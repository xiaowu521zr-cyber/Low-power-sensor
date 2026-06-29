/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴 IMU I2C 驱动程序实现
 *
 * 通过 I2C 总线与 MPU6050 进行通信。
 * 支持原始数据读取、传感器数据转换、软件校准以及用于休眠唤醒的运动检测中断配置。
 */

#include "drivers/mpu6050.h"
#include <math.h>
#include <string.h>

// 用于 ESP_LOG 日志打印的标签
static const char *TAG = "mpu6050";

// 全局静态变量，用于存储从 NVS 加载或现场校准得到的传感器零偏数据
static mpu6050_calib_t g_calib = {0};

/* ================================================================
 * I2C 底层辅助函数
 * ================================================================ */

/**
 * @brief 向 MPU6050 寄存器写入单个字节
 * @param reg 目标寄存器地址
 * @param value 要写入的数据
 * @return esp_err_t ESP_OK 表示成功，其他值表示失败
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    // 将寄存器地址和数据组合在同一个缓冲区中连续发送
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                      buf, 2, pdMS_TO_TICKS(100));
}

/**
 * @brief 从 MPU6050 寄存器读取单个字节
 * @param reg 目标寄存器地址
 * @param value 用于存储读取到数据的指针
 * @return esp_err_t ESP_OK 表示成功
 */
static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *value)
{
    // 先写入要读取的寄存器地址，然后执行复合传输读取 1 个字节
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

/**
 * @brief 从指定寄存器开始连续读取多个字节
 * @param reg 起始寄存器地址
 * @param data 存储读取数据的缓冲区指针
 * @param len 期待读取的字节长度
 */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    // MPU6050 支持寄存器地址自动递增，因此可以通过一次 I2C 传输连续读取多个寄存器（如 14 字节的传感器数据）
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/* ================================================================
 * 初始化
 * ================================================================ */

/**
 * @brief 主机 I2C 总线初始化配置
 */
esp_err_t mpu6050_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                         // 配置为 I2C 主机模式
        .sda_io_num = I2C_MASTER_SDA_IO,                 // SDA 引脚号
        .scl_io_num = I2C_MASTER_SCL_IO,                 // SCL 引脚号
        .sda_pullup_en = GPIO_PULLUP_DISABLE,             // 关闭内部上拉
        .scl_pullup_en = GPIO_PULLUP_DISABLE,             // 关闭内部上拉
        .master.clk_speed = I2C_MASTER_FREQ_HZ,          // I2C 工作频率（如 400kHz）
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,        // 默认时钟源选择
    };

    // 应用配置并安装 I2C 驱动
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C 初始化成功: SDA=GPIO%d, SCL=GPIO%d, Freq=%dHz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

/**
 * @brief MPU6050 芯片级配置初始化
 */
esp_err_t mpu6050_init(void)
{
    uint8_t whoami = 0;

    /* 验证器件身份 WHO_AM_I */
    esp_err_t ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &whoami);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 通信失败，无法读取 WHO_AM_I 寄存器!");
        return ret;
    }

    // MPU6050 的默认合法身份 ID 值为 0x68
    if (whoami != 0x68) {
        ESP_LOGE(TAG, "非法的 WHO_AM_I 身份码: 0x%02X (期望值: 0x68)", whoami);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "成功检测到 MPU6050 (WHO_AM_I: 0x%02X)", whoami);

    /* 唤醒 MPU6050 (清除 SLEEP 位，并推荐选择 X 轴陀螺仪作为时钟源以获得更稳定的时钟) */
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100)); // 留出足够的时间等待芯片内部 PLL 稳定

    /* 确保电源管理 2 寄存器清零，使能加速度计和陀螺仪的所有轴 */
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_2, 0x00);

    /* 配置数字低通滤波器 (DLPF) 级别 */
    mpu6050_write_reg(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG);

    /* 配置陀螺仪满量程范围 (Full-Scale Range) */
    mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS);

    /* 配置加速度计满量程范围 */
    mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS);

    /* 配置数据采样率分频器 
     * 采样率计算公式：Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
     * 注意：当启用 DLPF 时，Gyro Output Rate 恒为 1kHz */
    uint8_t smplrt_div = (1000 / MPU6050_SAMPLE_RATE) - 1;
    if (smplrt_div > 255) smplrt_div = 255; // 防止字节溢出
    mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, smplrt_div);

    /* 从非易失性存储中加载现有的校准数据 */
    mpu6050_calibrate(&g_calib);

    ESP_LOGI(TAG, "MPU6050 初始化完成: 采样率=%dHz, 陀螺仪量程代码=%d, 加速度计量程代码=%d",
             MPU6050_SAMPLE_RATE, MPU6050_GYRO_FS, MPU6050_ACCEL_FS);
    return ESP_OK;
}

/* ================================================================
 * 数据读取与解析
 * ================================================================ */

/**
 * @brief 读取 MPU6050 内部 14 字节的原始数据（加速度、温度、陀螺仪）
 */
esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data)
{
    uint8_t buf[14];
    esp_err_t ret;

    // 获取 I2C 互斥锁以确保多任务环境下的线程安全，超时时间设为 50ms
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // 一口气连续读取 14 个寄存器（从 ACCEL_XOUT_H 开始，直到 GYRO_ZOUT_L 结束）
    ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    xSemaphoreGive(g_i2c_mutex); // 释放锁

    if (ret != ESP_OK) return ret;

    /* 解析大端序 (Big-Endian) 的 16 位有符号整型数值
     * MPU6050 寄存器的高字节(H)在前，低字节(L)在后，需要通过位移拼接 */
    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->temperature = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

/**
 * @brief 读取并转化成标准物理单位的传感器数据，包含静态姿态结算
 */
esp_err_t mpu6050_read_sensor_data(sensor_data_t *data)
{
    mpu6050_raw_data_t raw;
    esp_err_t ret = mpu6050_read_raw(&raw);
    if (ret != ESP_OK) return ret;

    /* 将原始数据转换为物理单位 */
    /* 加速度计：当前设置为 ±16g 量程 → 对应 LSB 灵敏度 = 2048 LSB/g */
    float accel_x_g = (float)raw.accel_x / 2048.0f;
    float accel_y_g = (float)raw.accel_y / 2048.0f;
    float accel_z_g = (float)raw.accel_z / 2048.0f;

    /* 应用零偏校准（减去静止状态下的偏置） */
    accel_x_g -= (float)g_calib.accel_bias_x / 2048.0f;
    accel_y_g -= (float)g_calib.accel_bias_y / 2048.0f;
    accel_z_g -= (float)g_calib.accel_bias_z / 2048.0f;

    /* 陀螺仪：当前设置为 ±2000°/s 量程 → 对应 LSB 灵敏度 = 16.4 LSB/(°/s) */
    float gyro_x_dps = (float)raw.gyro_x / 16.4f;
    float gyro_y_dps = (float)raw.gyro_y / 16.4f;
    float gyro_z_dps = (float)raw.gyro_z / 16.4f;

    /* 应用陀螺仪零偏校准 */
    gyro_x_dps -= (float)g_calib.gyro_bias_x / 16.4f;
    gyro_y_dps -= (float)g_calib.gyro_bias_y / 16.4f;
    gyro_z_dps -= (float)g_calib.gyro_bias_z / 16.4f;

    /* 温度转换公式：摄氏度 = (TEMP_OUT / 340) + 36.53 (依据官方芯片手册) */
    float temp_c = ((float)raw.temperature / 340.0f) + 36.53f;

    /* 通过加速度计数据计算静态重力矢量下的 横滚角(Roll) 和 俯仰角(Pitch) */
    // Roll (横滚角): 绕 X 轴旋转的角度
    data->roll = atan2f(accel_y_g, accel_z_g) * 180.0f / M_PI;
    // Pitch (俯仰角): 绕 Y 轴旋转的角度
    data->pitch = atan2f(-accel_x_g, sqrtf(accel_y_g * accel_y_g +
                                            accel_z_g * accel_z_g)) * 180.0f / M_PI;

    /* 航向角(Yaw): 加速度计无法提供绝对水平几何参照，故静态设为 0，需后续通过陀螺仪积分或卡尔曼滤波进行动态更新 */
    float yaw = 0.0f;
    data->yaw = yaw;

    /* 填充传感器合矢量强度及其他元数据 */
    data->accel_mag = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
    data->gyro_mag = sqrtf(gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps);
    data->temperature_c = temp_c;
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS; // 记录当前系统毫秒时间戳
    data->sensor_id = DEVICE_TYPE;

    return ESP_OK;
}

/* ================================================================
 * 运动检测配置 (用于低功耗 Deep Sleep 外部中断唤醒)
 * ================================================================ */

/**
 * @brief 将 MPU6050 配置为低功耗加速度计突发中断模式（用于低功耗及震动唤醒）
 */
esp_err_t mpu6050_configure_motion_detect(void)
{
    esp_err_t ret;

    /* 正常唤醒 MPU6050 */
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 设置电源管理 2：关闭陀螺仪以省电，保持加速度计全轴激活 */
    uint8_t pwr_mgmt2 = 0x00;  
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_2, pwr_mgmt2);
    ESP_ERROR_CHECK(ret);

    /* 设置运动检测阈值 (Motion Threshold)：1 LSB = 1mg，该寄存器独立于加速度计的量程设置 */
    ret = mpu6050_write_reg(MPU6050_REG_MOT_THR, MPU6050_MOT_THRESHOLD);
    ESP_ERROR_CHECK(ret);

    /* 设置运动检测持续时间 (Motion Duration)：1 LSB = 1ms */
    ret = mpu6050_write_reg(MPU6050_REG_MOT_DUR, MPU6050_MOT_DURATION);
    ESP_ERROR_CHECK(ret);

    /* 先行读取并清除现存的所有中断状态位，防止历史未读中断导致引脚电平锁定 */
    uint8_t int_status;
    mpu6050_read_reg(MPU6050_REG_INT_STATUS, &int_status);

    /* 配置 INT 硬件引脚行为：高电平有效、推挽输出、锁存模式(LATCH_INT_EN=1，直到软件读取状态才清除中断) */
    ret = mpu6050_write_reg(MPU6050_REG_INT_PIN_CFG, 0x10);  
    ESP_ERROR_CHECK(ret);

    /* 开启运动检测中断使能 */
    ret = mpu6050_write_reg(MPU6050_REG_INT_ENABLE, MPU6050_INT_MOTION);
    ESP_ERROR_CHECK(ret);

    /* 启动低功耗循环休眠检测模式 (Cycle Mode)
     * 配合电源管理 2 中的唤醒频率控制（如 1.25Hz），芯片会自动定时唤醒加速度计检测震动 */
    uint8_t pwr_mgmt1 = MPU6050_CLOCK_PLL_XGYRO | 0x20;  // 0x20 对应第5位 CYCLE 比特
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, pwr_mgmt1);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "突发运动检测中断配置成功: 阈值=%dmg, 持续时间=%dms",
             MPU6050_MOT_THRESHOLD, MPU6050_MOT_DURATION);
    return ESP_OK;
}

/**
 * @brief 动态调节运动中断的震动阈值
 */
esp_err_t mpu6050_set_motion_threshold(uint8_t threshold)
{
    esp_err_t ret = mpu6050_write_reg(MPU6050_REG_MOT_THR, threshold);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "运动检测阈值成功更新为 %d mg", threshold);
    }
    return ret;
}

/**
 * @brief 动态调节常规数据输出的采样率
 */
esp_err_t mpu6050_set_sample_rate(uint16_t rate_hz)
{
    uint8_t smplrt_div = (1000 / rate_hz) - 1;
    if (smplrt_div > 255) smplrt_div = 255;
    esp_err_t ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, smplrt_div);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "采样率成功更新为 %d Hz", rate_hz);
    }
    return ret;
}

/* ================================================================
 * 传感器校准
 * ================================================================ */
/**
 * @brief 零偏校准函数，并在校准成功后保存偏置数据至NVS    
 */
esp_err_t mpu6050_calibrate(mpu6050_calib_t *calib)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    /* 首先尝试以只读模式打开 NVS，如果内部已有校准历史，则直接加载跳过现场校准 */
    ret = nvs_open("mpu6050_cal", NVS_READONLY, &nvs_handle);
    if (ret == ESP_OK) {
        size_t size = sizeof(mpu6050_calib_t);
        ret = nvs_get_blob(nvs_handle, "calib", calib, &size);
        nvs_close(nvs_handle);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "成功从永久存储 (NVS) 中加载历史校准数据");
            return ESP_OK;
        }
    }

    /* 现场自主校准：在传感器绝对静止时连续采集 200 个样本取平均值作为基准零偏 */
    ESP_LOGI(TAG, "未发现历史校准数据，正在启动现场校准（请保持传感器绝对静止）...");
    memset(calib, 0, sizeof(mpu6050_calib_t));

    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    const int samples = 200;

    for (int i = 0; i < samples; i++) {
        mpu6050_raw_data_t raw;
        if (mpu6050_read_raw(&raw) == ESP_OK) {
            gyro_sum_x += raw.gyro_x;
            gyro_sum_y += raw.gyro_y;
            gyro_sum_z += raw.gyro_z;
            accel_sum_x += raw.accel_x;
            accel_sum_y += raw.accel_y;
            accel_sum_z += raw.accel_z;
        }
        vTaskDelay(pdMS_TO_TICKS(5));  /* 采样间隔约 5ms (对应约 200Hz 频率) */
    }

    // 计算陀螺仪平均三轴零偏（静止时理论上应全为 0）
    calib->gyro_bias_x = (int16_t)(gyro_sum_x / samples);
    calib->gyro_bias_y = (int16_t)(gyro_sum_y / samples);
    calib->gyro_bias_z = (int16_t)(gyro_sum_z / samples);

    /* 加速度计偏置计算：核心注意 Z 轴 */
    calib->accel_bias_x = (int16_t)(accel_sum_x / samples);
    calib->accel_bias_y = (int16_t)(accel_sum_y / samples);
    /* 当芯片水平静止放置时，Z 轴会承受标准的 1g 地球重力。
     * 在 ±16g 量程下，1g 对应的数字原始值为 2048 LSB。
     * 因此校准 Z 轴偏置时，必须扣除这 1g 的理论物理值。 */
    calib->accel_bias_z = (int16_t)((accel_sum_z / samples) - 2048);  

    // 默认缩放系数设为 1.0（此处仅做线性位移偏置校准，未做多面旋转斜率校准）
    calib->gyro_scale_x = 1.0f;
    calib->gyro_scale_y = 1.0f;
    calib->gyro_scale_z = 1.0f;

    /* 以读写模式打开 NVS，将全新的校准数据写入非易失性闪存，防止掉电丢失 */
    ret = nvs_open("mpu6050_cal", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_set_blob(nvs_handle, "calib", calib, sizeof(mpu6050_calib_t));
        nvs_commit(nvs_handle); // 确保数据真正提交写入 Flash
        nvs_close(nvs_handle);
    }

    ESP_LOGI(TAG, "现场校准完毕并已保存。陀螺仪偏置=(%d,%d,%d), 加速度计偏置=(%d,%d,%d)",
             calib->gyro_bias_x, calib->gyro_bias_y, calib->gyro_bias_z,
             calib->accel_bias_x, calib->accel_bias_y, calib->accel_bias_z);
    return ESP_OK;
}

/**
 * @brief 器件在线自检
 */
esp_err_t mpu6050_self_test(void)
{
    uint8_t whoami;
    esp_err_t ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &whoami);
    if (ret != ESP_OK || whoami != 0x68) {
        return ESP_ERR_NOT_FOUND; // 通信不畅或找不到器件
    }
    return ESP_OK;
}

/**
 * @brief 控制 MPU6050 进入或退出低功耗完全睡眠模式
 * @param enable true 为进入睡眠状态（全芯片关断，仅保留 I2C 响应），false 为退出睡眠
 */
esp_err_t mpu6050_sleep(bool enable)
{
    uint8_t val;
    esp_err_t ret;

    ret = mpu6050_read_reg(MPU6050_REG_PWR_MGMT_1, &val);
    if (ret != ESP_OK) return ret;

    if (enable) {
        val |= 0x40;  /* 将 PWR_MGMT_1 的 SLEEP 位置 1 */
    } else {
        val &= ~0x40; /* 将 PWR_MGMT_1 的 SLEEP 位清零 */
    }

    return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, val);
}

/**
 * @brief 检查 ESP32 侧连接 MPU6050 INT 引脚的 GPIO 电平状态
 * @return true 表示当前有中断锁存触发，false 表示无中断
 */
bool mpu6050_check_int_pin(void)
{
    return (gpio_get_level(MPU6050_INT_GPIO) == 1);
}