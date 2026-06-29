/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴 IMU I2C 驱动程序头文件
 *
 * 支持功能：加速度计、陀螺仪、温度数据读取
 * 特性：支持基于 INT 引脚的运动唤醒中断配置、数字低通滤波器配置、软件校准管理
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "config/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * MPU6050 寄存器映射表 (Register Map)
 * ================================================================ */
#define MPU6050_REG_SMPLRT_DIV         0x19    /* 采样率分频器寄存器 */
#define MPU6050_REG_CONFIG             0x1A    /* 常规配置寄存器（数字低通滤波器 DLPF 设置） */
#define MPU6050_REG_GYRO_CONFIG        0x1B    /* 陀螺仪自检及满量程配置寄存器 */
#define MPU6050_REG_ACCEL_CONFIG       0x1C    /* 加速度计自检及满量程配置寄存器 */
#define MPU6050_REG_MOT_THR            0x1F    /* 运动检测触发阈值寄存器 (Wake-on-Motion) */
#define MPU6050_REG_MOT_DUR            0x20    /* 运动检测持续时间寄存器 */
#define MPU6050_REG_FIFO_EN            0x23    /* FIFO 使能寄存器 */
#define MPU6050_REG_INT_PIN_CFG        0x37    /* 中断引脚旁路与电平行为配置寄存器 */
#define MPU6050_REG_INT_ENABLE         0x38    /* 中断使能寄存器 */
#define MPU6050_REG_INT_STATUS         0x3A    /* 中断状态寄存器（读取后对应硬件位会自动清零） */

/* 传感器数据输出寄存器（皆为高低字节成对出现，拼接组成 16 位有符号大端序整型） */
#define MPU6050_REG_ACCEL_XOUT_H       0x3B    /* 加速度计 X 轴高 8 位 */
#define MPU6050_REG_ACCEL_XOUT_L       0x3C    /* 加速度计 X 轴低 8 位 */
#define MPU6050_REG_ACCEL_YOUT_H       0x3D    /* 加速度计 Y 轴高 8 位 */
#define MPU6050_REG_ACCEL_YOUT_L       0x3E    /* 加速度计 Y 轴低 8 位 */
#define MPU6050_REG_ACCEL_ZOUT_H       0x3F    /* 加速度计 Z 轴高 8 位 */
#define MPU6050_REG_ACCEL_ZOUT_L       0x40    /* 加速度计 Z 轴低 8 位 */
#define MPU6050_REG_TEMP_OUT_H         0x41    /* 内部温度传感器高 8 位 */
#define MPU6050_REG_TEMP_OUT_L         0x42    /* 内部温度传感器低 8 位 */
#define MPU6050_REG_GYRO_XOUT_H        0x43    /* 陀螺仪 X 轴高 8 位 */
#define MPU6050_REG_GYRO_XOUT_L        0x44    /* 陀螺仪 X 轴低 8 位 */
#define MPU6050_REG_GYRO_YOUT_H        0x45    /* 陀螺仪 Y 轴高 8 位 */
#define MPU6050_REG_GYRO_YOUT_L        0x46    /* 陀螺仪 Y 轴低 8 位 */
#define MPU6050_REG_GYRO_ZOUT_H        0x47    /* 陀螺仪 Z 轴高 8 位 */
#define MPU6050_REG_GYRO_ZOUT_L        0x48    /* 陀螺仪 Z 轴低 8 位 */

#define MPU6050_REG_PWR_MGMT_1         0x6B    /* 电源管理 1 寄存器（设备复位、休眠模式控制、时钟源配置） */
#define MPU6050_REG_PWR_MGMT_2         0x6C    /* 电源管理 2 寄存器（各轴低功耗待机模式及循环唤醒频率配置） */
#define MPU6050_REG_WHO_AM_I           0x75    /* 器件身份验证寄存器（合法设备默认返回固定值 0x68） */

/* ================================================================
 * MPU6050 配置可选参数值
 * ================================================================ */

/* 陀螺仪满量程选择 (Full-Scale Range) */
#define MPU6050_GYRO_FS_250            0x00    /* ±250°/s，  对应灵敏度：131 LSB/(°/s) */
#define MPU6050_GYRO_FS_500            0x08    /* ±500°/s，  对应灵敏度：65.5 LSB/(°/s) */
#define MPU6050_GYRO_FS_1000           0x10    /* ±1000°/s， 对应灵敏度：32.8 LSB/(°/s) */
#define MPU6050_GYRO_FS_2000           0x18    /* ±2000°/s， 对应灵敏度：16.4 LSB/(°/s) */

/* 加速度计满量程选择 (Full-Scale Range) */
#define MPU6050_ACCEL_FS_2G            0x00    /* ±2g，  对应灵敏度：16384 LSB/g */
#define MPU6050_ACCEL_FS_4G            0x08    /* ±4g，  对应灵敏度：8192 LSB/g */
#define MPU6050_ACCEL_FS_8G            0x10    /* ±8g，  对应灵敏度：4096 LSB/g */
#define MPU6050_ACCEL_FS_16G           0x18    /* ±16g， 对应灵敏度：2048 LSB/g */

/* DLPF (数字低通滤波器) 配置档位（值越小硬件滤波截止频率越低，抗噪好但延迟增加） */
#define MPU6050_DLPF_260_256           0x00    /* 加速度计带宽: 260Hz, 陀螺仪带宽: 256Hz */
#define MPU6050_DLPF_184_188           0x01    /* 加速度计带宽: 184Hz, 陀螺仪带宽: 188Hz */
#define MPU6050_DLPF_94_98             0x02    /* 加速度计带宽: 94Hz,  陀螺仪带宽: 98Hz */
#define MPU6050_DLPF_44_42             0x03    /* 加速度计带宽: 44Hz,  陀螺仪带宽: 42Hz */
#define MPU6050_DLPF_21_20             0x04    /* 加速度计带宽: 21Hz,  陀螺仪带宽: 20Hz */
#define MPU6050_DLPF_10_10             0x05    /* 加速度计带宽: 10Hz,  陀螺仪带宽: 10Hz */
#define MPU6050_DLPF_5_5               0x06    /* 加速度计带宽: 5Hz,   陀螺仪带宽: 5Hz */

/* 系统时钟源选择 */
#define MPU6050_CLOCK_INTERNAL         0x00    /* 内部 8MHz 阻容振荡器（温漂大，不推荐） */
#define MPU6050_CLOCK_PLL_XGYRO        0x01    /* 采用 X 轴陀螺仪锁相环作为时钟（推荐，时钟最稳定） */
#define MPU6050_CLOCK_PLL_YGYRO        0x02    /* 采用 Y 轴陀螺仪锁相环作为时钟 */
#define MPU6050_CLOCK_PLL_ZGYRO        0x03    /* 采用 Z 轴陀螺仪锁相环作为时钟 */

/* 中断标志触发控制位 */
#define MPU6050_INT_MOTION             0x40    /* 运动检测触发中断使能位 (Motion Detect) */
#define MPU6050_INT_DATA_RDY           0x01    /* 传感器新数据就绪中断使能位 (Data Ready) */

/* ================================================================
 * 传感器零偏校准数据结构体
 * ================================================================ */
typedef struct {
    int16_t gyro_bias_x;    /* 陀螺仪 X 轴静态零偏（寄存器原始 LSB 偏置值） */
    int16_t gyro_bias_y;    /* 陀螺仪 Y 轴静态零偏 */
    int16_t gyro_bias_z;    /* 陀螺仪 Z 轴静态零偏 */
    int16_t accel_bias_x;   /* 加速度计 X 轴静态零偏 */
    int16_t accel_bias_y;   /* 加速度计 Y 轴静态零偏 */
    int16_t accel_bias_z;   /* 加速度计 Z 轴静态零偏（已剔除 1g 地球重力基础值） */
    float gyro_scale_x;     /* 陀螺仪 X 轴灵敏度缩放比例系数（线性修正使用，默认 1.0） */
    float gyro_scale_y;     /* 陀螺仪 Y 轴灵敏度缩放比例系数 */
    float gyro_scale_z;     /* 陀螺仪 Z 轴灵敏度缩放比例系数 */
} mpu6050_calib_t;

/* ================================================================
 * 驱动级函数原型声明
 * ================================================================ */

/**
 * @brief 主机 I2C 硬件外设初始化
 * @return esp_err_t ESP_OK 表示配置并安装驱动成功
 */
esp_err_t mpu6050_i2c_init(void);

/**
 * @brief MPU6050 芯片上电配置与参数初始化
 * @note  包括唤醒、DLPF、量程以及自动加载 NVS 校准数据
 */
esp_err_t mpu6050_init(void);

/**
 * @brief 读取传感器 14 字节完整的寄存器原始数据 (精简高速)
 * @param data 接收原始 LSB 数据的结构体指针
 */
esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);

/**
 * @brief 读取传感器并转换为物理单位（包含零偏修正与静态 Roll/Pitch 姿态解算）
 * @param data 接收解算后标准物理单位数据的结构体指针
 */
esp_err_t mpu6050_read_sensor_data(sensor_data_t *data);

/**
 * @brief 配置 MPU6050 为低功耗运动检测突发中断模式
 * @note  主要用于 ESP32 芯片进入 Deep Sleep 时的外部震动硬件唤醒
 */
esp_err_t mpu6050_configure_motion_detect(void);

/**
 * @brief 动态调整运动检测中断的触发震动阈值
 * @param threshold 阈值大小 (1 LSB = 1mg)
 */
esp_err_t mpu6050_set_motion_threshold(uint8_t threshold);

/**
 * @brief 动态调整常规模式下的内部数据采样率
 * @param rate_hz 期望的采样频率（单位：Hz）
 */
esp_err_t mpu6050_set_sample_rate(uint16_t rate_hz);

/**
 * @brief 执行现场传感器零偏自主校准
 * @param calib 存放新计算出的偏置结构体指针，内部会自动同步固化到 NVS 中
 */
esp_err_t mpu6050_calibrate(mpu6050_calib_t *calib);

/**
 * @brief 传感器 I2C 在线通信连接自检 (WHO_AM_I 验证)
 * @return esp_err_t ESP_OK 代表芯片正常在线，ESP_ERR_NOT_FOUND 代表器件离线
 */
esp_err_t mpu6050_self_test(void);

/**
 * @brief 控制 MPU6050 整体进入或退出低功耗深度睡眠模式
 * @param enable true 为全芯片关断休眠，false 为退出休眠进入工作模式
 */
esp_err_t mpu6050_sleep(bool enable);

/**
 * @brief 检查主控板连接 MPU6050 INT 引脚的 GPIO 硬件电平状态
 * @return true 表示当前触发了锁存中断，false 表示当前无中断
 */
bool mpu6050_check_int_pin(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */