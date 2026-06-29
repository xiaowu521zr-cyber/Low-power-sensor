# ESP32-C6 MPU6050 Gyroscope Sensor Node
# LoRa Wireless Communication + ULP LP-Core Co-processor
# FreeRTOS based firmware
cmake_minimum_required(VERSION 3.16)

# Set project name
set(PROJECT_NAME "mpu6050_gyro_node")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})
