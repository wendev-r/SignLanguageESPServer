#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soft_i2c_master.h"

#define GPIO_INPUT_IO_0 12
#define GPIO_INPUT_IO_1 14
#define GPIO_INPUT_IO_2 27
#define GPIO_INPUT_IO_3 26
#define GPIO_INPUT_IO_4 25

#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0) | \
                            (1ULL << GPIO_INPUT_IO_1) | \
                            (1ULL << GPIO_INPUT_IO_2) | \
                            (1ULL << GPIO_INPUT_IO_3) | \
                            (1ULL << GPIO_INPUT_IO_4))

#define MPU6050_ADDR             0x68
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

void gpio_setup() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void read_glove_sensors(int *sensor_values) {
    sensor_values[0] = gpio_get_level(GPIO_INPUT_IO_0);
    sensor_values[1] = gpio_get_level(GPIO_INPUT_IO_1);
    sensor_values[2] = gpio_get_level(GPIO_INPUT_IO_2);
    sensor_values[3] = gpio_get_level(GPIO_INPUT_IO_3);
    sensor_values[4] = gpio_get_level(GPIO_INPUT_IO_4);
}

void mpu6050_read(soft_i2c_master_bus_t bus, int16_t *accel, int16_t *gyro) {
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[14];
    if (soft_i2c_master_write_read(bus, MPU6050_ADDR, &reg, 1, data, 14) != ESP_OK) {
        printf("Failed to read MPU6050 data\n");
        return;
    }
    accel[0] = (data[0] << 8) | data[1];
    accel[1] = (data[2] << 8) | data[3];
    accel[2] = (data[4] << 8) | data[5];
    gyro[0]  = (data[8] << 8) | data[9];
    gyro[1]  = (data[10] << 8) | data[11];
    gyro[2]  = (data[12] << 8) | data[13];
}


// void app_main(void) {
//     gpio_setup();

//     // Setup MPU6050 I2C
//     soft_i2c_master_config_t config = {
//         .scl_pin = 22, // Set to your SCL pin
//         .sda_pin = 21, // Set to your SDA pin
//         .freq = SOFT_I2C_100KHZ
//     };
//     soft_i2c_master_bus_t bus;
//     if (soft_i2c_master_new(&config, &bus) != ESP_OK) {
//         printf("Failed to init soft I2C bus\n");
//         return;
//     }

//     // Wake up MPU6050
//     uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0};
//     if (soft_i2c_master_write(bus, MPU6050_ADDR, wake_cmd, 2) != ESP_OK) {
//         printf("Failed to wake MPU6050\n");
//         soft_i2c_master_del(bus);
//         return;
//     }

//     int sensor_values[5];
//     int16_t accel[3], gyro[3];

//     while (1) {
//         read_glove_sensors(sensor_values);
//         printf("Glove sensors: %d %d %d %d %d\n",
//             sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4]);

//         mpu6050_read(bus, accel, gyro);
//         printf("Accel: X=%d Y=%d Z=%d\n", accel[0], accel[1], accel[2]);
//         printf("Gyro:  X=%d Y=%d Z=%d\n", gyro[0], gyro[1], gyro[2]);

//         vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
//     }

//     soft_i2c_master_del(bus);
// }