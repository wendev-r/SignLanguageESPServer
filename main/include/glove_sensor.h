#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soft_i2c_master.h"

void gpio_setup(void);
void read_glove_sensors(int *sensor_values);
void mpu6050_read(soft_i2c_master_bus_t bus, int16_t *accel, int16_t *gyro);