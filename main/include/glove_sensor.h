#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void gpio_setup(void);
void read_glove_sensors(int *sensor_values);
void mpu6050_read(int i2c_port, int16_t *accel, int16_t *gyro);