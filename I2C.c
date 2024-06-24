#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22
#define MPU_6050 0x68
#define MPU6050_ACCEL_X_H 0x3B
#define POWER 0x6B

i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

void i2c_config() {
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}
void mpu_wake(){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_6050 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, POWER, 1);
    i2c_master_write_byte(cmd, 0, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
void i2c_read(int16_t *ax,int16_t *ay,int16_t *az) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_6050 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_X_H, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_6050 << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd,data,6,I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
}

void app_main(void) {
    while(1){
    int16_t ax, ay, az;
    i2c_config();
    mpu_wake();
    i2c_read(&ax,&ay,&az);
    }
}
