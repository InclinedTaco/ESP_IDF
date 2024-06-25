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
#define POWER 0x6B
#define TAG "MPU6050"
#define ACCEL_CONFIG 0x1C

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

void set_accel_range(uint8_t range) {
    uint8_t data = range << 3; 
    i2c_master_write_to_device(I2C_NUM_0, MPU_6050,data, sizeof(data), pdMS_TO_TICKS(100));
} 
void mpu_wake(){
    uint8_t cmd[] = {0x6B, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, MPU_6050, cmd, sizeof(cmd), pdMS_TO_TICKS(100));
}
void i2c_read(int16_t *ax,int16_t *ay,int16_t *az) {
    uint8_t acc=0x3B;
    uint8_t data[6];
    i2c_master_read_device(I2C_NUM_0,MPU_6050,&acc,1,data,pdMS_TO_TICKS(100));
    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];
        
    *ax = raw_ax*9.8 / 1638;
    *ay = raw_ay*9.8 / 1638;
    *az = raw_az*9.8 / 1638;
}

void app_main(void) {
    mpu_wake();
    set_accel_range(0);
    while(1){
    int16_t ax, ay, az;
    i2c_config();
    i2c_read(&ax,&ay,&az);
    ESP_LOGI(TAG,"Aceeleration: x= %d,y=%d,z=%d",ax,ay,az);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
