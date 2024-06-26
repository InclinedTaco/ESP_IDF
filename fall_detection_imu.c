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
#define TAG "DATA"
#define ACCEL_CONFIG 0x1C
#define FF_THR 0x1D	
#define FF_DUR 0x1E
#define INT_PIN 17
#define GYRO_CONFIG 0x1B


static void IRAM_ATTR gpio_isr_handler(void* arg) {
    ESP_LOGI(TAG, "FALL DETECTED");
}

void i2c_config() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}

void config_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_PIN, gpio_isr_handler, NULL);
}

void register_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    i2c_master_write_to_device(I2C_NUM_0, MPU_6050, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

void set_accel_range(uint8_t range) {
    uint8_t data = range << 3; 
    register_write_byte(ACCEL_CONFIG, data);
}

void mpu_wake() {
    register_write_byte(POWER, 0x00);
}

void config_ff(uint8_t thrsh, uint8_t dur) {
    register_write_byte(FF_THR, thrsh);
    register_write_byte(FF_DUR, dur);
}

void i2c_read_acc(float *ax, float *ay, float *az) {
    uint8_t acc = 0x3B;
    uint8_t data[6];
    i2c_master_write_read_device(I2C_NUM_0, MPU_6050, &acc, 1, data, 6, pdMS_TO_TICKS(100));
    
    int16_t raw_ax = ((int16_t)data[0] << 8) | data[1];
    int16_t raw_ay = ((int16_t)data[2] << 8) | data[3];
    int16_t raw_az = ((int16_t)data[4] << 8) | data[5];
    
    *ax = ((float)raw_ax / 16384.0) * 9.81;
    *ay = ((float)raw_ay / 16384.0) * 9.81;
    *az = ((float)raw_az / 16384.0) * 9.81;
}

void i2c_read_gyro(float *ax, float *ay, float *az) {
    uint8_t acc = 0x43;
    uint8_t data[6];
    i2c_master_write_read_device(I2C_NUM_0, MPU_6050, &acc, 1, data, 6, pdMS_TO_TICKS(100));
    
    *ax = ((int16_t)data[0] << 8) | data[1];
    *ay = ((int16_t)data[2] << 8) | data[3];
    *az = ((int16_t)data[4] << 8) | data[5];
}




void app_main(void) {
    i2c_config();
    config_gpio();
    mpu_wake();
    set_accel_range(0);
    config_ff(0x1F4, 0x06); 

    while(1) {
        float ax, ay, az,gx,gy,gz;
        i2c_read_acc(&ax, &ay, &az);
        i2c_read_gyro(&gx,&gy,&gz);
        ESP_LOGI(TAG, "Acceleration: x=%f, y=%f, z=%f", ax, ay, az);
        ESP_LOGI(TAG, "Gyro Raw: x=%f, y=%f, z=%f", gx, gy, gz);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
