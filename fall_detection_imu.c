#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
#define INT_EN 0x38
#define INT_PIN_CFG 0x37
#define INT_STATUS 0x3A
#define MPU6050_MOT_THR 0x1F
#define MPU6050_MOT_DUR 0x20

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Motion Detected on GPIO %lu", io_num);
        }
    }
}

void i2c_config() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000  
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));
}

esp_err_t write_register(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_NUM_0, MPU_6050, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t read_register(uint8_t reg, uint8_t* data) {
    return i2c_master_write_read_device(I2C_NUM_0, MPU_6050, &reg, 1, data, 1, pdMS_TO_TICKS(100));
}

void setFFInterrupt(bool active) {
    uint8_t int_enable;
    ESP_ERROR_CHECK(read_register(INT_EN, &int_enable));
    int_enable = active ? (int_enable | (1 << 7)) : (int_enable & ~(1 << 7));
    ESP_ERROR_CHECK(write_register(INT_EN, int_enable));
}

bool getFFInterruptStatus(void) {
    uint8_t status;
    ESP_ERROR_CHECK(read_register(INT_STATUS, &status));
    return (status & (1 << 7)) != 0;
}

void setFFThreshold(uint8_t thr) {
    ESP_ERROR_CHECK(write_register(FF_THR, thr));
}

void setFFDuration(uint8_t dur) {
    ESP_ERROR_CHECK(write_register(FF_DUR, dur));
}

void config_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INT_PIN, gpio_isr_handler, (void*) INT_PIN));
}

void set_accel_range(uint8_t range) {
    uint8_t data = range << 3;
    ESP_ERROR_CHECK(write_register(ACCEL_CONFIG, data));
}

void mpu_wake() {
    ESP_ERROR_CHECK(write_register(POWER, 0x00));
}

void i2c_read_acc(float *ax, float *ay, float *az) {
    uint8_t acc = 0x3B;
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_NUM_0, MPU_6050, &acc, 1, data, 6, pdMS_TO_TICKS(100)));

    int16_t raw_ax = ((int16_t)data[0] << 8) | data[1];
    int16_t raw_ay = ((int16_t)data[2] << 8) | data[3];
    int16_t raw_az = ((int16_t)data[4] << 8) | data[5];

    *ax = ((float)raw_ax / 16384.0) * 9.81;
    *ay = ((float)raw_ay / 16384.0) * 9.81;
    *az = ((float)raw_az / 16384.0) * 9.81;
}

void i2c_read_gyro(float *gx, float *gy, float *gz) {
    uint8_t gyro = 0x43;
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_NUM_0, MPU_6050, &gyro, 1, data, 6, pdMS_TO_TICKS(100)));

    *gx = ((int16_t)data[0] << 8) | data[1];
    *gy = ((int16_t)data[2] << 8) | data[3];
    *gz = ((int16_t)data[4] << 8) | data[5];
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing MPU6050...");
    
    i2c_config();
    config_gpio();
    mpu_wake();
    set_accel_range(0); 

    ESP_ERROR_CHECK(write_register(INT_PIN_CFG, 0xA0));  
    setFFInterrupt(true);
    setFFThreshold(0);  
    setFFDuration(0x01);  

    ESP_LOGI(TAG, "MPU6050 initialized. Starting main loop...");

    while(1) {
        float ax, ay, az, gx, gy, gz;
        i2c_read_acc(&ax, &ay, &az);
        i2c_read_gyro(&gx, &gy, &gz);
        ESP_LOGI(TAG, "Free Fall Interrupt Status: %d", getFFInterruptStatus());
        ESP_LOGI(TAG, "Acceleration: x=%.2f, y=%.2f, z=%.2f", ax, ay, az);
        ESP_LOGI(TAG, "Gyro Raw: x=%.2f, y=%.2f, z=%.2f", gx, gy, gz);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
