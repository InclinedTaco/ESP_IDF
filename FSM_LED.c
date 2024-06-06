#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUTTON_PIN 23
#define LED_PIN 17


void app_main(void) {
   
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,  
        .clk_cfg          = LEDC_APB_CLK,
        .duty_resolution  = LEDC_TIMER_10_BIT 
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .channel   = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty      = 0,          
        .gpio_num  = LED_PIN,
        .hpoint    = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE
    };
    ledc_channel_config(&channel);

    
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_level(LED_PIN, 0); 

   
    int state = 0;
    int last_button_state = 1;
    int current_button_state=1;
    int button_press_detected = 0; 

    while (1) {  
        last_button_state = current_button_state;          
        current_button_state = gpio_get_level(BUTTON_PIN); 

        
        if (current_button_state != last_button_state && current_button_state == 1) { 
            button_press_detected = 1;  
        }

        
        if (button_press_detected) {
            button_press_detected = 0; 
            state = (state + 1) % 4;  
        }

        
        switch (state) {
            case 0:
                ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);        
                printf("OFF\n");                   
                break;
            case 1:
                ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,1023);
                ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);    
                printf("ONN\n");
                break;
            case 2:
                ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,1023);
                ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
                vTaskDelay(100/portTICK_PERIOD_MS);
                ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
                          
                printf("BLINK\n");
                break;
            case 3:
                 ledc_fade_func_install(0);
                 ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,1024,1000,LEDC_FADE_WAIT_DONE);
                 ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0,1000,LEDC_FADE_NO_WAIT);
                 state=0;
                 
                
                
        }        
        vTaskDelay(10 / portTICK_PERIOD_MS);               
    }
}
