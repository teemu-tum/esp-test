#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "esp_log.h"

#include "bmp180.h"
#include "freq.h"

#define COLOR_RED (0)
#define COLOR_GREEN (1)
#define COLOR_BLUE (2)

#define GPIO_RED (25)
#define GPIO_GREEN (26)
#define GPIO_BLUE (27)

#define GPIO_BUZZER (33)

#define I2C_DATA_PIN (32)
#define I2C_CLOCK_PIN (33)

#define DUTY_MAX (4095)


static const char* TAG = "TestApp";

typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} color_t;

uint32_t color_to_duty( uint16_t color ) {
    return DUTY_MAX - color * 16;
}



void init_i2c() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_DATA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_CLOCK_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config( I2C_NUM_0, &conf );
    i2c_driver_install( I2C_NUM_0, conf.mode, 0, 0, 0);
}

void test_bmp180() {
    ESP_LOGI( TAG, "Initializing I2C" );
    init_i2c();

    ESP_LOGI( TAG, "Reading BMP180 EEPROM" );
    bmp180_eeprom_t bmp180_eeprom = { 0 };
    ESP_LOGI( TAG, "Initial values: A1=%d, A2=%d, A3=%d, A4=%u, A5=%u, A6=%u, b1=%d, b2=%d, mb=%d, mc=%d, md=%d",
        bmp180_eeprom.ac1, bmp180_eeprom.ac2, bmp180_eeprom.ac3,
        bmp180_eeprom.ac4, bmp180_eeprom.ac5, bmp180_eeprom.ac6, bmp180_eeprom.b1, bmp180_eeprom.b2,
        bmp180_eeprom.mb, bmp180_eeprom.mc, bmp180_eeprom.md );
    esp_err_t result;
    result = bmp180_read_eeprom( I2C_NUM_0, &bmp180_eeprom );
    if ( result == ESP_OK ) {
        ESP_LOGI( TAG, "Reading successful: A1=%d, A2=%d, A3=%d, A4=%u, A5=%u, A6=%u, b1=%d, b2=%d, mb=%d, mc=%d, md=%d",
        bmp180_eeprom.ac1, bmp180_eeprom.ac2, bmp180_eeprom.ac3,
        bmp180_eeprom.ac4, bmp180_eeprom.ac5, bmp180_eeprom.ac6, bmp180_eeprom.b1, bmp180_eeprom.b2,
        bmp180_eeprom.mb, bmp180_eeprom.mc, bmp180_eeprom.md );
    } else {
        ESP_LOGI( TAG, "Reading failed." );
    }

    ESP_LOGI( TAG, "Reading raw temperature" );
    uint16_t raw_temperature = 0;
    result = bmp180_read_raw_temperature( I2C_NUM_0, &raw_temperature );
    if ( result == ESP_OK ) {
        ESP_LOGI( TAG, "Success: raw temperature = %u", raw_temperature );
        int32_t temperature = bmp180_true_temperature( raw_temperature, bmp180_eeprom );
        float tmp = temperature / 10.0;
        ESP_LOGI( TAG, "Calibrated temperature: %d (%f C)", temperature, tmp );
    } else {
        ESP_LOGI( TAG, "Failed to read the raw temperature." );
    }

    ESP_LOGI( TAG, "Reading raw pressure" );
    bmp180_oversampling_t sampling = BMP180_SAMPLING_HIGH;
    uint32_t raw_pressure = 0;
    result = bmp180_read_raw_pressure( I2C_NUM_0, sampling, &raw_pressure );
    if ( result == ESP_OK ) {
        ESP_LOGI( TAG, "Success: raw pressure = %u", raw_pressure );
        int64_t pressure = bmp180_true_pressure( raw_pressure, raw_temperature, bmp180_eeprom,
            sampling );
        ESP_LOGI( TAG, "Calibrated pressure: %lld", pressure );
    } else {
        ESP_LOGI( TAG, "Failed to read the raw pressure." );
    }
}

void app_main() {

    

    /* Configure timer for PWM */    
    ledc_timer_config_t ledc_timer = {
        //set timer counter bit number
        .duty_resolution = LEDC_TIMER_12_BIT,
        //set frequency of pwm
        .freq_hz = 1000,
        //timer mode,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //timer index
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config( &ledc_timer );

    ledc_timer_config_t buzzer_timer = {
        //set timer counter bit number
        .duty_resolution = LEDC_TIMER_12_BIT,
        //set frequency of pwm
        .freq_hz = 1000,
        //timer mode,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //timer index
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config( &buzzer_timer );

    ledc_channel_config_t ledc_channel[ 3 ] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = DUTY_MAX,
            .gpio_num   = GPIO_RED,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = DUTY_MAX,
            .gpio_num   = GPIO_GREEN,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = DUTY_MAX,
            .gpio_num   = GPIO_BLUE,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
    };

    //set the configuration
    ledc_channel_config( &ledc_channel[ COLOR_RED ] );
    ledc_channel_config( &ledc_channel[ COLOR_GREEN ] );
    ledc_channel_config( &ledc_channel[ COLOR_BLUE ] );

    ledc_channel_config_t buzzer_channel = {
        .channel    = LEDC_CHANNEL_3,
        .gpio_num   = GPIO_BUZZER,
        .duty       = DUTY_MAX / 2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_1
    };
    ledc_channel_config( &buzzer_channel );
    
    color_t colors[ 5 ] = {
        {
            .red = 0xf6,
            .green = 0x51,
            .blue = 0x1d
        },
        {
            .red = 0xff,
            .green = 0xb4,
            .blue = 0x00
        },
        {
            .red = 0x00,
            .green = 0xa6,
            .blue = 0xed
        },
        {
            .red = 0x7f,
            .green = 0xb8,
            .blue = 0x00
        },
        {
            .red = 0x0d,
            .green = 0x2c,
            .blue = 0x54
        },

    };

    
    int color_i = 0;
    int song_i = 0;
    while (1) {
        
        // RGB
        ledc_set_duty( ledc_channel[ COLOR_RED ].speed_mode, ledc_channel[ COLOR_RED ].channel,
                        color_to_duty( colors[ color_i ].red ) );
        ledc_update_duty( ledc_channel[ COLOR_RED ].speed_mode, ledc_channel[ COLOR_RED ].channel );

        ledc_set_duty( ledc_channel[ COLOR_GREEN ].speed_mode, ledc_channel[ COLOR_GREEN ].channel,
                        color_to_duty( colors[ color_i ].green ) );
        ledc_update_duty( ledc_channel[ COLOR_GREEN ].speed_mode, ledc_channel[ COLOR_GREEN ].channel );

        ledc_set_duty( ledc_channel[ COLOR_BLUE ].speed_mode, ledc_channel[ COLOR_BLUE ].channel,
                        color_to_duty( colors[ color_i ].blue ) );
        ledc_update_duty( ledc_channel[ COLOR_BLUE ].speed_mode, ledc_channel[ COLOR_BLUE ].channel );

        // Buzzer
        esp_err_t song_err = ledc_set_freq( buzzer_channel.speed_mode, buzzer_channel.timer_sel, BWV147[ song_i ] );
        if ( song_err == ESP_ERR_INVALID_ARG ) {
            ESP_LOGI( TAG, "ledc_set_freg -> ESP_ERR_INVALID_ARG" );
        } else if ( song_err == ESP_FAIL ) {
            ESP_LOGI( TAG, "ledc_set_freg -> ESP_FAIL" );
        }

        color_i = ( color_i + 1 ) % 5;
        song_i = ( song_i + 1 ) % BWV147_LENGTH;

        vTaskDelay( 300 / portTICK_PERIOD_MS );
        ledc_timer_pause( buzzer_channel.speed_mode, buzzer_channel.timer_sel );
        vTaskDelay( 30 / portTICK_PERIOD_MS );
        ledc_timer_resume( buzzer_channel.speed_mode, buzzer_channel.timer_sel );
    }
    

}


