/*
 * @author Abner Cordeiro 03/2024
 * 
 *      Código que implementa funções para utilizar o componente pcf8574p
 *      esse componente é um conversor I2C para paralelo de 8 bits( 8 GPIO )
 * 
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/i2c.h"
#include "i2c_def.h"
#include "pcf8574p.h"


esp_err_t i2c_master_init(void)
{
    esp_err_t esp_status = ESP_OK;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_status = i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);

    if (esp_status == ESP_OK)
    {
        esp_status = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);
    }

    return esp_status;
}

void app_main(void)
{
    uint8_t gpio_status = 0;
    uint8_t uiByte = 0x01;

    printf("Modulo: PCF8574P\n");

    i2c_master_init();
    pcf8574p_register_write_byte(PCF8574P_ADDR, 0xFF);

    for (char count_animat1 = 0; count_animat1 <= 10; count_animat1++)
    {
        for (char count = 0; count < 7; count++)
        {
            uiByte = ~uiByte;
            pcf8574p_register_write_byte(PCF8574P_ADDR, uiByte);
            uiByte = ~uiByte;
            uiByte <<= 1;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        for (char count = 0; count < 7; count++)
        {
            uiByte = ~uiByte;
            pcf8574p_register_write_byte(PCF8574P_ADDR, uiByte);
            uiByte = ~uiByte;
            uiByte >>= 1;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    while (1)
    {
        pcf8574p_set_gpio(PCF8574P_GPIO_P0);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_set_gpio(PCF8574P_GPIO_P1);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_set_gpio(PCF8574P_GPIO_P2);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_set_gpio(PCF8574P_GPIO_P3);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_clear_gpio(PCF8574P_GPIO_P0);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_clear_gpio(PCF8574P_GPIO_P1);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_clear_gpio(PCF8574P_GPIO_P2);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);

        pcf8574p_clear_gpio(PCF8574P_GPIO_P3);
        read_print_with_delay();

        pcf8574p_get_gpio(PCF8574P_GPIO_P0, &gpio_status);
    }
}
