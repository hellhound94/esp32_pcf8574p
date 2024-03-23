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
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c_def.h"
#include "pcf8574p.h"

/*! ************************************************************************************************************************\
 *  @brief envia 1 byte (data) para o endereço definido (pcf8574p_addr).
 * 
 *  @param pcf8574p_addr é um uint8_t utilizado para definir o endereço do PCF8574P.
 *  @param data é um uint8_t utilizado para passar o byte a ser escrito na saída do dispositivo.
 * 
 *  @return ESP_OK ou um código de erro, lista completa em: 
 *  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
************/
esp_err_t pcf8574p_register_write_byte(uint8_t pcf8574p_addr, uint8_t data)
{
    esp_err_t ret;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, pcf8574p_addr, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    return ret;
}

/*! ************************************************************************************************************************\
 *  @brief lê 1 byte armazenando em data do endereço definido pcf8574p_addr.
 * 
 *  @param pcf8574p_addr é um uint8_t utilizado para definir o endereço do PCF8574P.
 *  @param data é um uint8_t utilizado para receber o byte a ser lido da saída do dispositivo.
 * 
 *  @return ESP_OK ou um código de erro, lista completa em: 
 *  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
************/
esp_err_t pcf8574p_register_read_byte(uint8_t pcf8574p_addr, uint8_t *data)
{
    esp_err_t ret;

    ret = i2c_master_read_from_device(I2C_MASTER_NUM, pcf8574p_addr, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

    return ret;
}

/*! ************************************************************************************************************************\
 *  @brief seta o pino definido em gpio_pin.
 *  @param gpio_pin é um uint8_t utilizado para definir o pino que será setado.
 * 
 *  @return ESP_OK ou um código de erro, lista completa em: 
 *  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
************/
esp_err_t pcf8574p_set_gpio(uint8_t gpio_pin)
{
    uint8_t uiByteRead = 0;
    esp_err_t esp_status = pcf8574p_register_read_byte(PCF8574P_ADDR, &uiByteRead);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);

    if (esp_status == ESP_OK)
    {
        esp_status = pcf8574p_register_write_byte(PCF8574P_ADDR, uiByteRead | gpio_pin);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);
    }
    return esp_status;
}

/*! ************************************************************************************************************************\
 *  @brief zera o pino definido em gpio_pin.
 * 
 *  @param gpio_pin é um uint8_t utilizado para definir o pino que será zerado.
 * 
 *  @return ESP_OK ou um código de erro, lista completa em: 
 *  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
************/
esp_err_t pcf8574p_clear_gpio(uint8_t gpio_pin)
{
    uint8_t uiByteRead = 0;
    esp_err_t esp_status = pcf8574p_register_read_byte(PCF8574P_ADDR, &uiByteRead);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);

    if (esp_status == ESP_OK)
    {
        uiByteRead = ~uiByteRead;

        uiByteRead |= gpio_pin;

        uiByteRead = ~uiByteRead;

        esp_status = pcf8574p_register_write_byte(PCF8574P_ADDR, uiByteRead);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);
    }
    return esp_status;
}

/*! ************************************************************************************************************************\
 *  @brief lê o pino definido em gpio_pin para gpio_status.
 * 
 *  @param gpio_pin é um uint8_t utilizado para definir o pino que será lido.
 *  @param gpio_status é um uint8_t * onde receberá o status do pino.
 * 
 *  @return ESP_OK ou um código de erro, lista completa em: 
 *  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
************/
esp_err_t pcf8574p_get_gpio(uint8_t gpio_pin, uint8_t *gpio_status)
{
    uint8_t uiByteRead = 0;
    esp_err_t esp_status = pcf8574p_register_read_byte(PCF8574P_ADDR, &uiByteRead);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_status);

    *gpio_status = uiByteRead & gpio_pin;

    printf("BIT STATUS:%d\n\n", *gpio_status);

    return esp_status;
}

void print_bin(uint8_t ucByte)
{
    uint8_t ucFlag = 0b10000000;

    for (uint8_t iCount = 0; iCount < 8; iCount++, ucFlag >>= 1)
    {
        printf((ucByte & ucFlag) ? "1" : "0");
    }
    printf("\n");
}

void read_print_with_delay()
{
    uint8_t uiByteRead = 0;
    pcf8574p_register_read_byte(PCF8574P_ADDR, &uiByteRead);
    print_bin(uiByteRead);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
