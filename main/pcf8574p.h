

#define PCF8574P_GPIO_P0 0b00000001
#define PCF8574P_GPIO_P1 0b00000010
#define PCF8574P_GPIO_P2 0b00000100
#define PCF8574P_GPIO_P3 0b00001000
#define PCF8574P_GPIO_P4 0b00010000
#define PCF8574P_GPIO_P5 0b00100000
#define PCF8574P_GPIO_P6 0b01000000
#define PCF8574P_GPIO_P7 0b10000000

#define PCF8574P_ADDR       0x20

esp_err_t pcf8574p_register_write_byte(uint8_t pcf8574p_addr, uint8_t data);

esp_err_t pcf8574p_register_read_byte(uint8_t pcf8574p_addr, uint8_t *data);

esp_err_t pcf8574p_set_gpio(uint8_t gpio_pin);

esp_err_t pcf8574p_clear_gpio(uint8_t gpio_pin);

esp_err_t pcf8574p_get_gpio(uint8_t gpio_pin,uint8_t *gpio_status);



