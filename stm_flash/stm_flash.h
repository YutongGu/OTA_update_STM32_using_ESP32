#ifndef _STM_FLASH_H
#define _STM_FLASH_H

#include "stm_pro_mode.h"

/**
 * @brief Write the code into the flash memory of STM32Fxx
 * 
 * The data from the data_buff is written into the flash memory 
 * of the client, block-by-block 
 * 
 * @param data_buff Data buffer to be flashed
 * @param data_len Length of the data buffer
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t writeTask(uint8_t *data_buff, size_t data_len);

/**
 * @brief Read the flash memory of the STM32Fxx, for verification
 * 
 * It reads the flash memory of the STM32 block-by-block and 
 * checks it with the data from the file (with pointer passed)
 * 
 * @param data_buff Data buffer to be verified
 * @param data_len Length of the data buffer
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t readTask(uint8_t *data_buff, size_t data_len);

/**
 * @brief Flash the .bin file passed, to STM32Fxx, with read verification
 * 
 * @param data_buff Data buffer to be flashed
 * @param data_len Length of the data buffer
 * @param reset_pin Reset pin of the STM32
 * @param boot0_pin Boot0 pin of the STM32
 * @param uart_port UART port of the ESP32
 * @param txd_pin TXD pin of the ESP32
 * @param rxd_pin RXD pin of the ESP32
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t flashSTM(
    uint8_t *data_buff,
    size_t data_len,
    gpio_num_t reset_pin,
    gpio_num_t boot0_pin,
    uart_port_t uart_port,
    gpio_num_t txd_pin,
    gpio_num_t rxd_pin
);

#endif
