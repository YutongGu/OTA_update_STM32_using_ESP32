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
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t flashSTM(uint8_t *data_buff, size_t data_len);

#endif
