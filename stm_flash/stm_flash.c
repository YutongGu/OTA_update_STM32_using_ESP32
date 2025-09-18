#include "stm_flash.h"

static const char *TAG_STM_FLASH = "stm_flash";

esp_err_t writeTask(uint8_t *data_buff, size_t data_len)
{
    ESP_LOGI(TAG_STM_FLASH, "%s", "Write Task");

    char loadAddress[4] = {0x08, 0x00, 0x00, 0x00};
    char block[256] = {0};
    int curr_block = 0;

    setupSTM();

    int offset = 0;
    int data_remaining = data_len;
    while (data_remaining > 0)
    {
        int bytes_to_write = min(data_remaining, 256);
        if (bytes_to_write == 0) break;

        memcpy(block, data_buff + offset, bytes_to_write);
        curr_block++;
        ESP_LOGI(TAG_STM_FLASH, "Writing block: %d", curr_block);
        // ESP_LOG_BUFFER_HEXDUMP("Block:  ", block, sizeof(block), ESP_LOG_DEBUG);

        esp_err_t ret = flashPage(loadAddress, block);
        if (ret == ESP_FAIL)
        {
            return ESP_FAIL;
        }

        incrementLoadAddress(loadAddress);
        printf("\n");

        memset(block, 0xff, 256);
        offset += bytes_to_write;
        data_remaining -= bytes_to_write;
    }

    return ESP_OK;
}

esp_err_t readTask(uint8_t *data_buff, size_t data_len)
{
    ESP_LOGI(TAG_STM_FLASH, "%s", "Read & Verification Task");
    char readAddress[4] = {0x08, 0x00, 0x00, 0x00};

    char block[257] = {0};
    int curr_block = 0;

    int offset = 0;
    int data_remaining = data_len;
    while (data_remaining > 0)
    {
        int bytes_to_read = max(data_remaining, 256);
        if (bytes_to_read == 0) break;

        memcpy(block, data_buff + offset, bytes_to_read);
        curr_block++;
        ESP_LOGI(TAG_STM_FLASH, "Reading block: %d", curr_block);
        // ESP_LOG_BUFFER_HEXDUMP("Block:  ", block, sizeof(block), ESP_LOG_DEBUG);

        esp_err_t ret = readPage(readAddress, block);
        if (ret == ESP_FAIL)
        {
            return ESP_FAIL;
        }

        incrementLoadAddress(readAddress);
        printf("\n");

        memset(block, 0xff, 256);
        offset += bytes_to_read;
        data_remaining -= bytes_to_read;
    }

    return ESP_OK;
}

esp_err_t flashSTM(uint8_t *data_buff, size_t data_len)
{
    esp_err_t err = ESP_FAIL;

    initGPIO(GPIO_NUM_NC, GPIO_NUM_NC);
    initFlashUART(UART_NUM_MAX, GPIO_NUM_NC, GPIO_NUM_NC);

    err = writeTask(data_buff, data_len);
    if (err != ESP_OK)
    {
        return err;
    }

    err = readTask(data_buff, data_len);
    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(TAG_STM_FLASH, "%s", "Ending Connection");
    endConn();

    return err;
}