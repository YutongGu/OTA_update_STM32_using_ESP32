#include "stm_pro_mode.h"

static const char *TAG_STM_PRO = "stm_pro_mode";

static gpio_num_t g_reset_pin = GPIO_NUM_NC;
static gpio_num_t g_boot0_pin = GPIO_NUM_NC;

static uart_port_t g_uart_port = UART_NUM_MAX;

//Functions for custom adjustments
void initFlashUART(uart_port_t uart_port, gpio_num_t txd_pin, gpio_num_t rxd_pin)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    g_uart_port = uart_port;

    uart_driver_install(g_uart_port, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(g_uart_port, &uart_config);
    uart_set_pin(g_uart_port, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG_STM_PRO, "%s", "Initialized Flash UART");
}

void initGPIO(gpio_num_t reset_pin, gpio_num_t boot0_pin)
{
    g_reset_pin = reset_pin;
    g_boot0_pin = boot0_pin;

    gpio_set_direction(g_reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(g_reset_pin, HIGH);
    gpio_set_direction(g_boot0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(g_boot0_pin, HIGH);

    ESP_LOGI(TAG_STM_PRO, "%s", "GPIO Initialized");
}

void resetSTM(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "Starting RESET Procedure");

    gpio_set_level(g_reset_pin, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(g_reset_pin, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG_STM_PRO, "%s", "Finished RESET Procedure");
}

void endConn(void)
{
    gpio_set_level(g_reset_pin, LOW);
    gpio_set_level(g_boot0_pin, LOW);

    resetSTM();

    ESP_LOGI(TAG_STM_PRO, "%s", "Ending Connection");
}

void setupSTM(void)
{
    resetSTM();
    cmdSync();
    cmdGet();
    cmdVersion();
    cmdId();
    cmdErase();
    cmdExtErase();
}

int cmdSync(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "SYNC");

    char bytes[] = {0x7F};
    int resp = 1;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int cmdGet(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "GET");

    char bytes[] = {0x00, 0xFF};
    int resp = 15;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int cmdVersion(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "GET VERSION & READ PROTECTION STATUS");

    char bytes[] = {0x01, 0xFE};
    int resp = 5;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int cmdId(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "CHECK ID");
    char bytes[] = {0x02, 0xFD};
    int resp = 5;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int cmdErase(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "ERASE MEMORY");
    char bytes[] = {0x43, 0xBC};
    int resp = 1;
    int a = sendBytes(bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[] = {0xFF, 0x00};
        resp = 1;

        return sendBytes(params, sizeof(params), resp);
    }
    return 0;
}

int cmdExtErase(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "EXTENDED ERASE MEMORY");
    char bytes[] = {0x44, 0xBB};
    int resp = 1;
    int a = sendBytes(bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[] = {0xFF, 0xFF, 0x00};
        resp = 1;

        return sendBytes(params, sizeof(params), resp);
    }
    return 0;
}

int cmdWrite(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "WRITE MEMORY");
    char bytes[2] = {0x31, 0xCE};
    int resp = 1;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int cmdRead(void)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "READ MEMORY");
    char bytes[2] = {0x11, 0xEE};
    int resp = 1;
    return sendBytes(bytes, sizeof(bytes), resp);
}

int loadAddress(const char adrMS, const char adrMI, const char adrLI, const char adrLS)
{
    char xor = adrMS ^ adrMI ^ adrLI ^ adrLS;
    char params[] = {adrMS, adrMI, adrLI, adrLS, xor};
    int resp = 1;

    // ESP_LOG_BUFFER_HEXDUMP("LOAD ADDR", params, sizeof(params), ESP_LOG_DEBUG);
    return sendBytes(params, sizeof(params), resp);
}

int sendBytes(const char *bytes, int count, int resp)
{
    sendData(TAG_STM_PRO, bytes, count);
    int length = waitForSerialData(resp, SERIAL_TIMEOUT);

    if (length > 0)
    {
        uint8_t data[length];
        const int rxBytes = uart_read_bytes(g_uart_port, data, length, 1000 / portTICK_PERIOD_MS);

        if (rxBytes > 0 && data[0] == ACK)
        {
            ESP_LOGI(TAG_STM_PRO, "%s", "Sync Success");
            // ESP_LOG_BUFFER_HEXDUMP("SYNC", data, rxBytes, ESP_LOG_DEBUG);
            return 1;
        }
        else
        {
            ESP_LOGE(TAG_STM_PRO, "%s", "Sync Failure");
            return 0;
        }
    }
    else
    {
        ESP_LOGE(TAG_STM_PRO, "%s", "Serial Timeout");
        return 0;
    }

    return 0;
}

int sendData(const char *logName, const char *data, const int count)
{
    const int txBytes = uart_write_bytes(g_uart_port, data, count);
    //ESP_LOGD(logName, "Wrote %d bytes", count);
    return txBytes;
}

int waitForSerialData(int dataCount, int timeout)
{
    int timer = 0;
    int length = 0;
    while (timer < timeout)
    {
        uart_get_buffered_data_len(g_uart_port, (size_t *)&length);
        if (length >= dataCount)
        {
            return length;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        timer++;
    }
    return 0;
}

void incrementLoadAddress(char *loadAddr)
{
    loadAddr[2] += 0x1;

    if (loadAddr[2] == 0)
    {
        loadAddr[1] += 0x1;

        if (loadAddr[1] == 0)
        {
            loadAddr[0] += 0x1;
        }
    }
}

esp_err_t flashPage(const char *address, const char *data)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "Flashing Page");

    cmdWrite();

    loadAddress(address[0], address[1], address[2], address[3]);

    //ESP_LOG_BUFFER_HEXDUMP("FLASH PAGE", data, 256, ESP_LOG_DEBUG);

    char xor = 0xFF;
    char sz = 0xFF;

    sendData(TAG_STM_PRO, &sz, 1);

    for (int i = 0; i < 256; i++)
    {
        sendData(TAG_STM_PRO, &data[i], 1);
        xor ^= data[i];
    }

    sendData(TAG_STM_PRO, &xor, 1);

    int length = waitForSerialData(1, SERIAL_TIMEOUT);
    if (length > 0)
    {
        uint8_t data[length];
        const int rxBytes = uart_read_bytes(g_uart_port, data, length, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0 && data[0] == ACK)
        {
            ESP_LOGI(TAG_STM_PRO, "%s", "Flash Success");
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG_STM_PRO, "%s", "Flash Failure");
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG_STM_PRO, "%s", "Serial Timeout");
    }
    return ESP_FAIL;
}

esp_err_t readPage(const char *address, const char *data)
{
    ESP_LOGI(TAG_STM_PRO, "%s", "Reading page");
    char param[] = {0xFF, 0x00};

    cmdRead();

    loadAddress(address[0], address[1], address[2], address[3]);

    sendData(TAG_STM_PRO, param, sizeof(param));
    int length = waitForSerialData(257, SERIAL_TIMEOUT);
    if (length > 0)
    {
        uint8_t uart_data[length];
        const int rxBytes = uart_read_bytes(g_uart_port, uart_data, length, 1000 / portTICK_PERIOD_MS);

        if (rxBytes > 0 && uart_data[0] == 0x79)
        {
            ESP_LOGI(TAG_STM_PRO, "%s", "Success");
            if (!memcpy((void *)data, uart_data, 257))
            {
                return ESP_FAIL;
            }

            //ESP_LOG_BUFFER_HEXDUMP("READ MEMORY", data, rxBytes, ESP_LOG_DEBUG);
        }
        else
        {
            ESP_LOGE(TAG_STM_PRO, "%s", "Failure");
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG_STM_PRO, "%s", "Serial Timeout");
        return ESP_FAIL;
    }

    return ESP_OK;
}