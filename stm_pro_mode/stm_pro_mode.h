#ifndef _STM_PRO_MODE_H
#define _STM_PRO_MODE_H

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>

#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"

//Macro for error checking
#define IS_ESP_OK(x) if ((x) != ESP_OK) break;

#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

#define HIGH 1
#define LOW 0

#define ACK 0x79
#define SERIAL_TIMEOUT 5000

//Initialize UART functionalities
void initFlashUART(uart_port_t uart_port, gpio_num_t txd_pin, gpio_num_t rxd_pin);

//Initialize GPIO functionalities
void initGPIO(gpio_num_t reset_pin, gpio_num_t boot0_pin);

//Reset the STM32Fxx
void resetSTM(void);

//Increment the memory address for the next write operation
void incrementLoadAddress(char *loadAddr);

//End the connection with STM32Fxx
void endConn(void);

//Get in sync with STM32Fxx
int cmdSync(void);

//Get the version and the allowed commands supported by the current version of the bootloader
int cmdGet(void);

//Get the bootloader version and the Read Protection status of the Flash memory
int cmdVersion(void);

//Get the chip ID
int cmdId(void);

//Erase from one to all the Flash memory pages
int cmdErase(void);

//Erases from one to all the Flash memory pages using 2-byte addressing mode
int cmdExtErase(void);

//Setup STM32Fxx for the 'flashing' process
void setupSTM(void);

//Write data to flash memory address
int cmdWrite(void);

//Read data from flash memory address
int cmdRead(void);

//UART send data to STM32Fxx & wait for response
int sendBytes(const char *bytes, int count, int resp);

//UART send data byte-by-byte to STM32Fxx
int sendData(const char *logName, const char *data, const int count);

//Wait for response from STM32Fxx
int waitForSerialData(int dataCount, int timeout);

//Send the STM32Fxx the memory address, to be written
int loadAddress(const char adrMS, const char adrMI, const char adrLI, const char adrLS);

//UART write the flash memory address of the STM32Fxx with blocks of data 
esp_err_t flashPage(const char *address, const char *data);

//UART read the flash memory address of the STM32Fxx and verify with the given block of data 
esp_err_t readPage(const char *address, const char *data);

#endif
