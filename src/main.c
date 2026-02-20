#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define LCD_HOST    SPI2_HOST//the spi bus we are working with
//defines for the spi connections
#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  4
#define PIN_NUM_CS   7

void app_main(void)
{
    esp_err_t ret;//esp_err_t holds error codes, ret catches returns values of functions
    //if succeed, ret = ESP_OK (0), if not, it holds the error code
    spi_device_handle_t spi;
    //handler: a pointer to the spi device 
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,//extra pins for Quad SPI mode, -1 = ignore
        .quadhd_io_num = -1,//extra pins for Quad SPI mode, -1 = ignore
        .max_transfer_sz = 8 //max transfer in bits, increase if larger data
    };
    spi_device_interface_config_t devcfg = {
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz = 26 * 1000 * 1000,     //Clock out at 26 MHz
#else
        .clock_speed_hz = 10 * 1000 * 1000,     //Clock out at 10 MHz
#endif
        .mode = 0,                              //SPI mode 0
        .spics_io_num = PIN_NUM_CS,             //CS pin
        .queue_size = 7, 
        //execute tasks as they come in order, one at a time. We want to be able to queue 7 transactions at a time
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);//SPI_DMA_CH_AUTO assigns a DMA Channel automatically
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    //this function fills spi variable with reference to the spi device
    ESP_ERROR_CHECK(ret);//Crashes loudly if ret is not ok
    printf("SPI initialized\n");

        // Create transaction
    spi_transaction_t ID;//transaction struct for BMI ID
    memset(&ID, 0, sizeof(ID));//fills the entire struct with zeros before using it.
    //necessary when dealing with garbage data. Memset sets the entire struct lenght starting at &t to 0
    ID.length = 16;//how many bits to transmit (address+dummy for read)
    ID.tx_data[0] = 0x00|0x80;//the byte to send. // Register 0x00, with read bit set (0x80)
    ID.tx_data[1] = 0x00;  // Dummy byte
    ID.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;//tells driver how to handle data buffers
    //use_txdata = use internal tx_data array, use_rxdata = use internal rx_data array
    ID.rxlength = 16;//how many bits to receive, must match t.length for full duplex

    // Send and receive
    ret = spi_device_transmit(spi, &ID);
    ESP_ERROR_CHECK(ret);

    // Print result
    printf("Device ID: 0x%02X\n",ID.rx_data[1]);

    // Step 1: Wake up the accelerometer
    spi_transaction_t t_wake_accel;
    memset(&t_wake_accel, 0, sizeof(t_wake_accel));
    t_wake_accel.length = 16;
    t_wake_accel.tx_data[0] = 0x7E;  // CMD register (power mode)
    t_wake_accel.tx_data[1] = 0x11;  // Normal mode for accel
    t_wake_accel.flags = SPI_TRANS_USE_TXDATA;
    spi_device_transmit(spi, &t_wake_accel);

    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for sensor to wake

    printf("Accelerometer powered on\n");

        // Step 1: Wake up the gyroscope
    spi_transaction_t t_wake_gyro;
    memset(&t_wake_gyro, 0, sizeof(t_wake_gyro));
    t_wake_gyro.length = 16;
    t_wake_gyro.tx_data[0] = 0x7E;  // CMD register (power mode)
    t_wake_gyro.tx_data[1] = 0x15;  // Normal mode for gyro
    t_wake_gyro.flags = SPI_TRANS_USE_TXDATA;
    spi_device_transmit(spi, &t_wake_gyro);

    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for sensor to wake

    printf("Gyroscope powered on\n");

    uint8_t tx_accel_buf[7] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0};  // Address + 6 dummy bytes
    uint8_t rx_accel_buf[7] = {0};  // Buffer for received data

    uint8_t tx_gyro_buf[7] = {0x0C | 0x80, 0, 0, 0, 0, 0, 0};  // Address + 6 dummy bytes
    uint8_t rx_gyro_buf[7] = {0};  // Buffer for received data
    // Step 2: Read accelerometer X, Y, Z (registers 0x12-0x17)
    while(1) {
         spi_transaction_t t_gyro;
        memset(&t_gyro, 0, sizeof(t_gyro));
        t_gyro.length = 56;  // 7 bytes: 1 addr + 6 data (X,Y,Z = 2 bytes each)

        t_gyro.tx_buffer = tx_gyro_buf;  // Use external buffer
          t_gyro.rx_buffer = rx_gyro_buf;  // Use external buffer
        t_gyro.rxlength = 56;
        
        spi_device_transmit(spi, &t_gyro);
          ESP_ERROR_CHECK(ret);
        
        // BMI160 data is little-endian: low byte first, then high byte
    int16_t gyro_x = (int16_t)((rx_gyro_buf[2] << 8) | rx_gyro_buf[1]);
    int16_t gyro_y = (int16_t)((rx_gyro_buf[4] << 8) | rx_gyro_buf[3]);
    int16_t gyro_z = (int16_t)((rx_gyro_buf[6] << 8) | rx_gyro_buf[5]);

        //accel transaction
        spi_transaction_t t_accel;
        memset(&t_accel, 0, sizeof(t_accel));
        t_accel.length = 56;  // 7 bytes: 1 addr + 6 data (X,Y,Z = 2 bytes each)

        t_accel.tx_buffer = tx_accel_buf;  // Use external buffer
          t_accel.rx_buffer = rx_accel_buf;  // Use external buffer
        t_accel.rxlength = 56;
        
        spi_device_transmit(spi, &t_accel);
          ESP_ERROR_CHECK(ret);
        
        // BMI160 data is little-endian: low byte first, then high byte
    int16_t accel_x = (int16_t)((rx_accel_buf[2] << 8) | rx_accel_buf[1]);
    int16_t accel_y = (int16_t)((rx_accel_buf[4] << 8) | rx_accel_buf[3]);
    int16_t accel_z = (int16_t)((rx_accel_buf[6] << 8) | rx_accel_buf[5]);
        
        printf("Accel X: %6d  Y: %6d  Z: %6d\n", accel_x, accel_y, accel_z);
        printf("Gyro X: %6d  Y: %6d  Z: %6d\n", gyro_x, gyro_y, gyro_z);
        vTaskDelay(pdMS_TO_TICKS(100));  // Read every 100ms
    }
    /*
    %-start of format specifier
    0-pad with zeros if number is short
    2-minimum two characters wide
    X-prints as uppercase hexadecimal
    we already wrote '0x' for visual format only

    */

}

