#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "bme680.h"

#define PIN_NUM_MISO 13
#define PIN_NUM_CLK  12
#define PIN_NUM_MOSI 11
#define PIN_NUM_CS   10

#define TAG "main"
#define ENABLE_LOGGING      0

spi_device_handle_t spi;

/* SPI Config */
spi_device_handle_t spi_init(void) {
    spi_device_handle_t spi;
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (8 * 8)
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devCfg={
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .mode = 0,
        .clock_speed_hz = 2000000,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3
    };

    ret = spi_bus_add_device(SPI2_HOST, &devCfg, &spi);
    ESP_ERROR_CHECK(ret);

    return spi;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */

    vTaskDelay(pdMS_TO_TICKS(period));

}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

    uint8_t tx_data[len + 1];
    uint8_t rx_data[len + 1];

    tx_data[0] = reg_addr;

    for(int i = 1; i <= len; i++)
    {
        tx_data[i] = 0x00;
    }

    spi_transaction_t tM = {
        .tx_buffer = &tx_data,
        .rx_buffer = &rx_data,
        .length = 8 * (len + 1),
        .rxlength = 8 * (len + 1)
    };

    spi_device_acquire_bus(spi, portMAX_DELAY);
    spi_device_transmit(spi, &tM);
    spi_device_release_bus(spi);

    for(int i = 0; i < len; i++)
    {
        reg_data[i] = rx_data[i + 1];
    }

    if(ENABLE_LOGGING == 1)
    {
        printf("user_spi_read:\n");
        printf("tx_data = ");
        for(int i = 0; i < (len + 1); i++)
        {
            printf("0x%02x, ", tx_data[i]);
        }
        printf("\n");

        printf("rx_data = ");
        for(int i = 0; i < (len + 1); i++)
        {
            printf("0x%02x, ", rx_data[i]);
        }
        printf("\n");

        printf("reg_data = ");
        for(int i = 0; i < len; i++)
        {
            printf("0x%02x, ", reg_data[i]);
        }
        printf("\n");
    }

    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

    uint8_t tx_data[len + 1];

    tx_data[0] = reg_addr;

    for(int i = 0; i < len; i++)
    {
        tx_data[i + 1] = (uint8_t)reg_data[i];
    }

    spi_transaction_t tM = {
        .tx_buffer = &tx_data,
        .rx_buffer = NULL,
        .length = 8 * (len + 1),
    };

    spi_device_acquire_bus(spi, portMAX_DELAY);
    spi_device_transmit(spi, &tM);
    spi_device_release_bus(spi);

    if(ENABLE_LOGGING == 1)
    {
        printf("user_spi_write:\n");
        printf("tx_data = ");
        for(int i = 0; i < (len + 1); i++)
        {
            printf("0x%02x, ", tx_data[i]);
        }
        printf("\n");
    }

    return rslt;
}

void app_main(void)
{
    //spi_device_handle_t spi;
    spi = spi_init();

/*    uint8_t rx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t tx_data[8] = {0xD0, 0xAD, 0xBE, 0xEF, 0xC0, 0x00, 0xAA, 0x55};

    spi_transaction_t tM = {
        //.flags = SPI_TRANS_USE_TXDATA, //SPI_TRANS_USE_RXDATA,
        .tx_buffer = &tx_data,
        .rx_buffer = &rx_data,
        .length = 8 * 8,
        .rxlength = 8 * 8,
    };
*/
    // ///////////////////////////
    // BME680 related
    // ///////////////////////////

    // ///////////////////////////
    // Initializing the sensor
    // ///////////////////////////
    struct bme680_dev gas_sensor;

    /* You may assign a chip select identifier to be handled later */
    gas_sensor.dev_id = 0;
    gas_sensor.intf = BME680_SPI_INTF;
    gas_sensor.read = user_spi_read;
    gas_sensor.write = user_spi_write;
    gas_sensor.delay_ms = user_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor.amb_temp = 20;

    int8_t rslt = BME680_OK;
    rslt = bme680_init(&gas_sensor);

    // ///////////////////////////
    // Configuring the sensor
    // ///////////////////////////
    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);

    // ///////////////////////////
    // Reading sensor data
    // ///////////////////////////
    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    struct bme680_field_data data;

    while(1)
    {
        user_delay_ms(meas_period); /* Delay till the measurement is ready */

        rslt = bme680_get_sensor_data(&data, &gas_sensor);

        printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            printf(", G: %d ohms", data.gas_resistance);

        printf("\r\n");

        /* Trigger the next measurement if you would like to read data out continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(60000));
    }

/*    for (;;) {
        spi_device_acquire_bus(spi, portMAX_DELAY);
        spi_device_transmit(spi, &tM);
        spi_device_release_bus(spi);

        //int16_t res = (int16_t) SPI_SWAP_DATA_RX(rx_data, 16);
        //printf("Raw data = %d", res);
        printf("%d, %d, %d, %d, %d, %d, %d, %d\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

//    if (res & (1 << 2))
//      ESP_LOGE(TAG, "Sensor is not connected\n");
//    else {
//      res >>= 3;
//      printf("SPI res = %d temp=%f\n", res, res * 0.25);
//    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    }*/

    //xTaskCreate(&temp_task, "temperature_task", 4096, spi, 5, NULL);

}