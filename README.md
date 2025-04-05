# BMP280 Bosch sensor API

The BMP280 is an absolute barometric pressure sensor. In this project, it's used to measure temperature, pressure and indirectly estimate altitude.

## About

This API was developed for the ESP-WROOM-32 microcontroller to estimate the altitude of a drone. Depending on your application you may need to adjust the sensor's configuration parameters accordingly.

## Getting started

Clone the repository into your desired directory using:

```
    git clone https://github.com/Juanigrandinetti/BMP280.git
```

## Features

Key features of this implementation

* Sampling mode: Operates in "Forced Mode", meaning the microcontroller controls when a new measurement is taken (refer to Bosch datasheet for timing details).

* Communication: Uses I2C with the i2c_master API from ESP-IDF.

## Example usage

Below is an example of implementation to measure temperature, pressure and estimate altitude.

```
    #include <stdio.h>
    #include <bmp280.h>
    #include <esp_log.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>


    #define BMP280_ADDR     0x76    /* Sensor address */
    #define GPIO_SDA        21      /* I2C SDA data */
    #define GPIO_SCL        22      /* I2C SCL clock */

    i2c_master_bus_handle_t master_i2c_bus_handler; /* I2C bus handler for master device (MCU) */


    void vTaskBmp280Measure(void * bmp);


    void app_main(void) {
        bmp280_t * bmp = Bmp280();

        if(bmp->init(bmp, &master_i2c_bus_handler, BMP280_ADDR, GPIO_SDA, GPIO_SCL) != ESP_OK) {
            return;
        }

        double p0 = 1032.0; /* Reference pressure in hPa */
        double t = 0.0;
        double p = 0.0;
        double z = 0.0;

        xTaskCreatePinnedToCore(vTaskBmp280Measure, "task1", 1024 * 2, (void *) bmp, 1, NULL, 1);

        while(1) {

            t = bmp->get_temp();
            p = bmp->get_press();
            z = bmp->get_altitude(p, p0);

            printf("Temperature: %lf °C\tPressure: %lf hPa\tAltitude: %lf m\r\n", t, p, z);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }


    void vTaskBmp280Measure(void * _bmp) {
        bmp280_t * bmp = (bmp280_t *) _bmp;

        while(1) {
            bmp->measure(bmp->i2c.bmp280_i2c_bus_handler);

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
```
