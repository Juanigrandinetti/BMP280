# BMP280 Bosch sensor API

BMP280 is an absolute barometric pressure sensor. Here, its used to measure temperature, pressure and indirectly estimate altitude.

## About

This API was developed on an ESP-WROOM-32 for estimating the altitude of a drone therefore, deppending on your application you may want to change setted parameters.

## Get it to work

go to desire directory and run below line on terminal

```
    git clone https://github.com/Juanigrandinetti/BMP280.git
```

## Features

Below is a list of the most relevant features of the sensor

* Sample time: it operates in "Forced Mode" which means the microcontroller defines when to take a new measurement (See Bosch datasheet for maximum sample time).

* Communication: I2C interface with i2c_master api developed by esp-idf.