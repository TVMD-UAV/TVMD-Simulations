# TVMD Single Agent System Mother Board

This avionic system uses ESP32-s3 as the main MCU

- USB port for program upload

## Peripheral Devices
- IMU: ICM 20948 (SPI), IO voltage: 1.8V
- Barometer: BMP280 (SPI), IO voltage: 1.8V
- Distance Meter: VL53L0 (I2C)
- Optional Hall Sensors for servo angle measurement: QMC6308 (I2C)
    - X-axis Hall (EXT I2C)
    - y-axis Hall (I2C)
- 4 PWM Outputs
- UART port x1
- I2C port x1 (EXT I2C)

## Indicator
- WS2812 x2

## Pin Mapping

|  Function Name  |    GPIO    |   Notes   |
|-----------------|------------|-----------|
|   x-servo-pwm   |   GPIO39   |      |
|   y-servo-pwm   |   GPIO40   |      |
|   esc-p1-pwm    |   GPIO41   |      |
|   esc-p2-pwm    |   GPIO42   |      |
|   ws2812-input  |   GPIO21   |      |
|   I2C-SCL       |   GPIO8    |      |
|   I2C-SDA       |   GPIO9    |      |
|   EXT-I2C-SCL   |   GPIO6    |      |
|   EXT-I2C-SDA   |   GPIO7    |      |