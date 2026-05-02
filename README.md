# Documentation of Team TerraForce for the Catergory Future Engineers of the World Robot Olympiad 2026

## Teensy Pinout
|Pin|Analog|UART|I2C|SPI|WRO|
|---|---|---|---|---|---|
|0||RX1||CS1|LCD (CS)|
|1||TX1||SDI1|LCD (NC)|
|2|||||VL53L1X Shut FL + RC|
|3|||||VL53L1X INT FL|
|4|||||VL53L1X INT AL|
|5|||||VL53L1X Shut AL + RL|
|6|||||VL53L1X INT RL|
|7||RX2|||RPI5 Power|
|8||TX2|||Drive Rev|
|9|||||VL53L1X INT RC|
|10|||||TCS3472 INT|
|11||||SDO0|Gyro INT|
|12||||SDI0|LCD (LED - PWM)|
|13||||SCK0|Board LED|
|14|A0|TX3|||Button INT|
|15|A1|RX3|||NeoPixel|
|16|A2|RX4|SCL1||I2C1|
|17|A3|TX4|SDA1||I2C1|
|18|A4||SDA0||I2C0|
|19|A5||SCL0||I2C0|
|20|A6|TX5|||Total Current|
|21|A7|RX5|||Drive Current|
|22|A8||||Drive PWM|
|23|A9||||DS18B20 (OneWire)|
|24|A10|TX6|SCL2||I2C2|
|25|A11|RX6|SDA2||I2C2|
|26|A12|||SDO1|LCD (MOSI)|
|27|A13|||SCK1|LCD (SCK)|
|28|RX7||||RPI5 UART|
|29|TX7||||RPI5 UART|
|30|||||Drive Brake|
|31|||||LCD (RS)|
|32|||||LCD (RES)|
|33|||||VL53L1X INT FR|
|34||RX8|||VL53L1X INT RR|
|35||TX8|||VL53L1X INT FC|
|36|||||VL53L1X Shut FC + RR|
|37|||||Servo PWM|
|38|A14||||VL53L1X Shut AR + FR|
|39|A15||||VL53L1X INT AR|
|40|A16||||Battery Voltage|
|41|A17||||Button LED|

## TODO
- DS18B20 driver
- I2C driver
- MPU9250 driver
- ST7735 driver
- TCS3472 driver
- VL53L1X driver
- execution time measurements for functions in [car.hpp](Teensy/src/car.hpp)