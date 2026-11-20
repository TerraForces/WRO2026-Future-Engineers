# Additional hardware information
The following resources can help understand the PCB design

## Teensy 4.1 pinout
|Pin|Analog|UART|I2C|SPI|WRO|
|---|---|---|---|---|---|
|0||RX1||CS1|LCD (CS - out)|
|1||TX1||SDI1|LCD (MISO - in - NC)|
|2|||||VL53L1X shutdown FL + RC (out)|
|3|||||VL53L1X INT FL (in)|
|4|||||VL53L1X INT AL (in)|
|5|||||VL53L1X shutdown AL + RL (out)|
|6|||||VL53L1X INT RL (in)|
|7||RX2|||RPI5 power (out)|
|8||TX2|||drive reverse (out)|
|9|||||VL53L1X INT RC (in)|
|10|||||TCS3472 INT (in)|
|11||||SDO0|IMU INT (in)|
|12||||SDI0|LCD (LED - out)|
|13||||SCK0|board LED (out)|
|14|A0|TX3|||button INT (in)|
|15|A1|RX3|||NeoPixel (out)|
|16|A2|RX4|SCL1||I2C1 (SCL - bidirectional)|
|17|A3|TX4|SDA1||I2C1 (SDA - bidirectional)|
|18|A4||SDA0||I2C0 (SDA - bidirectional)|
|19|A5||SCL0||I2C0 (SCL - bidirectional)|
|20|A6|TX5|||converter current (ADC - in)|
|21|A7|RX5|||drive current (ADC - in)|
|22|A8||||drive PWM (out)|
|23|A9||||DS18B20 (OneWire - bidirectional)|
|24|A10|TX6|SCL2||I2C2 (SCL - bidirectional)|
|25|A11|RX6|SDA2||I2C2 (SDA - bidirectional)|
|26|A12|||SDO1|LCD (MOSI - out)|
|27|A13|||SCK1|LCD (SCK - out)|
|28|RX7||||RPI5 UART (RX - in)|
|29|TX7||||RPI5 UART (TX - out)|
|30|||||brake (out)|
|31|||||LCD (RS - out)|
|32|||||LCD (RES - out)|
|33|||||VL53L1X INT FR (in)|
|34||RX8|||VL53L1X INT RR (in)|
|35||TX8|||VL53L1X INT FC (in)|
|36|||||VL53L1X shutdown FC + RR (out)|
|37|||||servo PWM (out)|
|38|A14||||VL53L1X shutdown AR + FR (out)|
|39|A15||||VL53L1X INT AR (in)|
|40|A16||||battery voltage (ADC - in)|
|41|A17||||start button LED (out)|