# protocols
All of the following protocols run between the Teensy 4.1 and the specified sensors / controllers, the software implementations can be found in the [/src/Teensy/lib](/src/Teensy/lib/) directory:
## I2C
- <b>[ICM-20948](/src/Teensy/lib/ICM20948/)</b> based on the [SparkFun ICM-20948 library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main) and information provided in the [datasheet](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-03/ds-000189-icm-20948-datasheet.pdf?VersionId=DDZ6UQd2jbFGUCqi1PY4w_nR5jWpqKVN)
- <b>[TCS3472](/src/Teensy/lib/TCS3472/)</b> based on the information provided in the [datasheet](https://www.digikey.com/-/media/Images/Product%20Highlights/A/ams/sensor-driven-lighting/TCS3472_Datasheet_EN_v2.pdf?la=en-US&ts=3ffee11c-dfb5-4f09-aaca-5957eb0ff24b)
- <b>[VL53L1X](/src/Teensy/lib/VL53L1X/)</b> based on the [API library](https://www.st.com/en/embedded-software/stsw-img007.html) and information provided in the [datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
## NeoPixel
## SPI
- <b>[ST7735](/src/Teensy/lib/ST7735/)</b> based on the [Adafruit ST7735 library](https://github.com/adafruit/Adafruit-ST7735-Library)
## UART
- <b>[Raspberry Pi 5](/src/RPI5/main.cpp)</b>