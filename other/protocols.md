# Protocols
All of the following protocols run between the Teensy 4.1 and the specified sensors / controllers, the software implementations can be found in the [/src/Teensy/lib](/src/Teensy/lib/) directory, except for the communication with the Raspberry Pi 5:

## I2C (fast mode = 400 kHz)
- <b>[ICM-20948](/src/Teensy/lib/ICM20948/):</b> based on the [SparkFun ICM-20948 library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main) and information provided in the [datasheet](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-03/ds-000189-icm-20948-datasheet.pdf?VersionId=DDZ6UQd2jbFGUCqi1PY4w_nR5jWpqKVN)
- <b>[TCS3472](/src/Teensy/lib/TCS3472/):</b> based on the information provided in the [datasheet](https://www.digikey.com/-/media/Images/Product%20Highlights/A/ams/sensor-driven-lighting/TCS3472_Datasheet_EN_v2.pdf?la=en-US&ts=3ffee11c-dfb5-4f09-aaca-5957eb0ff24b)
- <b>[VL53L1X](/src/Teensy/lib/VL53L1X/):</b> based on the [API library](https://www.st.com/en/embedded-software/stsw-img007.html) and information provided in the [datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)

## NeoPixel
- <b>[Adafruit NeoPixel LED ring](/src/Teensy/lib/NeoPixel/):</b> based on the [Adafruit NeoPixel library](https://github.com/adafruit/Adafruit_NeoPixel/tree/master)

## SPI (32 MHz)
- <b>[ST7735](/src/Teensy/lib/ST7735/):</b> based on the [Adafruit ST7735 library](https://github.com/adafruit/Adafruit-ST7735-Library)

## UART (500 kHz)
The protocol for the communication between our main MCU [Teensy 4.1](/src/Teensy/src/car.cpp) and our SBC [Raspberry Pi 5](/src/RPI5/main.cpp) has been designed to keep the required MCU processing time as low as possible and is therefore unidirectional: The Raspberry Pi 5 sends information about detected objects whenever new data is available; the Teensy 4.1 can then simply read it from the UART receive buffer.

Transmissions consist of packets of 12 bytes representing one detected object each:
- 2 bytes: x position of the bounding rect in image pixels (0 - 1295)
- 2 bytes: y position of the bounding rect in image pixels (0 - 1295)
- 2 bytes: bounding rect width in image pixels (0 - 1295)
- 2 bytes: bounding rect height in image pixels (0 - 1295)
- 2 bytes: object color (1 = red, 2 = green, 3 = pink)
- 2 bytes: detection probability in 0.1% (0 - 1000)

Every transmission starts with a packet where at least the object color field is 0 - this clears the receive buffer, then the detected objects follow, sorted by relevance for our driving algorithm (nearest non-pink object first).