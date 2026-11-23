# Changelog (latest first)

## TODO
- add IMU temperature compensation
- write engineering journal

## commit #32 "extended code comments, fix for access to 16-bit register addresses over I2C"
- added [changelog](/CHANGELOG.md) of past commits back to the international final invitation
- extended comments in [car.cpp](/src/Teensy/src/car.cpp), [neopixel.cpp](/src/Teensy/lib/NeoPixel/neopixel.cpp) and [i2c.hpp](/src/Teensy/lib/I2C/i2c.hpp)
- changed color order for Neopixel from GRB to the standard RGB in [car.hpp](/src/Teensy/src/car.hpp) and further in the [library](/src/Teensy/lib/NeoPixel/)
- corrected multi-byte register transfer endianess in [i2c.cpp](/src/Teensy/lib/I2C/i2c.cpp)
- changed [VL53L1X I2C writes](/src/Teensy/lib/VL53L1X/lib/vl53l1_platform.cpp) to the corrected [I2C library](/src/Teensy/lib/I2C/i2c.cpp) function
- added more references in [README.md](/README.md)
- changed path visualisation inner track/lane color to gray in all [path diagrams](/other/diagrams/)
- emphasized path diagram design in [README.md](/README.md)

## commit #31 "videos, hardware assembly and improvements table"
- added [hailo_yolo11s_recording.json](/src/RPI5/hailo_yolo11s_recording.json) and changed [recording.cpp](/src/RPI5/recording.cpp) to draw boxes around objects in the video recording
- added new [videos](/video/video.md) for [open](/video/open_challenge.mp4) and [obstacle](/video/obstacle_challenge.mp4) challenge
- updated links in [video.md](/video/video.md)
- added many [hardware images](/other/images/)
- extended [hardware assembly](/other/preparation.md#hardware-assembly)
- added full [PCB part list](/other/PCB_part_list.ods)
- added hardware improvements table to [README.md](/README.md)

## commit #30 "IMU changed back to MPU9250"
- due to severe problems of the ICM-20948 with sharp turns (even at the highest FSR), the IMU has been changed back to the MPU9250
    - [MPU9250 library](/src/Teensy/lib/MPU9250/) has been added back with small improvements
    - IMU interrupt pin support has been added back to [car.cpp](/src/Teensy/src/car.cpp)
- rotation data is now being logged periodically together with the ADC data
- fixed wall correction always checking the sensor in the front right in [main.cpp](/src/Teensy/src/main.cpp)

## commit #29 "added title image, changed time unit for most functions back to ms"
- added title image
- changed all time measurements in [car.cpp](/src/Teensy/src/car.cpp) from milliseconds to microseconds
- increased default rotation calibration time from 30 to 60 seconds
- reduced default rotation calibration revolutions from 16 to 8
- removed empty line in log files ([car.cpp](/src/Teensy/src/car.cpp))

## commit #28 "partial hardware assembly, path diagrams"
- added partial [hardware assembly instructions](/other/preparation.md#hardware-assembly)
- added many [hardware images](/other/images/)
- added more [path diagrams](/other/diagrams/) for obstacle challenge rounds and description in [README.md](/README.md)
- renamed [differential_gear_cover_cap.stl](/models/differential_gear_cap.stl) and [vl53l1x_mount_front.stl](/models/vl53l1x_mount_deep.stl) to provide a better description
- moved important files from other/doc to [other/design](/other/design/) and [other/diagrams](/other/diagrams/)
- experimental quaternion to euler rotation conversion fix ([icm20948.cpp](/src/Teensy/lib/ICM20948/icm20948.cpp))
- removed row of zeros in log files ([car.cpp](/src/Teensy/src/car.cpp))

## commit #27 "added models, introduction and support for multiple traffic signs per section"
- added all [3D models](/models/)
- added software information to [introduction](/README.md#introduction) with object path visualisation
- added support for multiple traffic signs per section in [main.cpp](/src/Teensy/src/main.cpp)
- added parallel parking in [main.cpp](/src/Teensy/src/main.cpp)
- added German driving demonstrations to [video.md](/video/video.md)
- moved driving demonstrations to [video](/video/)
- removed old VL53L1X measurement visualisations (outdated because of major software improvements)
- updated team photos [official.png](/t-photos/official.png) and [funny.png](/t-photos/funny.png)

## commit #26 "IMU calibration fixes, half FSR"
- modified rotation full scale range from 2000°/s to 1000°/s (increases accuracy)
- fixed accidental removal of an important function in [icm20948.cpp](/src/Teensy/lib/ICM20948/icm20948.hpp)
- fixed problems with the IMU calibration in [car.cpp](/src/Teensy/src/car.cpp)
- changed obstacle challenge back from European Championship version to German national finals version in [main.cpp](/src/Teensy/src/main.cpp)

## commit #27 "RPI5 autostart over systemd, extended documentation"
- translated the power tree visualisation [energy.drawio](/other/diagrams/energy.drawio) from German to English
- added [dark power tree visualisation design](/schemes/energy.png)
- renamed [light power tree visualisation design](/schemes/energy_light.png)
- extended [software preparation instructions](/other/preparation.md#software-preparation)
- changed Raspberry Pi program startup from .desktop to systemd service
- changed user specific paths in many Raspberry Pi related files to match the instructions
- added [recording.cpp](/src/RPI5/recording.cpp) for video recording while driving
- deleted experimental code simulation project
- changed the time unit in all functions in [car.cpp](/src/Teensy/src/car.cpp) from ms to us
- removed Raspberry Pi program start handshake
- extended comments in [library files](/src/Teensy/lib/)
- experimental port of the obstacle challenge code of the last season's European Championships in [main.cpp](/src/Teensy/src/main.cpp)

## commit #26 "imu/lcd fixes and translated photo descriptions"
- fixed IMU startup problems with functions from the [SparkFun ICM-20948 library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main)
- removed IMU INT pin use in [car.cpp](/src/Teensy/src/car.cpp)
- removed large VL53L1X test program from [car.cpp](/src/Teensy/src/car.cpp)
- extended data logging in [car.cpp](/src/Teensy/src/car.cpp)
- extended [ST7735 library](/src/Teensy/lib/ST7735/st7735.cpp) documentation
- translated all [vehicle photo labels](/v-photos/)

## commit #25 "restructuring, new IMU"
- added experimental code simulation project
- removed DS18B20 library and added it as an external dependency

> [!NOTE]
> We were originally not nominated for international competitions but we received an unexpected invitation to the International Final in Puerto Rico after the number of participating teams has been raised. Because of that, no development has been happening between the end of the national competitions in June and the invitation in October.