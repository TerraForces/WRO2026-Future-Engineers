# Changelog (latest first)

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