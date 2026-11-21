# Hardware assembly
## complete part list
quantity|part name|image
-|-|-
1|[battery compartment printed from PLA](/models/battery_compartment.stl)|<img height="100" src="images/battery_compartment.png">
1|[camera mount printed from PLA](/models/camera_mount.stl)|<img height="100" src="images/camera_mount.png">
1|[camera mount base printed from PLA](/models/camera_mount_base.stl)|<img height="100" src="images/camera_mount_base.png">
1|[chassis printed from PLA](/models/chassis.stl)|<img height="100" src="images/chassis.png">
1|[differential gear cap printed from PLA](/models/differential_gear_cap.stl)|<img height="100" src="images/differential_gear_cap.png">
1|[differential gear cross printed from PLA](/models/differential_gear_cross.stl)|<img height="100" src="images/differential_gear_cross.png">
1|[differential gear enclosure printed from PLA](/models/differential_gear_enclosure.stl)|<img height="100" src="images/differential_gear_enclosure.png">
1|[drilling template printed from PLA](/models/drilling_template.stl)|<img height="100" src="images/drilling_template.png">
1|[drive shaft block left printed from PLA](/models/drive_shaft_block_left.stl)|<img height="100" src="images/drive_shaft_block_left.png">
1|[drive shaft block right printed from PLA](/models/drive_shaft_block_right.stl)|<img height="100" src="images/drive_shaft_block_right.png">
1|[motor mount printed from PLA](/models/motor_mount.stl)|<img height="100" src="images/motor_mount.png">
1|[servo gear printed from PLA](/models/servo_gear.stl)|<img height="100" src="images/servo_gear.png">
2|[servo spacer printed from PLA](/models/servo_spacer.stl)|<img height="100" src="images/servo_spacer.png">
2|[steering gear printed from PLA](/models/steering_gear.stl)|<img height="100" src="images/steering_gear.png">
1|[steering top plate printed from PLA](/models/steering_top_plate.stl)|<img height="100" src="images/steering_top_plate.png">
3|[VL53L1X mount printed from PLA](/models/vl53l1x_mount.stl)|<img height="100" src="images/vl53l1x_mount.png">
3|[VL53L1X mount (deep) printed from PLA](/models/vl53l1x_mount_deep.stl)|<img height="100" src="images/vl53l1x_mount_deep.png">
1|[M3 stainless steel flat head screws and nuts set](https://www.amazon.de/dp/B0DPKYXVWY)|<img height="100" src="images/M3_1.jpg">
1|[M3 stainless steel hex screw set](https://www.amazon.de/dp/B08F4SVS2Y)|<img height="100" src="images/M3_2.jpg">
1|[M3 brass spacer set](https://www.amazon.de/dp/B094337JLF)|<img height="100" src="images/M3_3.jpg">
2|[brass bevel gears 0.8 15T 3mm hole (2 pieces)](https://www.amazon.de/dp/B0DS1V1BCV)|<img height="100" src="images/bevel_gears.png">
1|[stainless steel round rods 3mm](https://www.amazon.de/dp/B0D1CFZ9PX)|<img height="100" src="images/steel_rods.png">
1|[chrome steel ball bearings 3 x 6 x 2.5 mm (20 pieces)](https://www.amazon.de/dp/B0CR1PY2W2)|<img height="100" src="images/ball_bearings.jpg">

## 1. differential gear
### required parts
1. 1x [differential gear enclosure](/models/differential_gear_enclosure.stl)
2. 5x M3 washer from [M3 stainless steel hex screw set](https://www.amazon.de/dp/B08F4SVS2Y)
3. [2x brass bevel gear 0.8 15T 3mm](https://www.amazon.de/dp/B0DS1V1BCV) without mounting provision
4. 1x [differential gear cross printed from PLA](/models/differential_gear_cross.stl)
5. [stainless steel round rods 3mm](https://www.amazon.de/dp/B0D1CFZ9PX) cut to length in vehicle
6. 1x ball bearing from [chrome steel ball bearings 3 x 6 x 2.5 mm (20 pieces)](https://www.amazon.de/dp/B0CR1PY2W2)
7. [2x brass bevel gear 0.8 15T 3mm](https://www.amazon.de/dp/B0DS1V1BCV)
8. 2x long screws from [M3 stainless steel hex screw set](https://www.amazon.de/dp/B08F4SVS2Y)
9.

### assembly
![](images/differential_gear_assembly_1.png)
![](images/differential_gear_assembly_2.png)
![](images/differential_gear_assembly_3.png)

## 2. drive motor
### required parts

### assembly
![](images/drive_motor_assembly_1.png)
![](images/drive_motor_assembly_2.png)

## 4. front wheels
### required parts

### assembly

## 5. steering gear
### required parts

### assembly

## 6. steering
### required parts

### assembly

## 7. chassis
### required parts

### assembly

## 2. drive shaft
### required parts

### assembly
![](images/drive_shaft_assembly_1.png)
![](images/drive_shaft_assembly_2.png)

## 8. PCBs
### required parts

### assembly

## 9. camera
### required parts

### assembly

## 10. battery
### required parts

### assembly

# Software preparation
## Required external parts
- laptop or desktop computer with Windows, macOS or a Linux distro of choice (macOS has not been tested but should be supported) and an internet connection
- cable with a USB 2 micro-B plug to connect to the Teensy 4.1
- USB keyboard and mouse (a USB hub with a slim connector is recommended for connecting to the Raspberry Pi 5 while it is installed in the vehicle)
- external monitor and a video cable with a micro-HDMI plug for the Raspberry Pi 5
- a WiFi network connected to the internet (or an Ethernet cable with the shielding of one plug removed, so it can't hit the fuse mount)

for the ATtiny841 (optional):
- AVR ISP programmer
- AVR 6-pin ISP 2.54 mm to 1.27 mm adapter

## Teensy 4.1
1. download and install [Visual Studio Code (VSCode)](https://code.visualstudio.com/download)
2. install the [PlatformIO](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode) extension in VSCode (might need a few minutes)
3. download and open the [/src/Teensy](/src/Teensy/) directory of this repository in VSCode, the PlatformIO extension should automatically download the necessary framework and library files (might need a few minutes too)
4. select the program by (un-)commenting the program selection lines (7/8) at the start of the [main.cpp](https://github.com/TerraForces/WRO2026-Future-Engineers/blob/master/src/Teensy/src/main.cpp) file:
    - for the calibration program / sensor tests:
        ```
        #define TEST
        //#define OBJECTS
        ```
    - for camera / object detection tests:
        ```
        //#define TEST
        #define OBJECTS
        ```
    - for the Open Challenge:
        ```
        //#define TEST
        //#define OBJECTS
        ```
    - for the Obstacle Challenge:
        ```
        //#define TEST
        #define OBJECTS
        ```
5. make sure that the vehicle is switched off, then put the [upload switch](/v-photos/top.png) into the "PGM" position and connect the Teensy 4.1 to the computer using the USB cable
6. press the upload button in the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode-toolbar)
7. on some operating systems additional steps might be needed (like installing configuration files for port access in Linux), please follow the instructions provided in the terminal output
8. the terminal or a small window should indicate a successful upload
9. disconnect the USB cable from the Teensy 4.1
10. put the [upload switch](/v-photos/top.png) into the "RUN" position, this reconnects the Teensy 4.1 to the vehicle power

## Raspberry Pi 5
1. plug a microSD card into your computer / laptop (make sure that the contents of this card are not needed anymore)
2. download the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and start it (administrator privileges might be required)
3. select the Raspberry Pi 5, then Raspberry Pi OS (64-bit)
4. select your microSD card as the storage device
5. customize the install - most things are not important, but the following ones are:
    - `keyboard layout` must match the layout of the keyboard you are planning to use with the Raspberry Pi 5
    - `username` must be `wro`, since all programs and commands expect this one
    - please remember what you wrote into `password`, you'll have to enter it most of the time when a `sudo` command needs to be executed
6. confirm that the data should be written and wait for it to finish (could take a few minutes)
7. once the data has been written and verified, put the microSD card into the microSD slot of the Raspberry Pi 5 and connect the keyboard, mouse and monitor
8. upload the object detection test program on the Teensy 4.1 according to the guide above and power everything on
9. wait for the Raspberry Pi 5 to boot into the desktop environment, then connect it to the internet (WiFi recommended)
10. open a command line and update the operating system and EEPROM according to [https://www.raspberrypi.com/documentation/computers/ai.html](https://www.raspberrypi.com/documentation/computers/ai.html#software) (this could take a few minutes):
    ```
    sudo apt update
    sudo apt full-upgrade -y
    sudo rpi-eeprom-update -a
    sudo reboot
    ```
    after the reboot, reopen the command line and install the required software packages:
    ```
    sudo apt install dkms -y
    sudo apt install hailo-all -y
    ```
11. check camera functionality, e.g. using `rpicam-hello` (shows camera video stream for a few seconds - the video stream should be rotated by 180° since the camera is mounted upside down)
12. check whether the AI accelerator is being detected using `hailortcli fw-control identify` (should print information about the connected accelerator when detected, otherwise prints nothing)
13. run `sudo raspi-config`:
    - in `3 Interface Options` select `I6 Serial Port`, then `No`, then `Yes` to enable the Serial Port / UART and exit by selecting `Ok`
    - in `6 Advanced Option` select `A8 PCIe Speed`, then `Yes` to enable PCIe 3.0 speeds and exit by selecting `Ok`
    - leave the configuration tool by selecting `Finish` and select `Yes` to reboot
14. download/copy the content of the [/src/RPI5](/src/RPI5/) directory of the repository into /home/wro/Documents
15. open a command line and compile the [code](/src/RPI5/main.cpp) using
    ```
    g++ ~/Documents/main.cpp -o ~/Documents/main -Ofast
    ```
16. enable autostart for the program by creating and starting a systemd service:
    ```
    sudo mv ~/Documents/WRO.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable --now WRO.service
    ```
17. when everything is working correctly, the lcd screen of the vehicle should now show that it's ready and display objects as they are visible to the camera

## ATtiny841 (optional)
1. completely disconnect the [battery check PCB](/schemes/batteryCheck.png) from the vehicle (while it's powered off)
2. remove the protection cap from the 1.27 mm ISP connector
3. connect the MCU to your computer (using the 1.27 to 2.54 mm adapter and the ISP programmer, the ground pin should face away from the battery level display)
4. upload the program to the ATtiny841, for example using Microchip Studio on Windows
5. disconnect the adapter and put the protection cap back on the 1.27 mm ISP connector
6. (re-)connect the [battery check PCB](/schemes/batteryCheck.png) to the vehicle (while it's powered off)

> [!NOTE]
> Our original intention was to use the ATtiny841 to automatically power off the entire vehicle when the battery voltage is critically low. This functionality is implemented in the code but for it to work, the shutdown pin on the [battery check PCB](/schemes/batteryCheck.png) has to be connected to the "EN 3V3" pin on the [main PCB](/schemes/main.png). We decided to not do that because we don't want to risk experiencing a shutdown during a race. That's why the ATtiny841 does currently not add any functionality and is therefore completely optional.