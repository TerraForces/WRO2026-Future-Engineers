# vehicle preparation
## software preparation
### required external parts
- laptop or desktop computer with Windows, macOS or a Linux distro of choice (macOS has not been tested but should be supported) and an internet connection
- cable with a USB 2 micro-B plug to connect to the Teensy 4.1
- USB keyboard and mouse (a USB hub with a slim connector is recommended for connecting to the Raspberry Pi 5 while it is installed in the vehicle)
- external monitor and a video cable with a micro-HDMI plug for the Raspberry Pi 5
- a WiFi network connected to the internet (or an Ethernet cable with the shielding of one plug removed, so it can't hit the fuse mount)

for the ATtiny841 (optional):
- AVR programmer
- adapter board / cable

### Teensy 4.1
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

### Raspberry Pi 5
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

### ATtiny841 (optional)
