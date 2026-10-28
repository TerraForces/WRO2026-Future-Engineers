### Raspberry Pi 5 preparation
1. install Raspberry Pi OS (64-bit) on a microSD card and perform the following steps on the Raspberry Pi
2. update packages and firmware (internet connection required):
```
sudo apt update
sudo apt full-upgrade -y
sudo rpi-eeprom-update -a
sudo reboot
```
3. install required packages (internet connection required):
```
sudo apt install dkms -y
sudo apt install hailo-all -y
```
4. check camera functionality, e.g. using `rpicam-hello` (shows camera video stream for a short time)
5. check whether the AI accelerator is being detected using `hailortcli fw-control identify` (should print information about the connected accelerator when detected, otherwise does nothing)
6. copy the content of the [Inference](RPI5/Inference/) folder into ~/Documents
7. compile the code using
```
cd ~/Documents
g++ main.cpp -o main -Ofast
```
8. set the program as autostart by placing the [wro.desktop](RPI5/Inference/wro.desktop) file into the autostart directory
```
mkdir -p ~/.config/autostart
mv ~/Documents/wro.desktop ~/.config/autostart/wro.desktop
```
