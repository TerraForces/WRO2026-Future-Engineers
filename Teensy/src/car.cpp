/**
 * car.cpp - the functional core of our autonomously driving car
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// include header file
#include "car.hpp"

// include Arduino framework headers
#include <Arduino.h>
#include <core_pins.h>
#include <SD.h>

// include sensor and display libraries
#include "ds18b20.hpp"
#include "font5x7.hpp"
#include "mpu9250.hpp"
#include "neopixel.hpp"
#include "st7735.hpp"
#include "tcs3472.hpp"
#include "vl53l1x.hpp"

// pin configuration
constexpr uint8_t Pin_ADC_Battery       = 40; // in     - battery voltage over voltage divider
constexpr uint8_t Pin_ADC_Current_Conv  = 20; // in     - conveter current over ACS712-5 (0A = 2.5V, 5A = 0V)
constexpr uint8_t Pin_ADC_Current_Drive = 21; // in     - drive current over ACS712-5 (0A = 2.5V, 5A = 0V)
constexpr uint8_t Pin_Board_LED         = 13; //    out - LED on Teensy PCB
constexpr uint8_t Pin_Button_INT        = 14; // in     - start button interrupt
constexpr uint8_t Pin_Button_LED        = 41; //    out - start button LED
constexpr uint8_t Pin_Drive_Brake       = 30; //    out - shorts motor to stop faster
constexpr uint8_t Pin_Drive_PWM         = 22; //    out - motor PWM signal
constexpr uint8_t Pin_Drive_Rev         = 8;  //    out - enable to drive backwards (only change at standstill)
constexpr uint8_t Pin_DS18B20_OneWire   = 23; // in out - OneWire bus for communication with the DS18B20 temperature sensors
constexpr uint8_t Pin_IMU_INT           = 11; // in     - IMU data ready interrupt signal
constexpr uint8_t Pin_I2C_0_SDA		    = 18; // in out - i2c bus 0 data
constexpr uint8_t Pin_I2C_0_SCL		    = 19; // in out - i2c bus 0 clock
constexpr uint8_t Pin_I2C_1_SDA		    = 17; // in out - i2c bus 1 data
constexpr uint8_t Pin_I2C_1_SCL		    = 16; // in out - i2c bus 1 clock
constexpr uint8_t Pin_I2C_2_SDA		    = 25; // in out - i2c bus 2 data
constexpr uint8_t Pin_I2C_2_SCL		    = 24; // in out - i2c bus 2 clock
constexpr uint8_t Pin_LCD_CS		    = 0;  //    out - LCD SPI Chip Select
constexpr uint8_t Pin_LCD_LED		    = 12; //    out - LCD Backlight Control PWM
constexpr uint8_t Pin_LCD_MISO		    = 1;  // in     - LCD SPI Master In Slave Out (unused)
constexpr uint8_t Pin_LCD_MOSI		    = 26; //    out - LCD SPI Master Out Slave In
constexpr uint8_t Pin_LCD_SCK		    = 27; //    out - LCD SPI Clock
constexpr uint8_t Pin_LCD_RST		    = 32; //    out - 
constexpr uint8_t Pin_LCD_DC		    = 31; //    out - 
constexpr uint8_t Pin_NeoPixel          = 15; //    out - NeoPixel data signal
constexpr uint8_t Pin_RPI5_PWR          = 7;  //    out - enables RPi5 voltage converter
constexpr uint8_t Pin_RPI5_RX           = 28; // in     - RPi5 UART receive
constexpr uint8_t Pin_RPI5_TX           = 29; //    out - RPi5 UART transmit
constexpr uint8_t Pin_Servo_PWM         = 37; //    out - servo PWM signal
constexpr uint8_t Pin_TCS3472_INT       = 10; // in     - TCS3472 data ready interrupt signal
constexpr uint8_t Pin_VL53L1X_INT_RR    = 34; // in     - rear right VL53L1X data ready interrupt pin
constexpr uint8_t Pin_VL53L1X_SHT_AL_RL = 5;  //    out - angled left and rear left VL53L1X combined shutdown pin
constexpr uint8_t Pin_VL53L1X_SHT_AR_FR = 38; //    out - angled right and front right VL53L1X combined shutdown pin
constexpr uint8_t Pin_VL53L1X_SHT_FC_RR = 36; //    out - front center and rear right VL53L1X combined shutdown pin
constexpr uint8_t Pin_VL53L1X_SHT_FL_RC = 2;  //    out - front left and rear center VL53L1X combined shutdown pin
constexpr uint8_t Pins_VL53L1X_INT[8]   = { 35, 4, 3, 6, 9, 34, 33, 39 }; // in     - VL53L1X data ready interrupt pins

// lcd color configuration
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3))
constexpr uint16_t Color_Background = RGB565(0x00, 0x00, 0x00);
constexpr uint16_t Color_Best_Bars  = RGB565(0x00, 0xFF, 0x00);
constexpr uint16_t Color_Diff_Neg   = RGB565(0x90, 0x00, 0x00);
constexpr uint16_t Color_Diff_Pos   = RGB565(0xFF, 0x00, 0x00);
constexpr uint16_t Color_Error      = RGB565(0xFF, 0x00, 0x00);
constexpr uint16_t Colors_Objects[] = { RGB565(0xFF, 0x00, 0x00), RGB565(0x00, 0xFF, 0x00), RGB565(0xFF, 0x00, 0xFF) };
constexpr uint16_t Color_Outlines   = RGB565(0x50, 0x50, 0x50);
constexpr uint16_t Color_Running    = RGB565(0x00, 0xFF, 0x00);
constexpr uint16_t Color_SD_Bar     = RGB565(0x00, 0x00, 0xFF);
constexpr uint16_t Color_Text       = RGB565(0xFF, 0xFF, 0xFF);
constexpr uint16_t Color_Warning    = RGB565(0xFF, 0xFF, 0x00);

// misc configuration
constexpr uint8_t  Config_DS18B20_Conv_Addr[8]   = { 0x28, 0x61, 0x64, 0x0a, 0x55, 0x27, 0xd1, 0x17 };
constexpr uint8_t  Config_DS18B20_MOSFET_Addr[8] = { 0x28, 0x61, 0x64, 0x0a, 0x4a, 0x1c, 0xf7, 0xf2 };
constexpr float    Config_Drive_PWM_Freq         = 2959.955f;
constexpr uint32_t Config_Drive_PWM_Update_Int   = 10000; // us
constexpr float    Config_Servo_Angle_To_Duty    = -220.0f;
constexpr uint16_t Config_Servo_Center_Duty      = 33350;
constexpr float    Config_Servo_Max_Angle        = 50.0f;
constexpr uint32_t Config_Temp_Update_Int        = 500000; // us
constexpr bool     Config_VL53L1X_LongMode[8]    = { 1, 1, 1, 1, 1, 1, 1, 1 };

// bus, display and sensor objects
OneWire oneWire(Pin_DS18B20_OneWire);
DS18B20 ds18b20(&oneWire);
MPU9250 imu(i2c2, 30.0f);
NeoPixel leds(Pin_NeoPixel);
ST7735 lcd(&SPI1, Pin_LCD_CS, Pin_LCD_DC, Pin_LCD_RST, true, true);
TCS3472 tcs3472(&i2c2);
VL53L1X vl53l1x[8] = { &i2c1, &i2c1, &i2c1, &i2c0, &i2c0, &i2c0, &i2c0, &i2c1 };

// global memory
int32_t currentVelocity = 0;
int32_t targetVelocity = 0;
int32_t acceleration = 1;
int32_t deceleration = -1;
uint32_t nextTempUpdateTime = 0;
uint32_t nextDriveUpdateTime = 0;
CAR::COLOR16 lastColor = {};
uint16_t lastDistance[8] = {};
std::vector<CAR::OBJECT> lastObjects, currentObjects;
float lastRotation = 0.0f;
float lastVoltage = 0.0f;
float rotationOffset = 0.0f, rotationTimeOffset = 0.0f, rotationFactor = 0.0f;
uint8_t vl53l1xViewportCenters[8] = { 199, 199, 199, 199, 199, 199, 199, 199 };
bool sensorActive[10] = {};
bool sensorWarning[10] = {};
bool rpiActive = false;
uint32_t startTime = 0;
uint8_t uartReadBuffer[50000] = {};

// reads battery voltage
// @returns battery voltage in V
float CAR::getBatteryVoltage() {
    return analogRead(Pin_ADC_Battery) * 0.002519843f;
}

// reads new color data if available, returns last value otherwise
// @returns color measurement as brightness values behind red, green, blue and clear filters
CAR::COLOR16 CAR::getColor() {
    if(sensorActive[8]) {
        if(!tcs3472.readColor(&lastColor)) sensorWarning[8] = true;
    }
    return lastColor;
}

// reads current flowing into the voltage converters
// @returns current in A
float CAR::getConverterCurrent() {
    return (3103.0303f - analogRead(Pin_ADC_Current_Conv)) * 0.002441406f;
}

// requests temperature next to the voltage converters
// @returns voltage converter temperature in °C
float CAR::getConverterTemp() {
    ds18b20.requestTemperaturesByAddress(Config_DS18B20_Conv_Addr);
    return ds18b20.getTempC(Config_DS18B20_Conv_Addr);
}

// reads Teensy core temperature
// @returns core temperature in °C
float CAR::getCoreTemp() {
    return tempmonGetTemp();
}

// reads new distance data if available, returns last value otherwise
// @param sensor sensor which should be read
// @returns distance in front of the specified sensor in mm
uint16_t CAR::getDistance(SENSORS sensor) {
    return lastDistance[sensor];
}

// reads current flowing through the MOSFET
// @returns motor current in A
float CAR::getDriveCurrent() {
    return (3103.0303f - analogRead(Pin_ADC_Current_Drive)) * 0.002441406f;
}

// requests MOSFET temperature
// @returns MOSFET temperature in °C
float CAR::getMosfetTemp() {
    ds18b20.requestTemperaturesByAddress(Config_DS18B20_MOSFET_Addr);
    return ds18b20.getTempC(Config_DS18B20_MOSFET_Addr);
}

// returns objects array (presorted by relevance)
// @returns vector of last detected objects
std::vector<CAR::OBJECT> CAR::getObjects() {
    return lastObjects;
}

// reads data and calculates rotation if available, returns last value otherwise
// @returns Z orientation in degrees
float CAR::getRotation() {
    return lastRotation;
}

// runs complete initialisation process - loads configuration on sensors and activates display elements
// @param rpi5 whether to activate the object detection on the Raspberry Pi 5
void CAR::init(bool rpi5) {

    // configure pin modes
    pinMode(0, OUTPUT);
    pinMode(1, INPUT);
    pinMode(2, OUTPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, OUTPUT);
    pinMode(6, INPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, INPUT);
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(14, INPUT_PULLUP);
    pinMode(15, OUTPUT);
    pinMode(20, INPUT_DISABLE);
    pinMode(21, INPUT_DISABLE);
    pinMode(22, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(28, INPUT);
    pinMode(29, OUTPUT);
    pinMode(30, OUTPUT);
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
    pinMode(36, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(39, INPUT);
    pinMode(40, INPUT_DISABLE);
    pinMode(41, OUTPUT);

    // configure output pin states
    digitalWriteFast(0, LOW);
    digitalWriteFast(2, LOW);
    digitalWriteFast(5, LOW);
    digitalWriteFast(7, rpi5);
    digitalWriteFast(8, LOW);
    digitalWriteFast(12, HIGH);
    digitalWriteFast(13, LOW);
    digitalWriteFast(15, LOW);
    digitalWriteFast(22, LOW);
    digitalWriteFast(26, LOW);
    digitalWriteFast(27, LOW);
    digitalWriteFast(29, LOW);
    digitalWriteFast(30, LOW);
    digitalWriteFast(31, LOW);
    digitalWriteFast(32, LOW);
    digitalWriteFast(36, LOW);
    digitalWriteFast(37, LOW);
    digitalWriteFast(38, LOW);
    digitalWriteFast(41, LOW);

    // configure ADC channels
    analogReadAveraging(16);
    analogReadRes(12);

    // start SD and read configuration data
    SD.begin(BUILTIN_SDCARD);
    File configFile = SD.open("/config.txt", FILE_READ);
    if(configFile) {
        for(uint8_t i = 0; i < 8; i++) vl53l1xViewportCenters[i] = configFile.readStringUntil('\n').toInt();
        rotationTimeOffset = configFile.readStringUntil('\n').toFloat();
        rotationFactor = configFile.readStringUntil('\n').toFloat();
        configFile.close();
    }

    // start display
    lcd.begin();
    lcd.fillScreen(0);

    // start I2C systems
    i2c0.begin(400000);
    i2c1.begin(400000);
    i2c2.begin(400000);

    // start IMU
    imu.begin();

    // start DS18B20 temperature sensors
    ds18b20.begin();
    ds18b20.setResolution(12);

    // start VL53L1X distance sensors
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_FL_RC, HIGH);
    delay(15);
    vl53l1x[FrontLeft].setAddress(0x2C);
    vl53l1x[RearCenter].setAddress(0x2C);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_FC_RR, HIGH);
    delay(15);
    vl53l1x[FrontCenter].setAddress(0x2B);
    vl53l1x[RearRight].setAddress(0x2B);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_AR_FR, HIGH);
    delay(15);
    vl53l1x[AngledRight].setAddress(0x2A);
    vl53l1x[FrontRight].setAddress(0x2A);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_AL_RL, HIGH);
    delay(15);
    for(uint8_t i = 0; i < 8; i++) sensorActive[i] = vl53l1x[i].begin(Config_VL53L1X_LongMode[i], vl53l1xViewportCenters[i]);
    delay(15);

    // start TCS3472 color sensor
    sensorActive[8] = tcs3472.begin();

    if(rpi5) {
        // wait for Raspberry Pi 5 to finish booting
        Serial7.addMemoryForRead(uartReadBuffer, sizeof(uartReadBuffer));
        Serial7.begin(500000, SERIAL_8N1);
        while(Serial7.available() < (int)sizeof(CAR::OBJECT)) yield();
        rpiActive = true;
    }
    else {
        // show battery voltage
        lastVoltage = getBatteryVoltage();
        setLEDs(CAR::COLOR8 { (uint8_t)(255 * (lastVoltage > 7.2f)), (uint8_t)(255 * (lastVoltage < 7.8f)), 0 });
    }
}

// appends a string to the logfile on the inserted sd card
// @param str string to append
void CAR::logSD(String str) {
    File logFile = SD.open("/log.txt", FILE_WRITE);
    logFile.println(str);
    logFile.close();
}

constexpr uint8_t testViewports[] = { 59, 60, 61, 62, 63, 199, 198, 197, 196, 195, 194 };
constexpr char directions[8][3] = { "FC", "AL", "FL", "RL", "RC", "RR", "FR", "AR" };

// runs interactive rotation and distance sensor calibration flow
// @param samples number of distance measurements read for each tested viewport center
// @param rotationTime time used to analyze noise in rotation measurements in ms
// @param rotationRounds number of rounds the car should be moved for rotation factor calibration
void CAR::runCalibration(const uint32_t samples, uint32_t rotationTime, uint32_t rotationRounds) {

    // tell user to not touch during first part of calibration
    lcd.fillScreen(Color_Background);
    lcd.drawText(2, 2, font5x7, "please don't touch", Color_Text, Color_Background);

    // delay measurement start to reduce vibrations caused by user
    delay(1000);

    // calculate rotation time offset
    float startRotation = 0.0f;
    imu.getRotation(&startRotation);
    uint32_t endMillis = millis() + rotationTime;
    while(millis() < endMillis) {
        if(digitalReadFast(Pin_IMU_INT)) imu.getRotation(&lastRotation);
    }
    rotationTimeOffset = (startRotation - lastRotation) / rotationTime;
    startRotation = lastRotation + (rotationTimeOffset * millis());

    // tell user to turn the vehicle for rotation factor calibration
    lcd.fillScreen(Color_Background);
    lcd.drawText(2, 2, font5x7, ("please turn " + String(rotationRounds) + " rounds\nand then press button").c_str(), Color_Text, Color_Background);
    while(digitalReadFast(Pin_Button_INT)) {
        if(digitalReadFast(Pin_IMU_INT)) imu.getRotation(&lastRotation);
    }
    lastRotation += rotationTimeOffset * millis();
    rotationFactor = (rotationRounds * 360.0f) / (lastRotation - startRotation);

    // run calibration flow for every ToF sensor
    if(samples > 0) {
        uint16_t data[samples] = {};
        for(uint8_t i = 0; i < 8; i++) {
            vl53l1x[i].stop();
            
            // show direction and distance to user
            uint32_t distance = Config_VL53L1X_LongMode[i] ? 2000 : 1000;
            lcd.fillScreen(Color_Background);
            lcd.drawChar(30, 2, font5x7, directions[i][0], Color_Text, Color_Background, 2);
            lcd.drawChar(42, 2, font5x7, directions[i][1], Color_Text, Color_Background, 2);
            lcd.drawChar(64, 2, font5x7, ':', Color_Text, Color_Background, 2);
            lcd.drawChar(88, 2, font5x7, Config_VL53L1X_LongMode[i] ? '2' : '1', Color_Text, Color_Background, 2);
            lcd.drawChar(100, 2, font5x7, 'm', Color_Text, Color_Background, 2);

            // wait for user input to start measurements
            delay(500);
            waitButtonPress();

            // draw bar chart area
            lcd.fillScreen(Color_Background);
            lcd.drawText(2, 2, font5x7, "59\n60\n61\n62\n63\n199\n198\n197\n196\n195\n194", Color_Text, Color_Background);
            lcd.drawChar(20, 92, font5x7, '1', Color_Text, Color_Background);
            lcd.drawChar(38, 92, font5x7, '2', Color_Text, Color_Background);
            lcd.drawChar(62, 92, font5x7, '5', Color_Text, Color_Background);
            lcd.drawText(80, 92, font5x7, "10", Color_Text, Color_Background);
            lcd.drawText(98, 92, font5x7, "20", Color_Text, Color_Background);
            lcd.drawText(122, 92, font5x7, "50", Color_Text, Color_Background);
            lcd.drawText(140, 92, font5x7, "100", Color_Text, Color_Background);
            lcd.drawRect(22, 1, 1, 89, Color_Text);
            lcd.drawRect(40, 1, 1, 89, Color_Text);
            lcd.drawRect(64, 1, 1, 89, Color_Text);
            lcd.drawRect(82, 1, 1, 89, Color_Text);
            lcd.drawRect(100, 1, 1, 89, Color_Text);
            lcd.drawRect(124, 1, 1, 89, Color_Text);
            lcd.drawRect(142, 1, 1, 89, Color_Text);

            // delay measurement start to reduce vibrations caused by user
            delay(1000);

            // record important metrics for every tested viewport center
            float minDiff = __FLT_MAX__;
            float minSD = __FLT_MAX__;
            uint8_t minDiffIdx = 0, minSDIdx = 0;
            float diffs[sizeof(testViewports)] = {};
            float sds[sizeof(testViewports)] = {};
            for(uint8_t j = 0; j < sizeof(testViewports); j++) {

                // set test viewport center
                vl53l1x[i].setROI(6, 6, testViewports[j]);

                // start measurements
                vl53l1x[i].start();
                float avg = 0;
                for(uint32_t k = 0; k < samples; k++) {
                    delay(1);
                    while(digitalReadFast(Pins_VL53L1X_INT[i]) == LOW) yield();
                    vl53l1x[i].getDistance(data + k);
                    avg += data[k];
                }
                vl53l1x[i].stop();

                // calculate average and difference to expected value
                avg /= samples;
                float diff = abs(avg - distance);
                diffs[j] = diff;
                if(diff < minDiff) {
                    minDiff = diff;
                    minDiffIdx = j;
                }

                // calculate standard deviation
                float sd = 0;
                for(uint32_t k = 0; k < samples; k++) sd += sq(data[k] - avg);
                sd = sqrt(sd / samples);
                sds[j] = sd;
                if(sd < minSD) {
                    minSD = sd;
                    minSDIdx = j;
                }

                // show bars
                if(diff > 1.0f) lcd.drawRect(22, 2 + (j * 8), (log10(diff) * 60.0f) + 0.5f, 3, avg > distance ? Color_Diff_Pos : Color_Diff_Neg);
                if(sd > 1.0f) lcd.drawRect(22, 6 + (j * 8), (log10(sd) * 60.0f) + 0.5f, 3, Color_SD_Bar);
            }
            
            // try to find best ROI center
            uint8_t bestIdx = minDiffIdx;
            if(diffs[minSDIdx] < (minDiff * 1.125f)) bestIdx = minSDIdx;

            // show best ROI center
            float bestDiff = diffs[bestIdx];
            float bestSD = sds[bestIdx];
            if(bestDiff > 1.0f) lcd.drawRect(22, 2 + (bestIdx * 8), (log10(bestDiff) * 60.0f) + 0.5f, 3, Color_Best_Bars);
            if(bestSD > 1.0f) lcd.drawRect(22, 6 + (bestIdx * 8), (log10(bestSD) * 60.0f) + 0.5f, 3, Color_Best_Bars);
            lcd.drawText(2, 109, font5x7, ("diff: " + String(bestDiff) + " mm\nsd: " + String(bestSD) + " mm").c_str(), Color_Best_Bars, Color_Background);

            // apply new viewport center
            vl53l1xViewportCenters[i] = testViewports[bestIdx];
            vl53l1x[i].setROI(6, 6, vl53l1xViewportCenters[i]);
            vl53l1x[i].start();

            // wait for user input to continue
            waitButtonPress();
        }
    }

    // write results to sd card
    File file = SD.open("/config.txt", FILE_WRITE_BEGIN);
    for(uint8_t i = 0; i < 8; i++) file.println(vl53l1xViewportCenters[i]);
    file.println(rotationTimeOffset, 7);
    file.println(rotationFactor, 7);
    file.close();
}

// sets camera LED ring colors (min 300us delay between writes required)
// @param color color to be displayed on all LEDs
bool CAR::setLEDs(COLOR8 color) {
    return leds.fill(*(NeoPixel::COLOR*)&color, 12);
}

// sets camera LED ring colors (min 300us delay between writes required)
// @param colors array of LED colors to be displayed
bool CAR::setLEDs(COLOR8 colors[12]) {
    return leds.show((NeoPixel::COLOR*)colors, 12);
}

// changes motor speed and direction using automated speed control
// @param velocity new velocity (between -1.0f and 1.0f)
// @param brakes whether to short the motor to decelerate (works only with velocity = 0.0f)
// @param acceleration acceleration in relative speed / s² (has to be >0.0f)
// @param deceleration deceleration in relative speed / s² (has to be <0.0f)
void CAR::setVelocity(float _velocity, bool _brakes, float _acceleration, float _deceleration) {

    // activate active brakes if asked for
    if((_velocity == 0.0f) && _brakes) {
        analogWrite(Pin_Drive_PWM, 0);
        digitalWriteFast(Pin_Drive_Brake, HIGH);
        targetVelocity = 0;
        currentVelocity = 0;
    }

    // calculate PWM duty and duty change values for update()
    else {
        targetVelocity = constrain(_velocity, -1.0f, 1.0f) * 0xFFFF;
        acceleration = _acceleration * Config_Drive_PWM_Update_Int * 0xFFFF / 1000000.0f;
        deceleration = _deceleration * Config_Drive_PWM_Update_Int * 0xFFFF / 1000000.0f;
        if(acceleration <= 0) acceleration = 1;
        if(deceleration >= 0) deceleration = -1;
    }
}

// sets target servo angle
// @param angle target angle in degrees
void CAR::setServo(float angle) {
    analogWrite(Pin_Servo_PWM, Config_Servo_Center_Duty + (int)(((angle < -Config_Servo_Max_Angle) ? -Config_Servo_Max_Angle : ((angle > Config_Servo_Max_Angle) ? Config_Servo_Max_Angle : angle)) * Config_Servo_Angle_To_Duty));
}

// show finish screen with elapsed time and temperatures
// @param userData additional data to be displayed below
void CAR::showFinish(const char* userData) {
    uint32_t time = millis() - startTime;
    lcd.fillScreen(Color_Background);
    lcd.drawText(2, 2, font5x7, ("time: " + String(time / 1000.0f) + " s").c_str(), Color_Text, Color_Background, 2);
    lcd.drawText(2, 20, font5x7, ("converter: " + String(getConverterTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(2, 28, font5x7, ("core: " + String(getCoreTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(2, 36, font5x7, ("MOSFET: " + String(getMosfetTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(2, 44, font5x7, userData, Color_Text, Color_Background);
}

// displays detected object boxes according to camera view with additional data
void CAR::showObjects() {
    lcd.fillScreen(Color_Background);
    lcd.drawRect(9, 9, 1, 110, Color_Text);
    lcd.drawRect(119, 9, 1, 110, Color_Text);
    lcd.drawRect(10, 9, 108, 1, Color_Text);
    lcd.drawRect(10, 119, 108, 1, Color_Text);
    uint8_t objectCounts[3] = {};
    for(int32_t i = lastObjects.size() - 1; i >= 0; i--) {
        CAR::OBJECT object = lastObjects[i];
        if(object.c) {
            object.c--;
            if(object.c <= 2) {
                uint16_t color = Colors_Objects[object.c];
                lcd.drawRect(10 + (object.x / 12), 10 + (object.y / 12), object.w / 12, object.h / 12, color);
                lcd.drawText(11 + (object.x / 12), 11 + (object.y / 12), font5x7, (String(object.p / 10.0f, 1) + "%").c_str(), Color_Text, color);
                objectCounts[object.c]++;

                // data of first detected object (temp)
                if(i == 0) lcd.drawText(2, 121, font5x7, ("x=" + String(object.x) + " y=" + String(object.y) + " w=" + String(object.w) + " h=" + String(object.h)).c_str(), Color_Text, Color_Background);
            }
        }
    }
    for(uint8_t i = 0; i < 3; i++) {
        lcd.drawRect(130, 9 + (i * 8), 7, 7, Colors_Objects[i]);
        lcd.drawText(138, 9 + (i * 8), font5x7, String(objectCounts[i]).c_str(), Color_Text, Color_Background);
    }
    lcd.drawText(130, 33, font5x7, (String(getBatteryVoltage()) + "V").c_str(), Color_Text, Color_Background);
}

// displays latest measurements of distance, rotation and color
// @param complete whether to draw elements like data labels (waits corresponding time if not)
void CAR::showSensorData(bool complete) {
    if(complete) {
        lcd.fillScreen(Color_Background);
        for(uint8_t i = 0; i < 8; i++) lcd.drawText(2, 2 + (i * 8), font5x7, (directions[i] + String(": ")).c_str(), Color_Text, Color_Background);
        lcd.drawText(2, 66, font5x7, "rotation: ", Color_Text, Color_Background);
        lcd.drawText(2, 74, font5x7, "battery:", Color_Text, Color_Background);
        lcd.drawText(2, 82, font5x7, "color: r:", Color_Text, Color_Background);
        lcd.drawText(44, 90, font5x7, "g:", Color_Text, Color_Background);
        lcd.drawText(44, 98, font5x7, "b:", Color_Text, Color_Background);
        lcd.drawText(44, 106, font5x7, "c:", Color_Text, Color_Background);
    }
    for(uint8_t i = 0; i < 8; i++) lcd.drawText(26, 2 + (i * 8), font5x7, (String(lastDistance[i]) + " mm  ").c_str(), Color_Text, Color_Background);
    lcd.drawText(62, 66, font5x7, (String(lastRotation) + "  ").c_str(), Color_Text, Color_Background);
    lcd.drawText(56, 74, font5x7, (String(getBatteryVoltage()) + " V ").c_str(), Color_Text, Color_Background);
    tcs3472.readColor(&lastColor);
    lcd.drawText(62, 82, font5x7, (String(lastColor.r) + "  ").c_str(), Color_Text, Color_Background);
    lcd.drawText(62, 90, font5x7, (String(lastColor.g) + "  ").c_str(), Color_Text, Color_Background);
    lcd.drawText(62, 98, font5x7, (String(lastColor.b) + "  ").c_str(), Color_Text, Color_Background);
    lcd.drawText(62, 106, font5x7, (String(lastColor.c) + "  ").c_str(), Color_Text, Color_Background);
    lcd.drawRect(2, 90, 41, 23, RGB565(lastColor.r >> 2, lastColor.g >> 2, lastColor.b >> 2));
}

// starts drive and servo control - only run after start button has been pressed
void CAR::start() {

    // set rotation offset
    while(!imu.getRotation(&lastRotation)) delay(1);
    lastRotation += millis() * rotationTimeOffset;
    lastRotation *= rotationFactor;
    rotationOffset = lastRotation;

    // configure drive pins and PWM
    analogWriteRes(16);
    analogWriteFrequency(Pin_Drive_PWM, Config_Drive_PWM_Freq);
    analogWrite(Pin_Drive_PWM, 0);

    // configure servo pin and PWM
    analogWriteFrequency(Pin_Servo_PWM, 333.0f);
    analogWrite(Pin_Servo_PWM, Config_Servo_Center_Duty);

    // update sensors multiple times
    uint32_t endMillis = millis() + 200;
    while(millis() < endMillis) {
        for(uint8_t i = 0; i < 8; i++) {
            if(digitalRead(Pins_VL53L1X_INT[i])) vl53l1x[i].getDistance(lastDistance + i);
        }
        if(digitalReadFast(Pin_IMU_INT)) {
            if(imu.getRotation(&lastRotation)) {
                lastRotation += millis() * rotationTimeOffset;
                lastRotation *= rotationFactor;
                lastRotation -= rotationOffset;
            }
        }
    }
    startTime = millis();
}

// updates drive speed and distance and rotation data - has to be called freqently
// @param showData show sensor data after update (needs additionally 20 ms)
void CAR::update() {

    // update drive PWM duty
    uint32_t time = micros();
    if(time > nextDriveUpdateTime) {
        nextDriveUpdateTime = time + Config_Drive_PWM_Update_Int;
        if(currentVelocity != targetVelocity) {
            digitalWriteFast(Pin_Drive_Brake, LOW);
            if(currentVelocity < 0) {
                if(targetVelocity < currentVelocity) currentVelocity = max(currentVelocity - acceleration, targetVelocity);
                else currentVelocity = min(currentVelocity - deceleration, 0);
            }
            else if(currentVelocity > 0) {
                if(targetVelocity < currentVelocity) currentVelocity = max(currentVelocity + deceleration, 0);
                else currentVelocity = min(currentVelocity + acceleration, targetVelocity);
            }
            else {
                if(targetVelocity > 0) {
                    digitalWriteFast(Pin_Drive_Rev, LOW);
                    currentVelocity = min(acceleration, targetVelocity);
                }
                else {
                    digitalWriteFast(Pin_Drive_Rev, HIGH);
                    currentVelocity = max(-acceleration, targetVelocity);
                }
            }
            analogWrite(Pin_Drive_PWM, abs(currentVelocity));
        }
    }

    // update battery voltage display
    if(time > nextTempUpdateTime) {
        nextTempUpdateTime = time + Config_Temp_Update_Int;
        float voltage = getBatteryVoltage();
        if(voltage < 7.2f) {
            if(lastVoltage > 7.2f) {
                setLEDs({ 0, 255, 0 });
                lastVoltage = voltage;
            }
        }
        if(voltage < 7.8f) {
            if(lastVoltage > 7.8f) {
                setLEDs({ 255, 255, 0 });
                lastVoltage = voltage;
            }
        }
    }

    // update event handlers
    yield();

    // update UART to RPI5
    if(rpiActive) {
        while(Serial7.available() >= (int)sizeof(CAR::OBJECT)) {
            CAR::OBJECT object = {};
            Serial7.readBytes((uint8_t*)&object, sizeof(CAR::OBJECT));
            if(object.c) currentObjects.push_back(object);
            else {
                lastObjects = currentObjects;
                currentObjects.clear();
            }
        }
    }

    // update distances
    for(uint8_t i = 0; i < 8; i++) {
        if(digitalRead(Pins_VL53L1X_INT[i])) vl53l1x[i].getDistance(lastDistance + i);
    }

    // update rotation
    if(digitalReadFast(Pin_IMU_INT)) {
        if(imu.getRotation(&lastRotation)) {
            lastRotation += millis() * rotationTimeOffset;
            lastRotation *= rotationFactor;
            lastRotation -= rotationOffset;
        }
    }
}

// waits until the start button is pressed (blocking) - activates LED while waiting
void CAR::waitButtonPress() {
    digitalWriteFast(Pin_Button_LED, HIGH);
    while(digitalReadFast(Pin_Button_INT)) yield();
    digitalWriteFast(Pin_Button_LED, LOW);
}