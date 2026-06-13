/**
 * car.hpp - interface containing many useful functions for controlling our autonomously driving car
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// prevent including multiple times
#ifndef CAR_HPP
#define CAR_HPP

// include integer type names and basic framework functions
#include <Arduino.h>
#include <stdint.h>

namespace CAR {

    // possible object colors
    enum COLORS : uint16_t {
        EMPTY, // no object detected
        RED,   // red traffic sign
        GREEN, // green traffic sign
        PINK   // pink parking space borders
    };

    // VL53L1X sensor positions (ordered counterclockwise) and other sensors
    enum SENSORS : uint8_t {
        FrontCenter,
        AngledLeft,
        FrontLeft,
        RearLeft,
        RearCenter,
        RearRight,
        FrontRight,
        AngledRight,
        TCS3472,
        MPU9250,
        DS18B20Conv,
        DS18B20MOSFET
    };

    // description of detected objects
    struct OBJECT {
        uint16_t x; // left x position of the bounding rect in image pixels (0 - 1295)
        uint16_t y; // top y position of the bounding rect in image pixels (0 - 1295)
        uint16_t w; // bounding rect width in image pixels (0 - 1295)
        uint16_t h; // bounding rect height in image pixels (0 - 1295)
        uint16_t c; // object color (see possible object colors above)
        uint16_t p; // detection probability in 0.1% (0 - 1000)
    };

    // NeoPixel LED color (warning: order is GRB)
    struct COLOR8 {
        uint8_t g, r, b;
    };

    // TCS3472 color values
    struct COLOR16 {
        uint16_t c, r, g, b; // color channel values ranging from 0 to 1023
    };

    /// @brief reads battery voltage
    /// @returns battery voltage in V
    float getBatteryVoltage();

    /// @brief reads color data from sensor
    /// @returns color measurement as brightness values behind colored filters
    COLOR16 getColor();

    /// @brief reads current flowing into the voltage converters
    /// @returns current in A
    float getConverterCurrent();

    /// @brief requests temperature next to the voltage converters
    /// @returns voltage converter temperature in °C
    float getConverterTemp();

    /// @brief reads Teensy core temperature
    /// @returns core temperature in °C
    float getCoreTemp();

    /// @brief returns last distance measurement
    /// @param sensor sensor which should be read
    /// @returns distance in front of the specified sensor in mm
    uint16_t getDistance(SENSORS sensor);

    /// @brief reads current flowing through the MOSFET
    /// @returns current in A
    float getDriveCurrent();

    /// @brief requests MOSFET temperature
    /// @returns MOSFET temperature in °C
    float getMosfetTemp();

    /// @brief returns objects array (presorted by relevance - closest non-pink objects)
    /// @returns vector of last detected objects
    std::vector<OBJECT> getObjects();

    /// @brief returns last rotation measurement
    /// @returns z axis orientation in degrees (positive values = left)
    float getRotation();

    /// @brief runs complete initialisation process - loads configuration on sensors and activates display elements
    /// @param rpi5 whether to activate the object detection on the Raspberry Pi 5
    void init(bool rpi5 = false);

    /// @brief appends a string to the logfile on the inserted sd card
    /// @param str string to append
    void logSD(String str);

    /// @brief changes the rotation offset by the specified amount
    /// @param change change of the rotation offset in degrees (positive values result in higher calculated rotation values)
    void modifyRotationOffset(float change);

    /// @brief runs interactive rotation and distance sensor calibration flow
    /// @param preSamples number of distance measurements read for each sensor in the first test round (or 0 to skip)
    /// @param mainSamples number of distance measurements read for each sensor in the main test (or 0 to skip)
    /// @param distance expected distance in front of the sensor while calibration
    /// @param rotationDuration duration used to analyze noise in rotation measurements in ms (or 0 to skip)
    /// @param rotationRounds number of revolutions the car should be turned for rotation factor calibration (or 0 to skip)
    /// @param servoCalibrationDuration duration of accelerating/driving for each step in ms (or 0 to skip)
    void runCalibration(const uint32_t preSamples = 4, const uint32_t mainSamples = 40, uint32_t distance = 2000, uint32_t rotationDuration = 30000, uint32_t rotationRounds = 16, uint32_t servoCalibrationDuration = 3000);

    /// @brief runs a complete I2C scan on all bus systems and shows the results on the display
    void runI2cScan();

    /// @brief runs complete VL53L1X sensor test and records data as .obj files (import in Blender)
    void runTest();

    /// @brief sets camera LED ring colors (min 300us delay between writes required)
    /// @param color color to be displayed on all LEDs
    /// @returns true when the color was applied or false when the minimum time between writes hasn't finished yet
    bool setLEDs(COLOR8 color);

    /// @brief sets camera LED ring colors (min 300us delay between writes required)
    /// @param colors array of LED colors to be displayed
    /// @returns true when the colors were applied or false when the minimum time between writes hasn't finished yet
    bool setLEDs(COLOR8 colors[12]);

    /// @brief changes motor speed and direction using automated speed control
    /// @param velocity new velocity (between -1.0f and 1.0f)
    /// @param brakes whether to short the motor to decelerate (works only with velocity = 0.0f)
    /// @param acceleration acceleration in relative speed / s²
    /// @param deceleration deceleration in relative speed / s²
    void setVelocity(float velocity, bool brakes = false, float acceleration = 0.25f, float deceleration = -0.5f);

    /// @brief sets target servo angle
    /// @param angle target angle in degrees (positive = car will turn left)
    void setServo(float angle);

    /// @brief show finish screen with elapsed time and current temperature measurements
    /// @param userData additional data to be displayed below the other values
    void showFinish(const char* userData = "");

    /// @brief displays detected object boxes according to camera view with detection probablility
    void showObjects();

    /// @brief displays latest measurements of distance, rotation, battery voltage and color
    /// @param complete whether to draw data labels (not required to be refreshed)
    void showSensorData(bool complete = false);

    /// @brief starts drive and servo control and prepares sensors - only run after start button has been pressed
    void start();

    /// @brief updates drive speed, distance, objects and rotation data - has to be called as freqently as possible
    void update();

    /// @brief waits until the start button is pressed (blocking) - activates button LED while waiting
    /// @param duration if not 0 this function blocks until the button is released or duration has passed (min 40 ms to debounce)
    /// @returns whether the button has been pressed for duration or longer
    bool waitButtonPress(uint32_t duration = 0);
}

#endif