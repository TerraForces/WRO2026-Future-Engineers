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

    // VL53L1X sensor positions
    enum SENSORS : uint8_t {
        FrontCenter,
        AngledLeft,
        FrontLeft,
        RearLeft,
        RearCenter,
        RearRight,
        FrontRight,
        AngledRight
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

    // NeoPixel LED color
    struct COLOR8 {
        uint8_t g, r, b;
    };

    // TCS3472 color values
    struct COLOR16 {
        uint16_t r, g, b, c;
    };

    // reads battery voltage
    // @returns battery voltage in V
    float getBatteryVoltage();

    // reads new color data if available, returns last value otherwise
    // @returns color measurement as brightness values behind red, green, blue and clear filters
    COLOR16 getColor();

    // reads current flowing into the voltage converters
    // @returns current in A
    float getConverterCurrent();

    // requests temperature next to the voltage converters
    // @returns voltage converter temperature in °C
    float getConverterTemp();

    // reads Teensy core temperature
    // @returns core temperature in °C
    float getCoreTemp();

    // reads new distance data if available, returns last value otherwise
    // @param sensor sensor which should be read
    // @returns distance in front of the specified sensor in mm
    uint16_t getDistance(SENSORS sensor);

    // reads current flowing through the MOSFET
    // @returns motor current in A
    float getDriveCurrent();

    // requests MOSFET temperature
    // @returns MOSFET temperature in °C
    float getMosfetTemp();

    // returns objects array (presorted by relevance)
    // @returns vector of last detected objects
    std::vector<OBJECT> getObjects();

    // reads data and calculates rotation if available, returns last value otherwise
    // @returns Z orientation in degrees
    float getRotation();

    // runs complete initialisation process - loads configuration on sensors and activates display elements
    // @param rpi5 whether to activate the object detection on the Raspberry Pi 5
    void init(bool rpi5 = false);

    // appends a string to the logfile on the inserted sd card
    // @param str string to append
    void logSD(String str);

    // runs interactive rotation and distance sensor calibration flow
    // @param samples number of distance measurements read for each tested viewport center
    // @param rotationTime time used to analyze noise in rotation measurements in ms
    // @param rotationRounds number of rounds the car should be moved for rotation factor calibration
    void runCalibration(const uint32_t samples = 50, uint32_t rotationTime = 10000, uint32_t rotationRounds = 8);

    // sets camera LED ring colors (min 300us delay between writes required)
    // @param color color to be displayed on all LEDs
    // @returns true when the color was applied or false when the minimum time between writes hasn't finished yet
    bool setLEDs(COLOR8 color);

    // sets camera LED ring colors (min 300us delay between writes required)
    // @param colors array of LED colors to be displayed
    // @returns true when the colors were applied or false when the minimum time between writes hasn't finished yet
    bool setLEDs(COLOR8 colors[12]);

    // changes motor speed and direction using automated speed control
    // @param velocity new velocity (between -1.0f and 1.0f)
    // @param brakes whether to short the motor to decelerate (works only with velocity = 0.0f)
    // @param acceleration acceleration in relative speed / s² (has to be >0.0f)
    // @param deceleration deceleration in relative speed / s² (has to be <0.0f)
    void setVelocity(float velocity, bool brakes = false, float acceleration = 0.25f, float deceleration = -0.5f);

    // sets target servo angle
    // @param angle target angle in degrees
    void setServo(float angle);

    // show finish screen with elapsed time and temperatures
    // @param userData additional data to be displayed below
    void showFinish(const char* userData = "");

    // displays detected object boxes according to camera view with additional data
    void showObjects();

    // displays latest measurements of distance, rotation and color
    // @param complete whether to draw elements like data labels (waits corresponding time if not)
    void showSensorData(bool complete = false);

    // starts drive and servo control - only run after start button has been pressed
    void start();

    // updates drive speed and distance and rotation data - has to be called freqently
    // @param showData show sensor data after update (needs additionally 20 ms)
    void update(bool showData = true);

    // waits until the start button is pressed (blocking) - activates LED while waiting
    void waitButtonPress();
}

#endif