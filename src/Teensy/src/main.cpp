/**
 * main.cpp - driving strategy and logic
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// select program
//#define TEST
//#define OBJECTS

#include "car.hpp"

#ifdef TEST
#ifndef OBJECTS

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::start();
    CAR::runCalibration();
    CAR::showSensorData(true);
}

void loop() {
    CAR::update();
    CAR::showSensorData();
}

#else

void setup() {
    CAR::init(true);
    CAR::waitButtonPress();
    CAR::start();
}

void loop() {
    CAR::update();
    CAR::showObjects();
}

#endif
#else

using namespace CAR;

#ifndef OBJECTS

// angle configuration
constexpr float curveEnd          =  38.0f;
constexpr float curveServoAngle   =  35.0f;
constexpr float reactionFactor    = -0.23f;
constexpr float wallReactionAngle =  8.0f;

// speed configuration
constexpr float boostSpeed  = 1.0f;
constexpr float normalSpeed = 1.0f;

// global memory
bool boost = false; // whether the speed boost mode is active
bool direction; // 0 = counterclockwise, 1 = clockwise
uint8_t curveCount = 0; // number of entered curve sections
uint8_t sectionWidth[4] = {}; // 0 = unknown, 1 = 600 mm, 2 = 1000 mm
uint32_t timestamp = 0; // timestamp of last curve start (in ms since program start)

// function to drive a curve
void driveCurve() {

    // slow down at the start of the curve
    setVelocity(normalSpeed, false, 0.2f, -0.4f);
    setServo(direction ? -curveServoAngle : curveServoAngle);

    // drive until reaching end angle
    if(direction == 0) while(getRotation() < (( 90.0f * curveCount) + curveEnd)) update();
    else               while(getRotation() > ((-90.0f * curveCount) - curveEnd)) update();

    // reached end of curve - activate speed boost
    setServo(0.0f);
    timestamp = millis();
    setVelocity(boostSpeed, false, 1.0f, -0.3f);
    boost = true;
    curveCount++;
}

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::start();

    // start driving
    setVelocity(normalSpeed, false, 1.0f, -0.4f);

    // find starting section width
    sectionWidth[0] = ((getDistance(FrontLeft) + getDistance(FrontRight)) > 750) + 1;

    // drive until first curve has been detected
    while(true) {
        update();

        // try to detect driving direction only when the vehicle is parallel to the walls
        if((abs(getRotation()) < 5.0f) || (getDistance(FrontCenter) < 400)) {

            // detect open space on the left
            if(getDistance(FrontLeft) > 800) {
                direction = 0;
                break;
            }

            // detect open space on the right
            else if(getDistance(FrontRight) > 800) {
                direction = 1;
                break;
            }
        }

        // drive away from the left wall when getting too close (< 15 cm)
        else if(getDistance(FrontLeft) < 150) {
            setServo(-wallReactionAngle);
            while(getRotation() > -7.0f) update();
        }

        // drive away from the right wall when getting too close (< 15 cm)
        else if(getDistance(FrontRight) < 150) {
            setServo(wallReactionAngle);
            while(getRotation() < 7.0f) update();
        }

        // correct vehicle rotation (using rotation feedback)
        setServo(getRotation() * reactionFactor);
    }
    driveCurve();
}

void loop() {
    update();

    // start curve when detected (cooldown of 1.3 s between curves to prevent driving circles)
    if((timestamp + 1000) < millis()) {
        if(((direction ? (getDistance(FrontRight) > 800) : (getDistance(FrontLeft) > 800)) || (getDistance(FrontCenter) < ((sectionWidth[curveCount & 3] == 2) ? 900 : 400))) 
            && (getDistance(RearCenter) > 1700)
            && (getRotation() + (direction ? (curveCount * 90.0f) : (curveCount * -90.0f)) < 7.0f))
            driveCurve();
    }

    // drive away from the left wall when getting too close (< 15 cm) and next curve is far enough away (> 80 cm)
    else if((getDistance(FrontLeft) < 120) && (getDistance(FrontCenter) > 950)) {
        setServo(-wallReactionAngle);
        while(getRotation() > (((direction == 0) ? 90.0f : -90.0f) * curveCount) - 4.0f) update();
        if((timestamp + 800) < millis()) timestamp = millis() - 800;
    }

    // drive away from the right wall when getting too close (< 15 cm) and next curve is far enough away (> 80 cm)
    else if((getDistance(FrontRight) < 120) && (getDistance(FrontCenter) > 950)) {
        setServo(wallReactionAngle);
        while(getRotation() < (((direction == 0) ? 90.0f : -90.0f) * curveCount) + 4.0f) update();
        if((timestamp + 800) < millis()) timestamp = millis() - 800;
    }

    // correct vehicle rotation (using rotation feedback)
    setServo((getRotation() - (direction ? (curveCount * -90.0f) : (curveCount * 90.0f))) * reactionFactor);

    // measure width of current section
    if((sectionWidth[curveCount & 3] == 0) && (getDistance(RearCenter) > 1200) && (abs(abs(getRotation()) - (curveCount * 90.0f)) < 5.0f)) {
        sectionWidth[curveCount & 3] = ((getDistance(FrontLeft) + getDistance(FrontRight)) > 750) + 1;
    }

    // deactivate boost when approaching next curve
    if(boost && ((getDistance(RearCenter) > (((sectionWidth[(curveCount + 1) & 3] == 1)) ? 1900 : 2050))) && ((timestamp + 500) < millis())) {
        setVelocity(normalSpeed, false, 0.2f, -0.3f);
        boost = false;
    }

    // stop after 12 curves inside of the start/end section
    if((curveCount >= 12) && (getDistance(RearCenter) > 1150) && ((timestamp + 800) < millis())) {
        setServo(0.0f);

        // use active braking to stop faster
        setVelocity(0.0f, true);

        // show finish screen and detected section widths below for debug
        char output[42];
        sprintf(output, "%hhu %hhu %hhu %hhu", sectionWidth[0], sectionWidth[1], sectionWidth[2], sectionWidth[3]);
        showFinish(output);

        // wait until turned off
        while(true) update();
    }
}

#else

// functions to detect whether a line of this color is visible below the color sensor
bool onOrangeLine() {
    CAR::COLOR16 color = CAR::getColor();
    return color.r > (color.b * 2.625f);
}
bool onBlueLine() {
    CAR::COLOR16 color = CAR::getColor();
    return color.b > (color.r * 1.1f);
}

// angle configuration
constexpr float curveServoAngle         = 37.5f;
constexpr float reactionFactor          = 0.3f;
constexpr float wallReactionAngle       = 8.0f;
constexpr float wallReactionTargetAngle = 4.0f;

// distance configuration
constexpr uint16_t parkingSpaceDetectionThreshold   = 60;
constexpr uint16_t wallReactionDistance             = 100;
constexpr uint16_t wallReactionDistance2            = 170;
constexpr uint16_t wallReactionDistanceStartSector  = 270;
constexpr uint16_t wallReactionDistance2StartSector = 470;
constexpr uint16_t wallReactionDistanceThreshold    = 650;

// speed configuration
constexpr float normalSpeed = 0.4f;
constexpr float fastSpeed   = 0.82f;
constexpr float slowSpeed   = 0.275f;

bool direction = 0;             // 0 = counterclockwise, 1 = clockwise
bool parkingSpace = 0;          // 0 = parking space at the end of the section, 1 = parking space at the beginning of the section (in driving direction)
uint8_t knownObstacles[4] = {}; // 0 = unknown, 1 = drive next to the inner border, 2 = drive next to the outer border
bool nearInnerBorder = false;   // indicates where the vehicle is currently driving
uint8_t curveCount = 0;
uint32_t timestamp = 0;

void wallCorrection() {
    update();
    if(((curveCount & 3) == 0) && (knownObstacles[curveCount & 3] != 1)) {
        if(direction) {
            if(getDistance(FrontLeft) < wallReactionDistanceStartSector) {
                setServo(-wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > -wallReactionTargetAngle) update();
            }
            else if(getDistance(FrontLeft) > wallReactionDistance2StartSector) {
                setServo(wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionTargetAngle) update();
            }
        }
        else {
            if(getDistance(FrontRight) < wallReactionDistanceStartSector) {
                setServo(wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionTargetAngle) update();
            }
            else if(getDistance(FrontRight) > wallReactionDistance2StartSector) {
                setServo(-wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > -wallReactionTargetAngle) update();
            }
        }
    }
    else if((knownObstacles[curveCount & 3] == 1) == direction) {
        if(getDistance(FrontRight) < wallReactionDistance) {
            setServo(wallReactionAngle);
            while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionTargetAngle) update();
        }
        else if(knownObstacles[curveCount & 3] != 1) {
            if(getDistance(FrontRight) > wallReactionDistance2) {
                setServo(-wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > -wallReactionTargetAngle) update();
            }
        }
        else {
            if((getDistance(FrontRight) > wallReactionDistance2) && (getDistance(FrontRight) < wallReactionDistanceThreshold) && (getDistance(RearCenter) > 1250)) {
                setServo(-wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > -wallReactionTargetAngle) update();
            }
        }

    }
    else {
        if(getDistance(FrontLeft) < wallReactionDistance) {
            setServo(-wallReactionAngle);
            while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > -wallReactionTargetAngle) update();
        }
        else if(knownObstacles[curveCount & 3] != 1) {
            if(getDistance(FrontRight) > wallReactionDistance2) {
                setServo(wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionTargetAngle) update();
            }
        }
        else {
            if((getDistance(FrontRight) > wallReactionDistance2) && (getDistance(FrontRight) < wallReactionDistanceThreshold) && (getDistance(RearCenter) > 1250)) {
                setServo(wallReactionAngle);
                while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionTargetAngle) update();
            }
        }
    }
    setServo(0.0f);
}

void readObject(uint32_t minHeight = 0, uint8_t imageSide = 0) { // 1 = right half; 2 = left half
    if(knownObstacles[curveCount & 3] == 0) {
        if(getObjects().size() > 0) {
            for(uint8_t i = 0; (knownObstacles[curveCount & 3] == 0) && (i < getObjects().size()); i++) {
                OBJECT object = getObjects()[i];
                if(object.h > minHeight) {
                    if (((((object.w / 2) + object.x) < 647) && (imageSide == 2)) || (imageSide == 0) || ((((object.w / 2) + object.x) > 647) && (imageSide == 1))) {
                        if(object.c == (direction ? RED : GREEN)) knownObstacles[curveCount & 3] = 1;
                        else if(object.c == (direction ? GREEN : RED)) knownObstacles[curveCount & 3] = 2;
                    }
                }
            }
        }
    }
}

// update sensors and drive while waiting
void wait(uint32_t ms) {
    timestamp = millis() + ms;
    while(millis() < timestamp) update();
}

// update sensors, drive and servo while waiting
void waitReaction(uint32_t ms) {
    timestamp = millis() + ms;
    while(millis() < timestamp) {
        update();
        setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor);
    }
}

void setup() {
    CAR::init(true);
    CAR::waitButtonPress();
    CAR::start();

    // adjust park speed accoding to battery voltage
    float parkSpeed = 0.18f + ((8.4f - getBatteryVoltage()) * 0.067f);

    // detect driving direction
    direction = getDistance(FrontRight) > 200;

    // leave parking space - first move
    setServo(direction ? -50.0f : 50.0f);
    setVelocity(parkSpeed, false, 0.5f);
    while(direction ? (getRotation() > -20.0f) : (getRotation() < 20.0f)) update();
    setVelocity(0.0f, true);
    wait(250);

    // leave parking space - second move
    setServo(direction ? 50.0f : -50.0f);
    setVelocity(-parkSpeed, false, 0.5f);
    while((direction ? (getRotation() > -40.0f) : (getRotation() < 40.0f))) update();
    setVelocity(0.0f, true);
    setServo(0.0f);
    wait(250);

    // leave parking space - third move
    setServo(direction ? -50.0f : 50.0f);
    setVelocity(parkSpeed, false, 0.5f);
    while(direction ? (getRotation() > -65.0f) : (getRotation() < 65.0f)) update();
    setVelocity(0.0f, true);
    wait(250);

    // leave parking space - fourth move
    setServo(direction ? 50.0f : -50.0f);
    setVelocity(-parkSpeed, false, 0.5f);
    while((direction ? (getRotation() > -85.0f) : (getRotation() < 85.0f))) update();
    setVelocity(0.0f, true);
    wait(250);
    setVelocity(parkSpeed, false, 0.5f);

    // drive until the sensors on the front side have full left the parking space
    while((getDistance(FrontLeft) < 200) || (getDistance(FrontRight) < 200) || (getDistance(RearCenter) < 115)) {
        setServo((getRotation() - (direction ? -90.0f : 90.0f)) * (-reactionFactor));
        update();
    }

    // detect parking space position
    parkingSpace = getDistance(direction ? FrontLeft : FrontRight) > getDistance(direction ? FrontRight : FrontLeft);

    if(parkingSpace) {
        setServo(direction ? 45.0f : -45.0f);
        setVelocity(parkSpeed);
        while(direction ? (getRotation() < -32.0f) : (getRotation() > 32.0f)) update();
        setVelocity(0.0f, true);
        wait(300);
        readObject(120);
        if(knownObstacles[0] == 1) {
            setServo(direction ? -45.0f : 45.0f);
            setVelocity(slowSpeed);
            while(direction ? (getRotation() > -80.0f) : (getRotation() < 80.0f)) update();
            timestamp = millis();
            while((getDistance(FrontCenter) > 300) || (millis() < (timestamp + 500))) {
                setServo(((getRotation() - (direction ? -90.0f : 90.0f))) * (-reactionFactor));
                update();
            }
            setServo(direction ? 45.0f : -45.0f);
            while(direction ? (getRotation() < -10.0f) : (getRotation() > 10.0f)) update();
            setVelocity(normalSpeed);
            timestamp = millis();
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 1150)) || (millis() < (timestamp + 1000))) {
                setServo(getRotation() * (-reactionFactor));
                update();
            }
            nearInnerBorder = true;
        }
        else {
            setServo(direction ? 45.0f : -45.0f);
            setVelocity(parkSpeed);
            while(direction ? (getRotation() < 15.0f) : (getRotation() > -15.0f)) update();
            setVelocity(normalSpeed);
            timestamp = millis();
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 1150)) || (millis() < (timestamp + 1000))) {
                setServo(getRotation() * (-reactionFactor));
                update();
            }
            nearInnerBorder = false;
        }
    }
    else {
        setVelocity(0.0f, true);
        wait(300);
        readObject(200);
        if(knownObstacles[0] == 1) {
            setVelocity(parkSpeed);
            timestamp = millis();
            while((getDistance(RearCenter) < 250) || (millis() < (timestamp + 500))) {
                setServo((getRotation() - (direction ? -90.0f : 90.0f)) * (-reactionFactor));
                update();
            }
            setServo(direction ? -40.0f : 40.0f);
            while(direction ? (getRotation() > -135.0f) : (getRotation() < 135.0f)) update();
            setServo(direction ? 40.0f : -40.0f);
            while(direction ? (getRotation() < -90.0f) : (getRotation() > 90.0f)) update();
            timestamp = millis();
            while((getDistance(FrontCenter) > 300) || (millis() < (timestamp + 500))) {
                setServo((getRotation() - (direction ? -90.0f : 90.0f)) * (-reactionFactor));
                update();
            }
            setServo(direction ? 45.0f : -45.0f);
            while(direction ? (getRotation() < -10.0f) : (getRotation() > 10.0f)) update();
            setVelocity(slowSpeed);
            timestamp = millis();
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < (timestamp + 1000))) {
                setServo(getRotation() * (-reactionFactor));
                update();
            }
            nearInnerBorder = true;
        }
        else {
            setVelocity(parkSpeed);
            setServo(direction ? 45.0f : -45.0f);
            while(direction ? (getRotation() < -2.0f) : (getRotation() > 2.0f)) update();
            setServo(0.0f);
            nearInnerBorder = false;
        }
    }
    setVelocity(0.0f, true);

    while(curveCount < 11) {

        // first round
        if(knownObstacles[(curveCount + 1) & 3] == 0) {
            if(nearInnerBorder) {
                curveCount++;
                readObject();
                if(knownObstacles[curveCount & 3] == 0) {
                    wait(50);
                    readObject();
                }
                curveCount--;
                if(knownObstacles[(curveCount + 1) & 3] == 1) {
                    nearInnerBorder = true;
                    curveCount++;
                    setVelocity(normalSpeed);
                    setServo(direction ? -45.0f : 45.0f);
                    while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                }
                else if(knownObstacles[(curveCount + 1) & 3] == 2) {
                    nearInnerBorder = false;
                    setVelocity(normalSpeed);
                    timestamp = millis();
                    while((getDistance(FrontCenter) > ((((curveCount + 1) & 3) == 0) ? 550 : 360)) || (millis() < (timestamp + 200))) {
                        setServo((getRotation() - ((curveCount) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                        update();
                    }

                    curveCount++;
                    setServo(direction ? -45.0f : 45.0f);
                    while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                }
                else if(knownObstacles[(curveCount + 1) & 3] == 0) goto middle;
                goto end;
            }
            else {
            middle:
                curveCount++;
                setVelocity(normalSpeed);
                setServo(direction ? -40.0f : 40.0f);
                while(direction ? (getRotation() > ((curveCount * -90.0f) + 23.0f)) : (getRotation() < ((curveCount * 90.0f) - 23.0f))) update();
                
                readObject();

                if(knownObstacles[curveCount & 3] == 1) {
                    nearInnerBorder = true;
                    while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                }
                else {
                    float startRotation = getRotation();
                    setServo(direction ? 20.0f : -20.0f);
                    while(abs(getRotation() - startRotation) < (((curveCount & 3) != 0) ? 15.0f : 2.5f)) update();
                    setServo(direction ? 40.0f : -40.0f);

                    if(((curveCount) & 3) != 0) {
                        while(direction ? (getRotation() < (((curveCount - 1) * -90.0f) - 7.0f)) : (getRotation() > (((curveCount - 1) * 90.0f) + 7.0f))) update();
                        timestamp = millis();
                        while((getDistance(FrontCenter) > 380) || (millis() < (timestamp + 100))) {
                            setServo((getRotation() - ((curveCount - 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                            update();
                        }
                        setServo(direction ? -40.0f : 40.0f);
                        while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                        nearInnerBorder = false;
                    }
                    else {
                        setVelocity(slowSpeed);
                        while(direction ? (getRotation() < (((curveCount - 1) * -90.0f) - 55.0f)) : (getRotation() > (((curveCount - 1) * 90.0f) + 55.0f))) update();
                        timestamp = millis();
                        while((getDistance(direction ? AngledLeft : AngledRight) > 500) || (millis() < (timestamp + 100))) {
                            setServo(((getRotation() - ((curveCount) * (direction ? -90.0f : 90.0f))) + (direction ? -50 : 50)) * reactionFactor * -2.0f);
                            update();
                        }
                        setServo(direction ? -40.0f : 40.0f);
                        while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                        nearInnerBorder = false;
                        setVelocity(normalSpeed);
                    }
                }
            end:
                timestamp = millis();
                setVelocity(fastSpeed);
                while((getDistance(FrontCenter) > 1350) || (millis() < (timestamp + 250))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    if((getDistance(FrontCenter) > 1400) && (getDistance(RearCenter) < 1550)) wallCorrection();
                    else update();
                }
                setVelocity(0.0f, true);
                wait(15);
                setVelocity(slowSpeed);
                uint32_t timestamp2 = millis();
                while((getDistance(direction ? FrontRight : FrontLeft) < 1100) || (getDistance(FrontCenter) > 1100) || (millis() < (timestamp2 + 50))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    if(getDistance(FrontCenter) < 110) {
                        curveCount++;
                        readObject(0, (int)(!direction) + 1);
                        if((nearInnerBorder) && (knownObstacles[curveCount & 3] == 0)) {
                            wait(50);
                            readObject(0, (int)(!direction) + 1);
                        }
                        curveCount--;
                    }
                    if((timestamp2 + 4000) > millis()) showSensorData();
                    update();
                }
            }
        }

        // second and third round
        else {
            if(knownObstacles[(curveCount + 1) & 3] == 2) {
                timestamp = millis();
                while((getDistance(FrontCenter) > ((((curveCount + 1) & 3) == 0) ? 550 : 360)) || (millis() < (timestamp + 100))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
                nearInnerBorder = false;
            }
            else {
                timestamp = millis();
                while(millis() < (timestamp + 25)) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
                nearInnerBorder = true;
            }
            setVelocity(normalSpeed);
            curveCount++;
            setServo(direction ? -45.0f : 45.0f);
            while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
            setVelocity(fastSpeed);
            timestamp = millis();
            while((getDistance(FrontCenter) > ((knownObstacles[(curveCount + 1) & 3] == 1) ? 1450 : ((((curveCount + 1) & 3) == 0) ? 1000 : 800))) || (millis() < (timestamp + 1500))) {
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                if((getDistance(FrontCenter) > 1400) && (getDistance(RearCenter) < 1550)) wallCorrection();
                else update();
            }
            setVelocity(slowSpeed, false, 1.0f, -1.0f);
            timestamp = millis();
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1300) || (getDistance(FrontCenter) > 1300)) || (millis() < (timestamp + 100))) {
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                update();
            }
            
        }
    }

    if(knownObstacles[(curveCount + 1) & 3] == 2) {
        timestamp = millis();
        while((getDistance(FrontCenter) > 550) || (millis() < (timestamp + 100))) {
            setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
            update();
        }
    }
    else {
        timestamp = millis();
        while(millis() < (timestamp + 25)) {
            setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
            update();
        }
    }
    setVelocity(normalSpeed);
    curveCount++;
    setServo(direction ? -45.0f : 45.0f);

    while(direction ? (getRotation() > (curveCount * -90.0f) + 10.0f) : (getRotation() < (curveCount * 90) - 10.0f)) update();
    setVelocity(fastSpeed);
    timestamp = millis();
    while(((getDistance(FrontCenter) > 1800) || (getDistance(RearCenter) < 1300)) || (millis() < (timestamp + 800))) {
        setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
        update();
    }
    
    // stop for > 3 seconds
    setVelocity(0.0f, true);
    update();
    showFinish();
    wait(3400);
    
    // find parking space
    setVelocity(slowSpeed);
    while((getDistance(FrontCenter) > 1500) || (getDistance(RearCenter) < 1350)) {
        setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
        update();
    }
    setVelocity(0.0f, true);
    uint32_t lastDistance = 0;
    uint32_t secondLastDistance = 0;
    if(knownObstacles[0] == 2) {
        if(parkingSpace) {
            setVelocity(-parkSpeed);
            lastDistance = getDistance(direction ? FrontLeft : FrontRight);
            secondLastDistance = lastDistance;
            while((getDistance(direction ? FrontLeft : FrontRight) < (lastDistance + parkingSpaceDetectionThreshold)) && (getDistance(direction ? FrontLeft : FrontRight) < (secondLastDistance + parkingSpaceDetectionThreshold))) {
                update();
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * 2.0f);
                if(getDistance(direction ? FrontLeft : FrontRight) != lastDistance) {
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction ? FrontLeft : FrontRight);
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -25.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 25.0f)) update();
            setVelocity(0.0f, true);
            setServo(direction ? -50.0f : 50.0f);
            wait(250);
            setVelocity(parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -40.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 40.0f)) update();
            setVelocity(0.0f, true);
            setServo(direction ? 50.0f : -50.0f);
            wait(250);
            setVelocity(-parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -85.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 85.0f)) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 35) || (getDistance(FrontRight) > 200) || (getDistance(FrontLeft) > 200)) {
                update();
                setServo((getRotation() - ((curveCount + 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * 2.0f);
            }
        }
        else {
            setVelocity(parkSpeed);
            lastDistance = getDistance(direction ? RearLeft : RearRight);
            secondLastDistance = lastDistance;
            while((getDistance(direction ? RearLeft : RearRight) < (lastDistance + parkingSpaceDetectionThreshold)) && (getDistance(direction ? RearLeft : RearRight) < (secondLastDistance + parkingSpaceDetectionThreshold))) {
                update();
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                if(getDistance(direction ? RearLeft : RearRight) != lastDistance) {
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction ? RearLeft : RearRight);
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 25.5f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -25.5f)) update();
            setVelocity(0.0f, true);
            wait(250);
            setServo(direction ? -50.0f : 50.0f);
            setVelocity(-parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 85.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -85.0f)) update();
            setVelocity(0.0f, true);
            wait(250);
            setVelocity(parkSpeed);
            while((getDistance(FrontCenter) > 35) || (getDistance(FrontRight) > 200) || (getDistance(FrontLeft) > 200)) {
                update();
                setServo((getRotation() - ((curveCount - 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
            }
        }
    }
    if(knownObstacles[0] == 1) {
        setServo(direction ? 50.0f : -50.0f);
        setVelocity(parkSpeed);
        while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 85.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -85.0f)) update();
        timestamp = millis();
        setServo(0.0f);
        while(((timestamp + 250) > millis()) || (getDistance(RearCenter) < 400)) {
            update();
            setServo((getRotation() - ((curveCount - 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
        }
        setServo(direction ? -50.0f : 50.0f);
        while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > 5.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 5.0f)) update();
        setVelocity(0.0f, true);
        wait(250);
        setServo(0.0f);
        setVelocity(-slowSpeed, false, 0.8f);
        timestamp = millis();
        while(((timestamp + 1000) > millis()) || (getDistance(FrontCenter) < (parkingSpace ? 1400 : 1600))) {
            update();
            setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * 2.0f);
        }
        if(parkingSpace) {
            setVelocity(-parkSpeed);
            lastDistance = getDistance(direction ? FrontLeft : FrontRight);
            secondLastDistance = lastDistance;
            while((getDistance(direction ? FrontLeft : FrontRight) < (lastDistance + parkingSpaceDetectionThreshold)) && (getDistance(direction ? FrontLeft : FrontRight) < (secondLastDistance + parkingSpaceDetectionThreshold))) {
                update();
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * 2.0f);
                if(getDistance(direction ? FrontLeft : FrontRight) != lastDistance) {
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction ? FrontLeft : FrontRight);
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -25.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 25.0f)) update();
            setVelocity(0.0f, true);
            setServo(direction ? -50.0f : 50.0f);
            wait(250);
            setVelocity(parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -40.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 40.0f)) update();
            setVelocity(0.0f, true);
            setServo(direction ? 50.0f : -50.0f);
            wait(250);
            setVelocity(-parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -85.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 85.0f)) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 35) || (getDistance(FrontRight) > 200) || (getDistance(FrontLeft) > 200)) {
                update();
                setServo((getRotation() - ((curveCount + 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * 2.0f);
            }
        }
        else {
            setVelocity(parkSpeed);
            lastDistance = getDistance(direction ? RearLeft : RearRight);
            secondLastDistance = getDistance(direction ? RearLeft : RearRight);
            while((getDistance(direction ? RearLeft : RearRight) < (lastDistance + parkingSpaceDetectionThreshold)) && (getDistance(direction ? RearLeft : RearRight) < (secondLastDistance + parkingSpaceDetectionThreshold))) {
                update();
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                if(getDistance(direction ? RearLeft : RearRight) != lastDistance) {
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction ? RearLeft : RearRight);
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 25.5f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -25.5f)) update();
            setVelocity(0.0f, true);
            wait(250);
            setServo(direction ? -50.0f : 50.0f);
            setVelocity(-parkSpeed);
            while(direction ? ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) < 85.0f) : ((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) > -85.0f)) update();
            setVelocity(0.0f, true);
            wait(250);
            setVelocity(parkSpeed);
            while((getDistance(FrontCenter) > 35) || (getDistance(FrontRight) > 200) || (getDistance(FrontLeft) > 200)) {
                update();
                setServo((getRotation() - ((curveCount - 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
            }
        }
    }
    setVelocity(0.0f, true);
    showFinish();
}

void loop() {
    update();
}

#endif
#endif