/**
 * main.cpp - driving strategy and logic
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// select program
//#define TEST
#define OBJECTS

#include "car.hpp"

#ifdef TEST
#ifndef OBJECTS

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::runCalibration();
    CAR::start();
}

void loop() {
    CAR::update();
    CAR::showSensorData();
    digitalToggleFast(13);
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
    digitalToggleFast(13);
}

#endif
#else

using namespace CAR;

#ifndef OBJECTS

// angle configuration
constexpr float curveEnd = 52.5f;
constexpr float curveServoAngle = 35.0f;
constexpr float reactionFactor = -0.25f;
constexpr float wallReactionAngle = 10.0f;

// speed configuration
constexpr float boostSpeed  = 1.0f;
constexpr float normalSpeed = 0.8f;

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
    setVelocity(boostSpeed, false, 0.5f, -0.3f);
    boost = true;
    curveCount++;
}

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::start();

    // start driving
    setVelocity(normalSpeed, false, 0.4f, -0.4f);

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
    if((timestamp + 1300) < millis()) {
        if(((direction ? (getDistance(FrontRight) > 800) : (getDistance(FrontLeft) > 800)) || (getDistance(FrontCenter) < ((sectionWidth[curveCount & 3] == 2) ? 450 : 350))) 
            && (getDistance(RearCenter) > 1700)
            && (getRotation() + (direction ? (curveCount * 90.0f) : (curveCount * -90.0f)) < 7.0f))
            driveCurve();
    }

    // drive away from the left wall when getting too close (< 15 cm) and next curve is far enough away (> 80 cm)
    else if((getDistance(FrontLeft) < 150) && (getDistance(FrontCenter) > 800)) {
        setServo(-wallReactionAngle);
        while(getRotation() > (((direction == 0) ? 90.0f : -90.0f) * curveCount) - 5.0f) update();
        if((timestamp + 800) < millis()) timestamp = millis() - 800;
    }

    // drive away from the right wall when getting too close (< 15 cm) and next curve is far enough away (> 80 cm)
    else if((getDistance(FrontRight) < 150) && (getDistance(FrontCenter) > 800)) {
        setServo(wallReactionAngle);
        while(getRotation() < (((direction == 0) ? 90.0f : -90.0f) * curveCount) + 5.0f) update();
        if((timestamp + 800) < millis()) timestamp = millis() - 800;
    }

    // correct vehicle rotation (using rotation feedback)
    setServo((getRotation() - (direction ? (curveCount * -90.0f) : (curveCount * 90.0f))) * reactionFactor);

    // measure width of current section
    if((sectionWidth[curveCount & 3] == 0) && (getDistance(RearCenter) > 1200) && (abs(abs(getRotation()) - (curveCount * 90.0f)) < 5.0f)) {
        sectionWidth[curveCount & 3] = ((getDistance(FrontLeft) + getDistance(FrontRight)) > 750) + 1;
    }

    // deactivate boost when approaching next curve
    if(boost && ((getDistance(RearCenter) > (((sectionWidth[(curveCount + 1) & 3] == 1)) ? 1800 : 2050))) && ((timestamp + 500) < millis())) {
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
constexpr float curveServoAngle    = 37.5f;
constexpr float offsetModification = 0.5;
constexpr float reactionFactor     = 0.3f;
constexpr float wallReactionAngle  = 10.0f;
constexpr float wallReactionAngle2 = 6.0f;

// distance configuration
constexpr uint16_t wallReactionDistance = 100;

// speed configuration
constexpr float normalSpeed = 0.4f;
constexpr float fastSpeed   = 0.6f;
constexpr float parkSpeed   = 0.15f;
constexpr float slowSpeed   = 0.25f;

bool direction = 0; // 0 = counterclockwise, 1 = clockwise
bool parkingSpace = 0; // 1 = parking space at the end of the section, 1 = parking space at the beginning of the section (in driving direction)
uint8_t knownObstacles[4] = {}; // 0 = unknown, 1 = drive next to the inner border, 2 = drive next to the outer border
bool nearInnerBorder = false; // indicates where the vehicle is currently driving
uint8_t curveCount = 0;

inline void wallCorrection() {
    update();
    if((knownObstacles[curveCount & 3] == 1) == direction) {
        if(getDistance(FrontRight) < wallReactionDistance) {
            setServo(wallReactionAngle);
            while((getRotation() - (curveCount * (direction ? -90.0f : 90))) < wallReactionAngle2) update();
            modifyRotationOffset(offsetModification);
        }
    }
    else{
        if(getDistance(FrontLeft) < wallReactionDistance) {
            setServo(-wallReactionAngle);
            while((getRotation() - (curveCount * (direction ? -90.0f : 90))) > (-wallReactionAngle2)) update();
            modifyRotationOffset(-offsetModification);
        }
    }
    setServo(0.0f);
}

void setup() {
    CAR::init(true);
    CAR::waitButtonPress();
    CAR::start();

    // leave parking space and detect position
    direction = getDistance(FrontRight) > 200;
    setServo(direction ? -50.0f : 50.0f);
    setVelocity(parkSpeed);
    while(direction ? (getRotation() > -25.0f) : (getRotation() < 23.0f)) update();
    setVelocity(0.0f, true);
    uint32_t timestamp = millis();
    while(millis() < (timestamp + 250)) update();
    setServo(direction ? 50.0f : -50.0f);
    setVelocity(-parkSpeed);
    while((direction ? (getRotation() > -55.0f) : (getRotation() < 55.0f))) update();
    setVelocity(0.0f, true);
    setServo(0.0f);
    timestamp = millis();
    while(millis() < (timestamp + 250)) update();
    setServo(direction ? -50.0f : 50.0f);
    setVelocity(parkSpeed);
    while(direction ? (getRotation() > -85.0f) : (getRotation() < 85.0f)) update();
    setVelocity(0.0f, true);
    timestamp = millis();
    while(millis() < (timestamp + 250)) update();
    setVelocity(parkSpeed);
    while((getDistance(FrontLeft) < 200) || (getDistance(FrontRight) < 200) || (getDistance(RearCenter) < 115)) {
        setServo((getRotation() - (direction ? -90.0f : 90.0f)) * (-reactionFactor));
        update();
    }
    parkingSpace = getDistance(direction ? FrontLeft : FrontRight) > getDistance(direction ? FrontRight : FrontLeft);

    if(parkingSpace) {
        setServo(direction ? 45.0f : -45.0f);
        setVelocity(parkSpeed);
        while(direction ? (getRotation() < -32.0f) : (getRotation() > 32.0f)) update();
        setVelocity(0.0f, true);
        timestamp = millis();
        while(millis() < (timestamp + 300)) update();
        std::vector<OBJECT> objects = getObjects();
        bool inside = false;
        for(OBJECT object : objects) {
            if((object.c == (direction ? RED : GREEN)) && (object.h > 120)) {
                inside = true;
                knownObstacles[0] = 1;
            }
            else if((object.c == (direction ? GREEN : RED)) && (object.h > 120)) knownObstacles[0] = 2;
        }
        if(inside) {
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
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < (timestamp + 2000))) {
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
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < (timestamp + 2000))) {
                setServo(getRotation() * (-reactionFactor));
                update();
            }
            nearInnerBorder = false;
        }
    }
    else {
        setVelocity(0.0f, true);
        timestamp = millis();
        while(millis() < (timestamp + 300)) update();
        std::vector<OBJECT> objects = getObjects();
        bool inside = false;
        for(OBJECT object : objects) {
            if((object.c == (direction ? RED : GREEN)) && (object.h > 200)) {
                inside = true;
                knownObstacles[0] = 1;
            }
            else if((object.c == (direction ? GREEN : RED)) && (object.h > 200)) knownObstacles[0] = 2;
        }
        if(inside) {
            setVelocity(parkSpeed);
            timestamp = millis();
            while((getDistance(RearCenter) < 250) || (millis() < (timestamp + 500))) {
                setServo((getRotation() - (direction ? -90.0f : 90.0f)) * (-reactionFactor));
                update();
            }
            setServo(direction ? -40.0f : 40.0f);
            setVelocity(slowSpeed);
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
            setVelocity(parkSpeed);
            while(direction ? (getRotation() < -2.0f) : (getRotation() > 2.0f)) update();
            setServo(0.0f);
            nearInnerBorder = false;
        }
    }
    setVelocity(0.0f, true);

    while(curveCount < 11) {
        if(knownObstacles[(curveCount + 1) & 3] == 0) {
            if(nearInnerBorder) {
                while((getDistance(FrontCenter) > 925) && (getDistance(direction ? RearRight : RearLeft) < 1000)) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * (-reactionFactor));
                    update();
                }
                if(getDistance(direction ? RearRight : RearLeft) < 1000) {
                    setVelocity(0.0f, true);
                    timestamp = millis();
                    while((timestamp + 100) > millis()) update();
                    setVelocity(-slowSpeed);
                    timestamp = millis();
                    while((timestamp + 600) > millis()) {
                        setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor);
                        update();
                    } 
                }
                setVelocity(parkSpeed);
                setServo(direction ? -45.0f : 45.0f);
                while(direction ? (getRotation() > ((curveCount * -90.0f) - 60.0f)) : (getRotation() < ((curveCount * 90.0f) + 60.0f))) update();
                timestamp = millis();
                setVelocity(0.0f, true);
                while((timestamp + 150) > millis()) update();
                setVelocity(-parkSpeed);
                while(direction ? (!onOrangeLine()) : (!onBlueLine())) {
                    setServo((getRotation() - ((curveCount * (direction ? -90.0f : 90.0f)) - (direction ? 60.0f : -60.0f))) * reactionFactor * 2.0f);
                    update();
                }
                setVelocity(0.0f, true);
                curveCount++;
            }
            else {
                setVelocity(slowSpeed);
                while((getDistance(FrontCenter) > 925) && (getDistance(direction ? RearRight : RearLeft) < 1000)) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * (-reactionFactor));
                    update();
                }
                if(getDistance(direction ? RearRight : RearLeft) < 1000) {
                    setVelocity(0.0f, true);
                    timestamp = millis();
                    while((timestamp + 100) > millis()) update();
                    setVelocity(-slowSpeed);
                    timestamp = millis();
                    while((timestamp + 600) > millis()) {
                        setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor);
                        update();
                    }
                }
                curveCount++;
                setVelocity(normalSpeed);
                setServo(direction ? -45.0f : 45.0f);
                while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                timestamp = millis();
                while((getDistance(RearCenter) < 600) || (millis() < (timestamp + 200))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
                timestamp = millis();
                setVelocity(0.0f, true);
                while((timestamp + 250) > millis()) update();
                setVelocity(-slowSpeed);
                setServo(direction ? -45.0f : 45.0f);
                while(direction ? (getRotation() < ((curveCount * -90.0f) + 25.0f)) : (getRotation() > ((curveCount * 90.0f) - 25.0f))) update();
                timestamp = millis();
                setVelocity(0.0f, true);
                while((timestamp + 250) > millis()) update();
            }
            std::vector<OBJECT> objects = getObjects();
            for(OBJECT object : objects) {
                if(object.c == (direction ? RED : GREEN)) knownObstacles[curveCount & 3] = 1;
                else if(object.c == (direction ? GREEN : RED)) knownObstacles[curveCount & 3] = 2;
            }
            if(knownObstacles[curveCount & 3] == 0) {
                timestamp = millis();
                while((timestamp + 200) > millis()) update();
                std::vector<OBJECT> objects = getObjects();
                for(OBJECT object : objects) {
                    if(object.c == (direction ? RED : GREEN)) knownObstacles[curveCount & 3] = 1;
                    else if(object.c == (direction ? GREEN : RED)) knownObstacles[curveCount & 3] = 2;
                }
            }
            if(knownObstacles[curveCount & 3] == 1) {
                nearInnerBorder = true;
                setVelocity(normalSpeed);
                setServo(direction ? -45.0f : 45.0f);
                while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                setVelocity(fastSpeed);
                timestamp = millis();
                while((getDistance(FrontCenter) > 1400) || (millis() < (timestamp + 200))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    wallCorrection();
                }
                setVelocity(0.0f, true);
                timestamp = millis();
                while((timestamp + 15) > millis()) update();
                setVelocity(slowSpeed);
                timestamp = millis();
                while((getDistance(direction ? FrontRight : FrontLeft) < 1000) || (getDistance(FrontCenter) > 940) || (millis() < (timestamp + 100))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
            }
            else {
                nearInnerBorder = false;
                setVelocity(slowSpeed);
                setServo(direction ? 45.0f : -45.0f);
                while(direction ? (getRotation() < ((curveCount * -90.0f) + 85.0f)) : (getRotation() > ((curveCount * 90.0f) - 85.0f))) update();
                timestamp = millis();
                while((getDistance(FrontCenter) > (((curveCount & 3) == 0) ? 550 : 200)) || (millis() < (timestamp + 200))) {
                    setServo((getRotation() - ((curveCount - 1) * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
                setVelocity(normalSpeed);
                setServo(direction ? -45.0f : 45.0f);
                while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
                setVelocity(fastSpeed);
                timestamp = millis();
                while((getDistance(FrontCenter) > 1500) || (millis() < (timestamp + 200))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    wallCorrection();
                }
                setVelocity(0.0f, true);
                timestamp = millis();
                while((timestamp + 15) > millis()) update();
                setVelocity(slowSpeed);
                timestamp = millis();
                while((getDistance(direction ? FrontRight : FrontLeft) < 1000) || (getDistance(FrontCenter) > 940) || (millis() < (timestamp + 100))) {
                    setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                    update();
                }
            }
        }
        else {
            if(knownObstacles[(curveCount + 1) & 3] == 2) {
                timestamp = millis();
                while((getDistance(FrontCenter) > ((((curveCount + 1) & 3) == 0) ? 500 : 250)) || (millis() < (timestamp + 100))) {
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
            while(direction ? (getRotation() > ((curveCount * -90.0f) + 10.0f)) : (getRotation() < ((curveCount * 90.0f) - 10.0f))) update();
            setVelocity(fastSpeed);
            timestamp = millis();
            while((getDistance(FrontCenter) > ((knownObstacles[(curveCount + 1) & 3] == 1) ? 1400 : 800)) || (millis() < (timestamp + 1500))) {
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                wallCorrection();
            }
            setVelocity(slowSpeed, false, 0.25f, -4.0f);
            timestamp = millis();
            while(((getDistance(direction ? FrontRight : FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < (timestamp + 100))) {
                setServo((getRotation() - (curveCount * (direction ? -90.0f : 90.0f))) * reactionFactor * -2.0f);
                update();
            }
            
        }
    }

    if(knownObstacles[(curveCount + 1) & 3] == 2) {
        timestamp = millis();
        while((getDistance(FrontCenter) > ((((curveCount + 1) & 3) == 0) ? 500 : 250)) || (millis() < (timestamp + 100))) {
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

    // unformatted!!! //
    while(direction ? (getRotation() > (curveCount*(-90))+10.0f) : (getRotation() < (curveCount*(90))-10.0f)) update();
    setVelocity(fastSpeed);
    timestamp = millis();
    while(((getDistance(FrontCenter) > 1800) || (getDistance(RearCenter) < 1300)) || (millis() < (timestamp + 200))) {
        setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
        update();
    }
    
    // stop for > 3 seconds
    timestamp = millis();
    setVelocity(0.0f, true);
    update();
    showFinish();
    while((timestamp + 3500) > millis()) update();
    setVelocity(slowSpeed);

    // find parking space
    while((getDistance(FrontCenter) > 1500) || (getDistance(RearCenter) < 1350)) {
        setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
        update();
    }
    setVelocity(0.0f, true);

    uint32_t lastDistance = 0;
    uint32_t secondLastDistance = 0;
    if(knownObstacles[0] == 2){
        if(parkingSpace){
            setVelocity(-parkSpeed);
            lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            secondLastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            while((getDistance(direction?(FrontLeft):(FrontRight)) < lastDistance + 40)&&(getDistance(direction?(FrontLeft):(FrontRight)) < secondLastDistance + 40)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
                if(getDistance(direction?(FrontLeft):(FrontRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            setVelocity(-parkSpeed);
            while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((curveCount + 1)*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
            }
        }
        else{
            setVelocity(parkSpeed);
            lastDistance = getDistance(direction?(RearLeft):(RearRight));
            secondLastDistance = getDistance(direction?(RearLeft):(RearRight));
            while((getDistance(direction?(RearLeft):(RearRight)) < lastDistance + 40)&&(getDistance(direction?(RearLeft):(RearRight)) < secondLastDistance + 40)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
                if(getDistance(direction?(RearLeft):(RearRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(RearLeft):(RearRight));
                }
            }
            lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            secondLastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            while((getDistance(direction?(FrontLeft):(FrontRight)) < lastDistance + 20)&&(getDistance(direction?(FrontLeft):(FrontRight)) < secondLastDistance + 20)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
                if(getDistance(direction?(FrontLeft):(FrontRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
                }
            }
            setVelocity(0.0f,true);
            timestamp=millis();
            while(timestamp + 250 > millis())update();
            setServo(direction ? 50.0f : -50.0f);
            setVelocity(-parkSpeed);
            while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((curveCount + 1)*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
            }
        }
    }
    if(knownObstacles[0] == 1){
        setServo(direction ? 50.0f : -50.0f);
        setVelocity(parkSpeed);
        while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) < 85.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) > -85.0f))) update();
        timestamp=millis();
        setServo(0);
        while((timestamp + 250 > millis())||(getDistance(RearCenter) < 400)){
            update();
            setServo(((getRotation() - ((curveCount-1)*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
        }
        setServo(direction ? -50.0f : 50.0f);
        setVelocity(parkSpeed);
        while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) > 5.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) < 5.0f))) update();
        timestamp=millis();
        setVelocity(0.0f,true);
        while(timestamp+250 > millis())update();
        setServo(0);
        setVelocity(-slowSpeed);
        timestamp=millis();
        setServo(0);
        while((timestamp + 250 > millis())||(getDistance(FrontCenter) < ((parkingSpace)?1400:1700))){
            update();
            setServo(((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
        }

        if(parkingSpace){
            setVelocity(-parkSpeed);
            lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            secondLastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            while((getDistance(direction?(FrontLeft):(FrontRight)) < lastDistance + 40)&&(getDistance(direction?(FrontLeft):(FrontRight)) < secondLastDistance + 40)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
                if(getDistance(direction?(FrontLeft):(FrontRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
                }
            }
            setServo(direction ? 50.0f : -50.0f);
            setVelocity(-parkSpeed);
            while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((curveCount + 1)*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
            }
        }
        else{
            setVelocity(parkSpeed);
            lastDistance = getDistance(direction?(RearLeft):(RearRight));
            secondLastDistance = getDistance(direction?(RearLeft):(RearRight));
            while((getDistance(direction?(RearLeft):(RearRight)) < lastDistance + 40)&&(getDistance(direction?(RearLeft):(RearRight)) < secondLastDistance + 40)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
                if(getDistance(direction?(RearLeft):(RearRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(RearLeft):(RearRight));
                }
            }
            lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            secondLastDistance = getDistance(direction?(FrontLeft):(FrontRight));
            while((getDistance(direction?(FrontLeft):(FrontRight)) < lastDistance + 20)&&(getDistance(direction?(FrontLeft):(FrontRight)) < secondLastDistance + 20)){
                update();
                setServo(((getRotation() - (curveCount*(direction?-90.0f:90.0f))))*((-reactionFactor*2)));
                if(getDistance(direction?(FrontLeft):(FrontRight)) != lastDistance){
                    secondLastDistance = lastDistance;
                    lastDistance = getDistance(direction?(FrontLeft):(FrontRight));
                }
            }
            setVelocity(0.0f,true);
            timestamp=millis();
            while(timestamp + 250 > millis())update();

            setServo(direction ? 50.0f : -50.0f);
            setVelocity(-parkSpeed);
            while((direction ? ((getRotation() - ((curveCount)*(direction?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (curveCount*(direction?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((curveCount + 1)*(direction?-90.0f:90.0f))))*((reactionFactor*2)));
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