/**
 * main.cpp - driving strategy and logic
 * by TerraForces
 */

// select program
#define TEST
#define OBJECTS

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

// activate to enter parking space at the end (always starts from parking space)
int32_t lastDistance = 0;

// speed and angle configuration (speed as a fraction of the maximum speed; angle in degrees)
constexpr float PARK_ANGLE    = 45.0f; // servo angle used for leaving the parking space
constexpr float SPEED_PARK    = 0.15f; // motor speed while leaving the parking space
constexpr float SPEED_PARK_IN = 0.1f;  // motor speed while entering the parking space
constexpr float SPEED_DRIVE   = 0.40f; // regular driving speed
constexpr float MIN_RCR_ANGLE = 0.20f; // minimum rotation difference to be corrected

// global working variables
bool outsideBorder = 0;          // position of the outside border relative to the vehicle (left = 0; right = 1)
bool drivingSide = 0;            // current driving side (side of closest border; left = 0; right = 1)
int32_t curveTargetRotation = 0; // driving target rotation (always multiples of 90°)
int64_t startTime = 0;           // driving start time in microseconds since boot
uint8_t section[4][3] = {};      // array containing all necessary information for object avoidance (4 straigtforward section with 3 positions in driving order; values represent traffic sign colors: 0 = unknown, 1 = red, 2 = green, 3 = no traffic sign)

// works like a delay but updates while waiting
void delayUpdate(uint32_t ms) {
    uint32_t endMicros = micros() + (ms * 1000);
    while(endMicros < micros()) update();
}

// curve mode - drive a 90° curve in the specified direction
void curve(bool direction, float sharpness = 0.0f) {
	setVelocity(0.3f);
	setServo(direction ? 42.0f : -42.0f);
	curveTargetRotation += direction ? 90 : -90;

	// wait until rotation says that curve is completed
	while (direction ? (getRotation() < (curveTargetRotation - 24.0f + sharpness)) : (getRotation() > (curveTargetRotation + 24.0f - sharpness))) update();
	setServo(0.0f);
	setVelocity(SPEED_DRIVE);
}

// correct orientation using the rotation data provided by the gyro sensor
void rotationCorrection(bool invert = false) {
    update();
	float rotation = getRotation();
	if ((curveTargetRotation - rotation) < MIN_RCR_ANGLE) setServo(invert ? -max(curveTargetRotation - rotation + MIN_RCR_ANGLE, -10.0f) : max(curveTargetRotation - rotation + MIN_RCR_ANGLE, -10.0f));
	else if ((curveTargetRotation - rotation) > MIN_RCR_ANGLE) setServo(invert ? -min(curveTargetRotation - rotation - MIN_RCR_ANGLE,  10.0f) : min(curveTargetRotation - rotation - MIN_RCR_ANGLE,  10.0f));
    update();
}

void setup() {

	CAR::init();
    CAR::waitButtonPress();
    CAR::start();

	// detect outside border
    update();
	outsideBorder = getDistance(RearLeft) > 300;

	// leave parking space using 4 move consisting of inverting the servo and motor direction and driving until reaching specified rotations (determined experimentally)
	setServo(outsideBorder ? PARK_ANGLE : -PARK_ANGLE);
	setVelocity(-SPEED_PARK);
	while (outsideBorder ? (getRotation() > -20.0f) : (getRotation() < 20.0f)) update();
	setServo(outsideBorder ? -PARK_ANGLE : PARK_ANGLE);
	setVelocity(SPEED_PARK);
	while (outsideBorder ? (getRotation() > -40.0f) : (getRotation() < 40.0f)) update();
	setServo(outsideBorder ? PARK_ANGLE : -PARK_ANGLE);
	setVelocity(-SPEED_PARK);
	while (outsideBorder ? (getRotation() > -60.0f) : (getRotation() < 60.0f)) update();
	setServo(outsideBorder ? -PARK_ANGLE : PARK_ANGLE);
	setVelocity(SPEED_PARK);
	while (outsideBorder ? (getRotation() > -92.0f) : (getRotation() < 92.0f)) update();
	setServo(0.0f);

	// set new target rotation
	curveTargetRotation = outsideBorder ? -90 : 90;

	// set driving side to the inner border
	drivingSide = 1 - outsideBorder;

	// object avoidance code for the starting section
	#pragma region section 0
	// region defined for easier code navigation

	// different approaches for the two possible driving directions necessary
	// approach for the possibility of the car driving counterclockwise
	if(outsideBorder) {
		setVelocity(SPEED_DRIVE);

		// check whether there is an object to avoid while leaving the parking space
		for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[0][2] == 0); i++) {	// loop that checks every object detected
			if(getObjects()[i].c != PINK) {												// condition that checks whether the current object is not pink
				if(getObjects()[i].x > 650) {											// condition that checks whether the current object is on the left side of the picture
					if((getObjects()[i].y + getObjects()[i].h) > 520) section[0][2] = getObjects()[i].c;   	// condition that checks whether the current object starts near the bottom of the picture, and saves the object colour and position in the array, called section
				}
			}
		}
		if(section[0][2] == 0) section[0][2] = 3;								// save in the section array whether no object was detected
		if(section[0][2] == RED) {												// condition that checks whether the car has to pass an object at the side of the outside border

			// let the car drive until it is time to turn to pass the object at the outer side
			drivingSide = outsideBorder;
			while (getDistance(FrontCenter) > 650) rotationCorrection();
		}
		else {

			// let the car drive until it is time to turn to pass the object at the inner side
			drivingSide = !outsideBorder;
			while (getDistance(FrontCenter) > 350) rotationCorrection();
		}
		curve(outsideBorder); 	// turn into the direction of the outside border to align to drive into the next section
	}

	else {
		setVelocity(0.0f);
		delayUpdate(50);

		// checks whether there is an object to avoid at the start of the second round
		for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[0][0] == 0); i++) {	// loop that checks every object detected
			if(getObjects()[i].c != PINK) {												// condition that checks whether the current object is not pink
				if(getObjects()[i].x > 650) {											// condition that checks whether the current object is on the left side of the picture
					if((getObjects()[i].y + getObjects()[i].h) > 480) section[0][0] = getObjects()[i].c;	// condition that checks whether the current object starts near the bottom of the picture, and saves the object colour and position in the array, called section
				}
			}
		}

		// reposition in a 60° angle to check whether the car has to avoid a obstacle while leaving the first section
		setVelocity(0.45f);
		setServo(-40.0f);
		curveTargetRotation -= 60;
		while ((getRotation() > (curveTargetRotation + 25.0f))) update();
		setServo(0.0f);
		setVelocity(0.0f);
		delayUpdate(100);
		setVelocity(SPEED_DRIVE);

		// checks whether there is an object to avoid while leaving the first section
		for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[0][1] == 0); i++) {			// loop that checks every object detected
			if(getObjects()[i].c != PINK) {														// condition that checks whether the current object is not pink
				if(getObjects()[i].x < 650) {													// condition that checks whether the current object is on the right side of the picture
					if((getObjects()[i].y + getObjects()[i].h) > 500) section[0][1] = getObjects()[i].c;			// condition that checks whether the current object is in the middle of the current section, and saves the object colour and position in the array, called section
					else if ((getObjects()[i].y + getObjects()[i].h) > 440) section[0][2] = getObjects()[i].c;	// condition that checks whether the current object is at the end of the current section of the section, and saves the object colour and position in the array, called section
				}
			}
		}
		if(section[0][0] == 0) section[0][0] = 3;		// save in the section array whether no object was detected at the start of the section
		if(section[0][2] == 0) section[0][2] = 3;		// save in the section array whether no object was detected in the middle of the section
		if(section[0][1] == 0) section[0][1] = 3;		// save in the section array whether no object was detected at the end of the section

		// condition that checks whether the car has to pass an object at the side of the outside border
		if((section[0][1] == GREEN) || ((section[0][2] == GREEN) && (section[0][1] == 3))) {

			// align parallel to the outside border to pass the object at the side of the outside border
			drivingSide = outsideBorder;
			setVelocity(0.45f);
			setServo(-40.0f);
			curveTargetRotation -= 30;
			while ((getRotation() > (curveTargetRotation + 5.0f))) update();
			setServo(0.0f);
		}
		else {

			// drive to the inside border
			drivingSide = !outsideBorder;
			setVelocity(0.45f);
			setServo(20.0f);
			curveTargetRotation += 60;
			while (getRotation() < (curveTargetRotation - 5.0f)) update();
			setServo(0.0f);
			while (getDistance(FrontCenter) > 340) rotationCorrection();

			// align parallel to the inside border 
			curve(outsideBorder);
		}
	}
	while (getDistance(outsideBorder ? RearLeft : RearRight) < 1500) rotationCorrection(); 		// pass the object and drive into the next section

	if((section[0][0] != 3) && (section[0][0] != 0)) {
		if(section[0][1] != 3) {
			section[0][2] = section[0][1];
			section[0][1] = 3;
		}
	}

	// object avoidance code for the sections 1 to 3
	#pragma region sections 1 - 3

	// loop to repeat the code for the sections 1 to 3
	for(uint8_t j = 1; j <= 3; j++) {

		// align at the outside border to check the current section for obstacles
		if(drivingSide == outsideBorder) {										// check the current position of the car to decide how to align at the outside border
			while(getDistance(FrontCenter) > 975) rotationCorrection();		// drive into the corner section
			curve(!outsideBorder);												// turn in the direction of the new section
			setVelocity(-0.35f);	
			while(getDistance(RearCenter) > 110) rotationCorrection(true);	// drive backwards to the outside border to align at it
			setVelocity(0.0f);
		}
		else {
			while(getDistance(FrontCenter) > 500) rotationCorrection();		// drive into the corner section
			setVelocity(-SPEED_DRIVE);
			while(getDistance(FrontCenter) < 560) rotationCorrection();	
			
			// turn while driving backwards to face into the direction of the new section
			if(outsideBorder) {
				setVelocity(-0.3f);
				setServo(40.0f);
				curveTargetRotation += -90;
				while (getRotation() > (curveTargetRotation + 25.0f)) update();
			}
			else {
				setVelocity(-0.3f);
				setServo(-40.0f);
				curveTargetRotation += 90;
				while (getRotation() < (curveTargetRotation - 25.0f)) update();
			}
			
			setVelocity(-0.5f);
			while(getDistance(RearCenter) > 110) rotationCorrection(true); 	// drive backwards to the outside border to align at it
			setVelocity(0.0f);
		}
		int64_t waitTime_new = millis();

		drivingSide = !outsideBorder;		// reset the position variable

		// loop to detect whether there is an object at the nearest 2 positions in the new section
		for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[j][0] == 0); i++) {		// loop that checks every object detected
			if(getObjects()[i].c != PINK) {													// condition that checks whether the current object is not pink
				if(outsideBorder? getObjects()[i].x > 650 : getObjects()[i].x < 650) { 	// condition that checks whether the current object is on the side of the outsideborder
					if((getObjects()[i].y + getObjects()[i].h) > 437) section[j][0] = getObjects()[i].c;		// condition that checks whether the current object is at the start of the new section of the section, and saves the object colour and position in the array, called section
				}
			}
		}

		// detect the position of the object, if there was no object in the first position
		if(section[j][0] == 0) {
			section[j][0] = 3;

			// check whether there is an object in the middle of the section
			for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[j][1] == 0); i++) {
				if(getObjects()[i].c != PINK) {
					if(outsideBorder? getObjects()[i].x > 650 : getObjects()[i].x < 650) {
						if((getObjects()[i].y + getObjects()[i].h) > 380) section[j][1] = getObjects()[i].c;
					}
				}
			}

			// check whether there is an object at the end of the section
			if(section[j][1] == 0) {
				section[j][1] = 3;
				for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[j][2] == 0); i++) {
					if(getObjects()[i].c != PINK) {
						if(outsideBorder? getObjects()[i].x > 650 : getObjects()[i].x < 650) {
							if((getObjects()[i].y + getObjects()[i].h) > 350) section[j][2] = getObjects()[i].c;
						}
					}
				}	
				if(section[j][2] == 0) section[j][2] = 3;
			}
			else section[j][2] = 3;
		}

		if(section[j][0] != 3) {
			if(section[j][0] == (outsideBorder ? RED : GREEN)) {				// checks whether the car has to pass the object at the side of the outside border
				curve(outsideBorder);											// turn to the outside border
				while(getDistance(FrontCenter) > 340) rotationCorrection();	// drive to the outside border
				curve(!outsideBorder);											// realign parallel to the outside border
				drivingSide = outsideBorder;
			}
			setVelocity(0.5f);
			uint64_t wait_time = millis();
			while(wait_time > (millis() - 1500000)) rotationCorrection();
			while((getDistance(FrontCenter) > 1900) || (getDistance(RearCenter) < 900)) rotationCorrection();		// drive completely into the straight section
			delayUpdate(100);

			// check whether there is a second object in the current section
			for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[j][2] == 0); i++) {
				if(getObjects()[i].c != PINK) {
					if((getObjects()[i].y + getObjects()[i].h) > 390) section[j][2] = getObjects()[i].c;
				}
			}
			delayUpdate(100); // wait for a new image

			// ckeck again if there is a second object, in case the car does not recognise it at the first image
			for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[j][2] == 0); i++) {
				if(getObjects()[i].c != PINK) {
					if((getObjects()[i].y + getObjects()[i].h) > 390) section[j][2] = getObjects()[i].c;
				}
			}
			if(section[j][1] == 0) section[j][1] = 3;
			if(section[j][2] == 0) section[j][2] = 3;

			// if required drive back to the inside border
			if((section[j][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
				curve(!outsideBorder);
				while(getDistance(FrontCenter) > 350) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
				delayUpdate(50);
			}

			// if required drive to the outside border
			if((section[j][2] == (outsideBorder ? RED : GREEN)) && drivingSide != outsideBorder) {
				curve(outsideBorder);
				while(getDistance(FrontCenter) > 350) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
				delayUpdate(50);
			}
			setVelocity(0.5f);
			while(((getDistance(outsideBorder ? RearLeft : RearRight) < 1100) || (getDistance(FrontCenter) > 1500))||((waitTime_new + 2000000) > millis())) rotationCorrection(); // pass the object and drive to the next corner section
		}

		// drive code for the current section if there only can be one object, because there was no object in the first position
		else {
			if((section[j][1] == (outsideBorder ? RED : GREEN)) || ((section[j][1] == 3) && (section[j][2] == (outsideBorder ? RED : GREEN)))) {	// checks whether the car has to pass the object at the side of the outside border
				curve(outsideBorder);												// turn to the outside border
				while(getDistance(FrontCenter) > 350) rotationCorrection();		// drive to the outsideborder
				curve(!outsideBorder);												// realign parallel to the outsideborder
				drivingSide = outsideBorder;
				delayUpdate(50);
			}
			setVelocity(0.5f);
			while(((getDistance(outsideBorder ? RearLeft : RearRight) < 1100) || (getDistance(FrontCenter) > 1500))||((waitTime_new + 2000000) > millis())) rotationCorrection(); // pass the object and drive to the next corner section
		}
	}

	#pragma region section 4

	// align at the outside border and check the position of the first object, the same way as in the last 3 sections
	if(drivingSide == outsideBorder) {
		while(getDistance(FrontCenter) > 975) rotationCorrection();
		curve(!outsideBorder);
		setVelocity(-0.35f);
		while(getDistance(RearCenter) > 110) rotationCorrection(true);
		setVelocity(0.0f);
	}
	else {
		while(getDistance(FrontCenter) > 500) rotationCorrection();
		setVelocity(-SPEED_DRIVE);
		while(getDistance(FrontCenter) < 620) rotationCorrection(true);
		if(outsideBorder) {
			setVelocity(-0.3f);
			setServo(40.0f);
			curveTargetRotation += -90;
			while (getRotation() > (curveTargetRotation + 25.0f)) update();
		}
		else {
			setVelocity(-0.3f);
			setServo(-40.0f);
			curveTargetRotation += 90;
			while (getRotation() < (curveTargetRotation - 25.0f)) update();
		}
		setVelocity(-0.4f);
		while(getDistance(RearCenter) > 110) rotationCorrection(true);
		setVelocity(0.0f);
	}

	drivingSide = !outsideBorder;

	if(outsideBorder) {
		for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[0][0] == 0); i++) {
			if(getObjects()[i].c != PINK) {
				if(outsideBorder ? (getObjects()[i].x > 650) : (getObjects()[i].x < 650)) {
					if((getObjects()[i].y + getObjects()[i].h) > (section[0][2] == 3) ? 440 : 380) section[0][0] = getObjects()[i].c;
				}
			}
		}
		if((section[0][0] == 0) && ((section[0][2] == 0) || (section[0][2] == 3))) {
			section[0][0] = 3;
			for (int i = 0; (getObjects()[i].c != 0) && (section[0][1] == 0); i++) {
				if(getObjects()[i].c != PINK) {
					//if(outsideBorder ? (getObjects()[i].x > 650) : (getObjects()[i].x < 650)) {
						if((getObjects()[i].y + getObjects()[i].h) > 380) section[0][1] = getObjects()[i].c;
					//}
				}
			}
			if(section[0][1] == 0) {
				section[0][1] = 3;
				for (uint16_t i = 0; (getObjects()[i].c != 0) && (section[0][2] == 0); i++) {
					if(getObjects()[i].c != PINK) {
						if(outsideBorder ? (getObjects()[i].x > 650) : (getObjects()[i].x < 650)) {
							if((getObjects()[i].y + getObjects()[i].h) > 350) section[0][2] = getObjects()[i].c;
						}
					}
				}
				if(section[0][2] == 0) section[0][2] = 3;
			}
			else section[0][2] = 3;
		}
	}
			
	// drive arround the objects, the same way as in the last three sections, but this time without looking for a second object, because if there is a second object it would already be known
	if(section[0][0] != 3) {
		if((section[0][0] == (outsideBorder ? RED : GREEN))) {
			curve(outsideBorder);
			while(getDistance(FrontCenter) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
		}
		setVelocity(0.5f);
		uint64_t wait_time = millis();
		while(wait_time > (millis() - 1000000)) rotationCorrection();
		while((getDistance(FrontCenter) > 1900) || (getDistance(RearCenter) < 1100)) rotationCorrection();
		delayUpdate(50);
		
		if((section[0][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
			curve(!outsideBorder);
			while(getDistance(FrontCenter) > 370) rotationCorrection();
			curve(outsideBorder);
			drivingSide = !outsideBorder;
			delayUpdate(50);
		}
		if((section[0][2] == (outsideBorder ? RED : GREEN)) && (drivingSide != outsideBorder)) {
			curve(outsideBorder);
			while(getDistance(FrontCenter) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			delayUpdate(50);
		}
		setVelocity(0.5f);
		wait_time = millis();
		while(wait_time > (millis() - 1000000)) rotationCorrection();
		while((getDistance(outsideBorder ? RearLeft : RearRight) < 1100) || (getDistance(FrontCenter) > 1500)) rotationCorrection();
	}
	else {
		if((section[0][1] == (outsideBorder ? RED : GREEN)) || ((section[0][1] == 3) && (section[0][2] == (outsideBorder ? RED : GREEN)))) {
			curve(outsideBorder);
			while(getDistance(FrontCenter) > 520) rotationCorrection();
			curve(!outsideBorder);
			drivingSide = outsideBorder;
			delayUpdate(50);
		}
		setVelocity(0.5f);
		int64_t t = millis() + 2000000;
		while(millis() < t) rotationCorrection();
		while((getDistance(outsideBorder ? RearLeft : RearRight) < 1100) || (getDistance(FrontCenter) > 1500)) rotationCorrection();
	}

	// object avoidance code for the sections 5 to 11
	#pragma region sections 5 - 11

	for(uint8_t k = 1; k <= 7; k++) {		//loop to repeat the code for the sections 5 to 11
		if(((section[k % 4][0] == (outsideBorder ? RED : GREEN)) || ((section[k % 4][0] == 3) && (section[k % 4][1] == (outsideBorder ? RED : GREEN)))) || ((section[k % 4][0] == 3) && (section[k % 4][1] == 3) && (section[k % 4][2] == (outsideBorder ? RED : GREEN)))) { // check if the car has to pass the first object of the next straight section at the side of the outside border
			drivingSide = outsideBorder;
			while(getDistance(FrontCenter) > (((k % 4) == 0) ? 520 : 270)) rotationCorrection(); 	// drive until nearly reaching the outsideborder to pass the first object after the turn at the side of the outside border
			curve(!outsideBorder, 10.0f);															// turn into the direction of the new section
		}
		else {
			drivingSide = !outsideBorder;
			curve(!outsideBorder, 20.0f);			// turn into the direction of the new section without waiting to pass the first object at the side of the inner border
		}
		int64_t t = millis() + 1000000;
		while(millis() < t) rotationCorrection();														// wait a second while correcting the alignment
		while((getDistance(FrontCenter) > 1800) || (getDistance(RearCenter) < 1200)) rotationCorrection(); 		// drive completely into the straight section
		if((section[k % 4][0] != 3) && (section[k % 4][0] != section[k % 4][2])) {									// check whether there is a second object the car has to pass on the other side
			
			// drive to the outside border and realign if the car has to pass the second object at the side of the outside border
			if((section[k % 4][2] == (outsideBorder ? RED : GREEN)) && (drivingSide != outsideBorder)) {
				curve(outsideBorder);
				while(getDistance(FrontCenter) > (((k % 4) == 0) ? 520 : 320)) rotationCorrection();
				curve(!outsideBorder);
				drivingSide = outsideBorder;
			}

			// drive to the inner border and realign if the car has to pass the second object at the side of the inner border
			else if((section[k % 4][2] == (outsideBorder ? GREEN : RED)) && (drivingSide == outsideBorder)) {
				curve(!outsideBorder);
				while(getDistance(FrontCenter) > 320) rotationCorrection();
				curve(outsideBorder);
				drivingSide = !outsideBorder;
			}
		}

		// drive into the next curve section
		delayUpdate(100);
		while((getDistance(outsideBorder ? RearLeft : RearRight) < 1000) || (getDistance(FrontCenter) > 1500) || (getDistance(RearCenter) < 800)) rotationCorrection();
		delayUpdate(100);																																		// wait for new distance data
		while((getDistance(outsideBorder ? RearLeft : RearRight) < 1000) || (getDistance(FrontCenter) > 1500) || (getDistance(RearCenter) < 800)) rotationCorrection(); 	// check again if the car reached the next curve section
	}

	#pragma region section 12

	if(((section[0][0] == (outsideBorder ? RED : GREEN)) || ((section[0][0] == 3) && (section[0][1] == (outsideBorder ? RED : GREEN)))) || ((section[0][0] == 3) && (section[0][1] == 3) && (section[0][2] == (outsideBorder ? RED : GREEN)))) { // ckeck whether the car has to pass the first object of the section at the side of the outside border
		drivingSide = outsideBorder;
		while(getDistance(FrontCenter) > 520) rotationCorrection(); 	// keep driving until the car passed the object
	}
	else drivingSide = !outsideBorder;
	curve(!outsideBorder);				// turn into the last section
	delayUpdate(150);

	// drive slower while approaching parking space for non-parallel parking
	setVelocity(0.3f);
	

	// different approach for the two different driving directions
	if(outsideBorder) {

		while((getDistance(FrontCenter) > 1600) || (getDistance(RearCenter) < 1400)) rotationCorrection();		// drive to the middle of the last section
		if((outsideBorder != drivingSide)) {
			while((getDistance(FrontCenter) > 1400) || (getDistance(RearCenter) < 1450)) rotationCorrection();	// if the car is at the side of the inner border the car keeps driving until it passed the position in the middle of the section where a obstacle can be placed
		}
		lastDistance = getDistance(outsideBorder ? RearRight : RearLeft);
		while(((lastDistance - getDistance(outsideBorder ? RearRight : RearLeft)) < 100)&&((lastDistance - getDistance(outsideBorder ? RearRight : RearLeft)) > -100)) rotationCorrection(); // the car drives until the measured distance to the outside border distinguishes the previous measurement by more than 100mm
		drivingSide = 2;
		setVelocity(-0.3f);
		delayUpdate((outsideBorder == drivingSide) ? 825 : 1350);
		curve(outsideBorder);		// turn towards the parking space

		setVelocity(SPEED_PARK);
		while(getDistance(FrontCenter) > 20) rotationCorrection();	// the car drives into the parking space until it reached the outside border
		delayUpdate(100);
		setVelocity(0.0f);

		if((getDistance(RearLeft) < 5)) {
			setVelocity(-0.2f);
			while(getDistance(FrontCenter) < 150) rotationCorrection(true);
			setServo(15.0f);
			while(getRotation() > (curveTargetRotation - 45.0f)) update();
			setVelocity(0.15f);
			setServo(40.0f);
			while(getRotation() < curveTargetRotation) update();
			while(getDistance(FrontCenter) > 20) rotationCorrection();
			
		}
		if((getDistance(RearRight) < 5)) {
			setVelocity(-0.2f);
			while(getDistance(FrontCenter) < 150) rotationCorrection(true);
			setServo(-15.0f);
			while(getRotation() < (curveTargetRotation + 45.0f)) update();
			setVelocity(0.15f);
			setServo(-40.0f);
			while(getRotation() > curveTargetRotation) update();
			while(getDistance(FrontCenter) > 20) rotationCorrection();
			
		}
		if(getDistance(RearLeft)> getDistance(RearRight)) {
			curveTargetRotation += 90;
			for(int i = 5; (getRotation() - curveTargetRotation) < 5; i+= 10){
				setVelocity(-SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while (getRotation() < curveTargetRotation -(90-i)) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while (getRotation() < curveTargetRotation -(90-(i+5))) if(getDistance(FrontCenter) < 5) break;
			}
			while((getRotation() - curveTargetRotation)>2){
				setVelocity(-SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)>1) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)>1) if(getDistance(FrontCenter) < 5) break;
			}
			setVelocity(0.0f);
			setServo(0);
		}
		else {
			curveTargetRotation -= 90;
			for(int i = 5;(getRotation() - curveTargetRotation) > 5; i+= 20){
				setVelocity(-SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while (getRotation() > curveTargetRotation +(90-i)) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while (getRotation() > curveTargetRotation +(90-(i+10))) if(getDistance(FrontCenter) < 5) break;
			}
			while((getRotation() - curveTargetRotation)<2){
				setVelocity(-SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)<1) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)<1) if(getDistance(FrontCenter) < 5) break;
			}
			setVelocity(0.0f);
			setServo(0);
		}
	}
	else {
		uint16_t wait_time = millis();
		while(wait_time > (millis() - 500000)) rotationCorrection();
		while(getDistance(RearCenter) < 900) rotationCorrection();

		setVelocity(-0.7f);
		while(getDistance(RearCenter) > 800) rotationCorrection(true);
		delayUpdate(100);
		setVelocity(0.2f);

		lastDistance = getDistance(outsideBorder ? RearRight : RearLeft);
		while(((lastDistance - getDistance(outsideBorder ? RearRight : RearLeft)) < 100)||(getDistance(outsideBorder ? RearLeft : RearRight) > 1000)) rotationCorrection(); // the car drives until the measured distance to the outside border distinguishes the previous measurement by more than 100mm
		drivingSide = 2;
		setVelocity(-0.3f);
		while(getDistance(RearCenter) > 950) rotationCorrection(true);
		setVelocity(0.3f);
		delayUpdate(50);
		while(getDistance(RearCenter) < ((drivingSide == outsideBorder) ? 985 : 965)) rotationCorrection();			
		curve(outsideBorder);		// turn towards the parking space
		setVelocity(SPEED_PARK);
		while(getDistance(FrontCenter) > 15) rotationCorrection();	// the car drives into the parking space until it reached the outside border
		delayUpdate(100);
		setVelocity(0.0f);

		if((getDistance(RearLeft) < 5)) {
			setVelocity(-0.2f);
			while(getDistance(FrontCenter) < 150) rotationCorrection(true);
			setServo(15.0f);
			while(getRotation() > (curveTargetRotation - 45.0f)) update();
			setVelocity(0.15f);
			setServo(40.0f);
			while(getRotation() < curveTargetRotation) update();
			while(getDistance(FrontCenter) > 20) rotationCorrection();
			
		}
		if((getDistance(RearRight) < 5)) {
			setVelocity(-0.2f);
			while(getDistance(FrontCenter) < 150) rotationCorrection(true);
			setServo(-15.0f);
			while(getRotation() < (curveTargetRotation + 45.0f)) update();
			setVelocity(0.15f);
			setServo(-40.0f);
			while(getRotation() > curveTargetRotation) update();
			while(getDistance(FrontCenter) > 20) rotationCorrection();
			
		}
		if(getDistance(RearLeft)> getDistance(RearRight)) {
			curveTargetRotation += 90;
			for(int i = 5; (getRotation() - curveTargetRotation) < 5; i+= 10){
				setVelocity(-SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while (getRotation() < curveTargetRotation -(90-i)) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while (getRotation() < curveTargetRotation -(90-(i+5))) if(getDistance(FrontCenter) < 5) break;
			}
			while((getRotation() - curveTargetRotation)>2){
				setVelocity(-SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)>1) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)>1) if(getDistance(FrontCenter) < 5) break;
			}
			setVelocity(0.0f);
			setServo(0);
		}
		else {
			curveTargetRotation -= 90;
			for(int i = 5;(getRotation() - curveTargetRotation) > 5; i+= 20){
				setVelocity(-SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while (getRotation() > curveTargetRotation +(90-i)) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while (getRotation() > curveTargetRotation +(90-(i+10))) if(getDistance(FrontCenter) < 5) break;
			}
			while((getRotation() - curveTargetRotation)<2){
				setVelocity(-SPEED_PARK_IN);
				setServo(-PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)<1) if(getDistance(RearCenter) < 5) break;
				setVelocity(SPEED_PARK_IN);
				setServo(PARK_ANGLE);
				while ((getRotation() - curveTargetRotation)<1) if(getDistance(FrontCenter) < 5) break;
			}
			setVelocity(0.0f);
			setServo(0);
		}
	}

	// stop driving and wait until standstill
	setVelocity(0.0f);
	
	// show finish with total drive time
	showFinish();
}

// wait until switched off or restarted
void loop() { delayUpdate(1000000); }

#endif
#endif