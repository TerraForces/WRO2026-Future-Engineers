/**
 * main.cpp - driving strategy and logic
 * by TerraForces
 */

// select program
//#define TEST
//#define OBJECTS

#include "car.hpp"

#ifdef TEST
#ifndef OBJECTS

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    //CAR::runCalibration();
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
    CAR::runObjectDemo();
    CAR::update();
    digitalToggleFast(13);
}

#endif
#else

using namespace CAR;

#ifndef OBJECTS

#define gegensteuern_multiplikator 0.225f
#define gegensteuern_wand_degree 30.0f
#define einlenken_kurve 35.0f

bool richtung; //0nach links abbiegen, 1 nach rechts abbiegen
uint8_t anzahl_Kurven = 0;
bool Kurve = 0;

uint8_t bereich_eng[4] = {0}; // 0 startbereich; 0 = unbekannt 1 = eng 2 = breit
uint64_t zeit = 0;
uint64_t zeitkorektur = 0;

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::start();
    setVelocity(95.0f,0,0.3f,-0.3f);
    update();
    showSensorData(true);
    bereich_eng[0] = ((getDistance(FrontLeft) + getDistance(FrontRight)) > 750) + 1;
    
    while(true){
        update();
        if((abs(getRotation()) < 5)||(getDistance(FrontCenter) < 300)){
            if(getDistance(FrontLeft) > 1000){
                Kurve = 1;
                richtung = 0;
                break;
            }
            else if(getDistance(FrontRight) > 1000){
                Kurve = 1;
                richtung = 1;
                break;
            }/*
            else if(getDistance(AngledLeft) > 1290){
                richtung = 0;
                break;
            }
            else if(getDistance(AngledRight) > 1290){
                richtung = 1;
                break;
            }*/
        }
        if(getDistance(FrontLeft) < 150){
            setServo(-gegensteuern_wand_degree);
            while(getRotation() > -10.0f){
                update();
            }
        }
        else if(getDistance(FrontRight) < 150){
            setServo(gegensteuern_wand_degree);
            while(getRotation() > 10.0f){
                update();
            }
        }
        setServo((getRotation())*((-gegensteuern_multiplikator)));
        
        //delay(100);
    }
}

float Korektur = 0;

void loop() {
    update();
    if(Kurve != 1 && zeit + 1300 < millis())if((((richtung == 0)&&(getDistance(FrontLeft) > 800))||((richtung == 1)&&(getDistance(FrontRight) > 800))||(getDistance(FrontCenter) < ((bereich_eng[anzahl_Kurven % 4] == 2)?350:300)))&&(getDistance(RearCenter) > 1700)&&((getRotation()+Korektur) + ((richtung)?(anzahl_Kurven * 90):(anzahl_Kurven*-90)) < 3)) Kurve = 1;
    if(Kurve == 1){
        
        anzahl_Kurven++;
        if(richtung!= 0) setServo(-einlenken_kurve);
        else setServo(einlenken_kurve);
        if(richtung == 0) while((getRotation() + Korektur) < ((90*anzahl_Kurven)-43)) update();
        else while((getRotation() + Korektur) > ((-90*anzahl_Kurven)+43)) update();
        Kurve = 0;
        setServo(0);
        zeit = millis();
    }
    else if(getDistance(FrontLeft) < 150){
        setServo(-gegensteuern_wand_degree);
        while(getRotation()+Korektur > (((richtung == 0)?90:-90)*anzahl_Kurven)-10.0f){
            update();
        }
    }
    else if(getDistance(FrontRight) < 150){
        setServo(gegensteuern_wand_degree);
        while(getRotation()+Korektur > (((richtung == 0)?90:-90)*anzahl_Kurven)+10.0f){
            update();
        }
    }
    setServo(((getRotation()+Korektur) - ((richtung)?(anzahl_Kurven * -90):(anzahl_Kurven*90)))*((-gegensteuern_multiplikator)));
    if(bereich_eng[anzahl_Kurven % 4] == 0) if(getDistance(RearCenter) > 1200 && (abs(abs(getRotation()) - (anzahl_Kurven*90)) < 5)){
        if((getDistance(FrontLeft)+getDistance(FrontRight)) < 750) bereich_eng[anzahl_Kurven % 4] = 1;
        else bereich_eng[anzahl_Kurven % 4] = 2;
    }
    if(anzahl_Kurven >= 12)if((getDistance(RearCenter)>1100) && (zeit + 350 < millis())){
        setServo(0);
        setVelocity(0.0f, 1, 0.3f, -4.0f);
        showFinish();
        while(true) update(false);
    }
    /*if(getDistance(FrontCenter)> 1300 && getDistance(RearCenter) > 1300 && zeitkorektur + 4000 < millis() && zeit + 1300 < millis() && (abs((getRotation()+Korektur) + ((richtung)?(anzahl_Kurven * 90):(anzahl_Kurven*-90))) < 3)){
        Korektur = (-(((atan((getDistance(richtung?RearLeft:FrontRight) - getDistance(richtung?FrontLeft:RearRight))/95)*(180/PI))+((richtung)?(anzahl_Kurven * -90):(anzahl_Kurven*90))))-getRotation());
        zeitkorektur = millis();
    }*/
}

#else

#define gegensteuern_multiplikator 0.35f
#define gegensteuern_wand_degree 30.0f
#define einlenken_kurve 37.5f

bool richtung; //0 nach links abbiegen; 1 nach rechts abbiegen

void setup() {
    CAR::init(true);
    CAR::waitButtonPress();
    CAR::start();
    update();
    richtung = (getDistance(FrontRight)> 200);
    setServo((richtung)?60.0f:-60.0f);
    setVelocity(0.5f,0,0.3f,0.4f);
    while(getRotation()< (richtung)?-82:82)update();
    setServo(0.0f);
    while(getDistance(richtung?FrontLeft:FrontRight)<170){
        update();
        setServo((getRotation()-((richtung)?-85:85))*((gegensteuern_multiplikator)));
    }
}

#endif

#endif