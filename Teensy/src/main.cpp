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
    CAR::update(false);
    CAR::showObjects();
    digitalToggleFast(13);
    delay(1);
}

#endif
#else

using namespace CAR;

#ifndef OBJECTS

#define gegensteuern_multiplikator 0.15f
#define gegensteuern_wand_degree 10.0f
#define einlenken_kurve 35.0f
#define kurvenEnde 45.0f

#define Geschwindigkeit_normal 0.85f
#define Geschwindigkeit_Boost 1.0f

bool richtung; //0nach links abbiegen, 1 nach rechts abbiegen
uint8_t anzahl_Kurven = 0;
bool Kurve = 0;

uint8_t bereich_eng[4] = {0}; // 0 startbereich; 0 = unbekannt 1 = eng 2 = breit
uint64_t zeit = 0;
uint64_t zeitkorektur = 0;

bool boost = false;

void setup() {
    CAR::init();
    CAR::waitButtonPress();
    CAR::start();
    setVelocity(Geschwindigkeit_normal,0,0.4f,-0.4f);
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
            while(getRotation() > -7.0f){
                update();
            }
        }
        else if(getDistance(FrontRight) < 150){
            setServo(gegensteuern_wand_degree);
            while(getRotation() < 7.0f){
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
    if(Kurve != 1 && zeit + 1300 < millis())if((((richtung == 0)&&(getDistance(FrontLeft) > 800))||((richtung == 1)&&(getDistance(FrontRight) > 800))||(getDistance(FrontCenter) < ((bereich_eng[anzahl_Kurven % 4] == 2)?450:350)))&&(getDistance(RearCenter) > 1700)&&((getRotation()+Korektur) + ((richtung)?(anzahl_Kurven * 90):(anzahl_Kurven*-90)) < 7)) Kurve = 1;
    if(Kurve == 1){
        setVelocity(Geschwindigkeit_normal, 0, 0.2f,-0.4f);
        anzahl_Kurven++;
        if(richtung!= 0) setServo(-einlenken_kurve);
        else setServo(einlenken_kurve);
        if(richtung == 0) while((getRotation() + Korektur) < ((90*anzahl_Kurven)-kurvenEnde)) update();
        else while((getRotation() + Korektur) > ((-90*anzahl_Kurven)+kurvenEnde)) update();
        Kurve = 0;
        setServo(0);
        zeit = millis();
        setVelocity(Geschwindigkeit_Boost,0,0.5f,-0.3f);
        boost = true;
    }
    else if((getDistance(FrontLeft) < 150)&&(getDistance(FrontCenter) > 800)){
        setServo(-gegensteuern_wand_degree);
        while(getRotation()+Korektur > (((richtung == 0)?90:-90)*anzahl_Kurven)-5.0f){
            update();
        }
    }
    else if((getDistance(FrontRight) < 150)&&(getDistance(FrontCenter) > 800)){
        setServo(gegensteuern_wand_degree);
        while(getRotation()+Korektur < (((richtung == 0)?90:-90)*anzahl_Kurven)+5.0f){
            update();
        }
    }
    setServo(((getRotation()+Korektur) - ((richtung)?(anzahl_Kurven * -90):(anzahl_Kurven*90)))*((-gegensteuern_multiplikator)));
    if(bereich_eng[anzahl_Kurven % 4] == 0) if(getDistance(RearCenter) > 1200 && (abs(abs(getRotation()) - (anzahl_Kurven*90)) < 5)){
        if((getDistance(FrontLeft)+getDistance(FrontRight)) < 750) bereich_eng[anzahl_Kurven % 4] = 1;
        else bereich_eng[anzahl_Kurven % 4] = 2;
    }
    if(boost &&((getDistance(RearCenter) > (((bereich_eng[(anzahl_Kurven+1)%4] == 1))?1750:1900))) && (zeit + 500 < millis())){
        setVelocity(Geschwindigkeit_normal,0,0.2f,-0.3f);
        boost = false;
    }
    if(anzahl_Kurven >= 12)if((getDistance(RearCenter)>1150) && (zeit + 400 < millis())){
        setServo(0);
        setVelocity(0.0f, 1, 0.3f, -4.0f);
        char output[42];
        sprintf(output, "%d %d %d %d", int(bereich_eng[0]), int(bereich_eng[1]), int(bereich_eng[2]), int(bereich_eng[3]));
        showFinish(output);
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

bool richtung = 0; //0 nach links abbiegen; 1 nach rechts abbiegen

void setup() {
    CAR::init(true);
    CAR::waitButtonPress();
    CAR::start();
    showSensorData(true);
    richtung = getDistance(FrontRight) > 200;
    setServo(richtung ? 45.0f : -45.0f);
    setVelocity(-0.5f);
    while(richtung ? (getRotation() > -35.0f) : (getRotation() < 35.0f)) update();
    setVelocity(0.0f, true);
    setServo(0.0f);
    uint64_t zeit = millis();
    while(millis() < zeit + 400) update();
    setServo(richtung ? -45.0f : 45.0f);
    setVelocity(0.5f);
    while(richtung ? (getRotation() > -80.0f) : (getRotation() < 80.0f)) update();
    setVelocity(0.0f, true);
    zeit = millis();
    while(millis() < zeit + 400) update();
    setVelocity(0.5f);
    while((getDistance(FrontLeft) < 200) || (getDistance(FrontRight) < 200)){
        setServo((getRotation() - (richtung?-90.0f:90.0f))*((-gegensteuern_multiplikator)));
        update();
    }
    setServo(richtung ? 45.0f : -45.0f);
    setVelocity(0.5f);
    while(richtung ? (getRotation() < -40.0f) : (getRotation() > 40.0f)) update();
    setVelocity(0.0f, true);
    zeit = millis();
    while(millis() < zeit + 400) update();
    while(true) update();
}

void loop() {

}

#endif

#endif