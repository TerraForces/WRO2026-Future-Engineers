/**
 * main.cpp - driving strategy and logic
 * by TerraForces
 */

// optimize program for higher execution speed
#pragma GCC optimize("-Ofast")

// select program
#define TEST
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
        if(zeit + 800 < millis()) zeit = millis()-800;
    }
    else if((getDistance(FrontRight) < 150)&&(getDistance(FrontCenter) > 800)){
        setServo(gegensteuern_wand_degree);
        while(getRotation()+Korektur < (((richtung == 0)?90:-90)*anzahl_Kurven)+5.0f){
            update();
        }
        if(zeit + 800 < millis()) zeit = millis()-800;
    }
    setServo(((getRotation()+Korektur) - ((richtung)?(anzahl_Kurven * -90):(anzahl_Kurven*90)))*((-gegensteuern_multiplikator)));
    if(bereich_eng[anzahl_Kurven % 4] == 0) if(getDistance(RearCenter) > 1200 && (abs(abs(getRotation()) - (anzahl_Kurven*90)) < 5)){
        if((getDistance(FrontLeft)+getDistance(FrontRight)) < 750) bereich_eng[anzahl_Kurven % 4] = 1;
        else bereich_eng[anzahl_Kurven % 4] = 2;
    }
    if(boost &&((getDistance(RearCenter) > (((bereich_eng[(anzahl_Kurven+1)%4] == 1))?1800:2050))) && (zeit + 500 < millis())){
        setVelocity(Geschwindigkeit_normal,0,0.2f,-0.3f);
        boost = false;
    }
    if(anzahl_Kurven >= 12)if((getDistance(RearCenter)>1150) && (zeit + 800 < millis())){
        setServo(0);
        setVelocity(0.0f, 1, 0.3f, -4.0f);
        char output[42];
        sprintf(output, "%d %d %d %d", int(bereich_eng[0]), int(bereich_eng[1]), int(bereich_eng[2]), int(bereich_eng[3]));
        showFinish(output);
        while(true) update();
    }
    /*if(getDistance(FrontCenter)> 1300 && getDistance(RearCenter) > 1300 && zeitkorektur + 4000 < millis() && zeit + 1300 < millis() && (abs((getRotation()+Korektur) + ((richtung)?(anzahl_Kurven * 90):(anzahl_Kurven*-90))) < 3)){
        Korektur = (-(((atan((getDistance(richtung?RearLeft:FrontRight) - getDistance(richtung?FrontLeft:RearRight))/95)*(180/PI))+((richtung)?(anzahl_Kurven * -90):(anzahl_Kurven*90))))-getRotation());
        zeitkorektur = millis();
    }*/
}

#else

CAR::COLOR16 letzte_farbe = {};
bool on_red_line(){
    CAR::COLOR16 farbe = CAR::getColor();
    //if(farbe.c != letzte_farbe.c) digitalToggleFast(13);
    letzte_farbe.c = farbe.c;
    return (uint16_t(double(farbe.b)*1.5f*1.75f) < farbe.r);
}
bool on_blue_line(){
    CAR::COLOR16 farbe = CAR::getColor();
    return (uint16_t(double(farbe.b)*1.5f) > (farbe.r*1.65));
}

#define gegensteuern_multiplikator 0.5f
#define gegensteuern_wand_degree 30.0f
#define einlenken_kurve 37.5f

#define Geschwindigkeit_ausparken 0.13f
#define Geschwindigkeit_normal 0.4f
#define Geschwindigkeit_langsam 0.25f
#define Geschwindigkeit_schnell 0.6f

bool richtung = 0; //0 nach links abbiegen; 1 nach rechts abbiegen
bool parkplatz = 0; // parkplatz = 1: Parkplatz am Anfang des Feldes (in Fahrtrichtung)

uint8_t bekannte_Hindernisse[4] = {}; //0unbekannt 1 innen 2 außen

bool near_inner_Border = false;

uint8_t anzahl_Kurven = 0;

void setup() {
    CAR::init(true);
    //CAR::setLEDs(COLOR8{135,255,50});
    CAR::waitButtonPress();
    CAR::start();/*
    richtung = getDistance(FrontRight) > 200;
    setServo(richtung ? 50.0f : -50.0f);
    setVelocity(-Geschwindigkeit_ausparken);
    while((richtung ? (getRotation() > -30.0f) : (getRotation() < 30.0f))) update();
    setVelocity(0.0f, true);
    setServo(0.0f);
    uint32_t zeit = millis();
    while(millis() < zeit + 300) update();
    setServo(richtung ? -50.0f : 50.0f);
    setVelocity(Geschwindigkeit_ausparken);
    while(richtung ? (getRotation() > -85.0f) : (getRotation() < 85.0f)) update();
    setVelocity(0.0f, true);
    zeit = millis();
    while(millis() < zeit + 300) update();*/


    richtung = getDistance(FrontRight) > 200;
    setServo(richtung ? -50.0f : 50.0f);
    setVelocity(Geschwindigkeit_ausparken);
    while(richtung ? (getRotation() > -25.0f) : (getRotation() < 23.0f)) update();
    setVelocity(0.0f, true);
    uint32_t zeit = millis();
    while(millis() < zeit + 250) update();
    setServo(richtung ? 50.0f : -50.0f);
    setVelocity(-Geschwindigkeit_ausparken);
    while((richtung ? (getRotation() > -55.0f) : (getRotation() < 55.0f))) update();
    setVelocity(0.0f, true);
    setServo(0.0f);
    zeit = millis();
    while(millis() < zeit + 250) update();
    setServo(richtung ? -50.0f : 50.0f);
    setVelocity(Geschwindigkeit_ausparken);
    while(richtung ? (getRotation() > -85.0f) : (getRotation() < 85.0f)) update();
    setVelocity(0.0f, true);
    zeit = millis();
    while(millis() < zeit + 250) update();


    setVelocity(Geschwindigkeit_ausparken);
    while((getDistance(FrontLeft) < 200) || (getDistance(FrontRight) < 200) || (getDistance(RearCenter) < 115)){
        setServo((getRotation() - (richtung?-90.0f:90.0f))*((-gegensteuern_multiplikator)));
        update();
    }
    parkplatz = (getDistance(richtung?FrontLeft:FrontRight) > getDistance(richtung?FrontRight:FrontLeft));
    /*
    // break point //
    setVelocity(0.0f, 1);
    showSensorData(true);
    while(true) delay(1);*/

    if(parkplatz){
        setServo(richtung ? 45.0f : -45.0f);
        setVelocity(Geschwindigkeit_ausparken);
        while(richtung ? (getRotation() < -32.0f) : (getRotation() > 32.0f)) update();
        setVelocity(0.0f, true);
        zeit = millis();
        while(millis() < zeit + 300) update();
        std::vector<OBJECT> Hindernisse = getObjects();
        bool innen = false;
        for(OBJECT hindernis:Hindernisse){
            if((hindernis.c == (richtung?RED:GREEN)) && (hindernis.h > 120)){
                innen = true;
                bekannte_Hindernisse[0] = 1;
            }
            else if((hindernis.c == (!richtung?RED:GREEN)) && (hindernis.h > 120)){
                bekannte_Hindernisse[0] = 2;
            }
        }
        if(!innen){
            setServo(richtung ? 45.0f : -45.0f);
            setVelocity(Geschwindigkeit_ausparken);
            while(richtung ? (getRotation() < 15.0f) : (getRotation() > -15.0f)) update();
            setVelocity(Geschwindigkeit_normal);
            zeit = millis();
            while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < zeit + 2000)){
                setServo((getRotation())*((-gegensteuern_multiplikator)));
                update();
            }
            near_inner_Border = false;
        }
        else{
            setServo(richtung ? -45.0f : 45.0f);
            setVelocity(Geschwindigkeit_langsam);
            while(richtung ? (getRotation() > -80.0f) : (getRotation() < 80.0f)) update();
            zeit = millis();
            while((getDistance(FrontCenter) > 300) || (millis() < zeit + 500)){
                setServo(((getRotation() - (richtung?-90.0f:90.0f)))*((-gegensteuern_multiplikator)));
                update();
            }
            setServo(richtung ? 45.0f : -45.0f);
            while(richtung ? (getRotation() < -10.0f) : (getRotation() > 10.0f)) update();
            setVelocity(Geschwindigkeit_normal);
            zeit = millis();
            while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < zeit + 2000)){
                setServo((getRotation())*((-gegensteuern_multiplikator)));
                update();
            }
            near_inner_Border = true;
        }
    }
    else{
        setVelocity(0.0f, true);
        zeit = millis();
        while(millis() < zeit + 300) update();
        std::vector<OBJECT> Hindernisse = getObjects();
        bool innen = false;
        for(OBJECT hindernis:Hindernisse){
            if((hindernis.c == (richtung?RED:GREEN)) && (hindernis.h > 200)) {
                innen = true;
                bekannte_Hindernisse[0] = 1;
            }
            else if((hindernis.c == (!richtung?RED:GREEN)) && (hindernis.h > 200)){
                bekannte_Hindernisse[0] = 2;
            }
        }
        if(!innen){
            setVelocity(Geschwindigkeit_ausparken);
            setServo(richtung ? 45.0f : -45.0f);
            setVelocity(Geschwindigkeit_ausparken);
            while(richtung ? (getRotation() <-2.0f) : (getRotation() > 2.0f)) update();
            setServo(0);
            near_inner_Border = false;
        }
        else{
            setVelocity(Geschwindigkeit_ausparken);
            zeit=millis();
            while((getDistance(RearCenter) < 250) || (millis() < zeit + 500)){
                setServo(((getRotation() - (richtung?-90.0f:90.0f)))*((-gegensteuern_multiplikator)));
                update();
            }
            setServo(richtung ? -40.0f : 40.0f);
            setVelocity(Geschwindigkeit_langsam);
            while(richtung ? (getRotation() > -135.0f) : (getRotation() < 135.0f)) update();
            setServo(richtung ? 40.0f : -40.0f);
            while(richtung ? (getRotation() < -90.0f) : (getRotation() > 90.0f)) update();
            zeit=millis();
            while((getDistance(FrontCenter) > 300) || (millis() < zeit + 500)){
                setServo(((getRotation() - (richtung?-90.0f:90.0f)))*((-gegensteuern_multiplikator)));
                update();
            }
            setServo(richtung ? 45.0f : -45.0f);
            while(richtung ? (getRotation() < -10.0f) : (getRotation() > 10.0f)) update();
            setVelocity(Geschwindigkeit_langsam);
            zeit = millis();
            while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < zeit + 1000)){
                setServo((getRotation())*((-gegensteuern_multiplikator)));
                update();
            }
            near_inner_Border = true;
        }
    }
    setVelocity(0.0f,true);
    while(anzahl_Kurven < 11){
        if(bekannte_Hindernisse[(anzahl_Kurven+1)%4] == 0){
            if(!near_inner_Border){
                setVelocity(Geschwindigkeit_langsam);
                while((getDistance(FrontCenter) > 925)&&(getDistance(richtung?(RearRight):(RearLeft)) < 1000)){
                    setServo((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f)))*((-gegensteuern_multiplikator)));
                    update();
                }
                if((getDistance(richtung?(RearRight):(RearLeft)) < 1000)){
                    setVelocity(0.0f,true);
                    zeit=millis();
                    while(zeit+100 > millis())update();
                    setVelocity(-Geschwindigkeit_langsam);
                    zeit=millis();
                    while(zeit + 600 > millis()) setServo((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f)))*((gegensteuern_multiplikator)));
                }
                /*if(anzahl_Kurven == 3){
                    setVelocity(0.0f,true);
                    while(true) update();
                }*/
                anzahl_Kurven++;
                setVelocity(Geschwindigkeit_normal);
                setServo(richtung ? -45.0f : 45.0f);
                while(richtung ? (getRotation() > (anzahl_Kurven*(-90))+10.0f) : (getRotation() < (anzahl_Kurven*(90))-10.0f)) update();
                zeit=millis();
                while((getDistance(RearCenter) < 600) || (millis() < zeit + 200)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
                zeit=millis();
                setVelocity(0.0f,true);
                while(zeit + 250 > millis()) update();
                setVelocity(-Geschwindigkeit_langsam);
                setServo(richtung ? -45.0f : 45.0f);
                while(richtung ? (getRotation() < (anzahl_Kurven*(-90))+25.0f) : (getRotation() > (anzahl_Kurven*(90))-25.0f)) update();
                zeit=millis();
                setVelocity(0.0f,true);
                while(zeit + 250 > millis()) update();
                
            }
            else{
                while((getDistance(FrontCenter) > 925)&&(getDistance(richtung?(RearRight):(RearLeft)) < 1000)){
                    setServo((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f)))*((-gegensteuern_multiplikator)));
                    update();
                }
                if((getDistance(richtung?(RearRight):(RearLeft)) < 1000)){
                    setVelocity(0.0f,true);
                    zeit=millis();
                    while(zeit+100 > millis())update();
                    setVelocity(-Geschwindigkeit_langsam);
                    zeit=millis();
                    while(zeit + 600 > millis()) setServo((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f)))*((gegensteuern_multiplikator)));
                }

                setVelocity(Geschwindigkeit_ausparken);
                setServo(richtung ? -45.0f : 45.0f);
                while(richtung ? (getRotation() > (anzahl_Kurven*(-90))-60.0f) : (getRotation() < (anzahl_Kurven*(90))+60.0f)) update();
                zeit = millis();
                setVelocity(0.0f,true);
                while(zeit+150 > millis())update();

                setVelocity(-Geschwindigkeit_ausparken);
                while(richtung?(!on_red_line()):(!on_blue_line())){
                    setServo(((getRotation() - ((anzahl_Kurven*(richtung?-90.0f:90.0f)) - (richtung?60:-60))))*((gegensteuern_multiplikator*2)));
                    update();
                }
                setVelocity(0.0f,true);
                anzahl_Kurven++;
                
                /*
                while(true){
                    update();
                    //showSensorData();
                    showObjects();
                }*/
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            std::vector<OBJECT> Hindernisse = getObjects();
            for(OBJECT hindernis:Hindernisse){
                if((hindernis.c == (richtung?RED:GREEN))){
                    bekannte_Hindernisse[anzahl_Kurven%4] = 1;
                }
                else if((hindernis.c == (!richtung?RED:GREEN))){
                    bekannte_Hindernisse[anzahl_Kurven%4] = 2;
                }
            }
            if(bekannte_Hindernisse[anzahl_Kurven%4] == 0){
                zeit=millis();
                while(zeit+200 > millis())update();
                std::vector<OBJECT> Hindernisse = getObjects();
                for(OBJECT hindernis:Hindernisse){
                    if((hindernis.c == (richtung?RED:GREEN))){
                        bekannte_Hindernisse[anzahl_Kurven%4] = 1;
                    }
                    else if((hindernis.c == (!richtung?RED:GREEN))){
                        bekannte_Hindernisse[anzahl_Kurven%4] = 2;
                    }
                }
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if(bekannte_Hindernisse[anzahl_Kurven%4] == 1){
                near_inner_Border = true;
                setVelocity(Geschwindigkeit_normal);
                setServo(richtung ? -45.0f : 45.0f);
                while(richtung ? (getRotation() > (anzahl_Kurven*(-90))+10.0f) : (getRotation() < (anzahl_Kurven*(90))-10.0f)) update();
                setVelocity(Geschwindigkeit_schnell);
                zeit=millis();
                while((getDistance(FrontCenter) > 1400) || (millis() < zeit + 200)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
                setVelocity(0.0f,true);
                zeit=millis();
                while(zeit + 15 > millis())update();
                setVelocity(Geschwindigkeit_langsam);
                zeit=millis();
                while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) || (getDistance(FrontCenter) > 940)) || (millis() < zeit + 100)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
            }
            else{
                near_inner_Border = false;
                setVelocity(Geschwindigkeit_langsam);
                setServo(richtung ? 45.0f : -45.0f);
                while(richtung ? (getRotation() < (anzahl_Kurven*(-90))+85.0f) : (getRotation() > (anzahl_Kurven*(90))-85.0f)) update();
                zeit=millis();
                while((getDistance(FrontCenter) > (((anzahl_Kurven%4) == 0)?(550):(200))) || (millis() < zeit + 200)){
                    setServo(((getRotation() - ((anzahl_Kurven-1)*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
                setVelocity(Geschwindigkeit_normal);
                setServo(richtung ? -45.0f : 45.0f);
                while(richtung ? (getRotation() > (anzahl_Kurven*(-90))+10.0f) : (getRotation() < (anzahl_Kurven*(90))-10.0f)) update();
                setVelocity(Geschwindigkeit_schnell);
                zeit=millis();
                while((getDistance(FrontCenter) > 1500) || (millis() < zeit + 200)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
                setVelocity(0.0f,true);
                zeit=millis();
                while(zeit + 15 > millis())update();
                setVelocity(Geschwindigkeit_langsam);
                zeit=millis();
                while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) || (getDistance(FrontCenter) > 940)) || (millis() < zeit + 100)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
            }
        }
        else{
            if(bekannte_Hindernisse[(anzahl_Kurven + 1)%4]==2){
                zeit=millis();
                while((getDistance(FrontCenter) > (((anzahl_Kurven+1)%4 == 0)?(550):(300))) || (millis() < zeit + 100)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
            }
            else{
                zeit=millis();
                while(/*(getDistance(FrontCenter) > (935)) || */(millis() < zeit + 25)){
                    setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                    update();
                }
            }
            setVelocity(Geschwindigkeit_normal);
            
            anzahl_Kurven++;
            setServo(richtung ? -45.0f : 45.0f);
            while(richtung ? (getRotation() > (anzahl_Kurven*(-90))+10.0f) : (getRotation() < (anzahl_Kurven*(90))-10.0f)) update();
            setVelocity(Geschwindigkeit_schnell);
            zeit=millis();
            while((getDistance(FrontCenter) > 1400) || (millis() < zeit + 1500)){
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                update();
            }
            setVelocity(Geschwindigkeit_langsam,0,0.25,-4.0f);
            zeit=millis();
            while(((getDistance(richtung?FrontRight:FrontLeft) < 1000) && (getDistance(FrontCenter) > 950)) || (millis() < zeit + 100)){
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                update();
            }
            
        }
    }

    if(bekannte_Hindernisse[(anzahl_Kurven + 1)%4]==2){
        zeit=millis();
        while((getDistance(FrontCenter) > (((anzahl_Kurven+1)%4 == 0)?(500):(300))) || (millis() < zeit + 100)){
            setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
            update();
        }
    }
    else{
        zeit=millis();
        while((getDistance(FrontCenter) > (910)) || (millis() < zeit + 25)){
            setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
            update();
        }
    }
    setVelocity(Geschwindigkeit_normal);
            
    anzahl_Kurven++;
    setServo(richtung ? -45.0f : 45.0f);
    while(richtung ? (getRotation() > (anzahl_Kurven*(-90))+10.0f) : (getRotation() < (anzahl_Kurven*(90))-10.0f)) update();
    setVelocity(Geschwindigkeit_schnell);
    zeit=millis();
    while(((getDistance(FrontCenter) > 1800)||(getDistance(RearCenter) < 1200)) || (millis() < zeit + 200)){
        setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
        update();
    }/*
    setVelocity(Geschwindigkeit_langsam,0,0.25,-4.0f);
    zeit=millis();
    while((getDistance(richtung?FrontRight:FrontLeft) < 1000) || (millis() < zeit + 100)){
        setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
        update();
    }*/
    

    uint32_t letzte_messung = 0;
    uint32_t vorletzte_messung = 0;

    zeit=millis();
    setVelocity(0.0f,true);
    update();
    showFinish();
    while(zeit + 3500 > millis())update();

    setVelocity(Geschwindigkeit_langsam);
    while((getDistance(FrontCenter) > 1500)||(getDistance(RearCenter) < 1250)){
        setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
        update();
    }

    if(bekannte_Hindernisse[0] == 2){
        if(parkplatz){
            setVelocity(-Geschwindigkeit_ausparken);
            letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            vorletzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            while((getDistance(richtung?(FrontLeft):(FrontRight)) < letzte_messung + 20)&&(getDistance(richtung?(FrontLeft):(FrontRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(FrontLeft):(FrontRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
                }
            }
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(-Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven + 1)*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
            }
        }
        else{
            setVelocity(Geschwindigkeit_ausparken);
            letzte_messung = getDistance(richtung?(RearLeft):(RearRight));
            vorletzte_messung = getDistance(richtung?(RearLeft):(RearRight));
            while((getDistance(richtung?(RearLeft):(RearRight)) < letzte_messung + 20)&&(getDistance(richtung?(RearLeft):(RearRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(RearLeft):(RearRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(RearLeft):(RearRight));
                }
            }
            letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            vorletzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            while((getDistance(richtung?(FrontLeft):(FrontRight)) < letzte_messung + 20)&&(getDistance(richtung?(FrontLeft):(FrontRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(FrontLeft):(FrontRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
                }
            }
            setVelocity(0.0f,true);
            zeit=millis();
            while(zeit + 250 > millis())update();

            /*
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) < 85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) > -85.0f))) update();
            setServo(0.0f);
            while((getDistance(FrontCenter) > 30)||(getDistance(RearRight) > 200)||(getDistance(RearLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven - 1)*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
            }*/
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(-Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven + 1)*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
            }
        }
    }
    if(bekannte_Hindernisse[0] == 1){
        setServo(richtung ? 50.0f : -50.0f);
        setVelocity(Geschwindigkeit_ausparken);
        while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) < 85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) > -85.0f))) update();
        zeit=millis();
        setServo(0);
        while((zeit + 250 > millis())||(getDistance(RearCenter) < 400)){
            update();
            setServo(((getRotation() - ((anzahl_Kurven-1)*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
        }
        setServo(richtung ? -50.0f : 50.0f);
        setVelocity(Geschwindigkeit_ausparken);
        while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) > 5.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) < 5.0f))) update();
        zeit=millis();
        setVelocity(0.0f,true);
        while(zeit+250 > millis())update();
        setServo(0);
        setVelocity(-Geschwindigkeit_langsam);
        zeit=millis();
        setServo(0);
        while((zeit + 250 > millis())||(getDistance(FrontCenter) < ((parkplatz)?1400:1700))){
            update();
            setServo(((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
        }

        if(parkplatz){
            setVelocity(-Geschwindigkeit_ausparken);
            letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            vorletzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            while((getDistance(richtung?(FrontLeft):(FrontRight)) < letzte_messung + 20)&&(getDistance(richtung?(FrontLeft):(FrontRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(FrontLeft):(FrontRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
                }
            }
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(-Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven + 1)*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
            }
        }
        else{
            setVelocity(Geschwindigkeit_ausparken);
            letzte_messung = getDistance(richtung?(RearLeft):(RearRight));
            vorletzte_messung = getDistance(richtung?(RearLeft):(RearRight));
            while((getDistance(richtung?(RearLeft):(RearRight)) < letzte_messung + 20)&&(getDistance(richtung?(RearLeft):(RearRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(RearLeft):(RearRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(RearLeft):(RearRight));
                }
            }
            letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            vorletzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
            while((getDistance(richtung?(FrontLeft):(FrontRight)) < letzte_messung + 20)&&(getDistance(richtung?(FrontLeft):(FrontRight)) < vorletzte_messung + 20)){
                update();
                setServo(((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
                if(getDistance(richtung?(FrontLeft):(FrontRight)) != letzte_messung){
                    vorletzte_messung = letzte_messung;
                    letzte_messung = getDistance(richtung?(FrontLeft):(FrontRight));
                }
            }
            setVelocity(0.0f,true);
            zeit=millis();
            while(zeit + 250 > millis())update();

            /*
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) < 85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) > -85.0f))) update();
            setServo(0.0f);
            while((getDistance(FrontCenter) > 30)||(getDistance(RearRight) > 200)||(getDistance(RearLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven - 1)*(richtung?-90.0f:90.0f))))*((-gegensteuern_multiplikator*2)));
            }*/
            setServo(richtung ? 50.0f : -50.0f);
            setVelocity(-Geschwindigkeit_ausparken);
            while((richtung ? ((getRotation() - ((anzahl_Kurven)*(richtung?-90.0f:90.0f))) > -85.0f) : ((getRotation() - (anzahl_Kurven*(richtung?-90.0f:90.0f))) < 85.0f))) update();
            setServo(0.0f);
            while((getDistance(RearCenter) > 15)||(getDistance(FrontRight) > 200)||(getDistance(FrontLeft) > 200)){
                update();
                setServo(((getRotation() - ((anzahl_Kurven + 1)*(richtung?-90.0f:90.0f))))*((gegensteuern_multiplikator*2)));
            }
        }
    }


    setVelocity(0.0f,true);
    showFinish();
    while(true){
        update();
        //showSensorData();
    }

}

void loop() {

}

#endif

#endif