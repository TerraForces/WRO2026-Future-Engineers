/**
 * car.cpp - the functional core of our autonomously driving car
 * by TerraForces
 */

// include header file
#include "../Teensy/src/car.hpp"

// misc configuration
constexpr uint32_t Config_Drive_PWM_Update_Int   = 10000; // interval at which the drive PWM duty cycle is being updated (in us)
constexpr float    Config_Servo_Max_Angle        = 50.0f; // max angle (to both directions from the center position) that is allowed for the servo to travel to

// configuration values stored on the sd card
int8_t vl53l1xViewportShift[8] = {};
float rotationTimeOffset = 0.0f;
float rotationFactor = 0.5f;
uint16_t servoCenterDuty = 33033;

// bus, display and sensor objects
ICM20948 imu(i2c2);
OneWire oneWire(Pin_DS18B20_OneWire);
DallasTemperature ds18b20(&oneWire);
NeoPixel leds(Pin_NeoPixel);
ST7735 lcd(&SPI1, Pin_LCD_CS, Pin_LCD_DC, Pin_LCD_RST, true);
TCS3472 tcs3472(i2c2);
VL53L1X vl53l1x[8] = { &i2c1, &i2c1, &i2c1, &i2c0, &i2c0, &i2c0, &i2c0, &i2c1 };

struct LOGDATA {
    uint32_t timestamp;
};

// global memory
std::vector<LOGDATA> logData;
int32_t currentVelocity = 0;
int32_t targetVelocity = 0;
int32_t acceleration = 1;
int32_t deceleration = -1;
uint32_t nextDriveUpdateTime = 0;
CAR::COLOR16 lastColor = {};
uint16_t lastDistance[8] = {};
std::vector<CAR::OBJECT> lastObjects, currentObjects;
float lastRotation = 0.0f;
float rotationOffset = 0.0f;
bool sensorActive[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
bool sensorWarning[12] = {};
bool sensorError[12] = {};
bool rpiActive = false;
uint32_t startTime = 0;
uint8_t uartReadBuffer[50000] = {};
uint32_t uartTest = 0;

float CAR::getBatteryVoltage() {
    return analogRead(Pin_ADC_Battery) * 0.002519843f;
}

CAR::COLOR16 CAR::getColor() {
    if(sensorActive[TCS3472]) {
        if(!tcs3472.readColor(&lastColor)) sensorWarning[TCS3472] = true;
    }
    return lastColor;
}

float CAR::getConverterCurrent() {
    return (3103.0303f - analogRead(Pin_ADC_Current_Conv)) * 0.004354941f;
}

float CAR::getConverterTemp() {
    if(!ds18b20.requestTemperaturesByAddress(Config_DS18B20_Conv_Addr)) sensorError[DS18B20Conv] = true;
    float temp =  ds18b20.getTempC(Config_DS18B20_Conv_Addr);
    if(temp <= DEVICE_DISCONNECTED_C) sensorWarning[DS18B20Conv] = true;
    return temp;
}

float CAR::getCoreTemp() {
    return tempmonGetTemp();
}

uint16_t CAR::getDistance(SENSORS sensor) {
    if(sensor >= 8) return 0;
    return lastDistance[sensor];
}

float CAR::getDriveCurrent() {
    return (3103.0303f - analogRead(Pin_ADC_Current_Drive)) * 0.004354941f;
}

float CAR::getMosfetTemp() {
    if(!ds18b20.requestTemperaturesByAddress(Config_DS18B20_MOSFET_Addr)) sensorError[DS18B20MOSFET] = true;
    float temp =  ds18b20.getTempC(Config_DS18B20_MOSFET_Addr);
    if(temp <= DEVICE_DISCONNECTED_C) sensorWarning[DS18B20MOSFET] = true;
    return temp;
}

std::vector<CAR::OBJECT> CAR::getObjects() {
    return lastObjects;
}

float CAR::getRotation() {
    return lastRotation;
}

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

    // record RPI5 max startup time
    uint32_t rpiTime = millis() + Config_RPI5_Startup_Millis;

    // configure ADC channels
    analogReadAveraging(16);
    analogReadRes(12);

    // start display
    lcd.begin();
    lcd.fillScreen(Color_Background);

    // start SD and read configuration data
    SD.begin(BUILTIN_SDCARD);
    File configFile = SD.open("/config.txt", FILE_READ);
    if(configFile) {
        for(uint8_t i = 0; i < 8; i++) vl53l1xViewportShift[i] = configFile.readStringUntil('\n').toInt();
        rotationTimeOffset = configFile.readStringUntil('\n').toFloat();
        rotationFactor = configFile.readStringUntil('\n').toFloat();
        servoCenterDuty = configFile.readStringUntil('\n').toInt();
        configFile.close();
    }

    // start I2C systems in fast mode
    i2c0.begin(400000);
    i2c1.begin(400000);
    i2c2.begin(400000);

    // start IMU
    sensorError[IMU] = !imu.begin();

    // start DS18B20 temperature sensors at 12-bit resolution
    ds18b20.begin();
    ds18b20.setResolution(12);

    // enable pairs of VL53L1X sensors (one per bus system) and change their addresses
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_FL_RC, HIGH);
    delay(15);
    sensorError[FrontLeft] = !vl53l1x[FrontLeft].setAddress(0x2C);
    sensorError[RearCenter] = !vl53l1x[RearCenter].setAddress(0x2C);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_FC_RR, HIGH);
    delay(15);
    sensorError[FrontCenter] = !vl53l1x[FrontCenter].setAddress(0x2B);
    sensorError[RearRight] = !vl53l1x[RearRight].setAddress(0x2B);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_AR_FR, HIGH);
    delay(15);
    sensorError[AngledRight] = !vl53l1x[AngledRight].setAddress(0x2A);
    sensorError[FrontRight] = !vl53l1x[FrontRight].setAddress(0x2A);
    delay(15);
    digitalWriteFast(Pin_VL53L1X_SHT_AL_RL, HIGH);
    delay(15);

    // run VL53L1X configuration for every sensor and start continous measurements
    for(uint8_t i = 0; i < 8; i++) {
        if(!vl53l1x[i].begin()) sensorError[i] = true;
        if(!vl53l1x[i].setROI(0, vl53l1xViewportShift[i], 6, 4)) sensorError[i] = true;
        if(!vl53l1x[i].start()) sensorError[i] = true;
    }
    delay(15);

    // start TCS3472 color sensor
    sensorError[TCS3472] = !tcs3472.begin();

    if(rpi5) {

        // start UART
        Serial7.addMemoryForRead(uartReadBuffer, sizeof(uartReadBuffer));
        Serial7.begin(500000, SERIAL_8N1);

        // wait for RPI5 to send the ready/handshake packet
        while(Serial7.available() < (int)sizeof(uartTest)) {

            // when enabled, force a reboot of the RPI5 after Config_RPI5_Startup_Millis milliseconds
            if(Config_RPI5_Startup_Millis && (millis() > rpiTime)) {
                digitalWriteFast(Pin_RPI5_PWR, LOW);
                delay(2000);
                digitalWriteFast(Pin_RPI5_PWR, HIGH);
                Serial7.clear();
                rpiTime = millis() + Config_RPI5_Startup_Millis;
            }

            // handle background tasks
            yield();
        }
        Serial7.readBytes((uint8_t*)&uartTest, sizeof(uartTest));
        rpiActive = true;
    }

    // show the user that the system is ready (with sensor states and battery voltage)
    for(uint8_t i = 0; i < 10; i++) lcd.drawRect(4 + (i * 16), 4, 8, 4, sensorError[i] ? Color_Error : Color_Running);
    lcd.drawText(4, 12, font5x7, "ready", Color_Text, Color_Background, 2);
    lcd.drawText(4, 32, font5x7, (String(millis() / 1000.0f, 2) + " s").c_str(), Color_Text, Color_Background, 2);
    lcd.drawText(4, 52, font5x7, (String(getBatteryVoltage(), 2) + " V").c_str(), Color_Text, Color_Background, 2);
}

void CAR::modifyRotationOffset(float change) {
    rotationOffset += change;
}

constexpr char directions[8][3] = { "FC", "AL", "FL", "RL", "RC", "RR", "FR", "AR" };
String modeNames[4] = { "increase value", "decrease value", "start/stop", "finish" };

uint32_t testPreSamples(uint8_t sensor, uint32_t preSamples, int8_t shift) {

    // start sensor with test configuration
    if(!vl53l1x[sensor].setROI(0, shift, 6, 4)) sensorError[sensor] = true;
    if(!vl53l1x[sensor].start()) sensorError[sensor] = true;

    // perform test measurements and calculate sum
    uint32_t sum = 0;
    for(uint32_t j = 0; j < preSamples; j++) {
        delay(1);
        while(digitalReadFast(Pins_VL53L1X_INT[sensor]) == LOW) yield();
        uint16_t measurement = 0;
        vl53l1x[sensor].getDistance(&measurement);
        sum += measurement;
    }
    if(!vl53l1x[sensor].stop()) sensorError[sensor] = true;
    return sum;
}

void CAR::runCalibration(const uint32_t preSamples, const uint32_t mainSamples, uint32_t distance, uint32_t rotationDuration, uint32_t rotationRounds, float servoCalibrationSpeed) {
    delay(200);

    if(rotationDuration) {

        // ask user whether to skip
        lcd.fillScreen(Color_Background);
        lcd.drawText(4, 4, font5x7, "calibration:\nrotation time offset", Color_Text, Color_Background);
        lcd.drawText(4, 24, font5x7, "press button to start\nhold button >1s to skip", Color_Text, Color_Background);
        if(!waitButtonPress(1000)) {

            // tell user to not touch during first part of calibration
            lcd.fillScreen(Color_Background);
            lcd.drawText(4, 4, font5x7, "do not touch", Color_Text, Color_Background, 2);

            // delay measurement start to reduce vibrations caused by user
            delay(2000);

            // calculate rotation time offset
            float startRotation = 0.0f;
            imu.getRotation(startRotation);
            uint32_t endMillis = millis() + rotationDuration;
            while(millis() < endMillis) {
                if(digitalReadFast(Pin_IMU_INT)) imu.getRotation(lastRotation);
            }
            rotationTimeOffset = (startRotation - lastRotation) / rotationDuration;
        }
    }

    if(rotationRounds) {

        // ask user whether to skip
        lcd.fillScreen(Color_Background);
        lcd.drawText(4, 4, font5x7, "calibration:\nrotation factor", Color_Text, Color_Background);
        lcd.drawText(4, 24, font5x7, "press button to start\nhold button >1s to skip", Color_Text, Color_Background);
        if(!waitButtonPress(1000)) {
        
            // calculate initial rotation
            while(lastRotation == 0.0f) {
                if(digitalReadFast(Pin_IMU_INT)) imu.getRotation(lastRotation);
            }
            float startRotation = lastRotation + (rotationTimeOffset * millis());

            // tell user to turn the vehicle for rotation factor calibration
            lcd.fillScreen(Color_Background);
            lcd.drawText(4, 4, font5x7, ("please turn " + String(rotationRounds) + " rounds\nand then press button").c_str(), Color_Text, Color_Background);
            delay(500);
            while(digitalReadFast(Pin_Button_INT)) {
                if(digitalReadFast(Pin_IMU_INT)) imu.getRotation(lastRotation);
            }
            lastRotation += rotationTimeOffset * millis();
            rotationFactor = (rotationRounds * 360.0f) / abs(lastRotation - startRotation);
        }
    }

    // run calibration flow for every ToF sensor
    if(preSamples || mainSamples) {

        // ask user whether to skip
        lcd.fillScreen(Color_Background);
        lcd.drawText(4, 4, font5x7, "calibration:\ndistance sensors", Color_Text, Color_Background);
        lcd.drawText(4, 24, font5x7, "press button to start\nhold button >1s to skip", Color_Text, Color_Background);
        if(!waitButtonPress(1000)) {
            
            uint16_t data[mainSamples] = {};
            const int32_t expectedSum = distance * preSamples;
            for(uint8_t i = 0; i < 8; i++) {
                if(!vl53l1x[i].stop()) sensorError[i] = true;
                
                // show direction and distance to user
                lcd.fillScreen(Color_Background);
                lcd.drawChar(4, 4, font5x7, directions[i][0], Color_Text, Color_Background, 2);
                lcd.drawChar(16, 4, font5x7, directions[i][1], Color_Text, Color_Background, 2);
                lcd.drawChar(28, 4, font5x7, ':', Color_Text, Color_Background, 2);
                lcd.drawText(40, 4, font5x7, String(distance).c_str(), Color_Text, Color_Background, 2);
                lcd.drawText(4, 24, font5x7, "press button to start\nhold button to skip", Color_Text, Color_Background);
                if(!waitButtonPress(1000)) {

                    // draw bar chart area
                    lcd.fillScreen(Color_Background);
                    lcd.drawChar(15, 93, font5x7, '5', Color_Text, Color_Background);
                    lcd.drawText(35, 93, font5x7, "10", Color_Text, Color_Background);
                    lcd.drawText(55, 93, font5x7, "20", Color_Text, Color_Background);
                    lcd.drawText(82, 93, font5x7, "50", Color_Text, Color_Background);
                    lcd.drawText(102, 93, font5x7, "100", Color_Text, Color_Background);
                    lcd.drawText(122, 93, font5x7, "200", Color_Text, Color_Background);
                    lcd.drawRect(17, 2, 1, 89, Color_Text);
                    lcd.drawRect(37, 2, 1, 89, Color_Text);
                    lcd.drawRect(57, 2, 1, 89, Color_Text);
                    lcd.drawRect(84, 2, 1, 89, Color_Text);
                    lcd.drawRect(104, 2, 1, 89, Color_Text);
                    lcd.drawRect(124, 2, 1, 89, Color_Text);

                    // delay measurement start to reduce vibrations caused by user
                    delay(1000);

                    // find best viewport center range
                    int8_t mid = 0;
                    int32_t buf = testPreSamples(i, preSamples, 0);
                    int32_t buf2 = buf;
                    if(buf < expectedSum) {
                        while((buf < expectedSum) && (mid > -4)) {
                            buf2 = buf;
                            buf = testPreSamples(i, preSamples, --mid);
                        }
                        if(abs(expectedSum - buf) > abs(expectedSum - buf2)) mid++;
                    }
                    else if(buf > expectedSum) {
                        while((buf > expectedSum) && (mid < 4)) {
                            buf2 = buf;
                            buf = testPreSamples(i, preSamples, ++mid);
                        }
                        if(abs(expectedSum - buf) > abs(expectedSum - buf2)) mid--;
                    }

                    for(int8_t j = -2; j <= 2; j++) {
                        if((mid + j) < 0) lcd.drawChar(3, 43 + (j * 18), font5x7, '-', Color_Text, Color_Background);
                        lcd.drawChar(9, 43 + (j * 18), font5x7, '0' + abs(mid + j), Color_Text, Color_Background);
                    }

                    // record important metrics for every tested viewport center
                    float minDiff = __FLT_MAX__;
                    bool minDiffPos = 0;
                    int8_t minDiffIdx = 0;
                    float minDiffAad = 0.0f;
                    float minDiffMin = 0.0f;
                    float minDiffMax = 0.0f;
                    for(int8_t j = 0; j < 5; j++) {

                        // start sensor with test configuration
                        if(!vl53l1x[i].setROI(0, mid + j - 2, 6, 4)) sensorError[i] = true;
                        if(!vl53l1x[i].start()) sensorError[i] = true;

                        // perform test measurements and find min/max
                        float avg = 0.0f;
                        uint16_t minData = 0xFFFF;
                        uint16_t maxData = 0;
                        for(uint32_t k = 0; k < mainSamples; k++) {
                            delay(1);
                            while(digitalReadFast(Pins_VL53L1X_INT[i]) == LOW) yield();
                            vl53l1x[i].getDistance(data + k);
                            avg += data[k];
                            if(data[k] < minData) minData = data[k];
                            if(data[k] > maxData) maxData = data[k];
                        }
                        if(!vl53l1x[i].stop()) sensorError[i] = true;

                        // calculate average and differences to expected value and min/max
                        avg /= mainSamples;
                        float min_ = avg - minData;
                        float max_ = maxData - avg;
                        float diff = abs(avg - distance);

                        // calculate average absolute deviation
                        float aad = 0.0f;
                        for(uint32_t k = 0; k < mainSamples; k++) aad += abs(data[k] - avg);
                        aad /= mainSamples;

                        // find lowest difference to expected value
                        if(diff < minDiff) {
                            minDiff = diff;
                            minDiffPos = (avg > distance);
                            minDiffIdx = j;
                            minDiffAad = aad;
                            minDiffMin = min_;
                            minDiffMax = max_;
                        }

                        // show bars
                        if(diff > 1.0f) lcd.drawRect(17,  3 + (j * 18), ((log10(diff) - 0.69897f) * 67.0f) + 0.5f, 3, (avg > distance) ? Color_Diff_Pos : Color_Diff_Neg);
                        if(aad  > 1.0f) lcd.drawRect(17,  7 + (j * 18), ((log10(aad ) - 0.69897f) * 67.0f) + 0.5f, 3, Color_AAD_Bar);
                        if(min_ > 1.0f) lcd.drawRect(17, 11 + (j * 18), ((log10(min_) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Diff_Neg);
                        if(max_ > 1.0f) lcd.drawRect(17, 15 + (j * 18), ((log10(max_) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Diff_Pos);
                    }

                    // show result and print values
                    if(minDiff    > 5.0f) lcd.drawRect(17,  3 + (minDiffIdx * 18), ((log10(minDiff   ) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Running);
                    if(minDiffAad > 5.0f) lcd.drawRect(17,  7 + (minDiffIdx * 18), ((log10(minDiffAad) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Running);
                    if(minDiffMin > 5.0f) lcd.drawRect(17, 11 + (minDiffIdx * 18), ((log10(minDiffMin) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Running);
                    if(minDiffMax > 5.0f) lcd.drawRect(17, 15 + (minDiffIdx * 18), ((log10(minDiffMax) - 0.69897f) * 67.0f) + 0.5f, 3, Color_Running);
                    if(!minDiffPos) minDiff = -minDiff;
                    lcd.drawText(3, 103, font5x7, ("diff: " + String(minDiff) + "  aad: " + String(minDiffAad) + "\nmin: " + String((uint16_t)(distance + minDiff + 0.5f - minDiffMin)) + "  max: " + String((uint16_t)(distance + minDiff + 0.5f + minDiffMax))).c_str(), Color_Running, Color_Background);
                    
                    // apply new viewport shift
                    vl53l1xViewportShift[i] = mid + minDiffIdx - 2;
                    vl53l1x[i].setROI(0, vl53l1xViewportShift[i], 6, 4);
                    vl53l1x[i].start();

                    // wait for user input to continue
                    waitButtonPress();
                    delay(200);
                }
            }
        }
    }

    if(servoCalibrationSpeed > 0.0f) {

        // ask user whether to skip
        lcd.fillScreen(Color_Background);
        lcd.drawText(4, 4, font5x7, "calibration:\nservo center", Color_Text, Color_Background);
        lcd.drawText(4, 24, font5x7, "press button to start\nhold button >1s to skip", Color_Text, Color_Background);
        if(!waitButtonPress(1000)) {

            // prepare user interface
            lcd.fillScreen(Color_Background);
            lcd.drawText(4, 24, font5x7, "press button to execute\nhold >1s to switch mode", Color_Text, Color_Background);

            // semi-automatic servo calibration loop
            uint8_t mode = 0;
            while(true) {
                lcd.drawText(4, 4, font5x7, ("mode: " + modeNames[mode] + "\nservo center: " + String(servoCenterDuty)).c_str(), Color_Text, Color_Background);
                if(waitButtonPress(1000)) {
                    if(mode == 2) setVelocity(0.0f, true);
                    if(mode < 3) mode++;
                    else mode = 0;
                }
                else if(mode == 0) servoCenterDuty++;
                else if(mode == 1) servoCenterDuty--;
                else if(mode == 2) {
                    if(targetVelocity == 0) {
                        setServo(0.0f);
                        setVelocity(servoCalibrationSpeed);
                    }
                    else setVelocity(0.0f, true);
                }
                else break;
                lcd.drawRect(4, 4, 156, 17, Color_Background);
            }
        }
    }

    // write results to sd card
    SD.remove("/config.txt");
    File file = SD.open("/config.txt", FILE_WRITE_BEGIN);
    for(uint8_t i = 0; i < 8; i++) file.println(vl53l1xViewportShift[i]);
    file.println(rotationTimeOffset, 8);
    file.println(rotationFactor, 8);
    file.println(servoCenterDuty);
    file.close();
}

void CAR::runI2cScan() {
    lcd.fillScreen(Color_Background);
    I2C *i2cs[] = { &i2c0, &i2c1, &i2c2 };
    for(uint8_t i = 0; i < (sizeof(i2cs) / sizeof(i2cs[0])); i++) {
        lcd.drawText(4 + (i * 50), 4, font5x7, ("I2C " + String(i)).c_str(), Color_Text, Color_Background);
        for(uint8_t j = 1; j < 127; j++) {
            i2cs[i]->beginTransmission(j);
            uint8_t result = i2cs[i]->endTransmission();
            lcd.drawRect(4 + (i * 50) + ((j % 16) * 3), 14 + ((j / 16) * 3), 2, 2, (result == 0) ? Color_Running : ((result == 4) ? Color_Error : Color_Outlines));
        }
    }
}

void writeFile(String filename, String data) {
    File file = SD.open(filename.c_str(), FILE_WRITE);
    file.print(data);
    file.close();
}

void CAR::runTest() {

    // test config
    /*constexpr uint8_t sensors[] = { 0, 2, 3 };
    constexpr uint8_t timings[][2] = { { 33, 33 }, { 50, 50 }, { 100, 100 } };
    constexpr uint32_t totalCycles = (sizeof(timings) / sizeof(timings[0])) * sizeof(testViewports) * 9 * 9;
    constexpr uint32_t sampleCount = 200;

    // test program
    lcd.fillScreen(Color_Background);
    delay(3000);
    uint32_t remaining = 0;
    for(uint8_t i = 0; i < (sizeof(timings) / sizeof(timings[0])); i++) remaining += sizeof(testViewports) * 9 * 9 * timings[i][1] * sampleCount;
    uint16_t data[sizeof(sensors)][sampleCount];
    uint32_t done = 0;
    for(uint8_t t = 0; t < (sizeof(timings) / sizeof(timings[0])); t++) {
        for(uint8_t i = 0; i < sizeof(sensors); i++) {
            vl53l1x[sensors[i]].setTimingBudget(timings[t][0]);
            vl53l1x[sensors[i]].setIntermeasurement(timings[t][1]);
        }
        for(uint8_t c = 0; c < sizeof(testViewports); c++) {
            for(uint8_t w = 4; w <= 12; w++) {
                for(uint8_t h = 4; h <= 12; h++) {
                    lcd.fillScreen(Color_Background);
                    lcd.drawText(2, 2, font5x7, (String(done * 100.0f / totalCycles, 1) + "% done\nmin. " + String(remaining / 60000) + "m " + String((remaining / 1000) % 60) + "s\nremaining\n" + String(getBatteryVoltage(), 2) + " V").c_str(), Color_Text, Color_Background, 2);
                    for(uint8_t i = 0; i < sizeof(sensors); i++) {
                        vl53l1x[sensors[i]].setROI(w, h, testViewports[c]);
                        vl53l1x[sensors[i]].start();
                    }
                    float avg[sizeof(sensors)] = {};
                    for(uint32_t s = 0; s < sampleCount; s++) {
                        delay(5);
                        for(uint8_t i = 0; i < sizeof(sensors); i++) {
                            while(digitalReadFast(Pins_VL53L1X_INT[sensors[i]]) == LOW) yield();
                            vl53l1x[sensors[i]].getDistance(data[i] + s);
                            avg[i] += data[i][s];
                        }
                    }
                    for(uint8_t i = 0; i < sizeof(sensors); i++) vl53l1x[sensors[i]].stop();
                    float sd[sizeof(sensors)] = {};
                    for(uint8_t i = 0; i < sizeof(sensors); i++) {
                        avg[i] /= sampleCount;
                        for(uint32_t s = 0; s < sampleCount; s++) sd[i] += sq(data[i][s] - avg[i]);
                        sd[i] = sqrt(sd[i] / sampleCount);
                        writeFile("avg" + String(directions[sensors[i]]) + ".obj", "v "+ String(w) + ' ' + String(avg[i]) + ' ' + String(h) + '\n');
                        writeFile("sd"  + String(directions[sensors[i]]) + ".obj", "v "+ String(w) + ' ' + String(sd[i]) + ' ' + String(h) + '\n');
                    }
                    done++;
                    remaining -= timings[t][1] * sampleCount;
                }
            }
        }
    }
    uint32_t groupBaseIdx = 0;
    for(uint8_t t = 0; t < (sizeof(timings) / sizeof(timings[0])); t++) {
        for(uint8_t c = 0; c < sizeof(testViewports); c++) {
            String object = "o TB"+ String(timings[t][0]) + "IM" + String(timings[t][1]) + "VPC" + String(testViewports[c]) + '\n';
            for(uint8_t i = 0; i < 72; i += 9) {
                for(uint8_t j = 0; j < 8; j++) {
                    object += "f " + String(groupBaseIdx + i + j + 1) + ' ' + String(groupBaseIdx + i + j + 2) + ' ' + String(groupBaseIdx + i + j + 10) + "\nf " + String(groupBaseIdx + i + j + 2) + ' ' + String(groupBaseIdx + i + j + 10) + ' ' + String(groupBaseIdx + i + j + 11) + '\n';
                }
            }
            for(uint8_t i = 0; i < sizeof(sensors); i++) {
                writeFile("avg" + String(directions[sensors[i]]) + ".obj", object);
                writeFile("sd"  + String(directions[sensors[i]]) + ".obj", object);
            }
            groupBaseIdx += 81;
        }
    }*/
}

bool CAR::setLEDs(COLOR8 color) {
    return leds.fill(*(NeoPixel::COLOR*)&color, 12);
}

bool CAR::setLEDs(COLOR8 colors[12]) {
    return leds.show((NeoPixel::COLOR*)colors, 12);
}

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
        if(_acceleration < 0) _acceleration = -_acceleration;
        if(_deceleration > 0) _deceleration = -_deceleration;
        acceleration = _acceleration * Config_Drive_PWM_Update_Int * 0xFFFF / 1000000.0f;
        deceleration = _deceleration * Config_Drive_PWM_Update_Int * 0xFFFF / 1000000.0f;
        if(acceleration <= 0) acceleration = 1;
        if(deceleration >= 0) deceleration = -1;
    }
}

void CAR::setServo(float angle) {
    analogWrite(Pin_Servo_PWM, servoCenterDuty + (int)(((angle < -Config_Servo_Max_Angle) ? -Config_Servo_Max_Angle : ((angle > Config_Servo_Max_Angle) ? Config_Servo_Max_Angle : angle)) * Config_Servo_Angle_To_Duty));
}

void CAR::showFinish(const char* userData) {
    uint32_t time = millis() - startTime;
    lcd.fillScreen(Color_Background);
    for(uint8_t i = 0; i < 12; i++) lcd.drawRect(4 + (i * 13), 4, 9, 4, sensorError[i] ? Color_Error : (sensorWarning[i] ? Color_Warning : Color_Running));
    lcd.drawText(4, 12, font5x7, ("time: " + String(time / 1000.0f) + " s").c_str(), Color_Text, Color_Background, 2);
    lcd.drawText(4, 32, font5x7, ("convs: " + String(getConverterTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(4, 40, font5x7, ("core: " + String(getCoreTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(4, 48, font5x7, ("MOSFET: " + String(getMosfetTemp()) + " C").c_str(), Color_Text, Color_Background);
    lcd.drawText(4, 56, font5x7, userData, Color_Text, Color_Background);
    lcd.drawRect(0, 126, 160, 2, Color_Warning);

    // write recorded log data to SD card
    File logFile = SD.open("/log.csv", FILE_WRITE);
    logFile.println("timestamp");
    for(LOGDATA data : logData) logFile.printf("%u\n", data.timestamp);
    logFile.close();

    // show save confirmation
    lcd.drawRect(0, 126, 160, 2, Color_Running);
}

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

void CAR::start() {

    if(rpiActive) {
        // start object detection
        Serial7.write((uint8_t*)&uartTest, sizeof(uartTest));
    }

    // set rotation offset
    while(!imu.getRotation(lastRotation)) delay(1);
    lastRotation += millis() * rotationTimeOffset;
    lastRotation *= rotationFactor;
    rotationOffset = -lastRotation;

    // configure drive pins and PWM
    analogWriteRes(16);
    analogWriteFrequency(Pin_Drive_PWM, Config_Drive_PWM_Freq);
    analogWrite(Pin_Drive_PWM, 0);

    // configure servo pin and PWM
    analogWriteFrequency(Pin_Servo_PWM, 333.0f);
    analogWrite(Pin_Servo_PWM, servoCenterDuty);

    // update sensors multiple times
    uint32_t endMillis = millis() + 200;
    while(millis() < endMillis) {
        for(uint8_t i = 0; i < 8; i++) {
            if(digitalRead(Pins_VL53L1X_INT[i])) vl53l1x[i].getDistance(lastDistance + i);
        }
        if(digitalReadFast(Pin_IMU_INT)) {
            if(imu.getRotation(lastRotation)) {
                lastRotation += millis() * rotationTimeOffset;
                lastRotation *= rotationFactor;
                lastRotation = rotationOffset;
            }
        }
    }
    startTime = millis();
}

void CAR::update() {

    // update drive PWM duty every Config_Drive_PWM_Update_Int microseconds
    uint32_t time = micros();
    if(time > nextDriveUpdateTime) {
        nextDriveUpdateTime = time + Config_Drive_PWM_Update_Int;

        // only update PWM duty if change is needed
        if(currentVelocity != targetVelocity) {

            // disable active breaking if enabled
            digitalWriteFast(Pin_Drive_Brake, LOW);

            // calculate new duty
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

            // update PWM duty
            analogWrite(Pin_Drive_PWM, abs(currentVelocity));
        }
    }

    // update event handlers
    yield();

    // update UART to RPI5 if active
    if(rpiActive) {

        // parse received object packets
        while(Serial7.available() >= (int)sizeof(CAR::OBJECT)) {
            CAR::OBJECT object = {};
            Serial7.readBytes((uint8_t*)&object, sizeof(CAR::OBJECT));
            if(object.c) currentObjects.push_back(object);

            // start new objects list when empty packet has been received
            else {
                lastObjects = currentObjects;
                currentObjects.clear();
            }
        }
    }

    // update distances when data ready interrupt has been sent
    for(uint8_t i = 0; i < 8; i++) {
        if(digitalRead(Pins_VL53L1X_INT[i])) vl53l1x[i].getDistance(lastDistance + i);
    }

    // update rotation when IMU data ready interrupt has been sent
    if(digitalReadFast(Pin_IMU_INT)) {
        if(imu.getRotation(lastRotation)) {
            lastRotation += millis() * rotationTimeOffset;
            lastRotation *= rotationFactor;
            lastRotation += rotationOffset;
        }
    }

    // toggle onboard LED (helps debugging whether the program is executing this update function)
    digitalToggleFast(13);
}

bool CAR::waitButtonPress(uint32_t duration) {

    // wait for last button press to end
    if(!digitalReadFast(Pin_Button_INT)) {
        while(!digitalReadFast(Pin_Button_INT)) update();
        delay(40);
    }

    // activate button LED
    digitalWriteFast(Pin_Button_LED, HIGH);

    // wait until button is pressed down
    while(digitalReadFast(Pin_Button_INT)) update();

    // wait until button is released
    uint32_t end = millis() + duration;
    if(duration) {
        delay(40);
        while(!digitalReadFast(Pin_Button_INT)) {
            update();
            if(millis() >= end) break;
        }
    }

    // deactivate button LED
    digitalWriteFast(Pin_Button_LED, LOW);

    // return duration in ms
    return millis() >= end;
}