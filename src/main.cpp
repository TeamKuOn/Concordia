/*
 * main.cpp

    using devivce
        INA226
        HW
        NEO6M
        V transmitter
 */

/* Main library */
#include "Arduino.h"
#include <Arduino_FreeRTOS.h>

#include "timers.h"
#include <semphr.h>

/* Communication library */
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>

/* Sensor & Device library */
#include <INA226_asukiaaa.h>
#include <JY901.h>
#include <TinyGPS++.h>

SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xVariableSemaphore;

/* Serial setting */
// #define imuSerial Serial1 // Rx(green) -> Tx2(18), Tx(yellow) -> Rx1(19)
#define gpsSerial Serial2 // Rx -> Tx2(16), Tx -> Rx2(17)
#define chipSelect 53
#define mainBattery A0

static const uint32_t SerialBaud = 115200;
static const uint32_t GPSBaud = 9600;
// static const uint32_t IMUBaud = 9600;
// const uint16_t ina226calib = INA226_asukiaaa::calcCalibByResistorMilliOhm(2); // Max 5120 milli ohm

/* Class init */
// MCP2515 mcp2515(chipSelect);
TinyGPSPlus gpsInfo;
// INA226_asukiaaa voltCurrMeter(INA226_ASUKIAAA_ADDR_A0_GND_A1_GND, ina226calib);

/* Struct Definition */
struct SerialMesg {
    char Date[40] = "";
    char Time[40] = "";
    
    // // Accelaration XYZ
    // char accX[20] = "";
    // char accY[20] = "";
    // char accZ[20] = "";

    // GPS position
    char lat[90] = "";
    char lng[90] = "";
    char altMeters[90] = "";
    char degree[90] = "";
    char speedkmph[90] = "";

    // GPS hrizontal dilution of precision
    char hdop[90] = "";

        // Battery
    // char voltage[20] = "";
    // char current[20] = "";
    // char power[20] = "";

};

// struct IMU {
//     float accX = 0;
//     float accY = 0;
//     float accZ = 0;
// };

struct NEO7M_GPS {
    // Date
    int year = 0;
    int month = 0;
    int day = 0;

    // Time
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

    // GPS position
    double lat = 0;
    double lng = 0;
    double altMeters = 0;
    double degree = 0;
    double speedkmph = 0;

    // GPS hrizontal dilution of precision
    double hdop = 0;
};

// struct INA226Data {
//     int16_t voltage = 0;
//     int16_t current = 0;
//     int16_t power = 0;
// };

// struct BatteryData {
//     double voltage = 0;
//     int16_t current = 0;
//     double power = 0;
// };

/* Struct init */
// struct can_frame canMsgIMU;
// struct can_frame canMsgGPS;
// struct can_frame canMsgVol;
// struct can_frame canMsgCurr;
// struct IMU carAcc;
struct NEO7M_GPS carPosition;
// struct BatteryData mainBatteryUnit;
struct SerialMesg Mesg4Raspi;


/* Task function init */
// void TaskCANSend(void *pvParameters);
// void TaskSerialSend(void *pvParameters);
// void TaskIMU(void *pvParameters);
void TaskCarPosition(void *pvParameters);
// void TaskINA226(void *pvParameters);
// void TaskVTransmitter(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup() {

    Serial.begin(SerialBaud);
    // imuSerial.begin(IMUBaud);
    gpsSerial.begin(GPSBaud);

    /* Semaphire Setting */
    if((xSerialSemaphore = xSemaphoreCreateMutex()) != NULL)
        xSemaphoreGive((xSerialSemaphore));

    /* CAN Bus Setting */
    // mcp2515.reset();
    // mcp2515.setBitrate(CAN_125KBPS);
    // mcp2515.setNormalMode();

    /* Task Setting */
    // xTaskCreate(TaskSerialSend,  "TaskSerialSend",  1000, NULL, 5, NULL);
    // xTaskCreate(TaskCANSend,     "sendCANBus",      128,  NULL, 2, NULL);
    xTaskCreate(TaskCarPosition,  "TaskCarPosition",  1000, NULL, 2, NULL);
    // xTaskCreate(TaskIMU,          "TaskIMU",          500,  NULL, 3, NULL);
    // xTaskCreate(TaskINA226,       "TaskINA226",       500,  NULL, 3, NULL);
    // xTaskCreate(TaskVTransmitter, "TaskVTransmitter", 500,  NULL, 1, NULL);
}

void loop() {}

// void TaskSerialSend(void *pvParameters) {
//     (void)pvParameters;

//     for(;;) {
//         if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//             Serial.println("Data Recieved");


//             // GPS time
//             // sprintf(Mesg4Raspi.Date, "gps.date:%04d-%02d-%02d", carPosition.year, carPosition.month, carPosition.day);
//             // Serial.println(Mesg4Raspi.Date);

//             sprintf(Mesg4Raspi.Time, "gps.time:%02d:%02d:%02d", carPosition.hour, carPosition.minute, carPosition.second);
//             Serial.println(Mesg4Raspi.Time);

            
//             // IMU
//             // sprintf(Mesg4Raspi.accX, "accX:%f", carAcc.accX);
//             // Serial.println(Mesg4Raspi.accX);

//             // sprintf(Mesg4Raspi.accY, "accY:%f", carAcc.accY);
//             // Serial.println(Mesg4Raspi.accY);

//             // sprintf(Mesg4Raspi.accZ, "accZ:%f", carAcc.accZ);
//             // Serial.println(Mesg4Raspi.accZ);


//             // GPS
//             sprintf(Mesg4Raspi.lat, "gps.lat:%f", carPosition.lat);
//             Serial.println(Mesg4Raspi.lat);

//             sprintf(Mesg4Raspi.lng, "gps.lng:%f", carPosition.lng);
//             Serial.println(Mesg4Raspi.lng);

//             sprintf(Mesg4Raspi.altMeters, "gps.altM:%f", carPosition.altMeters);
//             Serial.println(Mesg4Raspi.altMeters);

//             sprintf(Mesg4Raspi.degree, "gps.degree:%f", carPosition.degree);
//             Serial.println(Mesg4Raspi.altMeters);

//             sprintf(Mesg4Raspi.speedkmph, "gps.speedkmph:%f", carPosition.speedkmph);
//             Serial.println(Mesg4Raspi.altMeters);

//             sprintf(Mesg4Raspi.hdop, "gps.hdop:%f", carPosition.hdop);
//             Serial.println(Mesg4Raspi.hdop);


//             // Batterry Unit
//             // sprintf(Mesg4Raspi.voltage, "mainBat.V:%f", mainBatteryUnit.voltage);
//             // Serial.println(Mesg4Raspi.voltage);

//             // sprintf(Mesg4Raspi.current, "mainBat.A:%d", mainBatteryUnit.current);
//             // Serial.println(Mesg4Raspi.current);

//             // sprintf(Mesg4Raspi.power, "mainBat.W:%f", mainBatteryUnit.power);
//             // Serial.println(Mesg4Raspi.power);



//             xSemaphoreGive(xSerialSemaphore);
//         }
//     }

//     vTaskDelay(pdMS_TO_TICKS(1000));
// }

// void TaskCANSend(void *pvParameters) {
//     (void)pvParameters;

//     for(;;) {
//         if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//             mcp2515.sendMessage(&canMsg1);
//             xSemaphoreGive(xSerialSemaphore);
//         }
//     }

//     vTaskDelay(pdMS_TO_TICKS(100));
// }

void TaskCarPosition(void *pvParameters) {

    for(;;) {
        if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
            while(gpsSerial.available() > 0) {
                gpsInfo.encode(gpsSerial.read());
            }
            xSemaphoreGive(xSerialSemaphore);
        }

        // if (xSemaphoreTake(xVariableSemaphore, (TickType_t)10) == pdTRUE) {
        //     // carPosition.year = gpsInfo.date.year();
        //     // carPosition.month = gpsInfo.date.month();
        //     // carPosition.day = gpsInfo.date.day();
        //     carPosition.hour = gpsInfo.time.hour();
        //     carPosition.minute = gpsInfo.time.minute();
        //     carPosition.second = gpsInfo.time.second();
        //     carPosition.lat = gpsInfo.location.lat();
        //     carPosition.lng = gpsInfo.location.lng();
        //     carPosition.altMeters = gpsInfo.altitude.meters();
        //     carPosition.degree = gpsInfo.course.deg();
        //     carPosition.speedkmph = gpsInfo.speed.kmph();
        //     carPosition.hdop = gpsInfo.hdop.value();

        //     xSemaphoreGive(xVariableSemaphore);
        // }

        if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
            // GPS
            // sprintf(Mesg4Raspi.Date, "gps.date:%04d-%02d-%02d", carPosition.year, carPosition.month, carPosition.day);
            // sprintf(Mesg4Raspi.Date, "gps.date:%04d-%02d-%02d", gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day());
            // Serial.println(Mesg4Raspi.Date);

            // sprintf(Mesg4Raspi.Time, "gps.time:%02d:%02d:%02d", carPosition.hour, carPosition.minute, carPosition.second);
            sprintf(Mesg4Raspi.Time, "gps.time:%02d:%02d:%02d", gpsInfo.time.hour(), gpsInfo.time.minute(), gpsInfo.time.second());
            Serial.println(Mesg4Raspi.Time);

            // sprintf(Mesg4Raspi.lat, "gps.lat:%f", carPosition.lat);
            // sprintf(Mesg4Raspi.lat, "gps.lat:%f", gpsInfo.location.lat());
            // Serial.println(Mesg4Raspi.lat);
            Serial.print("gps.lat:");
            Serial.println(gpsInfo.location.lat());

            // sprintf(Mesg4Raspi.lng, "gps.lng:%f", carPosition.lng);
            // sprintf(Mesg4Raspi.lng, "gps.lng:%lf", gpsInfo.location.lng());
            // Serial.println(Mesg4Raspi.lng);
            Serial.print("gps.lng:");
            Serial.println(gpsInfo.location.lng());

            // sprintf(Mesg4Raspi.altMeters, "gps.altM:%f", carPosition.altMeters);
            // sprintf(Mesg4Raspi.altMeters, "gps.altM:%f", gpsInfo.altitude.meters());
            // Serial.println(Mesg4Raspi.altMeters);
            Serial.print("gps.altM:");
            Serial.println(gpsInfo.altitude.meters());

            // sprintf(Mesg4Raspi.degree, "gps.degree:%f", carPosition.degree);
            // sprintf(Mesg4Raspi.degree, "gps.degree:%f", gpsInfo.course.deg());
            // Serial.println(Mesg4Raspi.altMeters);
            Serial.print("gps.degree:");
            Serial.println(gpsInfo.course.deg());

            // sprintf(Mesg4Raspi.speedkmph, "gps.speedkmph:%f", carPosition.speedkmph);
            // sprintf(Mesg4Raspi.speedkmph, "gps.speedkmph:%f", gpsInfo.speed.kmph());
            // Serial.println(Mesg4Raspi.altMeters);
            Serial.print("gps.speedkmph:");
            Serial.println(gpsInfo.speed.kmph());

            sprintf(Mesg4Raspi.hdop, "gps.hdop:%ld", gpsInfo.hdop.value());
            Serial.println(Mesg4Raspi.hdop);

            xSemaphoreGive(xSerialSemaphore);
        }

        vTaskDelay(10);
    }
}


// void TaskIMU(void *pvParameters) {

//     for(;;) {
//         if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//             while(imuSerial.available()) {
//                 JY901.CopeSerialData(imuSerial.read());
//             }
//             xSemaphoreGive(xSerialSemaphore);
//         }

//         if(xSemaphoreTake(xVariableSemaphore, (TickType_t)10) == pdTRUE) {
//             carAcc.accX = (float)JY901.stcAcc.a[0] / 32768 * 16;
//             carAcc.accY = (float)JY901.stcAcc.a[1] / 32768 * 16;
//             carAcc.accZ = (float)JY901.stcAcc.a[2] / 32768 * 16;

//             xSemaphoreGive(xVariableSemaphore);
//         }

//         // if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//         //     Serial.print("accX:");
//         //     Serial.print(carAcc.accX);
//         //     Serial.print("accY:");
//         //     Serial.print(carAcc.accY);
//         //     Serial.print("accZ:");
//         //     Serial.println(carAcc.accZ);
//         //     xSemaphoreGive(xSerialSemaphore);
//         // }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void VTaskCarPosition(void *pvParameters) {

//     for(;;) {

//         if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//             while(gpsSerial.available() > 0) {
//                 gpsInfo.encode(gpsSerial.read());
//             }

//             xSemaphoreGive(xSerialSemaphore);
//         }

//         if(xSemaphoreTake(xVariableSemaphore, (TickType_t)10) == pdTRUE) {
//             carPosition.year = gpsInfo.date.year();
//             carPosition.month = gpsInfo.date.month();
//             carPosition.day = gpsInfo.date.day();

//             sprintf(carPosition.Date, "%04d-%02d-%02d", carPosition.year,
//                     carPosition.month, carPosition.day);
//             Serial.print("Date:");
//             Serial.println(carPosition.Date);

//             carPosition.hour = gpsInfo.time.hour();
//             carPosition.minute = gpsInfo.time.minute();
//             carPosition.second = gpsInfo.time.second();

//             sprintf(carPosition.Time, "%02d:%02d:%02d", carPosition.hour,
//                     carPosition.minute, carPosition.second);
//             Serial.print("Time:");
//             Serial.println(carPosition.Time);

//             carPosition.lat = gpsInfo.location.lat();
//             carPosition.lng = gpsInfo.location.lng();

//             Serial.print("lat:");
//             Serial.println(carPosition.lat, 8);
//             Serial.print("lng:");
//             Serial.println(carPosition.lng, 8);

//             carPosition.altMeters = gpsInfo.altitude.meters();

//             Serial.print("altM:");
//             Serial.println(carPosition.altMeters);

//             carPosition.hdop = gpsInfo.hdop.value();

//             Serial.print("hdop:");
//             Serial.println(carPosition.hdop);

//             xSemaphoreGive(xVariableSemaphore);
//         }

//         vTaskDelay(1);
//     }
// }

// void TaskINA226(void *pvParameters) {
//     for(;;) {
//         if(xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE) {
//             // voltCurrMeter.readMV(&mainBatteryUnit.voltage);
//             voltCurrMeter.readMA(&mainBatteryUnit.current);
//             // voltCurrMeter.readMW(&mainBatteryUnit.power);

//             sprintf(Mesg4Raspi.voltage, "mainBat.V:%f", mainBatteryUnit.voltage);
//             Serial.println(Mesg4Raspi.voltage);

//             sprintf(Mesg4Raspi.current, "mainBat.A:%d", mainBatteryUnit.current);
//             Serial.println(Mesg4Raspi.current);

//             sprintf(Mesg4Raspi.power, "mainBat.W:%f", mainBatteryUnit.power);
//             Serial.println(Mesg4Raspi.power);

//             xSemaphoreGive(xSerialSemaphore);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void TaskVTransmitter(void *pvParameters) {
//     for(;;) {
//         if(analogRead(mainBattery)) {
//             mainBatteryUnit.voltage = 40 * abs(analogRead(mainBattery)); // row val:200V max <-- transmitted val:5V max

//             if(xSemaphoreTake(xVariableSemaphore, (TickType_t)10) == pdTRUE) {
//                 mainBatteryUnit.power =
//                     mainBatteryUnit.voltage * mainBatteryUnit.current;

//                 xSemaphoreGive(xVariableSemaphore);
//             }

//             if(xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
//             sprintf(Mesg4Raspi.voltage, "mainBat.V:%f", mainBatteryUnit.voltage);
//             Serial.println(Mesg4Raspi.voltage);

//             sprintf(Mesg4Raspi.power, "mainBat.W:%f", mainBatteryUnit.power);
//             Serial.println(Mesg4Raspi.power);


//                 xSemaphoreGive(xSerialSemaphore);
//             }

//         } else {
//             mainBatteryUnit.voltage = 0;
//             mainBatteryUnit.power = 0;

//             sprintf(Mesg4Raspi.voltage, "mainBat.V:%f", mainBatteryUnit.voltage);
//             Serial.println(Mesg4Raspi.voltage);

//             sprintf(Mesg4Raspi.power, "mainBat.W:%f", mainBatteryUnit.power);
//             Serial.println(Mesg4Raspi.power);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }