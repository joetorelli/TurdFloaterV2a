
#ifndef SENSOR_READINGS_H

#define SENSOR_READINGS_H
#include <Arduino.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>
#include "INA3221.h"
#include "Adafruit_MPRLS.h" // air sensor
#include "settings.h"
// #include "OLED.h"
//  #include "sensor_readings.h"
////#include <Adafruit_INA219.h>
// #include <movingAvg.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// struct BME_Sensor
// {
//     float f_temperature = 0;
//     float f_humidity = 0;
//     float f_pressure = 0;
//     float f_altitude = 0;
// };

struct Select_SW
{
    int Switch_Auto = 0;
    int Switch_Alarm = 0;
    int Switch_Off = 0;
    int Switch_Pump = 0;
};

struct LevelSensor
{
    int ShuntVRaw = 0;
    float ShuntVmv = 0;
    int BusVRaw = 0;
    float BusV = 0;
    float ShuntImA = 0;
    float LoadV = 0;
    float power_mW = 0;
    float DepthIn = 0;
    int DepthMM = 0;
};
struct AirSensor
{
    float pressure_hPa = 0;
    float pressure_PSI = 0;
};

int ReadLevelSensor(SDL_Arduino_INA3221 *LevSensor, LevelSensor *SensorLevelVal, int CNum);
int ReadAirPump(Adafruit_MPRLS *AirSen, AirSensor *AirSenVal);
bool ReadCLSensor(int PIN);
bool ReadWaterFlowSensor(int PIN);
double mapf(double var, double InMin, double InMax, double OutMin, double OutMax);

#endif