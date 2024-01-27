
#ifndef SENSOR_READINGS_H

#define SENSOR_READINGS_H
#include <Arduino.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>
#include "INA3221.h"
#include "settings.h"
// #include "OLED.h"
//  #include "sensor_readings.h"

// #define SEALEVELPRESSURE_HPA (1013.25)

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

struct SensorData
{
    // int ShuntVRaw = 0;
    // float ShuntVmv = 0;
    // int BusVRaw = 0;
    // float LoadV[2][3];
    // float power_mW = 0;

    // values read from INA3221 boards
    float BusV[2][3];
    float ShuntImA[2][3];

    // values for WaterLevel sensor
    float DepthIn = 0;
    int DepthMM = 0;

    // value for AirPressure sensor
    float pressure_PSI = 0;

    // values for chlorine sensor
    int CLStatus = 0;
    // values for WaterFlow sensor
    int WFStatus = 0;
};

int ReadSensorIF(SDL_Arduino_INA3221 *SensorIFBoard, SensorData *SensorValue, int BNum, int CNum);
double mapf(double var, double InMin, double InMax, double OutMin, double OutMax);

#endif