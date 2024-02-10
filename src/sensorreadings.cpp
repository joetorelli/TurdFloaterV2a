
#include <Arduino.h>
#include "sensor_readings.h"
// #include "INA3221.h"
#include "OLED.h"

// //#include "mqttController.h"

// returns Status of sensor ans fill struc with values

void ReadSensorIF(SDL_Arduino_INA3221 *SensorIFBoard, SensorData *SensorValue, int BNum, int CNum)
{

    // int SensorFailType = 0;
    //  static int SensorFailCount = 0;
    //  float current_ma[2][3];
    //  float voltage[2][3];
    //  float shunt[2][3];
    //  float LoadV[2][3];

    // change array size for what is needed**********************************
    // maybe move to eprom
    // B1 Chan1 = WaterFlow
    // B1 Chan2 = CLSW
    // B1 Chan3 = AirFlow
    // B2 Chan1 = 3.3v
    // B2 Chan2 = 12v
    // B2 Chan3 = WaterLevel scale vals convert 4-20ma to mm
    //                         B1             B2
    //                      wf,  cl,  af     3v,  12v, wl
    double in_min[2][3] = {{0.0, 0.0, 0.5}, {0.0, 0.0, 4.98}};
    double in_max[2][3] = {{12.0, 12.0, 4.5}, {3.3, 12, 20.00}};
    double out_min[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 240.0}};
    double out_max[2][3] = {{1.0, 1.0, 30.0}, {3.3, 12.0, 3000}};

    // current_ma[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    // voltage[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);
    //  shunt[BNum][CNum] = SensorIFBoard->getShuntVoltage_mV(CNum + 1); /// 1000000
    //  LoadV[BNum][CNum] = voltage[BNum][CNum] + shunt[BNum][CNum];

    /*  // SensorLevelVal struct
    int ShuntVRaw = 0;
    float ShuntVmv
    int BusVRaw = 0;
    float BusV
    float ShuntImA
    float LoadV
    xfloat power_mW = 0;
    float DepthIn
    int DepthMM
    */

    SensorValue->ShuntImA[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    SensorValue->BusV[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);

    //////////////////////////////////////////////////// needs update
    // test for bad reading
    if (BNum == 0) // I/F Board 1
    {
        if (CNum == 0) // water flow
        {
            if (SensorValue->BusV[BNum][CNum] < 11) // set for low 12v
            {

                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 15) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }

            else if (SensorValue->ShuntImA[BNum][CNum] <= 0) // set for low 12v current
            {

                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 500) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }
            else //////////// good 12v
            {
                // SensorFailCount = 0;
                SensorValue->SensorFailType[BNum][CNum] = 0;
            }
        }

        ////// test for bad reading
        if (CNum == 1) // CL Switch
        {
            if (SensorValue->BusV[BNum][CNum] < 11) // set for low 12v
            {

                // SensorFailCount++;

                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 15) //  set for hi 12v
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }
            if (SensorValue->ShuntImA[BNum][CNum] < 3.5) // test for no sensor
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 21.0) // test for bad sensor
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {
                // SensorFailCount = 0;
                SensorValue->SensorFailType[BNum][CNum] = 0;

                // pass val
                SensorValue->DepthMM = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }

            /*         //  SensorFailCount = 0; ////////////////////////////////////
                    if (SensorFailCount > 5)
                    {

                        // Serial.print("Sensor Fail:");
                        SensorFailCount = 0;

                                     switch (SensorFailType)
                                    {
                                    case 0:
                                        Serial.println("Sensor OK");
                                        return SensorFailType;
                                    case 1:
                                        Serial.println("Sensor Not Found");
                                        return SensorFailType;
                                        break;
                                    case 2:
                                        Serial.println("Sensor Failed");
                                        return SensorFailType;
                                        break;
                                    default:
                                        Serial.println("Something went wrong");
                                        return SensorFailType;
                                        break;
                                    }
                    } */
        }

        // test for bad reading
        if (CNum == 2) // Air flow sensor
        {

            if (SensorValue->BusV[BNum][CNum] < .5) // set for no sensor
            {

                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 1) // set for low press
            {

                SensorValue->SensorFailType[BNum][CNum] = 2;
            }

            else if (SensorValue->BusV[BNum][CNum] > 4) //  set for hi press
            {

                SensorValue->SensorFailType[BNum][CNum] = 3;

                // pass bad val
                // SensorValue->pressure_PSI = 0;
            }
            else // good sensor
            {
                // SensorFailCount = 0;
                SensorValue->SensorFailType[BNum][CNum] = 0;

                // pass val
                SensorValue->DepthMM = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }
        }
        // return SensorFailType;
    }

    // test for bad reading
    if (BNum == 1) // I/F Board 2
    {
        if (CNum == 0) // 3.3v
        {
            if (SensorValue->BusV[BNum][CNum] < 3.1) // set for low v
            {
                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 3.5) //  set for hi v
            {
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }
            else // good sensor
            {

                SensorValue->SensorFailType[BNum][CNum] = 0;
            }
        }

        ////// test for 12v
        if (CNum == 1)
        {

            if (SensorValue->BusV[BNum][CNum] < 9) // set for low 12v
            {

                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 15) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }

            else if (SensorValue->ShuntImA[BNum][CNum] <= 0) // set for low 12v current
            {

                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 500) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorValue->SensorFailType[BNum][CNum] = 2;
            }
            else //////////// good 12v
            {
                // SensorFailCount = 0;
                SensorValue->SensorFailType[BNum][CNum] = 0;
            }
        }

        // test water level
        if (CNum == 2)
        {
            if (SensorValue->ShuntImA[BNum][CNum] < 3.5) // test for no sensor
            {

                SensorValue->SensorFailType[BNum][CNum] = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 21.0) // test for bad sensor
            {

                SensorValue->SensorFailType[BNum][CNum] = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {

                SensorValue->SensorFailType[BNum][CNum] = 0;

                // pass val
                SensorValue->DepthMM = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }
        }
    }
    // return SensorFailType;
}

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}