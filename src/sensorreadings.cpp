
#include <Arduino.h>
#include "sensor_readings.h"
// #include "INA3221.h"
#include "OLED.h"

// //#include "mqttController.h"

// returns Status of sensor ans fill struc with values

int ReadSensorIF(SDL_Arduino_INA3221 *SensorIFBoard, SensorData *SensorValue, int BNum, int CNum)
{

    int SensorFailType = 0;
    // static int SensorFailCount = 0;
    float current_ma[2][3];
    float voltage[2][3];
    float shunt[2][3];
    float LoadV[2][3];

    // B1 1st element = AirFlow
    // B1 2nd element = CL
    // B1 3rd element = WaterFlow
    // B1 2nd element =
    // 3rd element = scal vals convert ma to mm chan
    //                         B1             B2
    //                     wf, cl, af  3v,12v, wl
    double in_min[2][3] = {{0, 0, 0}, {0, 0, 4.98}};
    double in_max[2][3] = {{1, 1, 5}, {3, 12, 20.00}};
    double out_min[2][3] = {{0, 0, 0}, {0, 0, 240.0}};
    double out_max[2][3] = {{1, 1, 30}, {3, 12, 3000}};

    current_ma[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    voltage[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);
    shunt[BNum][CNum] = SensorIFBoard->getShuntVoltage_mV(CNum + 1); /// 1000000
    LoadV[BNum][CNum] = voltage[BNum][CNum] + shunt[BNum][CNum];

    /*  // SensorLevelVal struct
    int ShuntVRaw = 0;
    float ShuntVmv
    int BusVRaw = 0;
    float BusV
    float ShuntImA
    float LoadV
    float power_mW = 0;
    float DepthIn
    int DepthMM
    */

    SensorValue->ShuntImA = current_ma[BNum][CNum];
    SensorValue->BusV = voltage[BNum][CNum];
    SensorValue->ShuntVmv = shunt[BNum][CNum];
    SensorValue->LoadV = LoadV[BNum][CNum];
    //////////////////////////////////////////////////// needs update
    // test for bad reading
    if (BNum == 0) // I/F Board 1
    {
        if (CNum == 0) // water flow
        {
            if (SensorValue->BusV < 11) // set for low 12v
            {

                // SensorFailCount++;
                SensorFailType = 1;
            }
            else if (SensorValue->BusV > 15) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }

            else if (SensorValue->ShuntImA <= 0) // set for low 12v current
            {

                // SensorFailCount++;
                SensorFailType = 1;
            }
            else if (SensorValue->ShuntImA > 500) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }
            else //////////// good 12v
            {
                // SensorFailCount = 0;
                SensorFailType = 0;
            }
        }

        ////// test for bad reading
        if (CNum == 1) // CL Switch
        {
            if (SensorValue->BusV < 11) // set for low 12v
            {

                // SensorFailCount++;

                SensorFailType = 1;
            }
            else if (SensorValue->BusV > 15) //  set for hi 12v
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }
            if (SensorValue->ShuntImA < 3.5) // test for no sensor
            {
                // SensorFailCount++;
                SensorFailType = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA > 21.0) // test for bad sensor
            {
                // SensorFailCount++;
                SensorFailType = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {
                // SensorFailCount = 0;
                SensorFailType = 0;

                // pass val
                SensorValue->DepthMM = mapf(current_ma[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
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
            if (SensorValue->BusV < 0) // set for low 12v
            {

                // SensorFailCount++;

                SensorFailType = 1;
            }
            else if (SensorValue->BusV > 15) //  set for hi 12v
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }
            if (SensorValue->ShuntImA < 3.5) // test for no sensor
            {
                // SensorFailCount++;
                SensorFailType = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA > 21.0) // test for bad sensor
            {
                // SensorFailCount++;
                SensorFailType = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {
                // SensorFailCount = 0;
                SensorFailType = 0;

                // pass val
                SensorValue->DepthMM = mapf(current_ma[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }
            /*         if (SensorLevelVal->BusV < 4) // set for low 5v
                    {

                        // SensorFailCount++;
                        SensorFailType = 1;
                    }
                    else if (SensorLevelVal->BusV > 5.5) // set for hi 5v
                    {
                        // SensorFailCount++;
                        SensorFailType = 2;
                    }

                    if (SensorLevelVal->ShuntImA <= 0) //////////// set for low 5v current
                    {

                        // SensorFailCount++;
                        SensorFailType = 1;
                    }
                    else if (SensorLevelVal->ShuntImA > 750) //////////// set for hi 5v current
                    {
                        // SensorFailCount++;
                        SensorFailType = 2;
                    }
                    else //////////// good 5v
                    {
                        // SensorFailCount = 0;
                        SensorFailType = 0;
                    } */

            //  SensorFailCount = 0; ////////////////////////////////////
            /*         if (SensorFailCount > 5)
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
        return SensorFailType;
    }

    // test for bad reading
    if (BNum == 1) // I/F Board 2
    {
        if (CNum == 0) // 3.3v
        {
            if (SensorValue->BusV < 11) // set for low 12v
            {

                // SensorFailCount++;
                SensorFailType = 1;
            }
            else if (SensorValue->BusV > 15) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }

            else if (SensorValue->ShuntImA <= 0) // set for low 12v current
            {

                // SensorFailCount++;
                SensorFailType = 1;
            }
            else if (SensorValue->ShuntImA > 500) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }
            else //////////// good 12v
            {
                // SensorFailCount = 0;
                SensorFailType = 0;
            }
        }

        ////// test for Sensor bad reading
        if (CNum == 1) // 12v
        {
            if (SensorValue->BusV < 11) // set for low 12v
            {

                // SensorFailCount++;

                SensorFailType = 1;
            }
            else if (SensorValue->BusV > 15) //  set for hi 12v
            {
                // SensorFailCount++;
                SensorFailType = 2;
            }
        }

        if (CNum == 2) // water level
        {
            if (SensorValue->ShuntImA < 3.5) // test for no sensor
            {
                // SensorFailCount++;
                SensorFailType = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA > 21.0) // test for bad sensor
            {
                // SensorFailCount++;
                SensorFailType = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {
                // SensorFailCount = 0;
                SensorFailType = 0;

                // pass val
                SensorValue->DepthMM = mapf(current_ma[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
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
            /*         if (SensorLevelVal->BusV < 4) // set for low 5v
                    {

                        // SensorFailCount++;
                        SensorFailType = 1;
                    }
                    else if (SensorLevelVal->BusV > 5.5) // set for hi 5v
                    {
                        // SensorFailCount++;
                        SensorFailType = 2;
                    }

                    if (SensorLevelVal->ShuntImA <= 0) //////////// set for low 5v current
                    {

                        // SensorFailCount++;
                        SensorFailType = 1;
                    }
                    else if (SensorLevelVal->ShuntImA > 750) //////////// set for hi 5v current
                    {
                        // SensorFailCount++;
                        SensorFailType = 2;
                    }
                    else //////////// good 5v
                    {
                        // SensorFailCount = 0;
                        SensorFailType = 0;
                    } */

            //  SensorFailCount = 0; ////////////////////////////////////
            /*         if (SensorFailCount > 5)
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
    }
    return SensorFailType;
}

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}