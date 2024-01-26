
#include <Arduino.h>
#include "sensor_readings.h"
// #include "INA3221.h"
#include "OLED.h"

// //#include "mqttController.h"
bool ReadWaterFlowSensor(int PIN)
{
    if (digitalRead(PIN) == HIGH)
    {
        return false; // no flow
    }
    else
    {
        return true; // flow
    }
}

/* bool ReadCLSensor(int PIN)
{
    if (digitalRead(PIN) == HIGH)
    {
        return false; // no mag
    }
    else
    {
        return true; // magnet detected
    }
} */

/* void ReadSwitches(Select_SW *SwState) // Adafruit_SSD1306 *OLED_Display)
{
    int SSWAutoPin = 4;   // auto/man pos  BUTTON_A
    int SSWAlarmPin = 36; // alarm pos BUTTON_B
    int SSWOffPin = 39;   // off pos BUTTON_C
    int SSWPumpPin = 34;  // pump pos BUTTON_C

    if (digitalRead(SSWAutoPin) == 0)
    {
        SwState->Switch_Auto = 1;
    }
    else
    {
        SwState->Switch_Auto = 0;
    }

    if (!digitalRead(SSWAlarmPin))
    {
        SwState->Switch_Alarm = 1;
    }
    else
    {
        SwState->Switch_Alarm = 0;
    }

    if (!digitalRead(SSWOffPin))
    {
        SwState->Switch_Off = 1;
    }
    else
    {
        SwState->Switch_Off = 0;
    }

    if (!digitalRead(SSWPumpPin))
    {
        SwState->Switch_Pump = 1;
    }
    else
    {
        SwState->Switch_Pump = 0;
    }
}
 */

// pressure sensor
int ReadAirPump(Adafruit_MPRLS *AirSen, AirSensor *AirSenVal)
{
    int SensorFailType = 0;
    int Reading = 0;

    AirSenVal->pressure_hPa = AirSen->readPressure();
    AirSenVal->pressure_PSI = AirSenVal->pressure_hPa / 68.947572932;
    // Serial.print("Pressure (hPa): ");
    // Serial.println(AirSenVal->pressure_hPa);
    // Serial.print("Pressure (PSI): ");
    // Serial.println(AirSenVal->pressure_PSI);
    Reading = AirSenVal->pressure_hPa;

    if (isnan(AirSenVal->pressure_hPa)) // not found
    {
        Reading = 0;
        SensorFailType = 1;
        // Serial.printf("**************NAN Air Sensor: %d", SensorFailType);
    }
    else if (AirSenVal->pressure_hPa <= 950) // low filter clogged?

    {
        SensorFailType = 2;
        // Serial.printf("**************LOW Air Sensor: %d", SensorFailType);
    }
    else if (AirSenVal->pressure_hPa >= 1100) // hi air plugged?

    {
        SensorFailType = 3;
        // Serial.printf("**************HIGH Air Sensor: %d", SensorFailType);
    }
    else // good
    {
        SensorFailType = 0;
        // Serial.printf("**************GOOD Air Sensor: %d", SensorFailType);
    }

    return SensorFailType;
}

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
    double in_min[2][3] = {{0, 0, 4.98}, {0, 0, 4.98}};
    double in_max[2][3] = {{1, 1, 20.00}, {1, 1, 20.00}};
    double out_min[2][3] = {{0, 0, 240.0}, {0, 0, 240.0}};
    double out_max[2][3] = {{1, 1, 3000}, {1, 1, 3000}};

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
        if (CNum == 2) // Air flow switch
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