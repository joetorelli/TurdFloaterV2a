
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

bool ReadCLSensor(int PIN)
{
    if (digitalRead(PIN) == HIGH)
    {
        return false; // no mag
    }
    else
    {
        return true; // magnet detected
    }
}

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
int ReadLevelSensor(SDL_Arduino_INA3221 *LevSensor, LevelSensor *SensorLevelVal, int CNum)
{

    int SensorFailType = 0;
    // static int SensorFailCount = 0;
    float current_ma[3];
    float voltage[3];
    float shunt[3];
    float LoadV[3];

    // 1st element = scal vals for 12v
    // 2nd element = scal vals convert ma to mm chan
    // 3rd element = scal vals for 5v
    double in_min[3] = {0, 4.98, 0};
    double in_max[3] = {1, 20.00, 1};
    double out_min[3] = {0, 240.0, 0};
    double out_max[3] = {1, 3000, 1};

    current_ma[CNum] = LevSensor->getCurrent_mA(CNum + 1) * 1000;
    voltage[CNum] = LevSensor->getBusVoltage_V(CNum + 1);
    shunt[CNum] = LevSensor->getShuntVoltage_mV(CNum + 1); /// 1000000
    LoadV[CNum] = voltage[CNum] + (shunt[CNum]);

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

    SensorLevelVal->ShuntImA = current_ma[CNum];
    SensorLevelVal->BusV = voltage[CNum];
    SensorLevelVal->ShuntVmv = shunt[CNum];
    SensorLevelVal->LoadV = LoadV[CNum];

    // test for 12v bad reading
    if (CNum == 0)
    {
        if (SensorLevelVal->BusV < 11) // set for low 12v
        {

            // SensorFailCount++;
            SensorFailType = 1;
        }
        else if (SensorLevelVal->BusV > 15) // set for hi 12v over 350ma
        {
            // SensorFailCount++;
            SensorFailType = 2;
        }

        else if (SensorLevelVal->ShuntImA <= 0) // set for low 12v current
        {

            // SensorFailCount++;
            SensorFailType = 1;
        }
        else if (SensorLevelVal->ShuntImA > 500) // set for hi 12v over 350ma
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
    if (CNum == 1)
    {
        if (SensorLevelVal->BusV < 11) // set for low 12v
        {

            // SensorFailCount++;

            SensorFailType = 1;
        }
        else if (SensorLevelVal->BusV > 15) //  set for hi 12v
        {
            // SensorFailCount++;
            SensorFailType = 2;
        }
        if (SensorLevelVal->ShuntImA < 3.5) // test for no sensor
        {
            // SensorFailCount++;
            SensorFailType = 1;

            // pass bad val
            SensorLevelVal->DepthMM = -1;
            SensorLevelVal->DepthIn = -1;
        }
        else if (SensorLevelVal->ShuntImA > 21.0) // test for bad sensor
        {
            // SensorFailCount++;
            SensorFailType = 2;

            // pass bad val
            SensorLevelVal->DepthMM = -1;
            SensorLevelVal->DepthIn = -1;
        }
        else // good sensor
        {
            // SensorFailCount = 0;
            SensorFailType = 0;

            // pass val
            SensorLevelVal->DepthMM = mapf(current_ma[CNum], in_min[CNum], in_max[CNum], out_min[CNum], out_max[CNum]);
            SensorLevelVal->DepthIn = SensorLevelVal->DepthMM / 25.4;
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

    // test for ps 5v bad reading
    if (CNum == 2)
    {

        if (SensorLevelVal->BusV < 4) // set for low 5v
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
        }

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

double mapf(double var, double InMin, double InMax, double OutMin, double OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}