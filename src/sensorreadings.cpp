
#include <Arduino.h>
#include "sensor_readings.h"
// #include "INA3221.h"
#include "OLED.h"

// //#include "mqttController.h"

// returns Status of sensor ans fill struc with values

int ReadSensorIF(SDL_Arduino_INA3221 *SensorIFBoard, SensorData *SensorValue, int BNum, int CNum)
{

    int SensorStatus = 255;
    // static int SensorFailCount = 0;
    // float current_ma[2][3];
    // float voltage[2][3];
    // float shunt[2][3];
    // float LoadV[2][3];

    // change array size for what is needed**********************************
    // maybe move to eprom
    // B1 Chan1 = WaterFlow
    // B1 Chan2 = CLSW
    // B1 Chan3 = AirFlow scal vals convert .5-4.5v to 0-30psi
    // B2 Chan1 = 3.3v
    // B2 Chan2 = 12v
    // B2 Chan3 = WaterLevel scale vals convert 4-20ma to 0-3000mm
    //                         B1             B2
    //                     wf, cl, af  3v, 12v, wl
    /*     double in_min[2][3] = {{0, 0, 0}, {0, 0, 4.98}};
        double in_max[2][3] = {{1, 1, 5}, {3.3, 12, 20.00}};
        double out_min[2][3] = {{0, 0, 0}, {0, 0, 40.0}};
        double out_max[2][3] = {{1, 1, 30}, {3.3, 12, 3000}}; */
    float in_min[2][3] = {{0.0, 0.0, 0.5}, {0, 0, 4.1}};
    float in_max[2][3] = {{12.0, 5, 4.5}, {3.3, 12, 19.92}};
    float out_min[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 60.0}};
    float out_max[2][3] = {{1.0, 1.0, 30.0}, {3.3, 12.0, 3000}};
    // current_ma[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    // voltage[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);
    //  shunt[BNum][CNum] = SensorIFBoard->getShuntVoltage_mV(CNum + 1); /// 1000000
    //  LoadV[BNum][CNum] = voltage[BNum][CNum] + shunt[BNum][CNum];

    /*  // SensorLevelVal struct
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
    */

    SensorValue->ShuntImA[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    SensorValue->BusV[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);
    // test print
    Serial.println();
    Serial.printf("********************* Read Sensor Value, Board= %d  Chan= %d \n", BNum, CNum);
    Serial.printf("Bus V= %.2f  BusI= %.2f \n", SensorValue->BusV[BNum][CNum], SensorValue->ShuntImA[BNum][CNum]);
    Serial.printf("DepthIn= %d  DepthMM= %d  AP_PSI= %.2f  CLStatus= %d  WFStatus= %d \n", SensorValue->DepthIn, SensorValue->DepthMM, SensorValue->pressure_PSI, SensorValue->CLStatus, SensorValue->WFStatus);

    Serial.println();

    //////////////////////////////////////////////////// needs update

    if (BNum == 0) // I/F Board 1
    {
        if (CNum == 0) // water flow switch
        {
            // read the current
            int temp = SensorValue->ShuntImA[BNum][CNum];

            switch (temp)
            {
            case 0 ... 1:
                // no sensor
                SensorValue->WFStatus = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 2;

                break;
            case 2 ... 29:
                // good value
                if (SensorValue->ShuntImA[BNum][CNum] < 15) // switch open
                {
                    SensorValue->WFStatus = 0;
                    SensorStatus = 0;
                }
                else // closed
                {
                    SensorValue->WFStatus = 1;
                    SensorStatus = 0;
                }

                break;
            default:
                // bad sensor
                SensorValue->WFStatus = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 1;
            }

            Serial.printf("TestWFSwitchCurrent= %.2f    TestWFSwitchV= .2%f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            Serial.printf("WFStatus= %.2f    SensorStatus %d  \n", SensorValue->WFStatus, SensorStatus);
        }

        if (CNum == 1) // CL Switch
        {

            // read the current
            int temp = SensorValue->ShuntImA[BNum][CNum];

            switch (temp)
            {
            case 0 ... 1:
                // no sensor
                SensorValue->CLStatus = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 2;

                break;
            case 2 ... 29:
                // good value
                if (SensorValue->ShuntImA[BNum][CNum] < 15) // switch open
                {
                    SensorValue->CLStatus = 0;
                    SensorStatus = 0;
                }
                else // closed
                {
                    SensorValue->CLStatus = 1;
                    SensorStatus = 0;
                }

                break;
            default:
                // bad sensor
                SensorValue->CLStatus = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 1;
            }
            Serial.printf("TestCLSwitchCurrent= %.2f    TestCLSwitchV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            Serial.printf("CLStatus= %.2f    SensorStatus %d  \n", SensorValue->CLStatus, SensorStatus);

            /*         //  SensorFailCount = 0; ////////////////////////////////////
                    if (SensorFailCount > 5)
                    {

                        // Serial.print("Sensor Fail:");
                        SensorFailCount = 0;

                                     switch (SensorStatus)
                                    {
                                    case 0:
                                        Serial.println("Sensor OK");
                                        return SensorStatus;
                                    case 1:
                                        Serial.println("Sensor Not Found");
                                        return SensorStatus;
                                        break;
                                    case 2:
                                        Serial.println("Sensor Failed");
                                        return SensorStatus;
                                        break;
                                    default:
                                        Serial.println("Something went wrong");
                                        return SensorStatus;
                                        break;
                                    }
                    } */
        }

        // test for bad reading
        if (CNum == 2) // Air flow sensor
        {
            if (SensorValue->BusV[BNum][CNum] < 1) //   no sensor
            {
                SensorValue->pressure_PSI = 0;
                SensorStatus = 2;
            }
            else if (SensorValue->BusV[BNum][CNum] > 4.5) //  set for hi press voltage
            {
                SensorValue->pressure_PSI = mapf(SensorValue->BusV[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorStatus = 1;
            }
            else  // good range voltage
            {

                SensorValue->pressure_PSI = mapf(SensorValue->BusV[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorStatus = 0;
            }

            Serial.printf("TestAirFlowCurrent= %.2f    TestAirFlowV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            Serial.printf("pressure_PSI= %.2f    SensorStatus %d  \n", SensorValue->pressure_PSI, SensorStatus);
        }
        // return SensorStatus;
    }

    // test for bad reading
    if (BNum == 1) // I/F Board 2
    {
        if (CNum == 0) // 3.3v
        {
            if (SensorValue->BusV[BNum][CNum] < 3.1) // set for low v
            {
                SensorStatus = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 3.5) //  set for hi v
            {
                SensorStatus = 2;
            }
            else // good sensor
            {

                SensorStatus = 0;
            }

            // Serial.printf("Test3.3vCurrent= %f Type %d ",SensorValue->ShuntImA[BNum][CNum],SensorStatus);
            // Serial.printf("Test3.3v= %f Type %d  \n",SensorValue->BusV[BNum][CNum],SensorStatus);
        }

        ////// test for 12v
        if (CNum == 1)
        {

            if (SensorValue->BusV[BNum][CNum] < 9) // set for low 12v
            {
                // SensorFailCount++;
                SensorStatus = 1;
            }
            else if (SensorValue->BusV[BNum][CNum] > 15) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorStatus = 2;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] <= 0) // set for low 12v current
            {

                // SensorFailCount++;
                SensorStatus = 1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 500) // set for hi 12v over 350ma
            {
                // SensorFailCount++;
                SensorStatus = 2;
            }
            else //////////// good 12v
            {
                // SensorFailCount = 0;
                SensorStatus = 0;
            }
            // Serial.printf("Test12vCurrent= %f Type %d ",SensorValue->ShuntImA[BNum][CNum],SensorStatus);
            // Serial.printf("Test12v= %f Type %d  \n",SensorValue->BusV[BNum][CNum],SensorStatus);
        }

        // test water level
        if (CNum == 2)
        {

            if (SensorValue->ShuntImA[BNum][CNum] < 3.5) // test for no sensor
            {

                SensorStatus = 2;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else if (SensorValue->ShuntImA[BNum][CNum] > 21.0) // test for bad sensor
            {

                SensorStatus = 1;

                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }
            else // good sensor
            {

                SensorStatus = 0;

                // pass val

                SensorValue->DepthMM = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                // temp = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }

               Serial.printf("TestwaterLevelCurrent= %.2f    TestwaterLevelV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            Serial.printf("DepthMM= %.2f  DepthIN= %.2f    SensorStatus %d  \n", SensorValue->DepthMM, SensorValue->DepthIn,SensorStatus);         
       
            Serial.println();
        }
    }
    return SensorStatus;
}

float mapf(float var, float InMin, float InMax, float OutMin, float OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}