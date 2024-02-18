
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
    int CLValue = 0;
    // values for WaterFlow sensor
    int WFValue = 0;
    */

    SensorValue->ShuntImA[BNum][CNum] = SensorIFBoard->getCurrent_mA(CNum + 1) * 1000;
    SensorValue->BusV[BNum][CNum] = SensorIFBoard->getBusVoltage_V(CNum + 1);
    // test print
    // Serial.println();
    // Serial.printf("********************* Read Sensor Value, Board= %d  Chan= %d \n", BNum, CNum);
    // Serial.printf("Bus V= %.2f  BusI= %.2f \n", SensorValue->BusV[BNum][CNum], SensorValue->ShuntImA[BNum][CNum]);
    // Serial.printf("DepthIn= %d  DepthMM= %d  AP_PSI= %.2f  CLValue= %d  WFValue= %d \n", SensorValue->DepthIn, SensorValue->DepthMM, SensorValue->pressure_PSI, SensorValue->CLValue, SensorValue->WFValue);

    // Serial.println();

    //////////////////////////////////////////////////// needs update

    if (BNum == 0) // I/F Board 1
    {
        if (CNum == 0) // water flow switch
        {

            // no sensor
            if (SensorValue->ShuntImA[BNum][CNum] < 1)
            {

                SensorValue->WFValue = -1;
                SensorStatus = 2;
            }

            // good value
            else if (SensorValue->ShuntImA[BNum][CNum] >= 1 and SensorValue->ShuntImA[BNum][CNum] <= 29)
            {
                // switch open
                if (SensorValue->ShuntImA[BNum][CNum] < 15.0)
                {
                    SensorValue->WFValue = 0;
                    SensorStatus = 0;
                }
                else // switch closed
                {
                    SensorValue->WFValue = 1;
                    SensorStatus = 0;
                }
            }

            // bad sensor
            else
            {

                SensorValue->WFValue = 255;
                SensorStatus = 1;
            }

            /*             // read the current
                        int temp = SensorValue->ShuntImA[BNum][CNum];

                        switch (temp)
                        {
                        case 0 ... 1:
                            // no sensor
                            SensorValue->WFValue = SensorValue->ShuntImA[BNum][CNum];
                            SensorStatus = 2;

                            break;
                        case 2 ... 29:
                            // good value
                            if (SensorValue->ShuntImA[BNum][CNum] < 15) // switch open
                            {
                                SensorValue->WFValue = 0;
                                SensorStatus = 0;
                            }
                            else // closed
                            {
                                SensorValue->WFValue = 1;
                                SensorStatus = 0;
                            }

                            break;
                        default:
                            // bad sensor
                            SensorValue->WFValue = SensorValue->ShuntImA[BNum][CNum];
                            SensorStatus = 1;
                        } */

            // Serial.printf("TestWFSwitchCurrent= %.2f    TestWFSwitchV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            // Serial.printf("WFValue= %d   SensorStatus= %d \n", SensorValue->WFValue, SensorStatus);
        }

        if (CNum == 1) // CL Switch
        {
            // no sensor
            if (SensorValue->ShuntImA[BNum][CNum] <= 1.4)
            {
                SensorValue->CLValue = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 2;
            }
            
            // good value
            else if (SensorValue->ShuntImA[BNum][CNum] >= 1.5 and SensorValue->ShuntImA[BNum][CNum] <= 29)
            {
                // switch open
                if (SensorValue->ShuntImA[BNum][CNum] < 15.0) 
                {
                    SensorValue->CLValue = 0;
                    SensorStatus = 0;
                }
                // closed
                else 
                {
                    SensorValue->CLValue = 1;
                    SensorStatus = 0;
                }
            }
            
            // bad sensor
            else
            {
                SensorValue->CLValue = SensorValue->ShuntImA[BNum][CNum];
                SensorStatus = 1;
            }

            // read the current
            // int temp = SensorValue->ShuntImA[BNum][CNum];
            /*
                        switch (temp)
                        {
                        case 0 ... 1:
                            // no sensor
                            SensorValue->CLValue = SensorValue->ShuntImA[BNum][CNum];
                            SensorStatus = 2;

                            break;
                        case 2 ... 29:
                            // good value
                            if (SensorValue->ShuntImA[BNum][CNum] < 15) // switch open
                            {
                                SensorValue->CLValue = 0;
                                SensorStatus = 0;
                            }
                            else // closed
                            {
                                SensorValue->CLValue = 1;
                                SensorStatus = 0;
                            }

                            break;
                        default:
                            // bad sensor
                            SensorValue->CLValue = SensorValue->ShuntImA[BNum][CNum];
                            SensorStatus = 1;
                        } */
            // Serial.printf("TestCLSwitchCurrent= %.2f    TestCLSwitchV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            // Serial.printf("CLValue= %d    SensorStatus %d  \n", SensorValue->CLValue, SensorStatus);

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
            //   no sensor
            if (SensorValue->BusV[BNum][CNum] < 1) 
            {
                SensorValue->pressure_PSI = 0;
                SensorStatus = 2;
            }
            
            //  bad sensor  set for hi press voltage
            else if (SensorValue->BusV[BNum][CNum] > 4.5) 
            {
                SensorValue->pressure_PSI = mapf(SensorValue->BusV[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorStatus = 1;
            }
            
            // good range voltage
            else 
            {
                SensorValue->pressure_PSI = mapf(SensorValue->BusV[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorStatus = 0;
            }

            // Serial.printf("TestAirFlowCurrent= %.2f    TestAirFlowV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            // Serial.printf("pressure_PSI= %.2f    SensorStatus %d  \n", SensorValue->pressure_PSI, SensorStatus);
        }

        //return SensorStatus;
    }

    // test for bad reading
    if (BNum == 1) // I/F Board 2
    {
        // 3.3v
        if (CNum == 0) 
        {
            // no sensor  set for low v
            if (SensorValue->BusV[BNum][CNum] < 3.1) 
            {
                SensorStatus = 1;
            }
            
            //  bad sensor   set for hi v
            else if (SensorValue->BusV[BNum][CNum] > 3.5) 
            {
                SensorStatus = 2;
            }
            
            // good sensor
            else 
            {
                SensorStatus = 0;
            }

            // Serial.printf("Test3.3vCurrent= %f Type %d ",SensorValue->ShuntImA[BNum][CNum],SensorStatus);
            // Serial.printf("Test3.3v= %f Type %d  \n",SensorValue->BusV[BNum][CNum],SensorStatus);
        }

        ////// test for 12v
        if (CNum == 1)
        {
            // set for low 12v
            if (SensorValue->BusV[BNum][CNum] < 9) 
            {
                // SensorFailCount++;
                SensorStatus = 1;
            }
            
            // set for hi 12v
            else if (SensorValue->BusV[BNum][CNum] > 15) 
            {
                // SensorFailCount++;
                SensorStatus = 2;
            }
            
            // set for low 12v current
            else if (SensorValue->ShuntImA[BNum][CNum] <= 0) 
            {

                // SensorFailCount++;
                SensorStatus = 1;
            }
            
            // set for hi 12v over 350ma
            else if (SensorValue->ShuntImA[BNum][CNum] > 500) 
            {
                // SensorFailCount++;
                SensorStatus = 2;
            }
            
            //////////// good 12v
            else 
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
            // test for no sensor
            if (SensorValue->ShuntImA[BNum][CNum] < 3.5) 
            {
                SensorStatus = 2;
                // pass bad val
                SensorValue->DepthMM = 255;
                SensorValue->DepthIn = 255;
            }

            // test for bad sensor
            else if (SensorValue->ShuntImA[BNum][CNum] > 21.0) 
            {
                SensorStatus = 1;
                // pass bad val
                SensorValue->DepthMM = -1;
                SensorValue->DepthIn = -1;
            }

            // good sensor
            else 
            {
                SensorStatus = 0;
                // pass val
                SensorValue->DepthMM = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                // temp = mapf(SensorValue->ShuntImA[BNum][CNum], in_min[BNum][CNum], in_max[BNum][CNum], out_min[BNum][CNum], out_max[BNum][CNum]);
                SensorValue->DepthIn = SensorValue->DepthMM / 25.4;
            }

            // Serial.printf("TestwaterLevelCurrent= %.2f    TestwaterLevelV= %.2f \n", SensorValue->ShuntImA[BNum][CNum], SensorValue->BusV[BNum][CNum]);
            // Serial.printf("DepthMM= %.2f  DepthIN= %.2f    SensorStatus %d  \n", SensorValue->DepthMM, SensorValue->DepthIn, SensorStatus);

            // Serial.println();
        }
    }
    
    return SensorStatus;
}

float mapf(float var, float InMin, float InMax, float OutMin, float OutMax)
{
    return (var - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}