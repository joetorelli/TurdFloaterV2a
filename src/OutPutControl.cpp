#include <Arduino.h>
#include "OutPutControl.h"
#include "sensor_readings.h"

extern struct SensorData Sensor_Level_Values;
extern bool AutoManControl;
extern int PumpOnLevel;
extern byte StatusWaterPump;
extern int PumpOffLevel;
extern bool PumpManControl;
extern int AlarmOnLevel;
extern int AlarmOffLevel;
extern bool AlarmManControl;
extern byte StatusAlarm;

// Pump Control
void Pump(bool PumpOnOff)
{

	if (AutoManControl == ON) // auto
	{
		if (Sensor_Level_Values.DepthMM >= PumpOnLevel)
		{
			digitalWrite(PumpPin, ON);
			StatusWaterPump = ON;
			// Serial.println(" AutoPumpStatusON ");
			//  DEBUGPRINTLN(StatusWaterPump);
			// CLPumpRunOnce = ON;
			// delay(500);
			// StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
			// if (StatusWaterFlowSensor == OFF)
			// {
			//   TestWaterFlowSensor();
			// }
		}

		if (Sensor_Level_Values.DepthMM <= PumpOffLevel)
		{
			digitalWrite(PumpPin, OFF);
			StatusWaterPump = OFF;
			// //      delay(500);
			// StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
			// DEBUGPRINT(" AutoPumpStatusOFF ");
			//  DEBUGPRINTLN(StatusWaterPump);
		}
	}

	else // manual control
	{	 ///////////////////////////////////////////////////// maybe changes this to PumpToggle

		if (PumpManControl == ON)
		{
			digitalWrite(PumpPin, ON);
			StatusWaterPump = ON;
			// Serial.println("ManPumpStatusON ");
			//   DEBUGPRINTLN(StatusWaterPump);
			// delay(500);
			// StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
			// if (StatusWaterFlowSensor == OFF)
			// {
			//   TestWaterFlowSensor();
			// }
		}

		else
		{
			digitalWrite(PumpPin, OFF);
			StatusWaterPump = OFF;
			// Serial.println("ManPumpStatusOFF ");

			//   DEBUGPRINTLN(StatusWaterPump);
			// delay(500);
			// StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
			// if (StatusWaterFlowSensor == ON)
			// {
			//   TestWaterFlowSensor();
			// }
		}
	}
}

// Alaram Control
void Alarm(bool AlarmOnOff)
{

	if (AutoManControl == ON)
	{
		if (Sensor_Level_Values.DepthMM >= AlarmOnLevel)
		{
			digitalWrite(AlarmPin, ON);
			StatusAlarm = ON;
			//************************** led=LED_PumpFlow_RED_PIN, flashstate=flash,*

			// DEBUGPRINT(" AutoAlarmStatusON ");
			// DEBUGPRINTLN(StatusAlarm);
		}

		if (Sensor_Level_Values.DepthMM <= AlarmOffLevel)
		{
			digitalWrite(AlarmPin, OFF); // to relay board
			StatusAlarm = OFF;
			//************************** led=LED_PumpFlow_GRN_PIN, flashstate=on,*
			// DEBUGPRINT(" AutoAlarmStatusOFF ");
			// DEBUGPRINTLN(StatusAlarm);
		}
	}

	else // manual control
	{
		/*run timer for 1 hr the set alarm to remind to go to back to auto*/
		/************************** led=autogrn, flashstate=on,*/
		/************************** clearedled=autogrn, flashstate=on,*/

		if (AlarmManControl == ON)
		{
			digitalWrite(AlarmPin, ON);
			StatusAlarm = ON;
			// DEBUGPRINT(" ManAlarmStatusON ");
			//   DEBUGPRINTLN(StatusAlarm);
		}
		else
		{
			digitalWrite(AlarmPin, OFF);
			StatusAlarm = OFF;
			// DEBUGPRINT(" ManAlarmStatusOFF ");
			//   DEBUGPRINTLN(StatusAlarm);
		}
	}
}

// toggle pump on/off
void PumpToggle()
{

	PumpManControl = !PumpManControl;
	digitalWrite(PumpPin, PumpManControl);
}

// toggle clpump on/off
/* void CLPumpToggle()
{

  CLPumpManFlag = !CLPumpManFlag;
  digitalWrite(CLPumpPin, CLPumpManFlag);
} */

// toggle alram on/off
void AlarmToggle()
{

	AlarmManControl = !AlarmManControl;
	digitalWrite(AlarmPin, AlarmManControl);
}
