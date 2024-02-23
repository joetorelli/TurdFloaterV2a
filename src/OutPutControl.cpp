#include <Arduino.h>
#include "OutPutControl.h"
#include "sensor_readings.h"
#include "HMI.h"

extern struct SensorData Sensor_Level_Values;
extern bool AutoManControl;
extern int PumpOnLevel;
extern byte StatusWaterPump;
extern int PumpOffLevel;
extern bool PumpManControl;
extern int AlarmOnLevel;
extern int AlarmOffLevel;
extern bool AlarmManControl;
extern bool WFUpdateFlag;
extern byte StatusAlarm;

// Pump Control
void Pump(bool PumpOnOff)
{

	if (AutoManControl == ON) // auto
	{
		if (PumpOnOff == ON)
		{
			digitalWrite(PumpPin, ON);
			StatusWaterPump = ON;
		}

		if (PumpOnOff == OFF)
		{
			digitalWrite(PumpPin, OFF);
			StatusWaterPump = OFF;
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

// toggle pump on/off manual control
void PumpToggle()
{

	PumpManControl = !PumpManControl;
	digitalWrite(PumpPin, PumpManControl);

	if (PumpManControl == 1)
	{
		StatusWaterPump = ON;
	}
	else
	{
		StatusWaterPump = OFF;
	}
}

// toggle alram on/off manual control
void AlarmToggle()
{

	AlarmManControl = !AlarmManControl;
	digitalWrite(AlarmPin, AlarmManControl);
}
