#include <Arduino.h>
#include "HMI.h"
extern bool BlinkState;

//  LED_Alarm_RED_PIN 15
//  LED_Remote_RED_PIN 14
// LED_Auto_RED_PIN 13
//  LED_Auto_GRN_PIN 12
//  LED_WL_RED_PIN 11
//  LED_WL_GRN_PIN 10
//  LED_CL_RED_PIN 9
// LED_CL_GRN_PIN 8

//  LED_PumpFlow_RED_PIN 0
//  LED_PumpFlow_GRN_PIN 1
//  LED_AirFlow_RED_PIN 2
//  LED_AirFlow_GRN_PIN 3
//  LED_BT_BLU_PIN 4

/* set all leds to off at start when all outputs turned off*/
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int state)
{
	//Serial.println("****************** LED Control ********************");
	switch (type)
	{

	case 0: // AutoInManual

		if (state == ON)
		{ 
			//Serial.println("********   Man Flash Auto Yellow   **********");
			// flash yellow
			Expndr->digitalWrite(LED_Auto_GRN_PIN, BlinkState);
			Expndr->digitalWrite(LED_Auto_RED_PIN, BlinkState);
		}
		else
		{
			Serial.println("********  Auto Green  *********");
			Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
			Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
		}

		break;

	case 1: // LevelHi

		if (state == ON)
		{
			//Serial.println("*****  level hi  ON  ********");
			Expndr->digitalWrite(LED_WL_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_WL_RED_PIN, ON);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			//Serial.println("*****  level hi  OFF  ********");
			Expndr->digitalWrite(LED_WL_GRN_PIN, ON);
			Expndr->digitalWrite(LED_WL_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
			// Expndr->digitalWrite(LED_Alarm_GRN_PIN, OFF);
		}

		break;

	case 2: // PumpFlowFault

		if (state == ON)
		{
			//Serial.println("*****  pump no flow   ON  ******");
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, ON);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			//Serial.println("****  pump no flow   OFF  *****");
			//Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
		}

		break;

	case 3: // PumpFlow

		if (state == ON)
		{
			//Serial.println("*****  pump flow   ON  ******");
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, ON);
			//Expndr->digitalWrite(LED_PumpFlow_RED_PIN, OFF);
		}
		else
		{
			//Serial.println("****  pump flow   OFF  *****");
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, OFF);
			//Expndr->digitalWrite(LED_PumpFlow_RED_PIN, OFF);
		}

		break;

	case 4: // AirFlowFault

		if (state == ON)
		{
			//Serial.println("****  air no flow   ON  ****");
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, ON);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			//Serial.println("****  air no flow   OFF  ****");
			//Expndr->digitalWrite(LED_AirFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
		}

		break;

	case 5: // AirFlow

		if (state == ON)
		{
			//Serial.println("****  air flow   ON  ****");
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, ON);
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, OFF);
		}
		else
		{
			//Serial.println("****  air flow   OFF  ****");
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, ON);
		}

		break;

	case 6: // CL

		if (state == ON)
		{
			//Serial.println("*****  cl low   ON  ****");
			Expndr->digitalWrite(LED_CL_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_CL_RED_PIN, ON);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			//Serial.println("****  cl low   OFF  *****");
			Expndr->digitalWrite(LED_CL_GRN_PIN, ON);
			Expndr->digitalWrite(LED_CL_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
		}

		break;

	case 7: // Bluetooth

		if (state == ON)
		{
			//Serial.println("****  BT   ON  ****");
			Expndr->digitalWrite(LED_BT_BLU_PIN, ON);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_RED_PIN, BlinkState);
		}
		else
		{
			//Serial.println("***  BT   OFF  ****");
			Expndr->digitalWrite(LED_BT_BLU_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
			Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
		}

		break;

	case 8: // Wireless
		//Serial.println("******************Wireless********************");
		if (state == ON)
		{

			Expndr->digitalWrite(LED_Remote_RED_PIN, ON);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_RED_PIN, BlinkState);
		}
		else
		{
			Expndr->digitalWrite(LED_Remote_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
			Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
		}

		break;

		// default:
	}
}