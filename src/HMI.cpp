#include <Arduino.h>
#include "HMI.h"
extern bool BlinkState;
/* set all leds to off at start when all outputs turned off*/
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int state)
{
	Serial.println("****************** LED Control ********************");
	switch (type)
	{
	case 0: // AutoInManual

		if (state == ON)
		{
			Serial.println("********   NOT IN AUTO  ON   **********");
			Expndr->digitalWrite(LED_Auto_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_RED_PIN, BlinkState);
		}
		else
		{
			Serial.println("********  NOT IN AUTO  OFF  *********");
			Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
		}

		break;
	case 1: // LevelHi

		// if state = 3 then flash
		if (state == ON)
		{
			Serial.println("*****  level hi  ON  ********");
			// Expndr->digitalWrite(LED_Alarm_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			Serial.println("*****  level hi  OFF  ********");
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
			// Expndr->digitalWrite(LED_Alarm_GRN_PIN, OFF);
		}

		break;
	case 2: // PumpNoFlow

		if (state == ON)
		{
			Serial.println("*****  pump no flow   ON  ******");
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, BlinkState);
		}
		else
		{
			Serial.println("****  pump no flow   OFF  *****");
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, OFF);
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, ON);
		}

		break;
	case 3: // AirNoFlow

		if (state == ON)
		{
			Serial.println("****  air no flow   ON  ****");
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, BlinkState);
		}
		else
		{
			Serial.println("****  air no flow   OFF  ****");
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, OFF);
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, ON);
		}

		break;
	case 4: // CLLow

		if (state == ON)
		{
			Serial.println("*****  cl low   ON  ****");
			Expndr->digitalWrite(LED_CL_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_CL_RED_PIN, ON);
		}
		else
		{
			Serial.println("****  cl low   OFF  *****");
			Expndr->digitalWrite(LED_CL_RED_PIN, OFF);
			Expndr->digitalWrite(LED_CL_GRN_PIN, ON);
		}

		break;
	case 5: // Bluetooth

		if (state == ON)
		{
			Serial.println("****  BT   ON  ****");
			Expndr->digitalWrite(LED_BT_BLU_PIN, BlinkState);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_RED_PIN, ON);
		}
		else
		{
			Serial.println("***  BT   OFF  ****");
			Expndr->digitalWrite(LED_BT_BLU_PIN, OFF);
			Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
			Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
		}

		break;
	case 6: // Wireless
		Serial.println("******************Wireless********************");
		// if (state == ON)
		// {

		// 	Expndr->digitalWrite(LED_BT_GRN_PIN, BlinkState);
		// 	Expndr->digitalWrite(LED_Auto_GRN_PIN, OFF);
		// 	Expndr->digitalWrite(LED_Auto_RED_PIN, ON);
		// }
		// else
		// {
		// 	Expndr->digitalWrite(LED_BT_GRN_PIN, OFF);
		// 	Expndr->digitalWrite(LED_Auto_GRN_PIN, ON);
		// 	Expndr->digitalWrite(LED_Auto_RED_PIN, OFF);
		// }

		break;

		// default:
	}
}