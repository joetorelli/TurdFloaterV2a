#include <Arduino.h>
#include "HMI.h"
extern bool BlinkState;
/* set all leds to off at start when all outputs turned off*/
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int led, int state)
{

	switch (type)
	{
	case 0: // no alarm
		Expndr->digitalWrite(LED_Auto_RED_PIN, LOW);
		Expndr->digitalWrite(LED_PumpFlow_RED_PIN, LOW);
		Expndr->digitalWrite(LED_AirFlow_RED_PIN, LOW);
		Expndr->digitalWrite(LED_CL_RED_PIN, LOW);
		Expndr->digitalWrite(LED_Alarm_RED_PIN, LOW);
		Expndr->digitalWrite(led, state);
		break;
	case 1: // LevelHi
			// if state = 3 then flash
		if (state == ON)
		{
			Expndr->digitalWrite(LED_Alarm_RED_PIN, BlinkState);
		}
		else
		{
			Expndr->digitalWrite(LED_Alarm_RED_PIN, OFF);
		}

		break;
	case 2: // PumpNoFlow
		if (state == ON)
		{
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, BlinkState);
			Expndr->digitalWrite(LED_PumpFlow_GRN_PIN, LOW);
		}
		else
		{
			Expndr->digitalWrite(LED_PumpFlow_RED_PIN, OFF);
		}

		break;
	case 3: // AirNoFlow
		if (state == ON)
		{
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, BlinkState);
			Expndr->digitalWrite(LED_AirFlow_GRN_PIN, LOW);
		}
		else
		{
			Expndr->digitalWrite(LED_AirFlow_RED_PIN, OFF);
		}

		break;
	case 4: // CLLow
		if (state == ON)
		{
			Expndr->digitalWrite(LED_CL_RED_PIN, BlinkState);
		}
		else
		{
			Expndr->digitalWrite(LED_CL_RED_PIN, OFF);
		}

		break;

		// default:
	}
}