#include "HMI.h"
/* set all leds to off at start when all outputs turned off*/
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int led, int state)
{

	switch (type)
	{
	case 0: // no alarm
		Expndr->digitalWrite(LED_Auto_RED_PIN, LOW);
		Expndr->digitalWrite(LED_Pump_RED_PIN, LOW);
		Expndr->digitalWrite(LED_AirFlow_RED_PIN, LOW);
		Expndr->digitalWrite(LED_CL_RED_PIN, LOW);
		Expndr->digitalWrite(LED_Alarm_RED_PIN, LOW);
		Expndr->digitalWrite(led, state);
		break;
	case 1: // LevelHi
		// if state = 3 then flash
		Expndr->digitalWrite(LED_Alarm_RED_PIN, HIGH);

		break;
	case 2: // PumpNoFlow
		Expndr->digitalWrite(LED_Pump_RED_PIN, HIGH);
		Expndr->digitalWrite(LED_Pump_GRN_PIN, LOW);

		break;
	case 3: // AirNoFlow
		Expndr->digitalWrite(LED_AirFlow_RED_PIN, HIGH);
		Expndr->digitalWrite(LED_AirFlow_GRN_PIN, LOW);

		break;
	case 4: // CLLow
		Expndr->digitalWrite(LED_CL_RED_PIN, HIGH);

		break;

	default:
	}

	if (type != 2)
	{
		Expndr->digitalWrite(led, type);
	}
	else
	{
		// alarm
		// Expndr->digitalWrite(LED_Auto_GRN_PIN, x);
	}
}