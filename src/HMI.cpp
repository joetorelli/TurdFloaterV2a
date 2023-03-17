#include "HMI.h"
/* set all leds to off at start when all outputs turned off*/
void LEDControl(Adafruit_MCP23X17 *Expndr, int led, int type)
{

	if (type != 2)
	{
		Expndr->digitalWrite(LED_Auto_GRN_PIN, type);
	}
	else
	{
		// alarm
		Expndr->digitalWrite(LED_Auto_GRN_PIN, x);
	}
}