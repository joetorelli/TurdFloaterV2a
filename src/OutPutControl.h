
#ifndef OUTPUTCONTROL_H

#define OUTPUTCONTROL_H
#include <Arduino.h>
#define AlarmPin 12			 // Alarm
#define PumpPin 13			 //  Pump
void Alarm(bool AlarmOnOff); // alarm control auto/man & on/off
void Pump(bool PumpOnOff);	 // pump control auto/man & on/off

#endif