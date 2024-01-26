#ifndef HMI_H

#define HMI_H
#include <Arduino.h>
#include "settings.h"
#include <Adafruit_MCP23X17.h>

//  MCP23XXX pin LED is attached to
//  port GPA 0-7 pin 0-7
//  port GPB 0-7 pin 8-15


/* MCP23x17
Pin #	Pin Name	Pin ID
21	GPA0	0
22	GPA1	1
23	GPA2	2
24	GPA3	3
25	GPA4	4
26	GPA5	5
27	GPA6	6
28	GPA7	7
1	GPB0	8
2	GPB1	9
3	GPB2	10
4	GPB3	11
5	GPB4	12
6	GPB5	13
7	GPB6	14
8	GPB7	15 */

#define LED_Alarm_RED_PIN 15
#define LED_Remote_RED_PIN 14
#define LED_Auto_RED_PIN 13
#define LED_Auto_GRN_PIN 12
#define LED_WL_RED_PIN 11
#define LED_WL_GRN_PIN 10
#define LED_CL_RED_PIN 9
#define LED_CL_GRN_PIN 8

#define LED_PumpFlow_RED_PIN 0
#define LED_PumpFlow_GRN_PIN 1
#define LED_AirFlow_RED_PIN 2
#define LED_AirFlow_GRN_PIN 3
#define LED_BT_BLU_PIN 4

void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int state);

#endif