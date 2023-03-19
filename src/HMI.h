#ifndef HMI_H

#define HMI_H
#include <Arduino.h>
#include "settings.h"
#include <Adafruit_MCP23X17.h>


//  MCP23XXX pin LED is attached to
//  port GPA 0-7 pin 0-7
//  port GPB 0-7 pin 8-15

#define LED_Alarm_RED_PIN 8
#define LED_BT_BLU_PIN 5
// #define PIN 2
// #define PIN 3
// #define PIN 4
// #define PIN 5
// #define PIN 6
// #define PIN 7


#define LED_Auto_GRN_PIN 0
#define LED_Auto_RED_PIN 1
#define LED_PumpFlow_GRN_PIN 3
#define LED_PumpFlow_RED_PIN 4
#define LED_AirFlow_GRN_PIN 6
#define LED_AirFlow_RED_PIN 7
#define LED_CL_GRN_PIN 10
#define LED_CL_RED_PIN 11

// #define BUTTON_PIN 1  // MCP23XXX pin button is attached
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int led, int state);

#endif