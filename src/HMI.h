#include <Adafruit_MCP23X17.h>

// MCP23XXX pin LED is attached to
// port GPA pin 0-7
#define LED_Alarm_RED_PIN 10
#define LED_BT_BLU_PIN 11
// #define PIN 12
// #define PIN 13
// #define PIN 14
// #define PIN 15
// #define PIN 16
// #define PIN 17

// port GPB pin 0-7
#define LED_Auto_GRN_PIN 0
#define LED_Auto_RED_PIN 1
#define LED_Pump_GRN_PIN 2
#define LED_Pump_RED_PIN 3
#define LED_AirFlow_GRN_PIN 4
#define LED_AirFlow_RED_PIN 5
#define LED_CL_GRN_PIN 6
#define LED_CL_RED_PIN 7

// #define BUTTON_PIN 1  // MCP23XXX pin button is attached
void LEDControl(Adafruit_MCP23X17 *Expndr, int type, int led, int state);