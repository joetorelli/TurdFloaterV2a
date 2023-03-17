/******************************************************************
 Created - 11/2022
 Project     :  TurdFloaterFeather32
 Libraries   :
 Author      : JoeT
 Description : read level, control pump, sets alarm, BT and control panel  UI
 Version     : 1a
                changed AlarmLevel to AlarmOnLevel. changed value from 50 to 40
                added AlarmOffLevel. This should give a spread to the on and off
               1b
                adding blue tooth app "Bluetooth Electronics" www.keuwl.com
                2
                changed to feather32
                adding bluetooth - tried bt extern module but didn't work on 3.3v
                tried using internal bluetooth and consumed 50% memory
                got hc-05 BT mod working using it on Serial1. connect to 5v and logic still 3.3v
                3
                chaged max levels to 4096
                All status leds working
                level guage working
                all sliders working
                all switches working
                  when switching to man mode turn off pumps  -  done
                    show all sliders and switches and gone in auto - done
                changed BT module data

                b log data from tank done
                c add switches stopped reboot on wifi/ntp fail
                d  started menu
                d_1 same as _d with out buttons
                d_2 changing to ina3221
                  done - Adding OnBoard Averaging and Convertion Times
                  done - channel en/disable
                  * * got a 3chan board and changed the resistors to
                  8.20 * 20ma = 164mv
                  7.50 * 20ma = 150mv
                  6.19 * 20ma = 124mv
                E moved to ina3221
                  had conflict with bme and ina3221 - removed bme for now
                  ready for logging
                F intergrate parts and files
                  only using only chan 2 for now. Change readlevel sensor in sensorreadings.cpp
                  reinvented the wheel and reassigned the pins nums; can go to better pinout with external pull ups on 34,36,39
                  renamed a bunch of vars to make more sense, cleaned up
                  Back to all swithches working, BT working and Pump/CLPump/Alarm pins working
                  App screens working good
                  disable SSW while in BT
                  moving over to ticker - causing issues
                    when ticker calls, all the callbacks just set a flag, then loop reads the flags
                  this version seems stable, need to stop serial.print's
                G moved to git hub
                  BT =  has OLED Display and SSW inop


            Need to fix
              done - Pump LED flashes from man to auto
              done - add moving average -
              done - Store pump settings data in eeprom
              done - wifi
              done - oled
              done - rtc  use for time stamp on logger may not be used later
              done - ntp  use set time on rtc may not be used later
              done - bme
              done - analog sensor with 4-20 board
              done - sd card   use for storage on logger may not be used later
              done - added load voltage
              done - added map function
              done - Adjust range to inches/mm
              done - oled on/off control in software disabled for now in DisplayOff
              done - get minimum pump run level for auto and manual
              done - need to add timer to CLPump currently on delay() = Ticker
              done - test Pump/Alarm ON > Pump/Alarm Off
              done - show oled when connected to BT
              done - moved to ssd1327 1.5" 128x128 oled display
              done - added CLPump on/off
              done - adding diags to menu start, and monitor during run time
              done - working, need to add mprls pressure sensor to read air pump, menu's good
              done - add mag sw to check for CL tablet level and not use cl pump
                      need to check BT because removed CL Pump
              done - add water flow sw/cl sw status to off menu

              inwork -
                  add alarm flags and alarm timer beep
                  add air flow read/setting to pump menu

                  splitting files
                    HMI = OLED, LEDs
                    Actuator = Pump, Alarm
                    Menu = menusystem
                    Sensor = Water,Air Press, CL
                    INA3221 = Lib
                    Switches = SSW control
                    BlueTooth = BT COntrol, Screens
                    WIFICont = WIFI Control, WEB Pages

                  menu system setup


              open -
                  add air pressure level adj to warning trigger
                  looking into web page, will move to esp32v2
                  add alarm if not in auto after some time

                  Connect to ESPHome
                  Vol adj to PWM for alarm works on screen


              I2C Adr

                RTC PCF8523 - read 0xd1; write 0xd0
                MPRLS - 0x18

                OLED - 0X3C  change to 128x64 display************
                 solder bs from 0 to 1 for i2c
                * The connections when using I2C:
                    VCC->VCC
                    GND->GND
                    DIN ->SDA
                    CLK ->SCL
                    CS: Don't connect to anything
                    DC: Sets the I2C address. Connect to VCC to set address to 0x3D, and connect it to GND to set it to 0x3C.
                    RST: I didn't connect it to anything, and in software, assigned it to -1 in the constructor:
                    display = new Adafruit_SSD1327(128, 128, &Wire, -1, 1000000);

                ina3221/INA219 0x40,41
                * //Explanation of I2C address for INA3221:
                *   //INA3221_ADDR40_GND = 0b1000000, // A0 pin -> GND
                *   //INA3221_ADDR41_VCC = 0b1000001, // A0 pin -> VCC
                *   //INA3221_ADDR42_SDA = 0b1000010, // A0 pin -> SDA
                *   //INA3221_ADDR43_SCL = 0b1000011  // A0 pin -> SCL
                * * got a 3chan board and changed the resistors to
                * 8.20 * 20ma = 164mv
                * 7.50 * 20ma = 150mv
                * 6.19 * 20ma = 124mv

                BME 0x76 not used
                SRF = 0x70 not used

              Define all pins
              I2C             23 I2c_SDA
                              22 I2c_SCL

              SD_Card - SPI
                              5 SCK
                              18 MOSI
                              19 MISO
                              33 SD_CS
              Inputs -
                              //34 SensorPin analog
                              26,25,34,39  rotory select sw             //15,32,14,21
                              14 encoder sw
                              15,32 enca, encb
                              36 blue tooth connect from bt board
                              21 cl switch
                              4 waterflow

              Outputs -
                              12 AlarmPin
                              13 PumpPin
                              //27 CL Pump    // currently being used for toggle pin during SD_Update
              Bluetooth -
                              17 tx
                              16 rx
                              Name: TurdFloater
                              PSWD: 1234
                              Baud:19200,8,n,1
              debugger
                              GPIO12 — TDI
                              GPIO15 — TDO
                              GPIO13 — TCK
                              GPIO14 — TMS

              Relays: Pump, CL Pump
              Leds: Alarm, Pump, CL Pump, BTStatus
              switches: BT Connect(reset?), rotate knob auto/man,alarm,pump
12v out = 100ma on start, 250ma with relay on
******************************************************************/

/**********************************************
  Includes
**********************************************/
#include <Arduino.h>
#include <Preferences.h> //NVM
#include <WiFi.h>
#include <Ticker.h> //non blocking timer

#include "Wire.h"
#include "network_config.h"

#include "settings.h"        // The order is important! for nvm
#include "sensor_readings.h" // The order is important!

#include "time.h"
#include "RTClib.h"

#include "OLED.h"
#include "SD_Card.h"
#include "Simple_Menu.h"
#include "Button2.h"
#include "AiEsp32RotaryEncoder.h"

// #include <driver/adc.h> //adc
// #include "INA3221.h"   // included in sensor_READINGS.H

/**********************************************
  Pin Definitions
**********************************************/
/********************************* changed pin definition on ver F
 * stopped using analog and allowed for better connections  ****************/
#define AlarmPin 12 // Alarm
#define PumpPin 13  //  Pump
// #define CLPumpPin 27 //  Chlorine Pump
//  #define SensorPin 34 ////////////////// Sensor

#define SD_CS 33 // SD Card

#define BTStatusPin 36 // bluetooth

// assign i2c pin numbers
#define I2c_SDA 23
#define I2c_SCL 22

// NVM Mode - EEPROM on ESP32
#define RW_MODE false
#define RO_MODE true

/*********************** button2  **********************/
// SSW SelectSWitch
#define SSWAutoPin 39  // 21 // auto/man pos
#define SSWAlarmPin 34 // 4 // alarm pos
#define SSWOffPin 25   // off pos
#define SSWPumpPin 26  // pump pos

#define SWEncoderPin 14 // enc push button

#define CLLevelSW 21
#define WaterFlowSW 4

// Rotary Encoder
#define ROTARY_ENCODER_A_PIN 32
#define ROTARY_ENCODER_B_PIN 15
#define ROTARY_ENCODER_BUTTON_PIN SWEncoderPin // button handled by Button2
#define ROTARY_ENCODER_VCC_PIN -1              /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */
#define ROTARY_ENCODER_STEPS 10

/**********************************************
  Global Vars
**********************************************/

const byte ON = 1;
const byte OFF = 0;

int AlarmOnLevel = 0;  // value for AlarmLevel to ON
int AlarmOffLevel = 0; // value for AlarmLevel to OFF
int PumpOnLevel = 0;   // value for PumpOnLevel
int PumpOffLevel = 0;  // value for PumpOffLevel
int AlarmVol = 0;

// status flags
byte StatusWaterPump = OFF; // Pump On/Off
byte PumpManFlag = OFF;     // pump sw state
byte StatusAlarm = OFF;     // Alarm On/Off
byte AlarmManFlag = OFF;    // Alarm sw state
byte AutoManControl = ON;   // auto/manual sw state
// byte CLPumpStatus = OFF;  // CLPump On/Off
// byte CLPumpManFlag = OFF; // CLpump sw state
// byte CLPumpRunOnce = OFF; // run CLPump after Pump stops

/*************************** BT APP Vars ********************/
int BTStatusFlag = OFF;
char data_in; // data received from Serial1 link BT

// app flags
byte PumpManFl = OFF;
byte AlarmManFl = OFF;
byte AMSwitchFl = OFF;
// byte CLPumpManFl = OFF;

// sliders
int PumpOnLevelSliderValue;
int PumpOffLevelSliderValue;
int AlarmOnLevelSliderValue;
int AlarmOffLevelSliderValue;
int CLTimerSliderValue;
int AlarmVolSliderValue;

// digits
int PumpOnLevelDisplayValue;
int PumpOffLevelDisplayValue;
int AlarmOnLevelDisplayValue;
int AlarmOffLevelDisplayValue;
// int CLTimerDisplayValue;
int AlarmVolDisplayValue;
boolean DisplayState = ON;

// misc
String text;          // String for text elements
int red, green, blue; // RGB color
int bubbles;          // Bubble Gauge Value

int PumpPlotVal = 0;
int AlarmPlotVal = 0;
int CLPlotVal = 0;

// FLAGS
int Page1Once = 1;
int Page2Once = 0;

/************************ timer vars ************************/
Ticker SDTimer;         // how often to write to SD Card
Ticker APPTimer;        // how often to Update BT App
Ticker SensorTimer;     // how often to Read Sensor
Ticker DisPlayTimer;    // how often to update OLED
Ticker DisplayOffTimer; // when to blank display
Ticker ReadWaterSWTimer;
// Ticker CLPumpTimer;     // how long to run CLPump

//// timer intervals
float SD_interval = 600;            // sec for updating file on sd card
unsigned int APP_interval = 500;    // ms for updating BT panel
unsigned int Sensor_interval = 500; // ms for sensor reading
unsigned int DISP_interval = 250;   // ms for oled disp data update
float DISP_TimeOut = 150;           // sec how long before blank screen
// float CLPump_RunTime = 5;           // sec for CL Pump to run

/**************************** Switches ****************************/
struct Select_SW Switch_State; // switch position
byte SWEncoderFlag = OFF;      // encoder push button
byte SSWMode = 1;              // position of select switch and encoder

/****************** misc vars ********************/
double Count = 0;            // used for log save count
boolean SDConnectOK = ON;    // SD card valid
boolean WiFiConnected = OFF; // WIFI Connected
byte SetUpFlag = 0;          ///////////////// oled menu inwork
// int ChanNum = 0;

/******************* eeprom ******************/
Preferences Settings; // NVM

/**********************************************************************
*******************  Sub/Function Declarations
**********************************************************************/

void WriteData(void); // save to eprom

void Alarm(void); // alarm control auto/man & on/off
void Pump(void);  // pump control auto/man & on/off
// void CLPump(void);    // CLpump control on sets timer for off
// void CLPumpOFF(void); // CLPump off

void BuildPanel(void); // builds app panels on phone

void DisplayData(void); // send serial data debug
// DateTime OLEDClock = rtc.now();

void SystemSetUp(void); /////////////////// oled menu

/********  ticker timers callback functions  *********/
void DisplayUpdate(void);        // update oled data
void DisplayUpdateSetFlag(void); // set flag to run display update
boolean DisplayUpdateFlag = ON;  // update flag

void DisplayOff(void);        // blanks disp
void DisplayOffSetFlag();     // set flag to run displayoff
boolean DisplayOffFlag = OFF; // update flag

void DisplayOn(void); // update disp on start blank timer

void SD_Update();           // write to sd file
void SD_UpdateSetFlag();    // set flag to run SD update
boolean SDUpdateFlag = OFF; // update flag

// void SensorRead();            // read sensor value
void SensorReadSetFlag();     // set flag to run sd update
boolean SensorReadFlag = OFF; // update flag

void SendAppData();            // send data to app
void SendAppDataSetFlag();     // set flag to run app update
boolean SendAppDataFlag = OFF; // update flag

/*************************************************************************
 ********************** Init Hardware
 ************************************************************************/

/*******************   oled display   **************/
// Declaration for an SSD1306 OLED_Display connected to I2C (SDA, SCL pins)
// Adafruit_SSD1306 OLED_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// 1000000=i2c clk during ssd,  1000000=i2c clk after ssd
Adafruit_SSD1327 OLED_Display(128, 128, &Wire, OLED_RESET, 1000000);

/*******************  rtc  *************************/
RTC_PCF8523 rtc; // on feather logger board

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -6 * 60 * 60;
const int daylightOffset_sec = 0;
struct tm timeinfo;

/**********************  ina3221  ********************/
// #include "SDL_Arduino_INA3221.h"
static const uint8_t _INA_addr = 64; //  0x40 I2C address of sdl board

// tweeked the resistor value while measuring the current (@11.5ma center of 4-20ma) with a meter. To make the numbers very close.
// with sig gen
// ina3221(address, chan1 shunt miliohm, chan2 shunt miliohm, chan3 shunt miliohm)
SDL_Arduino_INA3221 ina3221(_INA_addr, 105, 7530, 105); // 6170, 7590, 8200);
// the 7.5ohm resistor works out he best. Shunt mv=~30-150, max out of register at 163.8mv.
//  this leaves some head room for when sensor fails and goes max
//  need to add test condition for <4ma(open) and >20ma (fault)
struct LevelSensor Sensor_Level_Values;
// values for 3 chan
const int Chan1 = 0;
const int Chan2 = 1;
const int Chan3 = 2;

int StatusLevelSensor = 0;

/***********************  encoder  *********************/
// AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
// AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();
//  void rotary_onButtonClick();
//  void rotary_loop();
void rotary_loop();
int ENCValue = 0;

/********************* encoder ISR  *********************/
void IRAM_ATTR readEncoderISR()
{
  // rotaryEncoder.readEncoder_ISR();    //old
  rotaryEncoder->readEncoder_ISR();
}

/*******************  switches **************************/
// Instantiate switch
// Button2 SWEncoder;
Button2 SSWAuto;
Button2 SSWAlarm;
Button2 SSWOff;
Button2 SSWPump;

// types of switch handlers
// void handler(Button2 &btn);
// void longpress(Button2 &btn);
void pressed(Button2 &btn); // when button/sw pressed
// void released(Button2 &btn);
// void changed(Button2 &btn);
// void tap(Button2 &btn);
// byte myButtonStateHandler();
// void myTapHandler(Button2 &btn);

/***********************  air (pressure) sensor  *****************/
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
// i2c adr 0x18
#define RESET_PIN -1 // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1   // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS AirFlowSensor = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
struct AirSensor AirPump;
int StatusAirSensor = 0;
bool StatusCLSensor = OFF;
bool StatusWaterFlowSensor = OFF;
/****************************  menu code  ***************************/
menuFrame AlarmMenu; // runs when SSW in Alarm Position
menuFrame PumpMenu;  // runs when SSW in Pump Position
menuFrame TestMenu;  // runs when Program Starts

// menu call functions
void testFunct();
void TestLevelSensor();
void TestAirSensor();
void TestCLSensor();
void TestPwrSupply();
void TestWaterFlowSensor();
void PumpOnAdjust();
void PumpOffAdjust();
void AlarmOnAdjust();
void AlarmOffAdjust();
// void CLTimeAdjust();
void VolumeAdjust();
void PumpToggle();
void AlarmToggle();
// void CLPumpToggle();
void MenuChoose(int Mode);

/********  physical poistion of SSW ***********/
byte AlarmPositionFlag = OFF;
byte OffPositionFlag = OFF;
byte PumpPositionFlag = OFF;
byte AutoPositionFlag = OFF;

/******************************************************************************
 **************************************  SetUp
 *******************************************************************************/
void setup()
{

  // serial ports
  Serial1.begin(9600); // bluetooth mod   needs to be 19200
  Serial.begin(57600); // debug
  DEBUGPRINTLN("Serial 0 Start");

  /***************************** pin properties  ******************/
  // relay
  pinMode(AlarmPin, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  // pinMode(CLPumpPin, OUTPUT);

  // select sw
  pinMode(SSWAutoPin, INPUT_PULLUP);
  pinMode(SSWAlarmPin, INPUT_PULLUP);
  pinMode(SSWOffPin, INPUT_PULLUP);
  pinMode(SSWPumpPin, INPUT_PULLUP);

  // BT mod
  pinMode(BTStatusPin, INPUT_PULLUP);

  // CL Mag SW
  pinMode(CLLevelSW, INPUT);

  // Water Flow SW
  pinMode(WaterFlowSW, INPUT_PULLUP);

  //// turn off outputs
  digitalWrite(AlarmPin, OFF);
  digitalWrite(PumpPin, OFF);
  // digitalWrite(CLPumpPin, OFF);

  /********************   init i2c  *****************/
  Wire.begin(I2c_SDA, I2c_SCL);
  // bool status; // connect status
  DEBUGPRINTLN("I2C INIT OK");

  /********************* oled  ********************/
  // SSD1306_SWITCHCAPVCC = generate OLED_Display voltage from 3.3V internally
  /*   if (!OLED_Display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) // Address 0x3C for 128x32 board
    {
      DEBUGPRINTLN(F("SSD1306 allocation failed"));
      for (;;)
        ; // Don't proceed, loop forever
    }
    else
    {
      DEBUGPRINTLN("SSD1306 Init");
    } */
  /************************************ oled  ********************/
  oledSystemInit(&OLED_Display); // in Simple_Menu.cpp
  // Clear the oled buffer.
  OLED_Display.clearDisplay();
  OLED_Display.display();

  // set up parameters
  OLED_Display.setRotation(ROTATION);
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(SSD1327_WHITE);
  OLED_Display.setContrast(0x7F);

  // **********************   wifi   *********************** //
  DEBUGPRINT("Connect to SSID: ");
  DEBUGPRINTLN(WIFI_SSID);
  DEBUGPRINT("Waiting for Network:");

  OLED_Display.setCursor(0, 0);
  OLED_Display.println("TurdF v G");           // line version displ
  OLED_Display.println("Connecting to SSID:"); // line 2
  OLED_Display.println(WIFI_SSID);             // line 3
  OLED_Display.print("Waiting for Network:");
  OLED_Display.println("");
  /////////////////////////////////////////////////OLED_Display.display();

  byte count = 0; // used for network and ntp timeout

  // connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUGPRINT(".");
    count++;
    OLED_Display.print(".");
    OLED_Display.display();

    if (count > 20) // if not connected reboot
    {
      OLED_Display.clearDisplay();
      OLED_Display.display();
      OLED_Display.print("Network Time out");
      OLED_Display.display();
      delay(1000);
      break;
    }
  }

  if (count > 20) // if not connected reboot
  {
    WiFiConnected = OFF;

    // ESP.restart();
  }
  else // continue
  {
    WiFiConnected = ON;
    count = 0;
  }

  if (WiFiConnected == ON)
  {
    DEBUGPRINTLN("");
    DEBUGPRINTLN("WIFI Connected");
    DEBUGPRINT("IP ADR:");
    DEBUGPRINTLN(WiFi.localIP());

    // display connection on oled

    // OLED_Display.println(""); // line 3
    OLED_Display.println("");
    OLED_Display.print("Connected");
    OLED_Display.println("IP:");
    OLED_Display.print(WiFi.localIP());
    OLED_Display.display();
    delay(1000);
    OLED_Display.clearDisplay();
    OLED_Display.display();

    // **********************  ntp   ****************** //
    // stop up date
    //// DEBUGPRINT("Waiting for NTP:");
    //// OLED_Display.setCursor(0, 0);
    //// OLED_Display.println("Waiting for NTP:");

    // init and get time - time.h
    /*********************************************** temp cut stop update clck
     configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
     do
     {
       count++;
       DEBUGPRINT(".");
       OLED_Display.print(".");
      OLED_Display.display();

      if (count > 10) // if not connected reboot
      {
        OLED_Display.clearDisplay();
        OLED_Display.print("Time out Restarting");
        OLED_Display.display();
        delay(1000);
        break;
      }
    } while (!getLocalTime(&timeinfo));
    */
    if (count > 10) // reboot
    {
      // ESP.restart();
    }
    else // contimue
    {
      count = 0;
    }

    // print ntp time
    // debug
  }

  /*******************  rtc  **************************/
  // convert string from ntp to int for rtc

  int Year = timeinfo.tm_year + 1900;
  int Month = timeinfo.tm_mon + 1;
  int Day = timeinfo.tm_mday;
  int Hour = timeinfo.tm_hour;
  int Min = timeinfo.tm_min;
  int Sec = timeinfo.tm_sec;

  // init rtc
  if (!rtc.begin())
  {
    DEBUGPRINTLN("Couldn't find RTC");

    OLED_Display.clearDisplay();
    OLED_Display.print("Couldn't find RTC");
    OLED_Display.display();

    delay(1000);
    // ESP.restart();
  }

  else
  {
    DEBUGPRINTLN("RTC Init");
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.display();
    OLED_Display.println("RTC Init");
    delay(1000);
  }

  if (!rtc.initialized())
  {
    // update rtc with ntp time
    DEBUGPRINTLN("RTC is NOT running! - Setting Clock to NTP");
    DEBUGPRINTLN(Year);
    DEBUGPRINTLN(Month);
    DEBUGPRINTLN(Day);
    DEBUGPRINTLN(Hour);
    DEBUGPRINTLN(Min);
    DEBUGPRINTLN(Sec);

    //// temp cut/////////////////////////// rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    DEBUGPRINTLN("Clock Set");
    OLED_Display.print("RTC set to NTP");

    // update rtc with compile time
    // DEBUGPRINTLN("RTC is NOT running! - Setting Clock to Compile");
    /////////////rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // DEBUGPRINTLN("Clock Set");
    // OLED_Display.println("RTC set to Compile");
  }

  else
  {
    // update rtc with ntp time
    DEBUGPRINTLN("RTC Running - ");
    DEBUGPRINTLN(Year);
    DEBUGPRINTLN(Month);
    DEBUGPRINTLN(Day);
    DEBUGPRINTLN(Hour);
    DEBUGPRINTLN(Min);
    DEBUGPRINTLN(Sec);
    //// temp cut rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    // DEBUGPRINTLN("Clock Set");
    // OLED_Display.println("RTC set to NTP");

    // update rtc with compile time
    // DEBUGPRINTLN("RTC is NOT running! - Setting Clock to Compile");
    /////////////////rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // DEBUGPRINTLN("Clock Set");
    // OLED_Display.println("RTC set to Compile");
  }
  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();

  // debug
  /* getLocalTime(&timeinfo);
  Serial.println("TimeInfo");
  DEBUGPRINTLN(&timeinfo);
  Serial.print(&timeinfo, "%A %B %d %Y %H:%M:%S");
  Serial.print(&timeinfo, "%m %d %Y / %H:%M:%S");

  OLED_Display.println(&timeinfo, "%A %B %d");
  OLED_Display.println(&timeinfo, "%H:%M:%S");
  OLED_Display.display();
  delay(2000); */

  ///////////////////////////// OLED_Display.println(rtc.now());

  // stop asking for internet ntp time
  void setInterval(uint16_t seconds = 0);

  OLED_Display.clearDisplay();
  OLED_Display.setCursor(0, 0);
  OLED_Display.display();

  // Initialize SD card
  OLED_Display.println("Init SDCard");
  OLED_Display.display();
  delay(1000);
  DEBUGPRINTLN("Initializing SD card...");
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS))
  {
    DEBUGPRINTLN("ERROR - SD card initialization failed!");
    OLED_Display.clearDisplay();
    OLED_Display.println("SDCard initialization failed!");
    OLED_Display.display();
    SDConnectOK = OFF;
    delay(1000);
    // ESP.restart();
    // return; // init failed
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    DEBUGPRINTLN("No SD card attached");
    // OLED_Display.clearDisplay();
    OLED_Display.println("No SD card attached");
    OLED_Display.display();
    SDConnectOK = OFF;
    delay(1000);
    // ESP.restart();
    // return;
  }

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  if (SDConnectOK)
  {
    File file = SD.open("/datalog.txt");
    if (!file)
    {
      DEBUGPRINTLN("File doens't exist");
      DEBUGPRINTLN("Creating file...");

      OLED_Display.println("Creating file...!");
      OLED_Display.display();
      // SDConnectOK = true;
      writeFile(SD, "/datalog.txt", " #\t\tDate\t\tTime\t\tMM\t\tIN\t\tMA\t\tMV\t\tPump\t\tAlarm\t\tCLPump\r\n");
      delay(1000);
    }
    else
    {
      DEBUGPRINTLN("File already exists");

      OLED_Display.println("File already exists");
      OLED_Display.display();
      // SDConnectOK = true;
      // delay(1000);
    }
    file.close();
  }

  /*************************  ina3221  ************************/
  // setup ina3221 SDL lib
  ina3221.begin();
  // Serial.println("ina3221.begin");
  //  en/dis channel as needed. effects response time
  ina3221.setChannelEnable(INA3221_CH1);
  ina3221.setChannelEnable(INA3221_CH2);
  ina3221.setChannelEnable(INA3221_CH3);

  // ina3221.setChannelDisable(INA3221_CH1);
  //  ina3221.setChannelDisable(INA3221_CH2);
  // ina3221.setChannelDisable(INA3221_CH3);

  // values for avg, effects response time
  ina3221.setAveragingMode(INA3221_REG_CONF_AVG_64);
  //  Sets bus-voltage conversion time.
  ina3221.setBusConversionTime(INA3221_REG_CONF_CT_1100US);
  //  Sets shunt-voltage conversion time.
  ina3221.setShuntConversionTime(INA3221_REG_CONF_CT_1100US);

  /****************************   NVM   ************************/
  // test for first run time
  Settings.begin("storage", RO_MODE); // nvm storage space, set to read
  bool doesExist = Settings.isKey("NVSInit");
  if (doesExist == false)
  {

    Serial1.println("-----------------NVM first time----------------");
    // first time run code to create keys & assign their values
    Settings.end();                     // close the namespace in RO mode.
    Settings.begin("storage", RW_MODE); //  create and open it in RW mode.

    // load NVM with default values
    Settings.putInt("PumpOnLevel", 2000);
    Settings.putInt("PumpOffLevel", 1500);
    Settings.putInt("AlarmOnLevel", 2200);
    Settings.putInt("AlarmOffLevel", 2000);
    // Settings.putInt("CLTimer", 5);
    Settings.putInt("AlarmVol", 50);
    Settings.putInt("NVSInit", true);
    Settings.end();                     // close the namespace
    Settings.begin("storage", RO_MODE); // nvm storage space, set to read
  }

  // load NVM into vars
  PumpOnLevel = Settings.getInt("PumpOnLevel");
  PumpOffLevel = Settings.getInt("PumpOffLevel");
  AlarmOnLevel = Settings.getInt("AlarmOnLevel");
  AlarmOffLevel = Settings.getInt("AlarmOffLevel");
  // CLPump_RunTime = Settings.getInt("CLTimer");
  AlarmVol = Settings.getInt("AlarmVol");
  Settings.end(); // close the namespace

  // Load Slider values
  PumpOnLevelSliderValue = PumpOnLevel;
  PumpOffLevelSliderValue = PumpOffLevel;
  AlarmOnLevelSliderValue = AlarmOnLevel;
  AlarmOffLevelSliderValue = AlarmOffLevel;
  // CLTimerSliderValue = CLPump_RunTime;
  AlarmVolSliderValue = AlarmVol;

  // Load Display values
  PumpOnLevelDisplayValue = PumpOnLevel;
  PumpOffLevelDisplayValue = PumpOffLevel;
  AlarmOnLevelDisplayValue = AlarmOnLevel;
  AlarmOffLevelDisplayValue = AlarmOffLevel;
  // CLTimerDisplayValue = CLPump_RunTime;
  AlarmVolDisplayValue = AlarmVol;

  /************************************ encoder  *********************/
  rotaryEncoder->begin();
  rotaryEncoder->setup(readEncoderISR);

  // minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  // rotaryEncoder->setBoundaries(0, 2, false); // dosen't work good in true
  rotaryEncoder->setAcceleration(0); // use this with ROTARY_ENCODER_STEPS, acts like debouce and changes response
  rotaryEncoder->setEncoderValue(0); // enc start value

  // numberSelector.attachEncoder(rotaryEncoder);
  //  example 1
  // numberSelector.setRange(0, 100, 1, false, 1);
  // numberSelector.setValue(50);

  // encoder switch
  /*   SWEncoder.begin(SWEncoderPin, INPUT_PULLUP);
    // SWEncoder.setLongClickTime(1000);
    SWEncoder.setDebounceTime(50); */

  // SWEncoder.setChangedHandler(changed);             // trigger on press and release
  // SWEncoder.setPressedHandler(pressed); // returns if still pressed

  /******************************** rotary select sw *************************/
  SSWAuto.begin(SSWAutoPin, INPUT_PULLUP);
  SSWAuto.setDebounceTime(50);
  // SSWAuto.setClickHandler(handler);                // bad with rotary
  //  SSWAuto.setLongClickDetectedHandler(handler);   // works good always thinks long
  // SSWAuto.setChangedHandler(changed);              // give current and next position
  SSWAuto.setPressedHandler(pressed); // works good with rotary

  SSWAlarm.begin(SSWAlarmPin, INPUT_PULLUP);
  SSWAlarm.setDebounceTime(50);
  // SSWAlarm.setClickHandler(handler);
  // SSWAlarm.setLongClickDetectedHandler(handler);
  // SSWAlarm.setChangedHandler(changed);
  SSWAlarm.setPressedHandler(pressed);

  SSWOff.begin(SSWOffPin, INPUT_PULLUP);
  SSWOff.setDebounceTime(50);
  // SSWOff.setClickHandler(handler);
  // SSWOff.setLongClickDetectedHandler(handler);
  // SSWOff.setChangedHandler(changed);
  SSWOff.setPressedHandler(pressed);
  // SSWOff.setReleasedHandler(released);
  // SSWOff.setTapHandler(tap);

  SSWPump.begin(SSWPumpPin, INPUT_PULLUP);
  SSWPump.setDebounceTime(50);
  // SSWPump.setClickHandler(handler);
  // SSWPump.setLongClickDetectedHandler(handler);
  // SSWPump.setChangedHandler(changed);
  SSWPump.setPressedHandler(pressed);

  /************************** Blue Tooth ******************/
  // this will download both screens to a new user
  // connect to bluetooth then press reset on eps32
  BTStatusFlag = digitalRead(BTStatusPin);
  if (BTStatusFlag == ON)
  {
    BuildPanel();
  }

  /********************* init pressure sensor ***************/
  Serial.println("MPRLS Simple Test");
  if (!AirFlowSensor.begin())
  {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");

    delay(1000);
  }
  else
  {
    Serial.println("Found MPRLS sensor");
    delay(1000);
  }
  /***********************************************************/
  // Start the timers
  SDTimer.attach(SD_interval, SD_UpdateSetFlag);               // set flag to write to sd
  APPTimer.attach_ms(APP_interval, SendAppData);               // update app data
  SensorTimer.attach_ms(Sensor_interval, SensorReadSetFlag);   // set flag to read sensor
  DisPlayTimer.attach_ms(DISP_interval, DisplayUpdateSetFlag); // set flag to update oled data
  // DisplayOffTimer.attach(DISP_TimeOut, DisplayOff);
  //     changer.once(30, change);

  // every 25 ms, call setPin(0)
  //   tickerSetLow.attach_ms(2000, setPin, 0);
  // blinker.once(10, setPin, 3);

  /********************  menu ********************/
  // Pump Menu
  PumpMenu.addMenu("Pump mm", 0);
  // mainMenu.addNode("Pump Levels", SUB_NODE, NULL);
  //  mainMenu.linkNode(1);
  //  // Submenu 1
  //  mainMenu.addMenu("Pump mm", 1);
  PumpMenu.addNode("ON/OFF", ACT_NODE, &PumpToggle);
  PumpMenu.addNode("On Level", ACT_NODE, &PumpOnAdjust);
  PumpMenu.addNode("Off Level", ACT_NODE, &PumpOffAdjust);
  // PumpMenu.addNode("CL ON/OFF", ACT_NODE, &CLPumpToggle);
  // PumpMenu.addNode("CL Time", ACT_NODE, &CLTimeAdjust);

  // Alarm Menu
  AlarmMenu.addMenu("Alarm", 0);
  AlarmMenu.addNode("ON/OFF", ACT_NODE, &AlarmToggle);
  AlarmMenu.addNode("On Level", ACT_NODE, &AlarmOnAdjust);
  AlarmMenu.addNode("Off Level", ACT_NODE, &AlarmOffAdjust);

  // Testing menu
  TestMenu.addMenu("Testing", 0);
  TestMenu.addNode("PowerSupply", ACT_NODE, &TestPwrSupply);
  TestMenu.addNode("LvlSensor", ACT_NODE, &TestLevelSensor);
  TestMenu.addNode("AirSensor", ACT_NODE, &TestAirSensor);
  TestMenu.addNode("CLSensor", ACT_NODE, &TestCLSensor);
  TestMenu.addNode("FlowSensor", ACT_NODE, &TestWaterFlowSensor);

  OLED_Display.setTextColor(SSD1327_WHITE);

  /****************   test menu run **********/
  TestMenu.nodeIndex = 0;
  TestMenu.build(&OLED_Display);
  delay(1000);
  TestMenu.choose(); // TestPwrSupply
  // OLED_Display.display();

  TestMenu.nodeIndex = 1;
  TestMenu.build(&OLED_Display);
  delay(1000);
  TestMenu.choose(); // Test level Sensor

  TestMenu.nodeIndex = 2;
  TestMenu.build(&OLED_Display);
  TestMenu.choose(); // test air sensor
  delay(1000);

  TestMenu.nodeIndex = 3;
  TestMenu.build(&OLED_Display);
  TestMenu.choose(); // test cl sensor
  delay(1000);

  TestMenu.nodeIndex = 4;
  TestMenu.build(&OLED_Display);
  TestMenu.choose(); // test water flow sensor
  delay(1000);

  OLED_Display.clearDisplay();
  OLED_Display.setCursor(0, 0);
  OLED_Display.display();

  TestMenu.nodeIndex = 0;

  // Force to Auto Position
  PumpPositionFlag = OFF;
  AlarmPositionFlag = OFF;
  OffPositionFlag = OFF;
  AutoPositionFlag = ON;
  SSWMode = 1;
  DisplayOn(); // update display and start screen time out
}

/**********************************************
  Run Loop
**********************************************/
void loop()
{

  /******************  encoder & PB ********************/
  rotary_loop();

  /*********************** Read Switches **********************/
  // SWEncoder.loop(); // Update Encoder Switch instance
  SSWAuto.loop();  // Update Select Switch Auto Position instance
  SSWAlarm.loop(); // Update Select Switch Alarm Position instance
  SSWOff.loop();   // Update Select Switch Off Position instance
  SSWPump.loop();  // Update Select Switch Pump Position instance

  /********************  blue tooth ************************/
  BTStatusFlag = digitalRead(BTStatusPin); // BTStatus from hc-05 mod

  // BT Status ON - read serial1
  if (BTStatusFlag == ON)
  {
    /////////////   Receive and Process APP Data
    while (Serial1.available() > 0)
    {
      //  runner.pause();
      data_in = Serial1.read(); // Get next character

      // man pump swi
      if (data_in == 'L') // sw on
      {
        PumpManFl = ON;
      }
      if (data_in == 'l') // sw off
      {
        PumpManFl = OFF;
      }

      // man alarm sw
      if (data_in == 'M') // sw on
      {
        AlarmManFl = ON;
      }
      if (data_in == 'm') // sw on
      {
        AlarmManFl = OFF;
      }

      // man cl pump sw
      // if (data_in == 'U') // sw on
      // {
      //   CLPumpManFl = ON;
      // }
      // if (data_in == 'u') // sw on
      // {
      //   CLPumpManFl = OFF;
      // }

      // auto/man switch
      if (data_in == 'K') // sw on
      {                   // Switch On
        AutoManControl = ON;
      }
      if (data_in == 'k') // sw off
      {                   // Switch Off
        AutoManControl = OFF;
      }

      // pump on level
      if (data_in == 'N')
      { //  Slider
        PumpOnLevelSliderValue = Serial1.parseInt();
      }

      // pump off level
      if (data_in == 'O')
      { //  Slider
        PumpOffLevelSliderValue = Serial1.parseInt();
      }

      // alarm on level
      if (data_in == 'P')
      { //  Slider
        AlarmOnLevelSliderValue = Serial1.parseInt();
      }

      // alarm off level
      if (data_in == 'Q')
      { //  Slider
        AlarmOffLevelSliderValue = Serial1.parseInt();
      }

      // cl level
      // if (data_in == 'R')
      // { //  Slider
      //   // CLTimerSliderValue = Serial1.parseInt(SKIP_NONE, 'R');
      //   CLTimerSliderValue = Serial1.parseInt();
      // }

      // alarm vol
      if (data_in == 'S')
      { //  Slider
        // AlarmVolSliderValue = Serial1.parseInt(SKIP_NONE, 'S');
        AlarmVolSliderValue = Serial1.parseInt();
      }
      // runner.resume();
    }
  }

  /************************** The following are called from timers********************/
  // Update OLED Display *********/
  // called from timer->DisplayUpdateSetFlag->DisplayUpdateFlag=ON
  if (DisplayUpdateFlag == ON)
  {
    DisplayUpdate();
    DisplayUpdateFlag = OFF;
    Serial.println("DisplayUpdate");
  }

  // Update SD Card *********/
  // called from timer->SDUpdateSetFlag->SDUpdateFlag=ON
  if (SDUpdateFlag == ON)
  {
    SD_Update();
    SDUpdateFlag = OFF;
    Serial.println("SD_Update");
  }

  // Update level Sensor *********/
  // called from timer->SensorReadSetFlag->SensorReadFlag=ON
  if (SensorReadFlag == ON)
  {
    // Sensor Level Read();
    StatusLevelSensor = ReadLevelSensor(&ina3221, &Sensor_Level_Values, Chan2);
    Serial.println("LevelSensorUpdate");
    // if bad reading run fault display
    if (StatusLevelSensor != 0)
    {
      if (AutoPositionFlag) // run test in auto only
      {
        TestPwrSupply();
        TestLevelSensor();
      }
    }

    // sensor air read
    StatusAirSensor = ReadAirPump(&AirFlowSensor, &AirPump);
    Serial.println("AirSensorUpdate");
    // Serial.printf("Status Air Sensor: %d", StatusAirSensor);
    //  if bad reading run fault display
    if (StatusAirSensor != 0)
    {
      if (AutoPositionFlag) // run test in auto only
      {
        TestPwrSupply();
        TestAirSensor();
      }
    }

    // sensor cl level read
    StatusCLSensor = ReadCLSensor(CLLevelSW);
    Serial.println("CLSensorUpdate");
    // Serial.printf("Status CL Sensor: %d", StatusCLSensor);
    //  if bad reading run fault display
    if (StatusCLSensor == ON) // mag detected
    {
      if (AutoPositionFlag) // run test in auto only
      {
        TestPwrSupply();
        TestCLSensor();
      }
    }

    // sensor water level read
    StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
    Serial.println("FLOWSensorUpdate");
    Serial.printf("Pump: %d, Water: %d\n", StatusWaterPump, StatusWaterFlowSensor);

    //  if bad reading run fault display
    if (StatusWaterPump == ON && StatusWaterFlowSensor == OFF)
    {
      if (AutoPositionFlag == ON) // run test in auto/pump only
      {
        TestWaterFlowSensor();
        // Serial.println("StatusWaterPump == ON && StatusWaterFlowSensor == OFF");
        // delay(1000);
      }
    }

    if (StatusWaterPump == OFF && StatusWaterFlowSensor == ON)
    {
      if (AutoPositionFlag == ON) // run test in auto/pump only
      {
        TestWaterFlowSensor();
        // Serial.println("StatusWaterPump == OFF && StatusWaterFlowSensor == ON");
        // delay(1000);
      }
    }

    SensorReadFlag = OFF;
  }

  // Update App Data *********/
  // called from timer->SendAppDataSetFlag->SendAppDataFlag=ON
  if (SendAppDataFlag == ON)
  {
    SendAppData();
    SendAppDataFlag = OFF;
  }

  // Blank Display
  if (DisplayOffFlag == ON)
  {
    DisplayOff();
    DisplayOffFlag = OFF;
  }
  /********* run every loop *********/
  Pump();
  // CLPump();
  Alarm();
}

/**************************************************************************************************
*******************************************  Sub/Function Definitions  ****************************
***************************************************************************************************/

void MenuChoose(int Mode)
{
  if (Mode == 2) // alarm
  {
    SWEncoderFlag = OFF;
    AlarmMenu.choose(); // run alarm menu
  }
  if (Mode == 4) // pump
  {
    SWEncoderFlag = OFF;
    PumpMenu.choose(); // run pump menu
  }
}

void PumpOnAdjust()
{
  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries((PumpOffLevel + 1), 3000, false);
  rotaryEncoder->setAcceleration(4000);
  // put current Pump Hi level here
  rotaryEncoder->setEncoderValue(PumpOnLevel);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {

    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("-Pump  ON-");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(80, 20);
    OLED_Display.println("MM");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  PumpOnLevel = ENCValue; // put encvalue into pump level
  WriteData();

  // go back to Pump menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 4, false); // set encoder range for number of menu items
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  PumpMenu.nodeIndex = 0;
  // PumpMenu.choose();
}

void PumpOffAdjust()
{

  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries(300, (PumpOnLevel - 1), false);
  rotaryEncoder->setAcceleration(2000);
  // put current Pump Hi level here
  rotaryEncoder->setEncoderValue(PumpOffLevel);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {

    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("-Pump OFF-");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(80, 20);
    OLED_Display.println("MM");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  PumpOffLevel = ENCValue; // put encvalue into pump level high
  WriteData();

  // go back to Pump menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 4, false);
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  PumpMenu.nodeIndex = 0;
  // PumpMenu.choose();
}

void AlarmOnAdjust()
{
  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries((AlarmOffLevel + 1), 3000, false);
  rotaryEncoder->setAcceleration(3000);
  // put current Pump Hi level here
  rotaryEncoder->setEncoderValue(AlarmOnLevel);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {

    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("-Alarm On-");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(80, 20);
    OLED_Display.println("MM");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  AlarmOnLevel = ENCValue;
  WriteData();
  // put encvalue into pump level high

  // go back to Alarm menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 2, false);
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  AlarmMenu.nodeIndex = 0;
  // PumpMenu.choose();
}

void AlarmOffAdjust()
{
  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries(0, (AlarmOnLevel - 1), false);
  rotaryEncoder->setAcceleration(3000);
  // put current Pump Hi level here
  rotaryEncoder->setEncoderValue(AlarmOffLevel);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {
    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("-Alrm OFF-");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(80, 20);
    OLED_Display.println("MM");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  AlarmOffLevel = ENCValue;
  WriteData();
  // put encvalue into pump level high

  // go back to Pump menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 2, false);
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  AlarmMenu.nodeIndex = 0;
  // PumpMenu.choose();
}

/* void CLTimeAdjust()
{

  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries(0, 30, false);
  rotaryEncoder->setAcceleration(0);
  // put current Pump Hi level here
  int x = CLPump_RunTime;
  rotaryEncoder->setEncoderValue(x);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {
    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("CL RunTime");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(75, 20);
    OLED_Display.println("SEC");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  CLPump_RunTime = ENCValue;
  WriteData();
  // put encvalue into pump level high

  // go back to Pump menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 3, false);
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  PumpMenu.nodeIndex = 0;
  // PumpMenu.choose();
} */

void VolumeAdjust()
{

  // turn flag off
  SWEncoderFlag = OFF;

  // menu settings
  rotaryEncoder->setBoundaries(0, 100, false);
  rotaryEncoder->setAcceleration(1);
  // put current Pump Hi level here
  rotaryEncoder->setEncoderValue(AlarmVol);

  // stay in loop while changing value exit when push button pressed
  while (SWEncoderFlag == OFF)
  {

    // check enc and pushbutton
    rotary_loop();

    // set up display
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.setTextSize(2);

    OLED_Display.println("- Volume -");
    OLED_Display.setCursor(0, 20);
    OLED_Display.printf(" %d", ENCValue);
    OLED_Display.setCursor(80, 20);
    OLED_Display.println("MM");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("  Rotate to change");
    OLED_Display.print(("   Press to enter"));
    OLED_Display.display();

    // Serial.printf("PumpHiAdjust while Loop SWEncoderFlag %d ENCVal %d\n\r", SWEncoderFlag, ENCValue);
  }
  // Serial.println("Left loop");
  //  may have to save changes here
  AlarmVol = ENCValue;
  WriteData();
  // put encvalue into pump level high

  // go back to Pump menu
  // set up menu settings for pump
  rotaryEncoder->setBoundaries(0, 3, false);
  rotaryEncoder->setAcceleration(0);
  rotaryEncoder->setEncoderValue(0); // stop indicator from jumping on next screen

  SWEncoderFlag = OFF;
  ENCValue = 0;
  PumpMenu.nodeIndex = 0;
  // PumpMenu.choose();
}

// toggle pump on/off
void PumpToggle()
{

  PumpManFlag = !PumpManFlag;
  digitalWrite(PumpPin, PumpManFlag);
}

// toggle clpump on/off
/* void CLPumpToggle()
{

  CLPumpManFlag = !CLPumpManFlag;
  digitalWrite(CLPumpPin, CLPumpManFlag);
} */

// toggle alram on/off
void AlarmToggle()
{

  AlarmManFlag = !AlarmManFlag;
  digitalWrite(AlarmPin, AlarmManFlag);
}

// Alaram Control
void Alarm(void)
{
  if (AutoManControl == ON)
  {
    if (Sensor_Level_Values.DepthMM >= AlarmOnLevel)
    {
      digitalWrite(AlarmPin, ON);
      StatusAlarm = ON;
      // DEBUGPRINT(" AutoAlarmStatusON ");
      // DEBUGPRINTLN(StatusAlarm);
    }

    if (Sensor_Level_Values.DepthMM <= AlarmOffLevel)
    {
      digitalWrite(AlarmPin, OFF);
      StatusAlarm = OFF;
      // DEBUGPRINT(" AutoAlarmStatusOFF ");
      // DEBUGPRINTLN(StatusAlarm);
    }
  }
  else // manual control
  {
    /*     if (AlarmManFlag == ON)
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
        } */
  }
}

// Pump Control
void Pump(void)
{

  if (AutoManControl == ON) // auto
  {
    if (Sensor_Level_Values.DepthMM >= PumpOnLevel)
    {
      digitalWrite(PumpPin, ON);
      StatusWaterPump = ON;
      // Serial.println(" AutoPumpStatusON ");
      //  DEBUGPRINTLN(StatusWaterPump);
      // CLPumpRunOnce = ON;
      // delay(500);
      // StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
      // if (StatusWaterFlowSensor == OFF)
      // {
      //   TestWaterFlowSensor();
      // }
    }

    if (Sensor_Level_Values.DepthMM <= PumpOffLevel)
    {
      digitalWrite(PumpPin, OFF);
      StatusWaterPump = OFF;
      // //      delay(500);
      // StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
      // DEBUGPRINT(" AutoPumpStatusOFF ");
      //  DEBUGPRINTLN(StatusWaterPump);
    }
  }
  else // manual control
  {    ///////////////////////////////////////////////////// maybe changes this to PumpToggle

    if (PumpManFlag == ON)
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

// CLPump Control
/* void CLPump(void)
{
  if (AutoManControl == ON) // auto
  {

    if (CLPumpRunOnce == ON && StatusWaterPump == OFF)
    {

      delay(500);
      digitalWrite(CLPumpPin, ON);
      DEBUGPRINTLN("CLPump ON Auto");
      CLPumpStatus = ON;
      CLPumpRunOnce = OFF; // set in pump()
      CLPumpTimer.once(CLPump_RunTime, CLPumpOFF);
    }
  }
  else // manual control
  {
    if (CLPumpManFlag == ON)
    {
      digitalWrite(CLPumpPin, ON);
      CLPumpStatus = ON;
      // DEBUGPRINT("CLPumpStatusON Man ");
      //  DEBUGPRINTLN(CLPumpStatus);
    }
    else
    {
      digitalWrite(CLPumpPin, OFF);
      CLPumpStatus = OFF;
      // DEBUGPRINT("CLPumpStatusOFF Man ");
      //  DEBUGPRINTLN(CLPumpStatus);
    }
  }
} */

// blank
void testFunct()
{
}

// read ina chans for power supply
void TestPwrSupply()
{
  // removed 5v test for now
  //  int PSType = 0;
  int StatusPS = 0;

  // pstype is chan1(0)=12v or chan3(2)=5v on ina3221 board

  String Type = "";

  /*************************************************************************/
  // for (int PSType = 0; PSType <= 2; PSType = PSType + 2)
  // {
  //   if (PSType == 0)
  //   {
  //     Type = "12v";
  //   }
  //   else if (PSType == 2)
  //   {
  //     Type = "5v";
  //   }
  /*************************************************************************/
  int PSType = 0;
  Type = "12v";
  TestMenu.nodeIndex = 0;
  TestMenu.build(&OLED_Display);

  OLED_Display.setTextSize(1);
  OLED_Display.println("Testing...");
  OLED_Display.display();

  for (int i = 0; i <= 6; i++)
  {

    StatusPS = ReadLevelSensor(&ina3221, &Sensor_Level_Values, PSType);
    delay(100);
  }
  // set up display
  //  OLED_Display.clearDisplay();
  //  OLED_Display.setCursor(0, 0);
  //  OLED_Display.setTextSize(2);

  // OLED_Display.println("-Pwr Tst-");
  //  OLED_Display.setCursor(0, 20);

  // OLED_Display.printf(" %d", ENCValue);
  // OLED_Display.setCursor(80, 20);
  // OLED_Display.println("MM");

  // OLED_Display.setTextSize(1);
  // OLED_Display.println("");

  // OLED_Display.display();

  switch (StatusPS)
  {

  case 0: // good
    // screen keeps blanking so reload to see again
    TestMenu.nodeIndex = 0;
    TestMenu.build(&OLED_Display);

    // 12v/5v staus display print below menu
    OLED_Display.setTextSize(2);
    OLED_Display.printf("%s OK \n\r", Type.c_str());
    OLED_Display.setTextSize(1);
    OLED_Display.print("Volts: ");
    OLED_Display.println(Sensor_Level_Values.BusV, 1);
    OLED_Display.print("Current: ");
    OLED_Display.println(Sensor_Level_Values.ShuntImA, 1);
    OLED_Display.display();
    delay(1000);
    break;

  case 1: // low
    OLED_Display.setTextSize(2);
    OLED_Display.printf("%s LOW \n", Type.c_str());
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.print("Volts: ");
    OLED_Display.println(Sensor_Level_Values.BusV, 1);
    // OLED_Display.printf("Volts: %d\n\r", Sensor_Level_Values.LoadV);
    // OLED_Display.printf("Current: %f.1\n\r", Sensor_Level_Values.ShuntImA);
    OLED_Display.print("Current: ");
    OLED_Display.println(Sensor_Level_Values.ShuntImA, 1);
    OLED_Display.println("");
    OLED_Display.println("Check Wiring");
    OLED_Display.println("");
    OLED_Display.printf("Replace %s \n", Type.c_str());
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  case 2: // high
    OLED_Display.setTextSize(2);
    OLED_Display.printf("%s Failed \n", Type.c_str());
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    // OLED_Display.printf("Volts: %d\n\r", Sensor_Level_Values.LoadV);
    OLED_Display.print("Volts: ");
    OLED_Display.println(Sensor_Level_Values.BusV, 1);
    // OLED_Display.printf("Current: %f.1\n\r", Sensor_Level_Values.ShuntImA);
    OLED_Display.print("Current: ");
    OLED_Display.println(Sensor_Level_Values.ShuntImA, 1);
    OLED_Display.println("");
    OLED_Display.printf("Replace %s \n", Type.c_str());
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  default:
    OLED_Display.setTextSize(2);
    OLED_Display.println("Something wrong");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Push Reset");
    OLED_Display.println("");
    OLED_Display.println("Check PS");
    OLED_Display.display();
    delay(5000);
    break;
  }
  OLED_Display.clearDisplay();
  OLED_Display.setCursor(0, 0);
  OLED_Display.display();
  //}
}

void TestLevelSensor()
{

  // set up display
  // OLED_Display.clearDisplay();
  // OLED_Display.setCursor(0, 0);
  // OLED_Display.setTextSize(2);

  // OLED_Display.println("-Snsr Tst-");
  // OLED_Display.setCursor(0, 20);
  // // OLED_Display.printf(" %d", ENCValue);
  // // OLED_Display.setCursor(80, 20);
  // // OLED_Display.println("MM");

  // OLED_Display.setTextSize(1);
  // // OLED_Display.println("");
  // OLED_Display.println("Please wait...");
  // OLED_Display.display();

  TestMenu.nodeIndex = 1;
  TestMenu.build(&OLED_Display);

  OLED_Display.setTextSize(1);
  OLED_Display.println("Testing...");
  OLED_Display.display();

  for (int i = 0; i <= 6; i++)
  {
    StatusLevelSensor = ReadLevelSensor(&ina3221, &Sensor_Level_Values, Chan2);
    delay(100);
  }

  switch (StatusLevelSensor)
  {

  case 0:

    // OLED_Display.printf("Volts: %d\n\r", Sensor_Level_Values.LoadV);
    // OLED_Display.printf("Current: %f.1\n\r", Sensor_Level_Values.ShuntImA);
    TestMenu.nodeIndex = 1;
    TestMenu.build(&OLED_Display);

    // OLED_Display.println("");
    OLED_Display.setTextSize(2);
    OLED_Display.println("Passed");
    OLED_Display.setTextSize(1);
    OLED_Display.print("Volts: ");
    OLED_Display.println(Sensor_Level_Values.BusV, 1);
    OLED_Display.print("Current: ");
    OLED_Display.println(Sensor_Level_Values.ShuntImA, 1);
    OLED_Display.display();
    delay(1000);
    break;

  case 1:
    OLED_Display.setTextSize(2);
    OLED_Display.println("LVL SensorNot Found");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Check Sensor I/F");
    OLED_Display.println(" Connections");
    OLED_Display.println("");
    OLED_Display.println("Check Sensor Wiring");
    OLED_Display.println("");
    OLED_Display.println("Replace Sensor");
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  case 2:
    OLED_Display.setTextSize(2);
    OLED_Display.println("Sensor");
    OLED_Display.println("Failed");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.print("Volts: ");
    OLED_Display.println(Sensor_Level_Values.BusV, 1);
    OLED_Display.print("Current: ");
    OLED_Display.println(Sensor_Level_Values.ShuntImA, 1);

    OLED_Display.println("");
    OLED_Display.print("Replace Sensor");

    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  default:
    OLED_Display.setTextSize(2);
    OLED_Display.println("Something wrong");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Push Reset");
    OLED_Display.println("");
    OLED_Display.println("Check PS");
    OLED_Display.display();
    delay(10000);
    break;
  }

  // OLED_Display.print(("   Press to enter"));
  // OLED_Display.display();
}

void TestAirSensor()
{

  // set up display
  // OLED_Display.clearDisplay();
  // OLED_Display.setCursor(0, 0);
  // OLED_Display.setTextSize(2);

  // OLED_Display.println("-Snsr Tst-");
  // OLED_Display.setCursor(0, 20);
  // // OLED_Display.printf(" %d", ENCValue);
  // // OLED_Display.setCursor(80, 20);
  // // OLED_Display.println("MM");

  // OLED_Display.setTextSize(1);
  // // OLED_Display.println("");
  // OLED_Display.println("Please wait...");
  // OLED_Display.display();
  OLED_Display.setTextSize(1);
  OLED_Display.println("Testing...");
  OLED_Display.display();

  for (int i = 0; i <= 6; i++)
  {
    // sensor air read
    StatusAirSensor = ReadAirPump(&AirFlowSensor, &AirPump);
    delay(100);
  }

  switch (StatusAirSensor)
  {

  case 0:
    TestMenu.nodeIndex = 2;
    TestMenu.build(&OLED_Display);
    // OLED_Display.printf("Volts: %d\n\r", Sensor_Level_Values.LoadV);
    // OLED_Display.printf("Current: %f.1\n\r", Sensor_Level_Values.ShuntImA);

    // OLED_Display.println("");
    OLED_Display.setTextSize(2);
    OLED_Display.println("Passed");
    OLED_Display.setTextSize(1);
    OLED_Display.print("HPA: ");
    OLED_Display.println(AirPump.pressure_hPa, 1);
    OLED_Display.print("PSI: ");
    OLED_Display.println(AirPump.pressure_PSI, 1);
    OLED_Display.display();
    delay(1000);
    break;

  case 1:

    OLED_Display.setTextSize(2);
    OLED_Display.println("Air SensorNot Found");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Check Sensor I/F");
    OLED_Display.println(" Connections");
    OLED_Display.println("");
    OLED_Display.println("Check Sensor Wiring");
    OLED_Display.println("");
    OLED_Display.println("Replace Sensor");
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  case 2:
    // OLED_Display.println("");
    OLED_Display.setTextSize(2);

    OLED_Display.println("LOW");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Check Air Filter");
    OLED_Display.print("HPA: ");
    OLED_Display.println(AirPump.pressure_hPa, 1);
    OLED_Display.print("PSI: ");
    OLED_Display.println(AirPump.pressure_PSI, 1);
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  case 3:
    // OLED_Display.println("");
    OLED_Display.setTextSize(2);

    OLED_Display.println("HI");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Check for plug");
    OLED_Display.print("HPA: ");
    OLED_Display.println(AirPump.pressure_hPa, 1);
    OLED_Display.print("PSI: ");
    OLED_Display.println(AirPump.pressure_PSI, 1);
    OLED_Display.display();
    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
    break;

  default:
    OLED_Display.setTextSize(2);
    OLED_Display.println("Something wrong");
    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Push Reset");
    OLED_Display.println("");
    OLED_Display.println("Check PS");
    OLED_Display.display();
    delay(10000);
    break;
  }

  // OLED_Display.print(("   Press to enter"));
  // OLED_Display.display();
}

void TestWaterFlowSensor()
{
  TestMenu.nodeIndex = 4;
  TestMenu.build(&OLED_Display);

  OLED_Display.setTextSize(1);
  OLED_Display.println("Testing...");
  OLED_Display.display();

  for (int i = 0; i <= 6; i++)
  {
    // sensor cl read
    StatusWaterFlowSensor = ReadWaterFlowSensor(WaterFlowSW);
    delay(100);
  }

  if (StatusWaterFlowSensor == ON) // flow detected
  {

    OLED_Display.setTextSize(2);
    OLED_Display.println("SW CLOSED");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Sensor Reading");
    OLED_Display.println("");
    OLED_Display.println("LOW Level");
    OLED_Display.println("");
    OLED_Display.println("Flow Detected");

    OLED_Display.display();

    // AlarmToggle();
    // delay(5000);
    // AlarmToggle();
    delay(1000);
  }

  else
  {

    TestMenu.nodeIndex = 4;
    TestMenu.build(&OLED_Display);

    OLED_Display.setTextSize(2);
    OLED_Display.println("SW OPEN");
    OLED_Display.setTextSize(1);
    OLED_Display.println("Sensor Reading");
    OLED_Display.println("HIGH Level");
    OLED_Display.print("NO Flow");

    OLED_Display.display();

    delay(1000);
  }
}

void TestCLSensor()
{
  TestMenu.nodeIndex = 3;
  TestMenu.build(&OLED_Display);
  OLED_Display.setTextSize(1);
  OLED_Display.println("Testing...");
  OLED_Display.display();

  for (int i = 0; i <= 6; i++)
  {
    // sensor cl read
    StatusCLSensor = ReadCLSensor(CLLevelSW);
    delay(100);
  }

  if (StatusCLSensor == ON) // mag detected
  {

    OLED_Display.setTextSize(2);
    OLED_Display.println("Mag CLOSED");

    OLED_Display.setTextSize(1);
    OLED_Display.println("");
    OLED_Display.println("Sensor Reading");
    OLED_Display.println("");
    OLED_Display.println("LOW Level");
    OLED_Display.println("");
    OLED_Display.println("Magnet Detected");
    OLED_Display.println("");
    OLED_Display.print("Add Tablet");
    OLED_Display.display();

    AlarmToggle();
    delay(5000);
    AlarmToggle();
    delay(1000);
  }

  else
  {

    TestMenu.nodeIndex = 3;
    TestMenu.build(&OLED_Display);
    OLED_Display.setTextSize(2);
    OLED_Display.println("Mag OPEN");
    OLED_Display.setTextSize(1);
    OLED_Display.println("Sensor Reading");
    OLED_Display.println("HIGH Level");
    OLED_Display.print("NO Magnet");

    OLED_Display.display();

    delay(1000);
  }
}

// Blank Display
void DisplayOff(void)
{

  // DisplayState = OFF;
  // Serial.println("DisplayOff");
  Serial.printf("DisplayOffSSWMode %i \n", SSWMode);
  //  did we time out while in AutoControl and not in BT
  if (SSWMode == 1) // && BTStatusFlag == OFF)
  {

    // Serial.printf("SSWMode %i \n",SSWMode);
    DisplayState = OFF;
    DisplayUpdateFlag = OFF;
    // blank disp
    OLED_Display.setCursor(0, 0);
    OLED_Display.clearDisplay();
    OLED_Display.display();

    // delay(10);

    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.display();
  }
}

// set off flag
void DisplayOffSetFlag(void)
{
  /* don't want to spend much time in callback
     so just set flag*/
  DisplayOffFlag = ON;
}

// set flag / timer
void DisplayOn(void)
{
  DisplayState = ON;

  // only start display timeout in auto
  if (AutoPositionFlag) //&& BTStatusFlag == OFF)
  {

    DisplayOffTimer.once(DISP_TimeOut, DisplayOffSetFlag);
  }
  else
  {

    DisplayOffTimer.detach();
  }
}

// stop CLPump
/* void CLPumpOFF(void)
{

  digitalWrite(CLPumpPin, OFF);
  CLPumpStatus = OFF;
} */

// set flag to send data to app
void DisplayUpdateSetFlag(void)
{
  /* don't want to spend much time in callback
     so just set flag*/
  DisplayUpdateFlag = ON;
}

// send data to app
void DisplayUpdate(void)
{
  // Serial.println("DisplayUpdate()");
  if (BTStatusFlag == ON) // mode sw to auto BT ON
  {

    // Serial.println("DisplayUpdate() / DisplayState=ON / BTStatusFlag=ON");
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    OLED_Display.println("*** BT Connected! ***");

    DisplayLevelSensor(&OLED_Display, &Sensor_Level_Values);

    OLED_Display.printf("Pump: %d\n", StatusWaterPump);
    OLED_Display.printf(" On: %d Off: %d\n", PumpOnLevel, PumpOffLevel);
    OLED_Display.printf("Alarm: %d\n", StatusAlarm);
    OLED_Display.printf(" On: %d Off: %d\n", AlarmOnLevel, AlarmOffLevel);
    // OLED_Display.printf("CLPmp: %d\n", CLPumpStatus);

    // int x = CLPump_RunTime;
    // OLED_Display.printf(" RunTime: %d \n", x);

    OLED_Display.display();
    return;
  }

  if (DisplayState == ON)
  {

    // Serial.println("DisplayUpdate() / DisplayState=ON");
    if (BTStatusFlag == OFF) //  BT OFF
    {

      // int CLPRT = CLPump_RunTime;
      //  Serial.println("DisplayUpdate() / DisplayState=ON / BTStatusFlag=OFF");
      switch (SSWMode)
      {

      case 0: // encoder sw

        break;

      case 1: // auto

        OLED_Display.clearDisplay();
        OLED_Display.setCursor(0, 0);

        DisplayLevelSensor(&OLED_Display, &Sensor_Level_Values); // level sensor value
        OLED_Light(&OLED_Display, Count, &Sensor_Level_Values);  // sd write count

        OLED_Display.println("");
        OLED_Display.printf("Alarm On:  %d\r\n\n", AlarmOnLevel);
        OLED_Display.printf("Alarm Off: %d\r\n\n", AlarmOffLevel);
        OLED_Display.printf("Pump On:   %d\r\n\n", PumpOnLevel);
        OLED_Display.printf("PumpOff:   %d\r\n\n", PumpOffLevel);
        // OLED_Display.printf("CL Time:   %d\r\n", CLPRT);

        OLED_Display.display();
        break;

      case 2: // mode sw alarm

        AlarmMenu.build(&OLED_Display);
        if (SWEncoderFlag)
        {
          MenuChoose(2);
          // Serial.println("                AlarmMenuChoose");
        }
        break;

      case 3: // off

        OLED_Display.clearDisplay();
        OLED_Display.setCursor(0, 0);
        OLED_Display.setTextSize(2);
        OLED_Display.println("System OFF");
        OLED_Display.setTextSize(1);
        OLED_Display.println("");
        OLED_Display.printf("Alarm Level On:  %d\r\n\n", AlarmOnLevel);
        OLED_Display.printf("Alarm Level Off: %d\r\n\n", AlarmOffLevel);
        OLED_Display.printf("Pump Level On:   %d\r\n\n", PumpOnLevel);
        OLED_Display.printf("Pump Level Off:  %d\r\n\n", PumpOffLevel);
        OLED_Display.printf("CL Switch:   %d\r\n\n", StatusCLSensor);
        OLED_Display.printf("Flow Switch:  %d\r", StatusWaterFlowSensor);
        // OLED_Display.printf("CL Time:         %d\r\n\n", CLPRT);

        OLED_Display.display();

        break;

      case 4: // pump

        // DisplayState = ON;

        PumpMenu.build(&OLED_Display); // display menu
        if (SWEncoderFlag)
        {

          MenuChoose(4);
          // Serial.printf("                PumpMenuChoose SWEncoderFlag %d", SWEncoderFlag);
        }

        break;

      default:

        OLED_Display.clearDisplay();
        OLED_Display.setCursor(0, 0);

        OLED_Display.print("Default = Bad");
        OLED_Display.display();

        break;
      }
    }
  }
  else // blank display
  {
    // Serial.println("DisplayUpdate() / DisplayState=OFF");
    // if (BTStatusFlag == ON)
    // {
    OLED_Display.clearDisplay();
    OLED_Display.setCursor(0, 0);
    // OLED_Display.println("BT Connected! DisplayStateOFF");
    OLED_Display.display();
    // }
  }
}

// read rotary encoder and Push Button
void rotary_loop()
{
  ///////////////////////////////////// encoder
  if (rotaryEncoder->encoderChanged())
  {

    ENCValue = rotaryEncoder->readEncoder();
    if (SSWMode == 4) // if in Pump position change pump menu
    {

      PumpMenu.nodeIndex = ENCValue; // move enc value to menu
      // Serial.printf("PMPENC: %d \n\r", ENCValue);
    }

    if (SSWMode == 2)
    {

      AlarmMenu.nodeIndex = ENCValue;
      // Serial.printf("ALMENC: %d \n\r", ENCValue);
    }
  }

  ///////////////////////////////////////////////// push button
  if (rotaryEncoder->isEncoderButtonClicked())
  {

    if (DisplayState == OFF)
    {

      Serial.println("SWEncoder DispON ");
      DisplayOn();
      DisplayUpdate();
    }
    else
    {

      // only set flag inAlarm or Pump position
      if (PumpPositionFlag || AlarmPositionFlag)
      {

        SWEncoderFlag = ON;
        Serial.println("SWEncoderFlag On Pressed");
      }
      else // if you leave the SSW in Auto or Off, don't set flag for now. causes menu to change after SSW moved to any menu position
      {

        SWEncoderFlag = OFF;
        Serial.println("SWEncoderFlag Off Pressed");
      }
    }
  }
}

// read  select switch  SSW
// only runs once when SSW changes position
void pressed(Button2 &btn)
{

  /*****************************************************************************
   * *****************  temp  ***************************************************
   * *************************************************************************/

  // Serial.print("pressed ");

  // look at switch when BT NOT connected
  if (BTStatusFlag == OFF)
  {

    if (btn == SSWAuto)
    {

      // Serial.println("SSWAuto");

      // keep track of SSW position
      PumpPositionFlag = OFF;
      AlarmPositionFlag = OFF;
      OffPositionFlag = OFF;
      AutoPositionFlag = ON;

      /* theses are set in prgam loop
             PumpManFlag
             AlarmManFlag
             CLPumpManFlag
        */

      AutoManControl = ON;

      if (SSWMode != 1) // comming from another switch position
      {

        digitalWrite(PumpPin, OFF);
        digitalWrite(AlarmPin, OFF);
        // digitalWrite(CLPumpPin, OFF);
        SSWMode = 1;
        DisplayOn();
        // DisplayState = ON;
        // DisplayUpdate();
      }
    }

    else if (btn == SSWAlarm)
    {

      // Serial.println("SSWAlarm");

      SSWMode = 2;
      // keep track of SSW position
      PumpPositionFlag = OFF;
      AlarmPositionFlag = ON;
      OffPositionFlag = OFF;
      AutoPositionFlag = OFF;

      PumpManFlag = OFF;
      // CLPumpManFlag = OFF;
      AlarmManFlag = OFF; // set in menu

      AutoManControl = OFF;

      if (DisplayState == OFF) // turn display on but do not restart screen off timer
      {

        DisplayOn();
        DisplayUpdate();
      }

      // blank display on SSW change
      // clear display and set up encoder for Alarm menu
      OLED_Display.clearDisplay();
      OLED_Display.setCursor(0, 0);
      OLED_Display.display();
      rotaryEncoder->setBoundaries(0, 2, false); // dosen't work good in true
    }

    else if (btn == SSWOff)
    {
      // Serial.println("SSWOff");
      SSWMode = 3;

      PumpPositionFlag = OFF;
      AlarmPositionFlag = OFF;
      OffPositionFlag = ON;
      AutoPositionFlag = OFF;

      PumpManFlag = OFF;
      // CLPumpManFlag = OFF;
      AlarmManFlag = OFF;

      AutoManControl = OFF;
      digitalWrite(PumpPin, OFF);
      digitalWrite(AlarmPin, OFF);
      // digitalWrite(CLPumpPin, OFF);
    }

    else if (btn == SSWPump)
    {
      // Serial.println("SSWPump");
      SSWMode = 4;

      PumpPositionFlag = ON;
      AlarmPositionFlag = OFF;
      OffPositionFlag = OFF;
      AutoPositionFlag = OFF;

      PumpManFlag = OFF; // will be controlled by menu
      // CLPumpManFlag = OFF; // will be controlled by menu
      AlarmManFlag = OFF;

      AutoManControl = OFF;

      if (DisplayState == OFF)
      {
        DisplayOn();
        DisplayUpdate();
      }

      OLED_Display.clearDisplay();
      OLED_Display.setCursor(0, 0);
      OLED_Display.display();

      // set rotary 0-4 menu items, also must match in all the Adjust functions
      rotaryEncoder->setBoundaries(0, 4, false); // dosen't work good in true
    }

    else
    {
      // Serial.println("no button");
      // //SSWMode = 0;
    }
  }

  else // BT mode pretend SSW always in Auto Position
  {

    SSWMode = 1;

    PumpPositionFlag = OFF;
    AlarmPositionFlag = OFF;
    OffPositionFlag = OFF;
    AutoPositionFlag = ON;

    AutoManControl = ON;
  }
}

void SendAppDataSetFlag()
{

  SendAppDataFlag = ON;
}

// updaye app screen data
void SendAppData()
{

  /////////////  Send Data to Android device   /////////////////

  if (BTStatusFlag == ON)
  {
    // load changes to screens when 1st selected.
    // store vals to nvm when going from page 2 to 1
    if (AutoManControl == ON) // auto
    {
      // give a digital representation on plot
      static int cntr = 0;
      if (StatusWaterPump)
      {
        PumpPlotVal = 750;
      }
      else
      {
        PumpPlotVal = 0;
      }
      if (StatusAlarm)
      {
        AlarmPlotVal = 500;
      }
      else
      {
        AlarmPlotVal = 0;
      }

      if (StatusCLSensor)
      {
        CLPlotVal = 250;
      }
      else
      {
        CLPlotVal = 0;
      }
      cntr++;
      if (cntr >= 1000)
      {
        cntr = 0;
      }
      Serial1.print("*VX" + String(cntr) + "Y" + String(Sensor_Level_Values.DepthMM) + ",X" + String(cntr) + "Y" + String(PumpPlotVal) + ",X" + String(cntr) + "Y" + String(AlarmPlotVal) + ",X" + String(cntr) + "Y" + String(CLPlotVal) + "*");

      // run this loop only when page first opens
      //
      // runner.pause();
      if (Page1Once == 1)
      {

        WriteData(); // write to eprom
        // draw switch in auto position
        Serial1.println("*.kwl");
        Serial1.println("select_panel(1)");
        Serial1.println("clear_location(4,4)");
        Serial1.println("add_switch(4,4,3,K,k,0,1)"); // auto/man sw on
        Serial1.println("run()");
        Serial1.println("*");

        // Update pump on level
        // text = PumpOnLevel;
        Serial1.print("*E");
        Serial1.print(PumpOnLevel);
        Serial1.print("*");

        // Update pump off level
        // text = PumpOffLevel;
        Serial1.print("*F");
        Serial1.print(PumpOffLevel);
        Serial1.print("*");

        // Update alarm on level
        // text = AlarmOnLevel;
        Serial1.print("*G");
        Serial1.print(AlarmOnLevel);
        Serial1.print("*");

        // Update alarm off level
        // text = "AlarmOffLevel";
        Serial1.print("*H");
        Serial1.print(AlarmOffLevel);
        Serial1.print("*");

        // Update cl level
        // text = "CLTimer";
        // Serial1.print("*I");
        // Serial1.print(CLPump_RunTime);
        // Serial1.print("*");

        // Update vol level
        // text = "AlarmVol";
        Serial1.print("*J");
        Serial1.print(AlarmVol);
        Serial1.println("*");

        //  plot

        // Serial1.print("*V" + String(3000) + "," + String(3000) + "," + String(3000) + "," + String(3000));
        // Serial1.print("*V" + String(Sensor_Level_Values.DepthMM) + "," + String(PumpPlotVal) + "," + String(AlarmPlotVal) + "," + String(CLPlotVal));

        // Serial1.print("*V" + String(Sensor_Level_Values.DepthMM) + "," + String(PumpPlotVal) + "," + String(AlarmPlotVal) + "," + String(CLPlotVal));
        //      Serial1.print("*VX" + String(cntr) + "Y" + String(Sensor_Level_Values.DepthMM) + ",X" + String(cntr) + "Y" + String(PumpPlotVal) + ",X" + String(cntr) + "Y" + String(AlarmPlotVal) + ",X" + String(cntr) + "Y" + String(CLPlotVal) + "*");

        StatusWaterPump = OFF;
        PumpManFlag = OFF;
        StatusAlarm = OFF;
        AlarmManFlag = OFF;
        // CLPumpStatus = OFF;
        // CLPumpManFlag = OFF;
        Page1Once = 0;
        Page2Once = 1;
      }
      // runner.resume();
    }
    else // manual
    {
      // runner.pause();
      if (Page2Once == 1)
      {

        // draw switch on man position
        Serial1.println("*.kwl");
        Serial1.println("select_panel(2)");
        Serial1.println("clear_location(4,4)");
        Serial1.println("add_switch(4,4,3,K,k,0,0)"); // auto/man sw off

        // draw pump, alarm, CL switches
        Serial1.println("clear_location(6,3)");
        Serial1.println("add_switch(6,3,2,L,l,0,0)"); // pump sw off
        Serial1.println("clear_location(6,5)");
        Serial1.println("add_switch(6,5,2,M,m,0,0)"); // alarm sw off
        Serial1.println("clear_location(6,7)");
        Serial1.println("add_switch(6,7,2,U,u,0,0)"); // CL sw

        Serial1.println("run()");
        Serial1.println("*");

        PumpManFl = OFF;
        AlarmManFl = OFF;
        // CLPumpManFl = OFF;
        Page1Once = 1;
        Page2Once = 0;
      }

      // move screen vals to vars
      /*     PumpManFlag = PumpManFl;
          AlarmManFlag = AlarmManFl;
          CLPumpManFlag = CLPumpManFl;
          PumpOnLevel = PumpOnLevelSliderValue;
          PumpOffLevel = PumpOffLevelSliderValue;
          AlarmOnLevel = AlarmOnLevelSliderValue;
          AlarmOffLevel = AlarmOffLevelSliderValue;
          CLPump_RunTime = CLTimerSliderValue;
          AlarmVol = AlarmVolSliderValue; */
      // runner.resume();
    }

    if (AutoManControl == OFF) // manual
    {

      PumpManFlag = PumpManFl;
      AlarmManFlag = AlarmManFl;
      // CLPumpManFlag = CLPumpManFl;
      PumpOnLevel = PumpOnLevelSliderValue;
      PumpOffLevel = PumpOffLevelSliderValue;
      AlarmOnLevel = AlarmOnLevelSliderValue;
      AlarmOffLevel = AlarmOffLevelSliderValue;
      // CLPump_RunTime = CLTimerSliderValue;
      AlarmVol = AlarmVolSliderValue;

      // Update pump on level
      // text = PumpOnLevel;
      Serial1.print("*E");
      Serial1.print(PumpOnLevel);
      Serial1.print("*");

      // Update pump off level
      // text = PumpOffLevel;
      Serial1.print("*F");
      Serial1.print(PumpOffLevel);
      Serial1.print("*");

      // Update alarm on level
      // text = AlarmOnLevel;
      Serial1.print("*G");
      Serial1.print(AlarmOnLevel);
      Serial1.print("*");

      // Update alarm off level
      // text = "AlarmOffLevel";
      Serial1.print("*H");
      Serial1.print(AlarmOffLevel);
      Serial1.print("*");

      // Update cl level
      // text = "CLTimer";
      // Serial1.print("*I");
      // Serial1.print(CLPump_RunTime);
      // Serial1.print("*");

      // Update vol level
      // text = "AlarmVol";
      Serial1.print("*J");
      Serial1.print(AlarmVol);
      Serial1.print("*");
    }

    // Update  level
    // text = "SensorLevel";
    Serial1.print("*T");
    Serial1.print(Sensor_Level_Values.DepthMM);
    Serial1.print("*");

    // Bubble Gauge (Range is from 0 to 4096)
    bubbles = Sensor_Level_Values.DepthMM; // <--- Set Bubble gauge value here
    Serial1.print("*D");                   // + String(bubbles) + "*");
    Serial1.print(bubbles);
    Serial1.print("*");

    // Alarm LED Color
    if (StatusAlarm == ON)
    { // <--- Set RGB color red
      red = 255;
      green = 0;
      blue = 0;
    }
    else
    { // <--- Set RGB color gray
      red = 127;
      green = 127;
      blue = 127;
    }

    // send string
    Serial1.print("*BR");
    Serial1.print(red);
    Serial1.print("G");
    Serial1.print(green);
    Serial1.print("B");
    Serial1.print(blue);
    Serial1.print("*");

    // Pump LED Color
    if (StatusWaterPump == ON)
    { // <--- Set RGB color grn
      red = 0;
      green = 255;
      blue = 0;
    }
    else
    { // <--- Set RGB color gry
      red = 127;
      green = 127;
      blue = 127;
    }

    // send string
    Serial1.print("*AR");
    Serial1.print(red);
    Serial1.print("G");
    Serial1.print(green);
    Serial1.print("B");
    Serial1.print(blue);
    Serial1.print("*");

    // CL LED Color
    if (StatusCLSensor == ON)
    { // <--- Set RGB color grn
      red = 0;
      green = 255;
      blue = 0;
    }
    else
    { // <--- Set RGB color gry
      red = 127;
      green = 127;
      blue = 127;
    }

    // send string
    Serial1.print("*CR");
    Serial1.print(red);
    Serial1.print("G");
    Serial1.print(green);
    Serial1.print("B");
    Serial1.print(blue);
    Serial1.print("*");

    Serial1.println();
  }
}

// send screen to BT
void BuildPanel(void)
{

  /////////////////////////////////////// Build panel 2 in app
  // start datablock
  Serial1.println("*.kwl");
  Serial1.println("clear_panel(1)");
  Serial1.println("clear_panel(2)");
  Serial1.println("select_panel(2)");
  Serial1.println("set_grid_size(18,9)");

  // add_text(X,Y,Size,Justification,Text,[Red],[Green],[Blue],[Receive Char])
  // add_led(X,Y,Size,Receive Char,[Red],[Green],[Blue])
  // add_gauge(X,Y,Type,Min,Max,Value,Receive Char,[Min Text],[Max Text], Major Divisions, Minor Divisions)
  // add_switch(X,Y,Type,[On Text],[Off Text],[Repeat Rate],State)
  //  add_slider(X,Y,Type,Min,Max,Value,[Start Text],[End Text],Mode)
  // add_roll_graph(X,Y,Size,Min Y, Max Y, Points,Receive Char,[Title Text],[X-Axis Text], [Y-Axis Text],Autoscale,Log Y,GridLines,Legend,MaxMin X,MaxMin Y, Line Size, Marker Size,Trace Count,Trace 1 Name, R, G, B, [Trace 2 Name, R, G, B,])
  // add_xy_graph(X,Y,Size,Min X,Max X,Min Y,Max Y,Points,Receive Char,[Title Text],[X-Axis Text],[Y-Axis Text Autoscale X, AutoScale Y, Log X, Log Y, GridLines X, Gridlines Y,Legend,MaxMin X,MaxMin Y, Line Size, Marker Size,Trace Count,Trace 1 Name, R, G, B, [Trace 2 Name, R, G, B,])

  //////////// static text
  // Serial1.println("add_text(15,1,medium,C,CL TIME,245,240,245,)");
  Serial1.println("add_text(15,2,small,C,SEC,245,240,245,)");
  Serial1.println("add_text(13,1,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(13,2,small,C,OFF,245,240,245,)");
  Serial1.println("add_text(12,1,medium,R,ALARM,245,240,245,)");
  Serial1.println("add_text(12,2,small,C,ON,245,240,245,)");
  Serial1.println("add_text(10,1,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(10,2,small,C,OFF,245,240,245,)");
  Serial1.println("add_text(17,1,medium,C,VOL,245,240,245,)");

  Serial1.println("add_text(2,5,small,L,1500,245,240,245,)");
  Serial1.println("add_text(9,2,small,C,ON,245,240,245,)");
  Serial1.println("add_text(9,1,medium,R,PUMP,245,240,245,)");
  Serial1.println("add_text(17,2,small,C,%,245,240,245,)");
  Serial1.println("add_text(0,1,medium,C,PUMP,245,240,245,)");
  // Serial1.println("add_text(6,1,medium,C,CL,245,240,245,)");
  Serial1.println("add_text(3,1,medium,C,ALARM,245,240,245,)");
  Serial1.println("add_text(4,0,large,C,STATUS,245,240,245,)");
  Serial1.println("add_text(7,2,medium,R,PUMP,245,240,245,)");
  Serial1.println("add_text(7,4,medium,L,ALARM,245,240,245,)");
  // Serial1.println("add_text(7,6,medium,L,CL,245,240,245,)");
  Serial1.println("add_text(4,3,medium,C,AUTO,245,240,245,)");
  Serial1.println("add_text(4,7,medium,C,MANUAL,245,255,245,)");
  Serial1.println("add_text(0,4,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(0,3,medium,C,TANK,245,240,245,)");
  Serial1.println("add_text(13,0,large,C,SETTINGS,245,240,245,)");

  ///////////// indicators

  Serial1.println("add_led(1,1,1,A,127,127,127)"); // pump led
  Serial1.println("add_led(4,1,1,B,127,127,127)"); // alarm led
  Serial1.println("add_led(7,1,1,C,127,127,127)"); // cl led

  ///////////// gauge
  // Serial1.println("add_gauge(1,3,3,0,1024,0,D,0,1024,8,2)"); // tank level value
  Serial1.println("add_gauge(1,3,3,0,3000,0,D,0,3000,8,2)"); // tank level value

  //////////////// active text
  // Serial1.println("add_text(9,3,small,C,0,245,240,245,E)");  // pump on value
  Serial1.print("add_text(9,3,small,C,");
  Serial1.print(PumpOnLevel);
  Serial1.println(",245,240,245,E)");

  //  Serial1.println("add_text(10,3,small,C,0,245,240,245,F)"); // pump off value
  Serial1.print("add_text(10,3,small,C,");
  Serial1.print(PumpOffLevel);
  Serial1.println(",245,240,245,F)");

  //  Serial1.println("add_text(12,3,small,C,0,245,240,245,G)"); // alarm on value
  Serial1.print("add_text(12,3,small,C,");
  Serial1.print(AlarmOnLevel);
  Serial1.println(",245,240,245,G)");

  // Serial1.println("add_text(13,3,small,C,0,245,240,245,H)"); // alarm off value
  Serial1.print("add_text(13,3,small,C,");
  Serial1.print(AlarmOffLevel);
  Serial1.println(",245,240,245,H)");

  // Serial1.println("add_text(15,3,small,C,0,245,240,245,I)"); // cl time value
  // Serial1.print("add_text(15,3,small,C,");
  // Serial1.print(CLPump_RunTime);
  // Serial1.println(",245,240,245,I)");

  // Serial1.println("add_text(17,3,small,C,0,245,240,245,J)"); // vol value
  Serial1.print("add_text(17,3,small,C,");
  Serial1.print(AlarmVol);
  Serial1.println(",245,240,245,J)");

  // Serial1.println("add_text(1,8,medium,C,0,245,240,245,T)"); // vol value
  Serial1.print("add_text(1,8,small,C,");
  Serial1.print(Sensor_Level_Values.DepthMM);
  Serial1.println(",245,240,245,T)");

  ////////////////// switches
  Serial1.println("add_switch(4,4,3,K,k,0,0)"); // auto/man sw
  Serial1.println("add_switch(6,3,2,L,l,0,0)"); // pump sw
  Serial1.println("add_switch(6,5,2,M,m,0,0)"); // alarm sw
  Serial1.println("add_switch(6,7,2,U,u,0,0)"); // CL sw

  ////////////////// sliders
  //   Serial1.println("add_slider(9,4,3,0,4096,0,N,n,0)");  // pump on level slider
  Serial1.print("add_slider(9,4,3,0,3000,");
  Serial1.print(PumpOnLevel);
  Serial1.println(",N,n,0)");

  // Serial1.println("add_slider(10,4,3,0,4096,0,O,o,0)"); // pump off level slider
  Serial1.print("add_slider(10,4,3,0,3000,");
  Serial1.print(PumpOffLevel);
  Serial1.println(",O,o,0)");

  // Serial1.println("add_slider(12,4,3,0,4096,AlarmOnLevel,P,p,0)"); // alarm on level slider
  Serial1.print("add_slider(12,4,3,0,3000,");
  Serial1.print(AlarmOnLevel);
  Serial1.println(",P,p,0)");

  // Serial1.println("add_slider(13,4,3,0,4096,AlarmOffLevel,Q,q,0)"); // alarm off level slider
  Serial1.print("add_slider(13,4,3,0,3000,");
  Serial1.print(AlarmOffLevel);
  Serial1.println(",Q,q,0)");

  // Serial1.println("add_slider(15,4,3,0,100,CLTimer,R,r,0)");  // cl time slider
  // Serial1.print("add_slider(15,4,3,0,60,");
  // int x = CLPump_RunTime;
  // Serial1.print(x);
  // Serial1.println(",R,r,0)");

  // Serial1.println("add_slider(17,4,3,0,100,AlarmVol,S,s,0)");  // vol slider */
  Serial1.print("add_slider(17,4,3,0,100,");
  Serial1.print(AlarmVol);
  Serial1.println(",S,s,0)");

  ////////////////////////////////////////// Build panel 1 in app

  Serial1.println("select_panel(1)");
  Serial1.println("set_grid_size(18,9)");

  //////////// static text
  // Serial1.println("add_text(15,1,medium,C,CL TIME,245,240,245,)");
  Serial1.println("add_text(15,2,small,C,SEC,245,240,245,)");
  Serial1.println("add_text(13,1,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(13,2,small,C,OFF,245,240,245,)");
  Serial1.println("add_text(12,1,medium,R,ALARM,245,240,245,)");
  Serial1.println("add_text(12,2,small,C,ON,245,240,245,)");
  Serial1.println("add_text(10,1,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(10,2,small,C,OFF,245,240,245,)");
  Serial1.println("add_text(17,1,medium,C,VOL,245,240,245,)");
  // Serial1.println("add_text(7,6,medium,L,ALARM,245,240,245,)");
  Serial1.println("add_text(2,5,small,L,1500,245,240,245,)");
  Serial1.println("add_text(9,2,small,C,ON,245,240,245,)");
  Serial1.println("add_text(9,1,medium,R,PUMP,245,240,245,)");
  Serial1.println("add_text(17,2,small,C,%,245,240,245,)");
  Serial1.println("add_text(0,1,medium,C,PUMP,245,240,245,)");
  // Serial1.println("add_text(6,1,medium,C,CL,245,240,245,)");
  Serial1.println("add_text(3,1,medium,C,ALARM,245,240,245,)");
  Serial1.println("add_text(4,0,large,C,STATUS,245,240,245,)");
  // Serial1.println("add_text(7,3,medium,R,PUMP,245,240,245,)");
  Serial1.println("add_text(4,3,medium,C,AUTO,245,240,245,)");
  Serial1.println("add_text(4,7,medium,C,MANUAL,245,255,245,)");
  Serial1.println("add_text(0,4,medium,L,LEVEL,245,240,245,)");
  Serial1.println("add_text(0,3,medium,C,TANK,245,240,245,)");
  Serial1.println("add_text(13,0,large,C,SETTINGS,245,240,245,)");

  ///////////// indicators
  Serial1.println("add_led(1,1,1,A,127,127,127)"); // pump led
  Serial1.println("add_led(4,1,1,B,127,127,127)"); // alarm led
  Serial1.println("add_led(7,1,1,C,127,127,127)"); // cl led

  ///////////// gauge
  Serial1.println("add_gauge(1,3,3,0,3000,0,D,0,3000,8,2)"); // tank level value

  //////////////// active text
  // Serial1.println("add_text(9,3,small,C,0,245,240,245,E)");  // pump on value
  Serial1.print("add_text(9,3,small,C,");
  Serial1.print(PumpOnLevel);
  Serial1.println(",245,240,245,E)");

  //  Serial1.println("add_text(10,3,small,C,0,245,240,245,F)"); // pump off value
  Serial1.print("add_text(10,3,small,C,");
  Serial1.print(PumpOffLevel);
  Serial1.println(",245,240,245,F)");

  //  Serial1.println("add_text(12,3,small,C,0,245,240,245,G)"); // alarm on value
  Serial1.print("add_text(12,3,small,C,");
  Serial1.print(AlarmOnLevel);
  Serial1.println(",245,240,245,G)");

  // Serial1.println("add_text(13,3,small,C,0,245,240,245,H)"); // alarm off value
  Serial1.print("add_text(13,3,small,C,");
  Serial1.print(AlarmOffLevel);
  Serial1.println(",245,240,245,H)");

  // Serial1.println("add_text(15,3,small,C,0,245,240,245,I)"); // cl time value
  // Serial1.print("add_text(15,3,small,C,");
  // Serial1.print(CLPump_RunTime);
  // Serial1.println(",245,240,245,I)");

  // Serial1.println("add_text(17,3,small,C,0,245,240,245,J)"); // vol value
  Serial1.print("add_text(17,3,small,C,");
  Serial1.print(AlarmVol);
  Serial1.println(",245,240,245,J)");

  // Serial1.println("add_text(1,8,medium,C,0,245,240,245,T)"); // vol value
  Serial1.print("add_text(1,8,small,C,");
  Serial1.print(Sensor_Level_Values.DepthMM);
  Serial1.println(",245,240,245,T)");

  ////////////////// switches
  Serial1.println("add_switch(4,4,3,K,k,0,1)"); // auto/man sw
  //  Serial1.println("add_switch(6,4,2,L,l,0,0)"); // pump sw
  //  Serial1.println("add_switch(6,7,2,M,m,0,0)"); // alarm sw

  // plot graph
  // Serial1.println("add_roll_graph(9,4,8,0.0,3000.0,1000,V,Last 30 Days,Time,Value,1,0,1,1,1,1,thin,none,4,Level,70,255,255,Pump,255,145,0,Alarm,255,255,0,CLPmp,220,180,225)");
  Serial1.println("add_xy_graph(9,4,8,0.0,1000.0,0.0,3000.0,1000,V,Usage,Time,Value,0,0,0,0,1,1,1,1,1,medium,small,4,Level,70,255,255,Pump,255,145,0,Alarm,255,255,0)"); //,CLPmp,220,180,225)");

  Serial1.println("run()");
  Serial1.println("*");
}

void SensorReadSetFlag()
{

  SensorReadFlag = ON;
}

void SD_UpdateSetFlag()
{

  SDUpdateFlag = ON;
}

// update log file
void SD_Update()
{

  if (SDConnectOK)
  {
    DEBUGPRINTLN("Write SD**************");
    DateTime RTClock = rtc.now();
    Count++;
    if (StatusWaterPump)
    {
      PumpPlotVal = 750;
    }
    else
    {
      PumpPlotVal = 0;
    }
    if (StatusAlarm)
    {
      AlarmPlotVal = 500;
    }
    else
    {
      AlarmPlotVal = 0;
    }

    if (StatusLevelSensor)
    {
      CLPlotVal = 250;
    }
    else
    {
      CLPlotVal = 0;
    }
    // Refresh_SD(&RTCClock, &Sensor_Env_Values, &Sensor_Level_Values, Count);
    Refresh_SD(&RTClock, &Sensor_Level_Values, Count, PumpPlotVal, AlarmPlotVal, CLPlotVal);
  }

  /*   Serial.print(RTCClock.year(), DEC);
    DEBUGPRINT('/');
    Serial.print(RTCClock.month(), DEC);
    DEBUGPRINT('/');
    Serial.print(RTCClock.day(), DEC);
    DEBUGPRINT(" (");
    DEBUGPRINT(daysOfTheWeek[RTCClock.dayOfTheWeek()]);
    DEBUGPRINT(") ");
    Serial.print(RTCClock.hour(), DEC);
    DEBUGPRINT(':');
    Serial.print(RTCClock.minute(), DEC);
    DEBUGPRINT(':');
    Serial.print(RTCClock.second(), DEC);
    DEBUGPRINT();

  // Full Timestamp
  // DEBUGPRINTln(String("DateTime::TIMESTAMP_FULL:\t")+RTCClock.timestamp(DateTime::TIMESTAMP_FULL));
  ////////////////////////////DEBUGPRINTln(String(RTCClock.timestamp(DateTime::TIMESTAMP_FULL)));
  //  //Date Only
  //  DEBUGPRINT(String("DateTime::TIMESTAMP_DATE:\t")+RTCClock.timestamp(DateTime::TIMESTAMP_DATE));

  //  //Full Timestamp
  //  DEBUGPRINT(String("DateTime::TIMESTAMP_TIME:\t")+RTCClock.timestamp(DateTime::TIMESTAMP_TIME));

  //  DEBUGPRINT("\n");

  //   char buf1[] = "hh:mm";
  //  DEBUGPRINTln(RTCClock.toString(buf1));

  //  char buf2[] = "YYMMDD-hh:mm:ss";
  //  DEBUGPRINTln(RTCClock.toString(buf2));

  //  char buf3[] = "Today is DDD, MMM DD YYYY";
  //  DEBUGPRINTln(RTCClock.toString(buf3));

  //  char buf4[] = "MM-DD-YYYY";
  //  DEBUGPRINTln(RTCClock.toString(buf4));

  //  Refresh_SD(&RTCClock, &Senor_Env_Values);
*/
}

void WriteData()
{

  Serial.println("--------- Write to NVM -----------");

  // write to NVM
  Settings.begin("storage", RW_MODE); //  create and open it in RW mode.
  // store NVM values
  Settings.putInt("PumpOnLevel", PumpOnLevel);
  Settings.putInt("PumpOffLevel", PumpOffLevel);
  Settings.putInt("AlarmOnLevel", AlarmOnLevel);
  Settings.putInt("AlarmOffLevel", AlarmOffLevel);
  // Settings.putInt("CLTimer", CLPump_RunTime);
  Settings.putInt("AlarmVol", AlarmVol);
  Settings.end(); // close the namespace
}

// send serial data
void DisplayData(void)
{

  // local
  /*  Serial.print("C1: ");
   Serial.print(current_ma[INA3221_CH1], 3);
   // Serial.print("/");
   // Serial.print(Avg_current_ma[INA3221_CH1], 3);
   Serial.print("mA, ");
   Serial.print(voltage[INA3221_CH1], 2);
   Serial.print("V, ");
   Serial.print("Sh ");
   Serial.print(shunt[INA3221_CH1], 2);
   Serial.print("mV, ");
   // Serial.print("LodV ");
   // Serial.print(LoadV[INA3221_CH1], 2);
   // Serial.print("V, ");
   Serial.print("D: ");
   Serial.print(Distance[INA3221_CH1]);
   Serial.print("mm /  ");
   Serial.print(Distance[INA3221_CH1] / 25.4);
   Serial.println("in");

   Serial.print("C2: ");
   Serial.print(current_ma[INA3221_CH2], 3);
   // Serial.print("/");
   // Serial.print(Avg_current_ma[INA3221_CH2], 3);
   Serial.print("mA, ");
   Serial.print(voltage[INA3221_CH2], 2);
   Serial.print("V, ");
   Serial.print("Sh ");
   Serial.print(shunt[INA3221_CH2], 2);
   Serial.print("mV, ");
   // Serial.print("LodV ");
   // Serial.print(LoadV[INA3221_CH2], 2);
   // Serial.print("V, ");
   Serial.print("D: ");
   Serial.print(Distance[INA3221_CH2]);
   Serial.print("mm /  ");
   Serial.print(Distance[INA3221_CH2] / 25.4);
   Serial.println("in");

   Serial.print("C3: ");
   Serial.print(current_ma[INA3221_CH3], 3);
   // Serial.print("/");
   // Serial.print(Avg_current_ma[INA3221_CH3], 3);
   Serial.print("mA, ");
   Serial.print(voltage[INA3221_CH3], 2);
   Serial.print("V, ");
   Serial.print("Sh ");
   Serial.print(shunt[INA3221_CH3], 2);
   Serial.print("mV, ");
   // Serial.print("LodV ");
   // Serial.print(LoadV[INA3221_CH3], 2);
   // Serial.print("V, ");
   Serial.print("D: ");
   Serial.print(Distance[INA3221_CH3]);
   Serial.print("mm /  ");
   Serial.print(Distance[INA3221_CH3] / 25.4);
   Serial.println("in");
  */
  /* Serial.println("---");
  Serial.print("IMma ");
  Serial.print(in_min[INA3221_CH1]);
  Serial.print(" IXma ");
  Serial.print(in_max[INA3221_CH1]);
  Serial.print(" OMmm ");
  Serial.print(out_min[INA3221_CH1]);
  Serial.print(" OXmm ");
  Serial.println(out_max[INA3221_CH1]);

  Serial.print("IMma ");
  Serial.print(in_min[INA3221_CH2]);
  Serial.print(" IXma ");
  Serial.print(in_max[INA3221_CH2]);
  Serial.print(" OMmm ");
  Serial.print(out_min[INA3221_CH2]);
  Serial.print(" OXmm ");
  Serial.println(out_max[INA3221_CH2]);

  Serial.print("IMma ");
  Serial.print(in_min[INA3221_CH3]);
  Serial.print(" IXma ");
  Serial.print(in_max[INA3221_CH3]);
  Serial.print(" OMmm ");
  Serial.print(out_min[INA3221_CH3]);
  Serial.print(" OXmm ");
  Serial.println(out_max[INA3221_CH3]);

  Serial.println("---");
 */

  ////////////////////////////////////// BT
  /* Serial1.print("1:");
  Serial1.print(current_ma[INA3221_CH1], 3);
  Serial1.print("/");
  Serial1.print(Avg_current_ma[INA3221_CH1], 3);
  Serial1.print("mA, ");
  Serial1.print(voltage[INA3221_CH1], 2);
  Serial1.print("V, ");
  Serial1.print(shunt[INA3221_CH1], 2);
  Serial1.print("SmV, ");
  // Serial1.print("LV ");
  // Serial1.print(LoadV[INA3221_CH1], 2);
  // Serial1.println("V");
  Serial1.print("D ");
  Serial1.println(Distance[INA3221_CH1]);
  Serial1.println("");

  Serial1.print("IM ");
  Serial1.print(in_min[INA3221_CH1]);
  Serial1.print("IX ");
  Serial1.print(in_max[INA3221_CH1]);
  Serial1.print("OM ");
  Serial1.print(out_min[INA3221_CH1]);
  Serial1.print("OX ");
  Serial1.print(out_max[INA3221_CH1]);
  Serial.println("---");

  Serial1.print("2:");
  Serial1.print(current_ma[INA3221_CH2], 3);
  Serial1.print("/");
  Serial1.print(Avg_current_ma[INA3221_CH2], 3);
  Serial1.print("mA, ");
  Serial1.print(voltage[INA3221_CH2], 2);
  Serial1.print("V, ");
  Serial1.print(shunt[INA3221_CH2], 2);
  Serial1.print("SmV, ");
  // Serial1.print("LV ");
  // Serial1.print(LoadV[INA3221_CH2], 2);
  // Serial1.println("V");
  Serial1.print("D ");
  Serial1.println(Distance[INA3221_CH2]);

  Serial1.println("");
  Serial1.print("IM ");
  Serial1.print(in_min[INA3221_CH2]);
  Serial1.print("IX ");
  Serial1.print(in_max[INA3221_CH2]);
  Serial1.print("OM ");
  Serial1.print(out_min[INA3221_CH2]);
  Serial1.print("OX ");
  Serial1.print(out_max[INA3221_CH2]);
  Serial.println("---");

  Serial1.print("3:");
  Serial1.print(current_ma[INA3221_CH3], 3);
  Serial1.print("/");
  Serial1.print(Avg_current_ma[INA3221_CH3], 3);
  Serial1.print("mA, ");
  Serial1.print(voltage[INA3221_CH3], 2);
  Serial1.print("V, ");
  Serial1.print(shunt[INA3221_CH3], 2);
  Serial1.print("SmV, ");
  // Serial1.print("LV ");
  // Serial1.print(LoadV[INA3221_CH3], 2);
  // Serial1.println("V");
  Serial1.print("D ");
  Serial1.println(Distance[INA3221_CH3]);

  Serial1.println("");
  Serial1.print("IM ");
  Serial1.print(in_min[INA3221_CH3]);
  Serial1.print("IX ");
  Serial1.print(in_max[INA3221_CH3]);
  Serial1.print("OM ");
  Serial1.print(out_min[INA3221_CH3]);
  Serial1.print("OX ");
  Serial1.print(out_max[INA3221_CH3]);
  Serial.println("---"); */
}
