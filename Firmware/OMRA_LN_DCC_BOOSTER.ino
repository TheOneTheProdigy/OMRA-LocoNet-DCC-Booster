//  DCC / LocoNet RailSync Booster - Dual 10A Booster Districts With OLED Screen and AR Relay Output
//  2024 Lance Bradley
//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club SPFD Mo

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Filters.h>
 
#include <AH/Hardware/FilteredAnalog.hpp>
#include <AH/Timing/MillisMicrosTimer.hpp>

#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

int MICRO_PWR_PIN = 5; // MICRO_PWR - External PWR - Pulled up Internally
int C_SENSE1_PIN = A1; // Booster 1 Current Sensing Pin
int C_SENSE2_PIN = A2; // Booster 1 Current Sensing Pin
int EN1_PIN = 21; // Pin to Enable Booster 1
int EN2_PIN = 18; // Pin To Enable Booster 2
int RPWM_DETECT_PIN = 4; // Pin To Detect Valid Railsync Packets and Shut Down Track Power If No Signal
int LN_RX_MICRO_PIN = 8; // Pin To Receive LocoNet Packets From The LocoNet Buss
int LN_TX_MICRO_PIN = 9; // Pin To Send LocoNet Packets To The LocoNet Buss
int AR_RLY_PIN = 10; // Pin To Reverse Polarity Via External DPDT Relay With Opto Isolator

float CURRENT_LIMIT = 3.5; // Constaint Current Limit in ma - Fast Trip Current Should Be 140 Percent Of This Value
float ULTRA_SLOW_BLOW_LIMIT = 4.2; // Upper Limit Of The Ultra Slow Blow Region
float ULTRA_SLOW_BLOW_TIME = 30000; // Milliseconds To Trip Ultra Slow Blow Fuse
float SLOW_BLOW_LIMIT = 4.9; // Upper Limit Of The Slow Blow Region In Amps
float SLOW_BLOW_TIME = 15000; // Milliseconds To Trip Slow Blow Fuse
float FAST_BLOW_LIMIT = 5.1; // Upper Limit Of The Fast Blow Region In Amps. Anything Above This Will Blow Ultra Fast
float FAST_BLOW_TIME = 250; // Milliseconds To Trip Fast Blow Fuse
float BOOSTER_SOFT_START_PULSE_TIME = 100; // Milliseconds To Pulse 
float BOOSTER_REBOOT_TIME = 2000; // Milliseconds To Wait 
float BOOSTER_TRIPPED_COUNTER_RESET = 65000; // Milliseconds To Go Without A Current Trip To Reset Trip Counter Must Be Higher Than Longest Current Cutout Duration Of 60 Seconds
float RPWM_TIMER_LIMIT = 2500; // Milliseconds To Go Without Valid Railsync Commands Before Boosters Shutdown 
int RPWM_SIG_EDGES = 10; // Edges To Trigger RailSync Active Or Not Within RPWM_TRIGGER_LIMIT Timeframe
int POWER_BTN_LAST = 0; // A Zero Here Will Power Up The Track On Startup. A 1 Here Will Power Up With Track Power Off
int BOOSTER1_SOFTSTART_COUNT = 20;
int BOOSTER2_SOFTSTART_COUNT = 20;
float BOOSTER1_SOFTSTART_MULT = 2;
float BOOSTER2_SOFTSTART_MULT = 2;
float BOOSTER1_SOFTSTART_TIME = 5;
float BOOSTER2_SOFTSTART_TIME = 5;
float BOOST1_CSENSE_OFFSET = 270;
float BOOST2_CSENSE_OFFSET = 270;
float BOOSTER1_CSENSE_MOD = 2.42;
float BOOSTER2_CSENSE_MOD = 2.42;


int BOOSTER1_REBOOT_COUNT = 0;
int BOOSTER2_REBOOT_COUNT = 0;
int RPWM_RX_COUNT = 0;
int RPWM_COUNT = 0;
int RPWM_DETECT = 1;
bool RPWM_RX_TIMER_ACTIVE = false;
bool BOOST1_ENABLED = false;
bool IS_POWER1_TRIPPED = false;
bool IS_POWER1_ULTRA_TRIPPED = false;
bool IS_POWER1_FAST_TRIPPED = false;
bool IS_POWER1_FAST_PRE_TRIPPED = false;
bool IS_POWER1_SLOW_TRIPPED = false;
bool IS_POWER1_SLOW_PRE_TRIPPED = false;
bool IS_POWER1_ULTRA_SLOW_TRIPPED = false;
bool IS_POWER1_ULTRA_SLOW_PRE_TRIPPED = false;
bool BOOST2_ENABLED = false;
bool IS_POWER2_TRIPPED = false;
bool IS_POWER2_ULTRA_TRIPPED = false;
bool IS_POWER2_FAST_TRIPPED = false;
bool IS_POWER2_FAST_PRE_TRIPPED = false;
bool IS_POWER2_SLOW_TRIPPED = false;
bool IS_POWER2_SLOW_PRE_TRIPPED = false;
bool IS_POWER2_ULTRA_SLOW_TRIPPED = false;
bool IS_POWER2_ULTRA_SLOW_PRE_TRIPPED = false;
bool RPWM_TIMER_ACTIVE = false;
int MICRO_PWR = 0;
float BOOST1_CURRENT = 0;
float BOOST2_CURRENT = 0;
float BOOST1_AMPS = 0;
float BOOST2_AMPS = 0;
float BOOST1_AMPS_RAW = 0;
float BOOST2_AMPS_RAW = 0;
float BOOST1_CURRENT_RAW = 0;
float BOOST2_CURRENT_RAW = 0;
int RPWM_LAST = 1;

FilteredAnalog<> BOOST1_FILTERED = C_SENSE1_PIN; // Booster 1 Current Sensing Pin
FilteredAnalog<> BOOST2_FILTERED = C_SENSE2_PIN; // Booster 2 Current Sensing Pin

unsigned long time1;
unsigned long time2;
unsigned long BOOST1_TIMER;
unsigned long BOOST2_TIMER;
unsigned long BOOSTER1_SHUTDOWN_TIME;
unsigned long BOOSTER2_SHUTDOWN_TIME;
unsigned long BOOSTER1_LAST_POWER_ON;
unsigned long BOOSTER2_LAST_POWER_ON;
unsigned long RPWM_TIMER;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OMRALOGO 128x48px
const unsigned char OMRALOGO[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 
0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 
0x00, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 
0x00, 0x07, 0xef, 0xe0, 0x00, 0x00, 0x01, 0xfe, 0x7f, 0x00, 0x00, 0x00, 0x0f, 0xe7, 0xe0, 0x00, 
0x00, 0x1f, 0x83, 0xfe, 0x00, 0x00, 0x1f, 0xf8, 0x1f, 0xf0, 0x00, 0x00, 0xff, 0xc1, 0xf0, 0x00, 
0x00, 0x3e, 0x00, 0x7f, 0xf8, 0x03, 0xff, 0xc0, 0x07, 0xff, 0xc0, 0x3f, 0xfe, 0x00, 0xfc, 0x00, 
0x00, 0x7c, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x3e, 0x00, 
0x01, 0xf8, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0x1f, 0x80, 
0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 
0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 
0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 
0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
0x00, 0x1f, 0x00, 0x00, 0x7e, 0x01, 0xf0, 0x07, 0xc3, 0xfe, 0x00, 0x3c, 0x00, 0x01, 0xf0, 0x00, 
0x00, 0x0f, 0x80, 0x03, 0xff, 0x81, 0xf8, 0x0f, 0xc3, 0xff, 0x80, 0x7e, 0x00, 0x01, 0xe0, 0x00, 
0x00, 0x07, 0x80, 0x07, 0xff, 0xc1, 0xf8, 0x0f, 0xc3, 0xff, 0xc0, 0x7e, 0x00, 0x03, 0xe0, 0x00, 
0x00, 0x03, 0xc0, 0x07, 0x83, 0xe1, 0xf8, 0x1f, 0xc3, 0xc1, 0xe0, 0xfe, 0x00, 0x03, 0xc0, 0x00, 
0x00, 0x03, 0xc0, 0x0f, 0x01, 0xe1, 0xfc, 0x1f, 0xc3, 0xc1, 0xe0, 0xff, 0x00, 0x07, 0xc0, 0x00, 
0x00, 0x03, 0xc0, 0x0f, 0x01, 0xe1, 0xfc, 0x3f, 0xc3, 0xc1, 0xe0, 0xef, 0x00, 0x07, 0x80, 0x00, 
0x00, 0x03, 0xe0, 0x1f, 0x01, 0xf1, 0xfe, 0x3b, 0xc3, 0xc3, 0xe1, 0xe7, 0x80, 0x07, 0x80, 0x00, 
0x00, 0x01, 0xe0, 0x1f, 0x01, 0xf1, 0xee, 0x3b, 0xc3, 0xff, 0xc1, 0xe7, 0x80, 0x07, 0x80, 0x00, 
0x00, 0x01, 0xe0, 0x1f, 0x01, 0xf1, 0xee, 0x7b, 0xc3, 0xff, 0x01, 0xc7, 0x80, 0x07, 0x80, 0x00, 
0x00, 0x03, 0xe0, 0x0f, 0x01, 0xf1, 0xef, 0x73, 0xc3, 0xcf, 0x03, 0xc3, 0xc0, 0x07, 0x80, 0x00, 
0x00, 0x03, 0xc0, 0x0f, 0x01, 0xe1, 0xe7, 0xf3, 0xc3, 0xc7, 0x83, 0xff, 0xc0, 0x07, 0x80, 0x00, 
0x00, 0x03, 0xc0, 0x0f, 0x81, 0xe1, 0xe7, 0xf3, 0xc3, 0xc7, 0x87, 0xff, 0xe0, 0x07, 0xc0, 0x00, 
0x00, 0x07, 0xc0, 0x07, 0x83, 0xe1, 0xe3, 0xe3, 0xc3, 0xc3, 0xc7, 0x81, 0xe0, 0x03, 0xc0, 0x00, 
0x00, 0x07, 0x80, 0x07, 0xff, 0xc1, 0xe3, 0xe3, 0xc3, 0xc3, 0xc7, 0x81, 0xe0, 0x01, 0xe0, 0x00, 
0x00, 0x0f, 0x00, 0x03, 0xff, 0x81, 0xe3, 0xe3, 0xc3, 0xc1, 0xef, 0x01, 0xf0, 0x01, 0xf0, 0x00, 
0x00, 0x1e, 0x00, 0x00, 0x7e, 0x00, 0xc1, 0x81, 0xc1, 0xc0, 0xee, 0x00, 0xe0, 0x00, 0xf8, 0x00, 
0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 
0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 
0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 
0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 
0x03, 0xf0, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x0f, 0x80, 
0x00, 0xf8, 0x00, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xc0, 0x00, 0x3f, 0x00, 
0x00, 0x7e, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7c, 0x00, 
0x00, 0x1f, 0x00, 0xff, 0xc0, 0x00, 0xff, 0xe0, 0x0f, 0xfe, 0x00, 0x07, 0xff, 0x01, 0xf8, 0x00, 
0x00, 0x0f, 0x87, 0xfc, 0x00, 0x00, 0x07, 0xfc, 0x3f, 0xc0, 0x00, 0x00, 0x3f, 0xc3, 0xf0, 0x00, 
0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 
0x00, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 
0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 
0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ModernSteamerBlack, 128x64px Amimation 1
const unsigned char ANIMATION1[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x0e, 0x03, 0x80, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x3f, 0x07, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x7f, 0x8f, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x7f, 0xdf, 0xf0, 0x0f, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1c, 0x3c, 0x73, 0xdc, 0x70, 0x0e, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x0e, 0x3c, 0x3c, 0x73, 0xdc, 0x70, 0x0e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0xfc, 0x3f, 0xf3, 0xfc, 0x7f, 0xfe, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xff, 0xf8, 0x1f, 0xf1, 0xfc, 0x7f, 0xfc, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 
0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xfc, 0x01, 0xc0, 0x00, 0xf0, 0x00, 0x38, 0x00, 0x1c, 0x00, 0x1c, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xf0, 0x07, 0xf8, 0x03, 0xfc, 0x01, 0xfe, 0x00, 0xff, 0x00, 0x38, 0x00, 0x00, 0x00, 
0x00, 0x0f, 0xc0, 0x0f, 0xfc, 0x0f, 0xff, 0x03, 0xff, 0x01, 0xff, 0xc0, 0x38, 0x00, 0x00, 0x00, 
0x00, 0x3f, 0x00, 0x1f, 0xfe, 0x1f, 0xff, 0x87, 0xff, 0xc3, 0xff, 0xe0, 0x38, 0x00, 0x00, 0x00, 
0x00, 0x7e, 0x00, 0x3c, 0x0f, 0x1e, 0x07, 0x8f, 0x03, 0xe7, 0x80, 0xe0, 0x70, 0x00, 0x00, 0x00, 
0x01, 0xf8, 0x00, 0x78, 0x07, 0x3c, 0x03, 0xde, 0x3f, 0xff, 0xf8, 0x70, 0x70, 0x00, 0x00, 0x00, 
0x07, 0xe0, 0x00, 0x70, 0x07, 0xb8, 0x01, 0xde, 0x3f, 0xff, 0xfc, 0x70, 0xe0, 0x00, 0x00, 0x00, 
0x0f, 0x80, 0x00, 0x70, 0x03, 0xb8, 0x01, 0xdc, 0x3f, 0xff, 0xf8, 0x30, 0xe0, 0x00, 0x00, 0x00, 
0x3f, 0xff, 0xf8, 0x70, 0x03, 0xb8, 0x01, 0xdc, 0x00, 0xff, 0x00, 0x30, 0xff, 0xff, 0xff, 0x00, 
0xff, 0xff, 0xfc, 0x70, 0x7f, 0xff, 0xc1, 0xdc, 0x00, 0xfe, 0x00, 0x31, 0xff, 0xff, 0xff, 0xc0, 
0x7f, 0xff, 0xfc, 0x71, 0xff, 0xff, 0xe1, 0xde, 0x00, 0xe7, 0x00, 0x70, 0xff, 0xff, 0xff, 0xc0, 
0x3f, 0xff, 0xf8, 0x70, 0xff, 0xff, 0xe3, 0xce, 0x01, 0xe7, 0x00, 0x70, 0xff, 0xff, 0xff, 0x80, 
0x00, 0x00, 0x00, 0x38, 0x0f, 0x1e, 0x07, 0x8f, 0x01, 0xc7, 0x80, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1e, 0x1e, 0x0f, 0x0f, 0x07, 0xc7, 0xc3, 0xc3, 0xc0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfc, 0x07, 0xfe, 0x03, 0xff, 0x81, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xf8, 0x03, 0xfc, 0x01, 0xfe, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ModernSteamerBlack2, 128x64px Amimation 2 
const unsigned char ANIMATION2[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x0e, 0x03, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x3f, 0x07, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x7f, 0x8f, 0xf0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x7f, 0xdf, 0xf0, 0x0f, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x18, 0x3c, 0x33, 0xde, 0x70, 0x0e, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x0e, 0x38, 0x3c, 0x73, 0xde, 0x70, 0x1e, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0xfc, 0x3f, 0xf3, 0xfe, 0x7f, 0xfe, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xff, 0xf8, 0x1f, 0xf1, 0xfc, 0x7f, 0xfc, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xf8, 0x00, 
0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xfc, 0x00, 0x60, 0x00, 0x30, 0x00, 0x1c, 0x00, 0x0e, 0x00, 0x1c, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xf0, 0x03, 0xfc, 0x01, 0xfe, 0x00, 0xff, 0x80, 0x7f, 0x80, 0x38, 0x00, 0x00, 0x00, 
0x00, 0x0f, 0xc0, 0x07, 0xfe, 0x03, 0xff, 0x81, 0xff, 0xc0, 0xff, 0xe0, 0x38, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0x80, 0x0f, 0xff, 0x07, 0xff, 0x83, 0xff, 0xe1, 0xff, 0xf0, 0x70, 0x00, 0x00, 0x00, 
0x00, 0x7e, 0x00, 0x1e, 0x07, 0x8f, 0x01, 0xc7, 0x81, 0xf3, 0xc0, 0x70, 0x70, 0x00, 0x00, 0x00, 
0x01, 0xf0, 0x00, 0x1c, 0x7f, 0xff, 0xf1, 0xe7, 0x00, 0x73, 0x80, 0x38, 0x70, 0x00, 0x00, 0x00, 
0x07, 0xe0, 0x00, 0x38, 0x7f, 0xff, 0xf8, 0xe7, 0x00, 0x73, 0x80, 0x38, 0xe0, 0x00, 0x00, 0x00, 
0x0f, 0x80, 0x00, 0x38, 0x7f, 0xff, 0xf0, 0xff, 0x00, 0x7f, 0x80, 0x18, 0xe0, 0x00, 0x00, 0x00, 
0x3f, 0xff, 0xf8, 0x38, 0x01, 0xcc, 0x00, 0xff, 0x00, 0x7f, 0x80, 0x18, 0xff, 0xff, 0xff, 0x00, 
0x7f, 0xff, 0xfc, 0x38, 0x01, 0xdc, 0x00, 0xfe, 0x07, 0xff, 0xf8, 0x19, 0xff, 0xff, 0xff, 0xc0, 
0x7f, 0xff, 0xfc, 0x38, 0x01, 0xdc, 0x00, 0xef, 0x1f, 0xff, 0xfe, 0x39, 0xff, 0xff, 0xff, 0x80, 
0x3f, 0xff, 0xf8, 0x3c, 0x03, 0xde, 0x01, 0xe7, 0x1f, 0xff, 0xfc, 0x38, 0xff, 0xff, 0xff, 0x00, 
0x00, 0x00, 0x00, 0x1e, 0x07, 0x8f, 0x01, 0xc7, 0x80, 0xf3, 0xe0, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x0f, 0x0f, 0x07, 0xc7, 0xc3, 0xe3, 0xe1, 0xf1, 0xf0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xfe, 0x03, 0xff, 0x81, 0xff, 0xc0, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x03, 0xfc, 0x01, 0xff, 0x00, 0x7f, 0x00, 0x7f, 0x80, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Power Icon 16x16
const unsigned char POWERICON[] PROGMEM = {
0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x19, 0x98, 0x39, 0x9c, 0x39, 0x9c, 0x71, 0x8e, 0x61, 0x86, 
0x61, 0x86, 0x60, 0x06, 0x70, 0x0e, 0x70, 0x0e, 0x38, 0x1c, 0x1f, 0xf8, 0x0f, 0xf0, 0x07, 0xe0
};

// Overload Icon 16x16px
const unsigned char OVERLOADICON [] PROGMEM = {
	0x07, 0xe0, 0x1f, 0xfa, 0x38, 0x1e, 0x71, 0x8e, 0x61, 0x9e, 0xc1, 0x80, 0xc1, 0x80, 0xc1, 0x80, 
	0x01, 0x83, 0x00, 0x03, 0x01, 0x83, 0x7b, 0xc6, 0x71, 0x8e, 0x78, 0x1c, 0x5f, 0xf8, 0x07, 0xe0
};

// Alarm1 Icon 16x16px
const unsigned char ALARM1ICON [] PROGMEM = {
	0x00, 0x80, 0x03, 0xc0, 0x02, 0x40, 0x06, 0x60, 0x05, 0xa0, 0x0d, 0xb0, 0x09, 0x90, 0x11, 0x88, 
	0x11, 0x88, 0x21, 0x84, 0x21, 0x84, 0x40, 0x02, 0x41, 0x82, 0x81, 0x81, 0xc0, 0x03, 0x3f, 0xfc
};

// Alarm2 Icon 16x16px
const unsigned char ALARM2ICON [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x11, 0x88, 0x23, 0xc4, 0x24, 0x24, 0x08, 0x10, 0x48, 0x12, 0x48, 0x12, 
	0x48, 0x12, 0x28, 0x10, 0x28, 0x14, 0x08, 0x10, 0x1f, 0xf8, 0x10, 0x08, 0x1f, 0xf8, 0x00, 0x00
};

// Start Setup

void setup() {
  
  // Start Serial

  Serial.begin(9600);

  // Setup IO Pins
  pinMode(MICRO_PWR_PIN, INPUT_PULLUP); 
  pinMode(EN1_PIN, OUTPUT); 
  pinMode(EN2_PIN, OUTPUT);  
  pinMode(LN_TX_MICRO_PIN, OUTPUT); 
  pinMode(AR_RLY_PIN, OUTPUT); 

  FilteredAnalog<>::setupADC();

  // Start Display

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // here the 0x3c is the I2C address, check your i2c address if u have multiple devices.
  
  // Prevent Screen Manufacture Logo From Displaying On the Bootup

  display.clearDisplay();
  display.display();
  delay(250);

  // Show Short Animation For Visual Appeal 5 Times Only On Boot / Setup

  for (int i = 0; i < 5; i++) {
    display.clearDisplay();
    display.drawBitmap(0, 0, ANIMATION1, 128, 64, WHITE);
    display.display();
    delay(100);
    display.clearDisplay();
    display.drawBitmap(0, 0, ANIMATION2, 128, 64, WHITE);
    display.display();
    delay(100);
  }

  // Clear Display

  display.clearDisplay();
  display.display();

  // Show LCD Startup Routine Start With Board Model

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(28, 0);
  display.setTextWrap(false);
  display.println("10A-X2");
  display.drawBitmap(0, 16, OMRALOGO, 128, 64, WHITE);
  display.display();
  delay(1000);

  // Show Board Model And Version

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(40, 0);
  display.setTextWrap(false);
  display.println("V1.2");
  display.drawBitmap(0, 16, OMRALOGO, 128, 64, WHITE);
  display.display();
  delay(1000);

}

// Functions To Use In Programs

void turnPowerOn() {
  turnPower1On();
  turnPower2On();
}

void turnPowerOff() {
  turnPower1Off();
  turnPower2Off();
}

void turnPower1On() {
  for ( int x = 0; x < BOOSTER1_SOFTSTART_COUNT; x++ ) {
  digitalWrite(EN1_PIN, HIGH);
  delay(BOOSTER1_SOFTSTART_TIME);
  digitalWrite(EN1_PIN, LOW);
  delay(BOOSTER1_SOFTSTART_TIME * BOOSTER1_SOFTSTART_MULT);
  }

  digitalWrite(EN1_PIN, HIGH);
  BOOST1_ENABLED = true;
  BOOSTER1_LAST_POWER_ON = millis();
  IS_POWER1_TRIPPED = false;
  IS_POWER1_ULTRA_TRIPPED = false;
  IS_POWER1_FAST_TRIPPED = false;
  IS_POWER1_FAST_PRE_TRIPPED = false;
  IS_POWER1_SLOW_TRIPPED = false;
  IS_POWER1_SLOW_PRE_TRIPPED = false;
  IS_POWER1_ULTRA_SLOW_TRIPPED = false;
  IS_POWER1_ULTRA_SLOW_PRE_TRIPPED = false;
}

void turnPower2On() {
    for ( int x = 0; x < BOOSTER2_SOFTSTART_COUNT; x++ ) {
  digitalWrite(EN2_PIN, HIGH);
  delay(BOOSTER2_SOFTSTART_TIME);
  digitalWrite(EN2_PIN, LOW);
  delay(BOOSTER2_SOFTSTART_TIME * BOOSTER2_SOFTSTART_MULT);
  }
  digitalWrite(EN2_PIN, HIGH);
  BOOST2_ENABLED = true;
  BOOSTER2_LAST_POWER_ON = millis();
  IS_POWER2_TRIPPED = false;
  IS_POWER2_ULTRA_TRIPPED = false;
  IS_POWER2_FAST_TRIPPED = false;
  IS_POWER2_FAST_PRE_TRIPPED = false;
  IS_POWER2_SLOW_TRIPPED = false;
  IS_POWER2_SLOW_PRE_TRIPPED = false;
  IS_POWER2_ULTRA_SLOW_TRIPPED = false;
  IS_POWER2_ULTRA_SLOW_PRE_TRIPPED = false;
}

void turnPower1Off() {
  digitalWrite(EN1_PIN, LOW);
  BOOST1_ENABLED = false;
}

void turnPower2Off() {
  digitalWrite(EN2_PIN, LOW);
  BOOST2_ENABLED = false;
}

// Start Main Loop
void loop() {

  // Detect External Power Button

  MICRO_PWR = digitalRead(MICRO_PWR_PIN);
  if (MICRO_PWR == LOW) {
    turnPowerOff();
  }

  // Detect Power Button High, Makes Sure Not To Turn On While Tripped, Only Tries To Power On If Not Tried Within A Set Period, And Only Activates On Switch Change From Off To On

  if ((MICRO_PWR == HIGH) && (IS_POWER1_TRIPPED == false) && ((millis() - BOOSTER1_LAST_POWER_ON) >= 250 ) && (MICRO_PWR != POWER_BTN_LAST)) {
    turnPower1On();
  }

 // Detect Power Button High, Makes Sure Not To Turn On While Tripped, Only Tries To Power On If Not Tried Within A Set Period, And Only Activates On Switch Change From Off To On

  if ((MICRO_PWR == HIGH) && (IS_POWER2_TRIPPED == false) && ((millis() - BOOSTER2_LAST_POWER_ON) >= 250 ) && (MICRO_PWR != POWER_BTN_LAST)) {
    turnPower2On();
  }

  // Check Current Of Boosters 1 and 2

  // Booster 1

  BOOST1_FILTERED.update();

  // Serial.print("     ");
  // Serial.print(BOOST1_FILTERED.getValue());
  // Serial.print("     ");

  BOOST1_CURRENT = BOOST1_FILTERED.getValue() - BOOST1_CSENSE_OFFSET;
  BOOST1_AMPS = BOOSTER1_CSENSE_MOD * ((BOOST1_CURRENT * .004887) * 2.5767); //  8500 to 1 Current Sense 1A Should Be .1176ma x 3.3K Resistance = .38808 Volts Per A 1/.38808 = 2.5767 Amps Per Volt

  BOOST1_CURRENT_RAW = analogRead(C_SENSE1_PIN) - BOOST1_CSENSE_OFFSET;
  BOOST1_AMPS_RAW = BOOSTER1_CSENSE_MOD * ((BOOST1_CURRENT_RAW * .004887) * 2.5767); //  8500 to 1 Current Sense 1A Should Be .1176ma x 3.3K Resistance = .38808 Volts Per A 1/.38808 = 2.5767 Amps Per Volt

  if (BOOST1_AMPS <= 0.04) {
    BOOST1_AMPS = 0.0;
  }

  if (BOOST1_AMPS_RAW <= 0.04) {
    BOOST1_AMPS_RAW = 0.0;
  }

  // Stage 1
  
  if (BOOST1_AMPS_RAW >= (FAST_BLOW_LIMIT + .01)) {
    turnPower1Off();
    IS_POWER1_TRIPPED = true;
    IS_POWER1_ULTRA_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = millis();
  } 
    
    // Stage 2
    
  else if (BOOST1_AMPS >= (SLOW_BLOW_LIMIT + .01)) {
    if(IS_POWER1_FAST_PRE_TRIPPED == false){
      BOOST1_TIMER = millis();
      IS_POWER1_FAST_PRE_TRIPPED = true;
    }
    time1 = millis();
    if ((time1 - BOOST1_TIMER) >= FAST_BLOW_TIME){
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      IS_POWER1_FAST_TRIPPED = true;
      IS_POWER1_FAST_PRE_TRIPPED = false;
      BOOSTER1_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST1_AMPS <= (SLOW_BLOW_LIMIT + .01)) {
    IS_POWER1_FAST_PRE_TRIPPED = false;
  } 
  
    // Stage 3

  else if (BOOST1_AMPS >= (ULTRA_SLOW_BLOW_LIMIT + .01)) {
    if(IS_POWER1_SLOW_PRE_TRIPPED == false){
      BOOST1_TIMER = millis();
      IS_POWER1_SLOW_PRE_TRIPPED = true;
    }
    time1 = millis();
    if ((time1 - BOOST1_TIMER) >= SLOW_BLOW_TIME){
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      IS_POWER1_SLOW_TRIPPED = true;
      IS_POWER1_SLOW_PRE_TRIPPED = false;
      BOOSTER1_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST1_AMPS <= (ULTRA_SLOW_BLOW_LIMIT + .01)) {
    IS_POWER1_SLOW_PRE_TRIPPED = false;
  } 
  
    // Stage 4
    
  else  if (BOOST1_AMPS >= (CURRENT_LIMIT + .01)) {
    if(IS_POWER1_ULTRA_SLOW_PRE_TRIPPED == false){
      BOOST1_TIMER = millis();
      IS_POWER1_ULTRA_SLOW_PRE_TRIPPED = true;
    }
    time1 = millis();
    if ((time1 - BOOST1_TIMER) >= ULTRA_SLOW_BLOW_TIME){
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      IS_POWER1_ULTRA_SLOW_TRIPPED = true;
      IS_POWER1_ULTRA_SLOW_PRE_TRIPPED = false;
      BOOSTER1_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST1_AMPS <= (CURRENT_LIMIT + .01)) {
    IS_POWER1_ULTRA_SLOW_PRE_TRIPPED = false;
  }

  // Booster 2
  
   BOOST2_FILTERED.update();

  // Serial.print("     ");
  // Serial.print(BOOST2_FILTERED.getValue());
  // Serial.print("     ");

  BOOST2_CURRENT = BOOST2_FILTERED.getValue() - BOOST2_CSENSE_OFFSET;
  BOOST2_AMPS = BOOSTER2_CSENSE_MOD * ((BOOST2_CURRENT * .004887) * 2.5767); //  8500 to 1 Current Sense 1A Should Be .1176ma x 3.3K Resistance = .38808 Volts Per A 1/.38808 = 2.5767 Amps Per Volt

  BOOST2_CURRENT_RAW = analogRead(C_SENSE2_PIN) - BOOST2_CSENSE_OFFSET;
  BOOST2_AMPS_RAW = BOOSTER2_CSENSE_MOD * ((BOOST2_CURRENT_RAW * .004887) * 2.5767); //  8500 to 1 Current Sense 1A Should Be .1176ma x 3.3K Resistance = .38808 Volts Per A 1/.38808 = 2.5767 Amps Per Volt

  if (BOOST2_AMPS <= 0.04) {
    BOOST2_AMPS = 0.0;
  }

  if (BOOST2_AMPS_RAW <= 0.04) {
    BOOST2_AMPS_RAW = 0.0;
  }

  // Stage 1
  
  if (BOOST2_AMPS_RAW >= (FAST_BLOW_LIMIT + .01)) {
    turnPower2Off();
    IS_POWER2_TRIPPED = true;
    IS_POWER2_ULTRA_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = millis();
  } 
    
    // Stage 2
    
  else if (BOOST2_AMPS >= (SLOW_BLOW_LIMIT + .01)) {
    if(IS_POWER2_FAST_PRE_TRIPPED == false){
      BOOST2_TIMER = millis();
      IS_POWER2_FAST_PRE_TRIPPED = true;
    }
    time2 = millis();
    if ((time1 - BOOST2_TIMER) >= FAST_BLOW_TIME){
      turnPower2Off();
      IS_POWER2_TRIPPED = true;
      IS_POWER2_FAST_TRIPPED = true;
      IS_POWER2_FAST_PRE_TRIPPED = false;
      BOOSTER2_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST2_AMPS <= (SLOW_BLOW_LIMIT + .01)) {
    IS_POWER2_FAST_PRE_TRIPPED = false;
  } 
  
    // Stage 3

  else if (BOOST2_AMPS >= (ULTRA_SLOW_BLOW_LIMIT + .01)) {
    if(IS_POWER2_SLOW_PRE_TRIPPED == false){
      BOOST2_TIMER = millis();
      IS_POWER2_SLOW_PRE_TRIPPED = true;
    }
    time2 = millis();
    if ((time2 - BOOST2_TIMER) >= SLOW_BLOW_TIME){
      turnPower2Off();
      IS_POWER2_TRIPPED = true;
      IS_POWER2_SLOW_TRIPPED = true;
      IS_POWER2_SLOW_PRE_TRIPPED = false;
      BOOSTER2_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST2_AMPS <= (ULTRA_SLOW_BLOW_LIMIT + .01)) {
    IS_POWER2_SLOW_PRE_TRIPPED = false;
  } 
  
    // Stage 4
    
  else  if (BOOST2_AMPS >= (CURRENT_LIMIT + .01)) {
    if(IS_POWER2_ULTRA_SLOW_PRE_TRIPPED == false){
      BOOST2_TIMER = millis();
      IS_POWER2_ULTRA_SLOW_PRE_TRIPPED = true;
    }
    time2 = millis();
    if ((time2 - BOOST2_TIMER) >= ULTRA_SLOW_BLOW_TIME){
      turnPower2Off();
      IS_POWER2_TRIPPED = true;
      IS_POWER2_ULTRA_SLOW_TRIPPED = true;
      IS_POWER2_ULTRA_SLOW_PRE_TRIPPED = false;
      BOOSTER2_SHUTDOWN_TIME = millis();
    }
  } 
  else if (BOOST2_AMPS <= (CURRENT_LIMIT + .01)) {
    IS_POWER2_ULTRA_SLOW_PRE_TRIPPED = false;
  }

  // Attempt To Repower The Boosters

  // Booster 1 Repower Procedure

  if ((IS_POWER1_TRIPPED == true) && (MICRO_PWR == true)) {
    if (millis() - BOOSTER1_LAST_POWER_ON >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER1_REBOOT_COUNT = 0;
    }
    if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER1_REBOOT_COUNT <= 15)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*5)) && (BOOSTER1_REBOOT_COUNT <= 30)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*15)) && (BOOSTER1_REBOOT_COUNT <= 45)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*30)) && (BOOSTER1_REBOOT_COUNT <= 52)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
  }

  // Booster 2 Repower Procedure

 if ((IS_POWER2_TRIPPED == true) && (MICRO_PWR == true)) {
    if (millis() - BOOSTER2_LAST_POWER_ON >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER2_REBOOT_COUNT = 0;
    }
    if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER2_REBOOT_COUNT <= 15)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*5)) && (BOOSTER2_REBOOT_COUNT <= 30)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*15)) && (BOOSTER2_REBOOT_COUNT <= 45)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*30)) && (BOOSTER2_REBOOT_COUNT <= 52)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
  }

  // Draw Display

  display.clearDisplay(); // Clear Previous Display

  // Display Icons In Yellow Bar
  
  if (BOOST1_ENABLED == true) {     // Booster 1 Enabled Icon
    display.drawBitmap(0, 0, POWERICON, 16, 16, WHITE);
  }
  if (BOOST2_ENABLED == true) {     // Booster 2 Enabled Icon
    display.drawBitmap(64, 0, POWERICON, 16, 16, WHITE);   
  }
  if (IS_POWER1_TRIPPED == true) {     // Booster 1 Overload Icon
    display.drawBitmap(16, 0, OVERLOADICON, 16, 16, WHITE);   
  }
  if (IS_POWER2_TRIPPED == true) {     // Booster 2 Overload Icon
    display.drawBitmap(80, 0, OVERLOADICON, 16, 16, WHITE);   
  }
  if (IS_POWER1_SLOW_PRE_TRIPPED == true) {     // Booster 1 Pre Slow Overload Icon
    display.drawBitmap(32, 0, ALARM1ICON, 16, 16, WHITE);   
  }
  if (IS_POWER2_SLOW_PRE_TRIPPED == true) {     // Booster 2 Pre Slow Overload Icon
    display.drawBitmap(96, 0, ALARM1ICON, 16, 16, WHITE);   
  }
  if (IS_POWER1_ULTRA_SLOW_PRE_TRIPPED == true) {     // Booster 1 Pre Ultra Slow Overload Icon
    display.drawBitmap(48, 0, ALARM2ICON, 16, 16, WHITE);   
  }
  if (IS_POWER2_ULTRA_SLOW_PRE_TRIPPED == true) {     // Booster 2 Pre Ultra Slow Overload Icon
    display.drawBitmap(112, 0, ALARM2ICON, 16, 16, WHITE);   
  }
  
  // Display Booster District Name and INFO Box

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.setCursor(0, 20);
  display.println("**Prototype Booster**");
  
  // Display Booster 1 Load In Amps

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.setCursor(4, 32);
  display.println("B1 AMPS B2");
  display.setCursor(0, 50);
  display.println(BOOST1_AMPS, 2);

  // Display Booster 2 Load In Amps

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(80, 50);
  display.setTextWrap(false);
  display.println(BOOST2_AMPS, 2);

  // Finally Draw the Screen

  display.display();

  // Save PWR Button State For Comparison Next Run
  
  POWER_BTN_LAST = MICRO_PWR;

  // Check For Railsync Activity And Turn Off Or ON Track Power Depending On Activity After So Many Seconds

  // if (RPWM_TIMER_ACTIVE == false){
  //   RPWM_TIMER = millis();
  //   RPWM_COUNT = 0;
  //   RPWM_TIMER_ACTIVE = true;
  // } 
  // RPWM_DETECT = digitalRead(RPWM_DETECT_PIN);
  // if ((millis() - RPWM_TIMER) <= RPWM_TIMER_LIMIT) {
  //   if (RPWM_LAST != RPWM_DETECT) {
  //     RPWM_COUNT = ++RPWM_COUNT;
  //     Serial.print("Hello");
  //     Serial.print(RPWM_COUNT);
  //   }
  // }
  // if ((millis() - RPWM_TIMER) > (RPWM_TIMER_LIMIT)) {
  //   if ((RPWM_COUNT >= RPWM_SIG_EDGES) && (MICRO_PWR == HIGH) && (IS_POWER1_TRIPPED == false) && (IS_POWER2_TRIPPED == false)) {
  //     turnPowerOn();
  //   }
  //   if (RPWM_COUNT <= RPWM_SIG_EDGES) {
  //     turnPowerOff();
  //   } 
  //   RPWM_TIMER_ACTIVE = false;   
  // }

  // RPWM_LAST = RPWM_DETECT;

}
