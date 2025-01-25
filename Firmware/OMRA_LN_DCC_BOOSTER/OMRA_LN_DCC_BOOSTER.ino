//  DCC / LocoNet RailSync Booster - Dual 5A Booster Districts With Overload Protection, Overload Alarm Output, And OLED Screen With Both Booster's Current and Status

//  2025 Lance Bradley  Ozarks Model Railroad Assocation
//  2024 Kurt Clement   Ozarks Model Railroad Assocation

//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club Springfield Missouri

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "avdweb_AnalogReadFast.h"
#include "ARTWORK.h"

#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

float CURRENT_LIMIT = 5.1; // Constant current limit in amps - Instant trip current should be a margin above this value.
float INSTANT_FUSE_LIMIT = 6.0; // Lower Limit Of The Instant Blow Region In Amps. Anything Above This Will Blow Instantly.
float SLOW_FUSE_TIME = 3000; // Milliseconds To Trip Slow Blow Fuse
float BOOSTER_REBOOT_TIME = 3000; // Milliseconds to wait to energize the track again between short circuits.
float BOOSTER_REBOOT_COUNTER = 20; // Number of overcurrent failures before "BOOSTER_TRIPPED_COUNTER_RESET" timeout.
float BOOSTER_TRIPPED_COUNTER_RESET = 10000; // Milliseconds to wait once "BOOSTER_REBOOT_COUNTER" counter has been hit before restarting the process.

float BOOST1_CSENSE_OFFSET = 0.00; // Range = +- ADC (0-1023) every whole number offsets current measurements by .0083 amps
float BOOST2_CSENSE_OFFSET = 0.00; // Range = +- ADC (0-1023) every whole number offsets current measurements by .0083 amps

float RPWM_TIMER_LIMIT = 100; // Milliseconds To Go Without Valid Railsync Commands Before Boosters Shutdown 
int RPWM_SIG_EDGES = 2; // Edges To Trigger RailSync Active Or Not Within RPWM_TRIGGER_LIMIT Timeframe
float PRINT_DISPLAY_DELAY_TIME = 500; // Refresh Screen Every 500 Micro Seconds

bool PRINT_DEBUG = false; // Print Debug Info To Serial
float DEBUG_REFRESH_TIME = 2500; // Refresh serial debug if enabled (ms)

// These Below Should Not Need Touched

// Pin Assignments

int C_SENSE1_MICRO = A1; // Booster 1 Current Sensing Pin
int C_SENSE2_MICRO = A2; // Booster 1 Current Sensing Pin
int EN1_MICRO = 21; // Pin to Enable Booster 1
int EN2_MICRO = 18; // Pin To Enable Booster 2
int RPWM_DETECT_MICRO = 4; // Pin To Detect Valid Railsync Packets and Shut Down Track Power If No Signal
int LN_RX_MICRO = 8; // Pin To Receive LocoNet Packets From The LocoNet Buss
int LN_TX_MICRO = 9; // Pin To Send LocoNet Packets To The LocoNet Buss
int ALM1_MICRO = 10; // Pin To Activate LED and Piezo Alarm For Booster 1 Short Circuit Trip
int ALM2_MICRO = 5;  // Pin To Activate LED and Piezo Alarm For Booster 2 Short Circuit Trip

// Variables

int BOOSTER1_REBOOT_COUNT = 0;
int BOOSTER2_REBOOT_COUNT = 0;
int RPWM_COUNT = 0;
int RPWM_DETECT = 1;
bool BOOST1_ENABLED = false;
bool IS_POWER1_TRIPPED = false;
bool IS_POWER1_SLOW_PRE_TRIPPED = false;
bool BOOST2_ENABLED = false;
bool IS_POWER2_TRIPPED = false;
bool IS_POWER2_SLOW_PRE_TRIPPED = false;
bool RPWM_TIMER_ACTIVE = false;
int RPWM_LAST = 1;
float BOOST1_CURRENT = 0;
float BOOST2_CURRENT = 0;
float BOOST1_AMPS = 0;
float BOOST2_AMPS = 0;
float BOOST1_CURRENT_AVG = 0;
float BOOST2_CURRENT_AVG = 0;
float LAST_PRINT_DISPLAY_TIME = 0;
float LAST_DEBUG_REFRESH_TIME = 0;
unsigned long POWER1_SLOW_PRE_TIME = 0;
unsigned long POWER2_SLOW_PRE_TIME = 0;
unsigned long BOOSTER1_SHUTDOWN_TIME = 0;
unsigned long BOOSTER2_SHUTDOWN_TIME = 0;
unsigned long RPWM_TIMER;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Start Setup

void setup() {

  Serial.begin(9600);

  // Setup IO Pins
  pinMode(EN1_MICRO, OUTPUT); 
  pinMode(EN2_MICRO, OUTPUT);  
  pinMode(LN_TX_MICRO, OUTPUT);
  pinMode(ALM1_MICRO, OUTPUT);
  pinMode(ALM2_MICRO, OUTPUT);

  
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
  display.setCursor(22, 0);
  display.setTextWrap(false);
  display.println("V2.0.02");
  display.drawBitmap(0, 16, OMRALOGO, 128, 64, WHITE);
  display.display();
  delay(1500);

}

// Functions To Use In Programs

void turnPowerOff() {
  turnPower1Off();
  turnPower2Off();
}

void turnPower1On() {
  digitalWrite(EN1_MICRO, HIGH);
  BOOST1_ENABLED = true;
  IS_POWER1_TRIPPED = false;
}

void turnPower2On() {
  digitalWrite(EN2_MICRO, HIGH);
  BOOST2_ENABLED = true;
  IS_POWER2_TRIPPED = false;
}

void turnPower1Off() {
  digitalWrite(EN1_MICRO, LOW);
  BOOST1_ENABLED = false;
}

void turnPower2Off() {
  digitalWrite(EN2_MICRO, LOW);
  BOOST2_ENABLED = false;
}

// Start Main Loop
void loop() {

  // Check Current Of Boosters 1 and 2

    // Booster 1

  BOOST1_CURRENT = analogReadFast(C_SENSE1_MICRO) - BOOST1_CSENSE_OFFSET;
  BOOST1_CURRENT_AVG = BOOST1_CURRENT_AVG + (BOOST1_CURRENT - BOOST1_CURRENT_AVG) / 10;
  BOOST1_AMPS = ((BOOST1_CURRENT_AVG / 204.6) / 0.588);

  // Stage 1 - Instant current fault trip
  
  if (BOOST1_AMPS > INSTANT_FUSE_LIMIT) {
    turnPower1Off();
    IS_POWER1_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = millis();
  }

  // Stage 2 - Over the current limit will fault within the time delay

  if (BOOST1_AMPS > CURRENT_LIMIT) {
    if (IS_POWER1_SLOW_PRE_TRIPPED == false) {
      POWER1_SLOW_PRE_TIME = millis();
      IS_POWER1_SLOW_PRE_TRIPPED = true;
    }
    if (IS_POWER1_SLOW_PRE_TRIPPED == true) {
      if (millis() >= POWER1_SLOW_PRE_TIME + SLOW_FUSE_TIME){
      POWER1_SLOW_PRE_TIME += SLOW_FUSE_TIME;
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      BOOSTER1_SHUTDOWN_TIME = millis();
      }
    } 
  }
  if (BOOST1_AMPS < CURRENT_LIMIT) {
    IS_POWER1_SLOW_PRE_TRIPPED = false;
  } 

  // Booster 2

  BOOST2_CURRENT = analogReadFast(C_SENSE2_MICRO) - BOOST2_CSENSE_OFFSET;
  BOOST2_CURRENT_AVG = BOOST2_CURRENT_AVG + (BOOST2_CURRENT - BOOST2_CURRENT_AVG) / 10;
  BOOST2_AMPS = ((BOOST2_CURRENT_AVG / 204.6) / 0.588);

  // Stage 1 - Instant current fault trip
  
  if (BOOST2_AMPS > INSTANT_FUSE_LIMIT) {
    turnPower2Off();
    IS_POWER2_TRIPPED = true;
    BOOSTER2_SHUTDOWN_TIME = millis();
  }

  // Stage 2 - Over the current limit will fault within the time delay

  if (BOOST2_AMPS > CURRENT_LIMIT) {
    if (IS_POWER2_SLOW_PRE_TRIPPED == false) {
      POWER2_SLOW_PRE_TIME = millis();
      IS_POWER2_SLOW_PRE_TRIPPED = true;
    }
    if (IS_POWER2_SLOW_PRE_TRIPPED == true) {
      if (millis() >= POWER2_SLOW_PRE_TIME + SLOW_FUSE_TIME){
        POWER2_SLOW_PRE_TIME += SLOW_FUSE_TIME;
        turnPower2Off();
        IS_POWER2_TRIPPED = true;
        BOOSTER2_SHUTDOWN_TIME = millis();
      }
    }
  }
  if (BOOST2_AMPS < CURRENT_LIMIT) {
    IS_POWER2_SLOW_PRE_TRIPPED = false;
  } 

  // Attempt To Repower The Boosters

  // Booster 1 Repower Procedure

 if ((millis() - BOOSTER1_SHUTDOWN_TIME) >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER1_REBOOT_COUNT = 0;
    }
 if (IS_POWER1_TRIPPED == true) {
    if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER1_REBOOT_COUNT <= BOOSTER_REBOOT_COUNTER)) { // Retry if time and not at counter limit.
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
  }

  // Booster 2 Repower Procedure

 if ((millis() - BOOSTER2_SHUTDOWN_TIME) >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER2_REBOOT_COUNT = 0;
    }
 if (IS_POWER2_TRIPPED == true) {
    if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER2_REBOOT_COUNT <= BOOSTER_REBOOT_COUNTER)) { // Retry if time and not at counter limit.
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
  }

  // Draw Display

  if (millis() - LAST_PRINT_DISPLAY_TIME >= PRINT_DISPLAY_DELAY_TIME) {

    display.clearDisplay(); // Clear Previous Display

    // Display Icons In Yellow Bar
    
    if (BOOST1_ENABLED == true) {     // Booster 1 Enabled Icon
      display.drawBitmap(0, 0, POWERICON, 16, 16, WHITE);
      digitalWrite(ALM1_MICRO, LOW);  // Deactivate The Short Circuit Alarm   
    }
    if (BOOST2_ENABLED == true) {     // Booster 2 Enabled Icon
      display.drawBitmap(64, 0, POWERICON, 16, 16, WHITE);   
      digitalWrite(ALM2_MICRO, LOW);  // Deactivate The Short Circuit Alarm   
    }
    if (IS_POWER1_TRIPPED == true) {     // Booster 1 Overload Icon
      display.drawBitmap(16, 0, OVERLOADICON, 16, 16, WHITE); 
      digitalWrite(ALM1_MICRO, HIGH); // Activate The Short Circuit alarm 
    }
    if (IS_POWER2_TRIPPED == true) {     // Booster 2 Overload Icon
      display.drawBitmap(80, 0, OVERLOADICON, 16, 16, WHITE);   
      digitalWrite(ALM2_MICRO, HIGH); // Activate The Short Circuit alarm 
    }
    if (IS_POWER1_SLOW_PRE_TRIPPED == true) {     // Booster 1 Pre Slow Overload Icon
      display.drawBitmap(32, 0, ALARM2ICON, 16, 16, WHITE);   
    }
    if (IS_POWER2_SLOW_PRE_TRIPPED == true) {     // Booster 2 Pre Slow Overload Icon
      display.drawBitmap(96, 0, ALARM2ICON, 16, 16, WHITE);   
    }
//    if () {     // Not Used
//      display.drawBitmap(48, 0, ALARM1ICON, 16, 16, WHITE);   
//    }
//    if () {     // Not Used
//      display.drawBitmap(112, 0, ALARM1ICON, 16, 16, WHITE);   
//    }
    
    // Display Booster District Name and INFO Box

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setTextWrap(false);
    display.setCursor(0, 20);
    display.println("5A X2 LOCONET BOOSTER");
    
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

    LAST_PRINT_DISPLAY_TIME = millis();
  }

  // Check For Railsync Activity And Turn Off Or ON Track Power Depending On Activity After Alloted Time

  if (RPWM_TIMER_ACTIVE == false){
    RPWM_TIMER = millis();
    RPWM_COUNT = 0;
    RPWM_TIMER_ACTIVE = true;
  } 
  RPWM_DETECT = digitalRead(RPWM_DETECT_MICRO);
  if ((millis() - RPWM_TIMER) <= RPWM_TIMER_LIMIT) {
    if (RPWM_LAST != RPWM_DETECT) {
      RPWM_COUNT = ++RPWM_COUNT;
    }
  }  
  if ((millis() - RPWM_TIMER) > (RPWM_TIMER_LIMIT)) {  
    if ((RPWM_COUNT >= RPWM_SIG_EDGES) && (IS_POWER1_TRIPPED == false)) {
      turnPower1On();
    }
    if ((RPWM_COUNT >= RPWM_SIG_EDGES) && (IS_POWER2_TRIPPED == false)) {
      turnPower2On();
    }
    if (RPWM_COUNT <= RPWM_SIG_EDGES) {
      turnPowerOff();
    } 
    RPWM_TIMER_ACTIVE = false;   
  }
  RPWM_LAST = RPWM_DETECT;

  // Debug Code

   if ((PRINT_DEBUG == true) && (millis() - LAST_DEBUG_REFRESH_TIME >= DEBUG_REFRESH_TIME)) {
    LAST_DEBUG_REFRESH_TIME = millis();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("OMRA Booster Debug Report");
    Serial.println();
    Serial.println();
    Serial.println("Booster 1 Enabled");
    Serial.println(BOOST1_ENABLED);
    Serial.println(); 
    Serial.println("Booster 1 Pre Tripped");
    Serial.println(IS_POWER1_SLOW_PRE_TRIPPED);
    Serial.println();
    Serial.println("Booster 1 Tripped");
    Serial.println(IS_POWER1_TRIPPED);
    Serial.println();  
    Serial.println("Booster 1 AMPS");
    Serial.println(BOOST1_AMPS, 2);
    Serial.println(); 
    Serial.println("Booster 1 RAW Analog");
    Serial.println(BOOST1_CURRENT_AVG);
    Serial.println();
    Serial.println("Booster 1 Reboot Count");
    Serial.println(BOOSTER1_REBOOT_COUNT);
    Serial.println();
    Serial.println();
    Serial.println("Booster 2 Enabled");
    Serial.println(BOOST2_ENABLED);
    Serial.println(); 
    Serial.println("Booster 2 Pre Tripped");
    Serial.println(IS_POWER2_SLOW_PRE_TRIPPED);
    Serial.println();
    Serial.println("Booster 2 Tripped");
    Serial.println(IS_POWER2_TRIPPED);
    Serial.println();
    Serial.println("Booster 2 AMPS");
    Serial.println(BOOST2_AMPS, 2);
    Serial.println();    
    Serial.println("Booster 2 RAW Analog");
    Serial.println(BOOST2_CURRENT_AVG);
    Serial.println();
    Serial.println("Booster 2 Reboot Count");
    Serial.println(BOOSTER2_REBOOT_COUNT);
   }

}
