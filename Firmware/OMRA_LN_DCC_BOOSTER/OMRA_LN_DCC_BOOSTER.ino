//  DCC / LocoNet RailSync Booster - Dual 5A Booster Districts With Overload Protection, Overload Output, And OLED Screen With Both Booster's Current and Status
//  2024 Lance Bradley
//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club SPFD Mo
//  kc Includes code for visual and sonic short circuit alarm - 04/25/2024

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "avdweb_AnalogReadFast.h"
#include "ARTWORK.h"

#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

float CURRENT_LIMIT = 4.5; // Constant Current Limit in A - Instant Trip Current Should Be 150 Percent Of This Value
float FAST_BLOW_LIMIT = 4.9; // Upper Limit Of The Fast Blow Region In Amps
float INSTANT_BLOW_LIMIT = 6.0; // Lower Limit Of The Instant Blow Region In Amps. Anything Above This Will Blow Instantly
float SLOW_BLOW_TIME = 30000; // Milliseconds To Trip Slow Blow Fuse
float FAST_BLOW_TIME = 5000; // Milliseconds To Trip Fast Blow Fuse
float BOOSTER_REBOOT_TIME = 1000; // Milliseconds To Wait 
float BOOSTER_TRIPPED_COUNTER_RESET = 75000; // Milliseconds To Go Without A Current Trip To Reset Trip Counter Must Be Higher Than Longest Current Cutout Duration Of 60 Seconds

float RPWM_TIMER_LIMIT = 250; // Milliseconds To Go Without Valid Railsync Commands Before Boosters Shutdown 
int RPWM_SIG_EDGES = 1; // Edges To Trigger RailSync Active Or Not Within RPWM_TRIGGER_LIMIT Timeframe

int POWER_BTN_LAST = 0; // A Zero Here Will Power Up The Track On Startup. A 1 Here Will Power Up With Track Power Off

float PRINT_DISPLAY_DELAY_TIME = 500; // Refresh Screen Every 500 Micro Seconds

float BOOST1_CSENSE_OFFSET = 403; // 350 to 400 A Good Starting Point
float BOOST2_CSENSE_OFFSET = 353; // 350 to 400 A Good Starting Point
float BOOSTER1_CURRENT_RATIO = 10250; // 8500 = 8500:1
float BOOSTER2_CURRENT_RATIO = 8500; // 8500 = 8500:1
float BOOST1_R_VALUE = 5000; // In OHMS
float BOOST2_R_VALUE = 5000; // In OHMS
bool PRINT_BOOST_OFFSET = false; // Put A 0 In The CSENSE_OFFSET and True Here To View Suggested Offsets In Serial Monitor.
float PRINT_BOOST_DELAY_TIME = 2000; // Delay To Print Zero Values To Serial Monitor

// These Below Should Not Need Touched

int MICRO_PWR_PIN = 5; // MICRO_PWR - External PWR - Pulled up Internally
int C_SENSE1_PIN = A1; // Booster 1 Current Sensing Pin
int C_SENSE2_PIN = A2; // Booster 1 Current Sensing Pin
int EN1_PIN = 21; // Pin to Enable Booster 1
int EN2_PIN = 18; // Pin To Enable Booster 2
int RPWM_DETECT_PIN = 4; // Pin To Detect Valid Railsync Packets and Shut Down Track Power If No Signal
int LN_RX_MICRO_PIN = 8; // Pin To Receive LocoNet Packets From The LocoNet Buss
int LN_TX_MICRO_PIN = 9; // Pin To Send LocoNet Packets To The LocoNet Buss
int AR_RLY_PIN = 10; // Pin To Reverse Polarity Via External DPDT Relay With Opto Isolator
int ALARM1_PIN = 6;		// kc Pin to activate LED and piezo alarm for booster 1 short circuit trip
int ALARM2_PIN = 7;		// kc Pin to activate LED and piezo alarm for booster 2 short circuit trip

int BOOSTER1_REBOOT_COUNT = 0;
int BOOSTER2_REBOOT_COUNT = 0;
int RPWM_RX_COUNT = 0;
int RPWM_COUNT = 0;
int RPWM_DETECT = 1;
bool RPWM_RX_TIMER_ACTIVE = false;
bool BOOST1_ENABLED = false;
bool IS_POWER1_TRIPPED = false;
bool IS_POWER1_FAST_PRE_TRIPPED = false;
bool IS_POWER1_SLOW_PRE_TRIPPED = false;
bool BOOST2_ENABLED = false;
bool IS_POWER2_TRIPPED = false;
bool IS_POWER2_FAST_PRE_TRIPPED = false;
bool IS_POWER2_SLOW_PRE_TRIPPED = false;
bool RPWM_TIMER_ACTIVE = false;
int MICRO_PWR = 0;
float BOOST1_CURRENT = 0;
float BOOST2_CURRENT = 0;
float BOOST1_AMPS = 0;
float BOOST2_AMPS = 0;
float BOOST1_AMPS_AVG = 0;
float BOOST2_AMPS_AVG = 0;
float BOOST1_AMPS_AVG_DISPLAY = 0;
float BOOST2_AMPS_AVG_DISPLAY = 0;
int RPWM_LAST = 1;
float PRINT_BOOST_LAST_TIME = 0;
float LAST_PRINT_DISPLAY_TIME = 0;
unsigned long POWER1_FAST_PRE_TIME = 0;
unsigned long POWER2_FAST_PRE_TIME = 0;
unsigned long POWER1_SLOW_PRE_TIME = 0;
unsigned long POWER2_SLOW_PRE_TIME = 0;
unsigned long BOOSTER1_SHUTDOWN_TIME;
unsigned long BOOSTER2_SHUTDOWN_TIME;
unsigned long BOOSTER1_LAST_POWER_ON;
unsigned long BOOSTER2_LAST_POWER_ON;
unsigned long RPWM_TIMER;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Start Setup

void setup() {

	// kc added serial output of calibration values at startup for documentation
  Serial.begin(9600);
  Serial.println("Calibration Values for OMRA Booster");
  Serial.print("BOOST1_CSENSE_OFFSET: ");
  Serial.println(BOOST1_CSENSE_OFFSET);
  Serial.print("BOOST2_CSENSE_OFFSET: ");
  Serial.println(BOOST2_CSENSE_OFFSET);
  Serial.print("BOOSTER1_CURRENT_RATIO: ");
  Serial.println(BOOSTER1_CURRENT_RATIO);
  Serial.print("BOOSTER2_CURRENT_RATIO: ");
  Serial.println(BOOSTER2_CURRENT_RATIO);
	
if (PRINT_BOOST_OFFSET == false) {
  //  if PRINT_BOOST_OFFSET == false then Serial is not needed, turn it off.
  Serial.flush();  
  Serial.end();	// kc Disable serial    
  }

  // Setup IO Pins
  pinMode(MICRO_PWR_PIN, INPUT_PULLUP); 
  pinMode(EN1_PIN, OUTPUT); 
  pinMode(EN2_PIN, OUTPUT);  
  pinMode(LN_TX_MICRO_PIN, OUTPUT); 
  pinMode(AR_RLY_PIN, OUTPUT); 
  pinMode(ALARM1_PIN, OUTPUT); 		// kc setup alarm pins
  pinMode(ALARM2_PIN, OUTPUT);		// kc setup alarm pins	

	
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
  display.println("V1.3");
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
  digitalWrite(EN1_PIN, HIGH);
  BOOST1_ENABLED = true;
  BOOSTER1_LAST_POWER_ON = millis();
  IS_POWER1_TRIPPED = false;
}

void turnPower2On() {
  digitalWrite(EN2_PIN, HIGH);
  BOOST2_ENABLED = true;
  BOOSTER2_LAST_POWER_ON = millis();
  IS_POWER2_TRIPPED = false;
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

  BOOST1_CURRENT = analogReadFast(C_SENSE1_PIN) - BOOST1_CSENSE_OFFSET;
  if (BOOST1_CURRENT <= 75) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 2.3);
  }
  else if (BOOST1_CURRENT <= 125) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 2.4);
  }
  else if (BOOST1_CURRENT <= 150) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 2.5);
  }
  else if (BOOST1_CURRENT <= 170) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 2.8);
  }  
  else if (BOOST1_CURRENT <= 190) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 3.0);
  }
  else if (BOOST1_CURRENT <= 205) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 3.1);
  }
  else if (BOOST1_CURRENT <= 225) {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 3.3);
  }
  else {
    BOOST1_AMPS = ((((BOOST1_CURRENT * .0049804688) / BOOST1_R_VALUE) / (1 / BOOSTER1_CURRENT_RATIO)) * 4.0);
  }

  BOOST1_AMPS_AVG = BOOST1_AMPS_AVG + (BOOST1_AMPS - BOOST1_AMPS_AVG) / 10;
  BOOST1_AMPS_AVG_DISPLAY = BOOST1_AMPS_AVG;

  if (BOOST1_AMPS_AVG_DISPLAY <= 0.0) {
    BOOST1_AMPS_AVG_DISPLAY = 0.0;
  }

  // Stage 1
  
  if (BOOST1_AMPS > INSTANT_BLOW_LIMIT) {
    turnPower1Off();
    IS_POWER1_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = millis();
  }

  // Stage 2

  if (BOOST1_AMPS_AVG > FAST_BLOW_LIMIT) {
    if (IS_POWER1_FAST_PRE_TRIPPED == false) {
      POWER1_FAST_PRE_TIME = millis();
      IS_POWER1_FAST_PRE_TRIPPED = true;
    }
    if (IS_POWER1_FAST_PRE_TRIPPED == true) {
      if (millis() >= POWER1_FAST_PRE_TIME + FAST_BLOW_TIME){
      POWER1_FAST_PRE_TIME += FAST_BLOW_TIME;
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      BOOSTER1_SHUTDOWN_TIME = millis();
      }
    }
  } 
  if (BOOST1_AMPS_AVG < FAST_BLOW_LIMIT) {
    IS_POWER1_FAST_PRE_TRIPPED = false;
  }

  // Stage 3

  if (BOOST1_AMPS_AVG > CURRENT_LIMIT) {
    if (IS_POWER1_SLOW_PRE_TRIPPED == false) {
      POWER1_SLOW_PRE_TIME = millis();
      IS_POWER1_SLOW_PRE_TRIPPED = true;
    }
    if (IS_POWER1_SLOW_PRE_TRIPPED == true) {
      if (millis() >= POWER1_SLOW_PRE_TIME + SLOW_BLOW_TIME){
      POWER1_SLOW_PRE_TIME += SLOW_BLOW_TIME;
      turnPower1Off();
      IS_POWER1_TRIPPED = true;
      BOOSTER1_SHUTDOWN_TIME = millis();
      }
    } 
  }
  if (BOOST1_AMPS_AVG < CURRENT_LIMIT) {
    IS_POWER1_SLOW_PRE_TRIPPED = false;
  } 

  // Booster 2

  BOOST2_CURRENT = analogReadFast(C_SENSE2_PIN) - BOOST2_CSENSE_OFFSET;
  if (BOOST2_CURRENT <= 75) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 2.3);
  }
  else if (BOOST2_CURRENT <= 125) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 2.4);
  }
  else if (BOOST2_CURRENT <= 150) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 2.5);
  }
  else if (BOOST2_CURRENT <= 170) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 2.8);
  }  
  else if (BOOST2_CURRENT <= 190) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 3.0);
  }
  else if (BOOST2_CURRENT <= 205) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 3.1);
  }
  else if (BOOST2_CURRENT <= 225) {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 3.3);
  }
  else {
    BOOST2_AMPS = ((((BOOST2_CURRENT * .0049804688) / BOOST2_R_VALUE) / (1 / BOOSTER2_CURRENT_RATIO)) * 4.0);
  }

  BOOST2_AMPS_AVG = BOOST2_AMPS_AVG + (BOOST2_AMPS - BOOST2_AMPS_AVG) / 10;
  BOOST2_AMPS_AVG_DISPLAY = BOOST2_AMPS_AVG;

  if (BOOST2_AMPS_AVG_DISPLAY <= 0.0) {
    BOOST2_AMPS_AVG_DISPLAY = 0.0;
  }

  // Stage 1
  
  if (BOOST2_AMPS > INSTANT_BLOW_LIMIT) {
    turnPower2Off();
    IS_POWER2_TRIPPED = true;
    BOOSTER2_SHUTDOWN_TIME = millis();
  }

  // Stage 2

  if (BOOST2_AMPS_AVG > FAST_BLOW_LIMIT) {
    if (IS_POWER2_FAST_PRE_TRIPPED == false) {
      POWER2_FAST_PRE_TIME = millis();
      IS_POWER2_FAST_PRE_TRIPPED = true;
    }
    if (IS_POWER2_FAST_PRE_TRIPPED == true) {
      if (millis() >= POWER2_FAST_PRE_TIME + FAST_BLOW_TIME){
      POWER2_FAST_PRE_TIME += FAST_BLOW_TIME;
      turnPower2Off();
      IS_POWER2_TRIPPED = true;
      BOOSTER2_SHUTDOWN_TIME = millis();
      }
    }
  } 
  if (BOOST2_AMPS_AVG < FAST_BLOW_LIMIT) {
    IS_POWER2_FAST_PRE_TRIPPED = false;
  }

  // Stage 3

  if (BOOST2_AMPS_AVG > CURRENT_LIMIT) {
    if (IS_POWER2_SLOW_PRE_TRIPPED == false) {
      POWER2_SLOW_PRE_TIME = millis();
      IS_POWER2_SLOW_PRE_TRIPPED = true;
    }
    if (IS_POWER2_SLOW_PRE_TRIPPED == true) {
      if (millis() >= POWER2_SLOW_PRE_TIME + SLOW_BLOW_TIME){
      POWER2_SLOW_PRE_TIME += SLOW_BLOW_TIME;
      turnPower2Off();
      IS_POWER2_TRIPPED = true;
      BOOSTER2_SHUTDOWN_TIME = millis();
      }
    }
  } 
  if (BOOST2_AMPS_AVG < CURRENT_LIMIT) {
    IS_POWER2_SLOW_PRE_TRIPPED = false;
  }  

  // Attempt To Repower The Boosters

  // Booster 1 Repower Procedure

  if ((IS_POWER1_TRIPPED == true) && (MICRO_PWR == true)) {
    if (millis() - BOOSTER1_LAST_POWER_ON >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER1_REBOOT_COUNT = 0;
    }
    else if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER1_REBOOT_COUNT <= 5)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*5)) && (BOOSTER1_REBOOT_COUNT <= 10)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*15)) && (BOOSTER1_REBOOT_COUNT <= 20)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER1_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*30)) && (BOOSTER1_REBOOT_COUNT <= 25)) {
      turnPower1On();
      BOOSTER1_REBOOT_COUNT = BOOSTER1_REBOOT_COUNT + 1;
    }
  }

  // Booster 2 Repower Procedure

 if ((IS_POWER2_TRIPPED == true) && (MICRO_PWR == true)) {
    if (millis() - BOOSTER2_LAST_POWER_ON >= BOOSTER_TRIPPED_COUNTER_RESET) {
      BOOSTER2_REBOOT_COUNT = 0;
    }
    else if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= BOOSTER_REBOOT_TIME) && (BOOSTER2_REBOOT_COUNT <= 5)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*5)) && (BOOSTER2_REBOOT_COUNT <= 10)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*15)) && (BOOSTER2_REBOOT_COUNT <= 20)) {
      turnPower2On();
      BOOSTER2_REBOOT_COUNT = BOOSTER2_REBOOT_COUNT + 1;
    }
    else if (((millis() - BOOSTER2_SHUTDOWN_TIME) >= (BOOSTER_REBOOT_TIME*30)) && (BOOSTER2_REBOOT_COUNT <= 25)) {
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
      digitalWrite(ALARM1_PIN, LOW);	// kc deactivate the short circit alarm		
    }
    if (BOOST2_ENABLED == true) {     // Booster 2 Enabled Icon
      display.drawBitmap(64, 0, POWERICON, 16, 16, WHITE);   
      digitalWrite(ALARM2_PIN, LOW);	// kc deactivate the short circit alarm		
    }
    if (IS_POWER1_TRIPPED == true) {     // Booster 1 Overload Icon
      display.drawBitmap(16, 0, OVERLOADICON, 16, 16, WHITE); 
      digitalWrite(ALARM1_PIN, HIGH);	// kc activate the short circit alarm	
    }
    if (IS_POWER2_TRIPPED == true) {     // Booster 2 Overload Icon
      display.drawBitmap(80, 0, OVERLOADICON, 16, 16, WHITE);   
      digitalWrite(ALARM2_PIN, HIGH);	// kc activate the short circit alarm	
    }
    if (IS_POWER1_FAST_PRE_TRIPPED == true) {     // Booster 1 Pre Fast Overload Icon
      display.drawBitmap(32, 0, ALARM1ICON, 16, 16, WHITE);   
    }
    if (IS_POWER2_FAST_PRE_TRIPPED == true) {     // Booster 2 Pre Fast Overload Icon
      display.drawBitmap(96, 0, ALARM1ICON, 16, 16, WHITE);   
    }
    if (IS_POWER1_SLOW_PRE_TRIPPED == true) {     // Booster 1 Pre Slow Overload Icon
      display.drawBitmap(48, 0, ALARM2ICON, 16, 16, WHITE);   
    }
    if (IS_POWER2_SLOW_PRE_TRIPPED == true) {     // Booster 2 Pre Slow Overload Icon
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
    display.println(BOOST1_AMPS_AVG_DISPLAY, 2);

    // Display Booster 2 Load In Amps

    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(80, 50);
    display.setTextWrap(false);
    display.println(BOOST2_AMPS_AVG_DISPLAY, 2);

    // Finally Draw the Screen

    display.display();

    LAST_PRINT_DISPLAY_TIME = millis();

  }

  // Save PWR Button State For Comparison Next Run
  
  POWER_BTN_LAST = MICRO_PWR;

  // Check For Railsync Activity And Turn Off Or ON Track Power Depending On Activity After So Many Seconds

  if (RPWM_TIMER_ACTIVE == false){
    RPWM_TIMER = millis();
    RPWM_COUNT = 0;
    RPWM_TIMER_ACTIVE = true;
  } 
  RPWM_DETECT = digitalRead(RPWM_DETECT_PIN);
  if ((millis() - RPWM_TIMER) <= RPWM_TIMER_LIMIT) {
    if (RPWM_LAST != RPWM_DETECT) {
      RPWM_COUNT = ++RPWM_COUNT;
    }
  }
  if ((millis() - RPWM_TIMER) > (RPWM_TIMER_LIMIT)) {
    if ((RPWM_COUNT >= RPWM_SIG_EDGES) && (MICRO_PWR == HIGH) && (IS_POWER1_TRIPPED == false) && (IS_POWER2_TRIPPED == false)) {
      turnPowerOn();
    }
    if (RPWM_COUNT <= RPWM_SIG_EDGES) {
      turnPowerOff();
    } 
    RPWM_TIMER_ACTIVE = false;   
  }

  RPWM_LAST = RPWM_DETECT;

  // Print Offset Values For Initial Config
  
  if (PRINT_BOOST_OFFSET == true) {
    if (millis() - PRINT_BOOST_LAST_TIME >= PRINT_BOOST_DELAY_TIME) {
      Serial.println();
      Serial.print("B1 ");
      Serial.print(BOOST1_CURRENT);
      Serial.print(" B2 ");
      Serial.print(BOOST2_CURRENT);
      PRINT_BOOST_LAST_TIME = millis();
    }
  }

}
