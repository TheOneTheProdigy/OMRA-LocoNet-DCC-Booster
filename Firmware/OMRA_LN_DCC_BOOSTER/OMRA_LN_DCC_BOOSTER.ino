//  DCC / LocoNet RailSync Booster - Dual 5A Booster Districts With Overload Protection, Overload Alarm Output, And OLED Screen With Both Booster's Current and Status

//  2025 Lance Bradley  Ozarks Model Railroad Assocation
//  2024 Kurt Clement   Ozarks Model Railroad Assocation

//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club In Springfield Missouri

// Micro Pin Assignments
int C_SENSE1_MICRO = A1; // Booster1 current sensing pin.
int C_SENSE2_MICRO = A2; // Booster2 current sensing pin.
int N_FAULT1_MICRO = 14; // Booster 1 instant DRV8874 fault pin.
int N_FAULT2_MICRO = 15; // Booster 2 instant DRV8874 fault pin.
int EN1_MICRO = 21; // Booster1 enable pin.
int EN2_MICRO = 18; // Booster2 enable pin
int PWM_DETECT_MICRO = 4; // Pin to detect valid railsync packets and shutdown track power if no signal.
int LN_RX_MICRO = 8; // Pin To Receive LocoNet Packets From The LocoNet Buss
int LN_TX_MICRO = 9; // Pin To Send LocoNet Packets To The LocoNet Buss
int ALM1_MICRO = 10; // Pin To Activate LED and Piezo Alarm For Booster 1 Short Circuit Trip
int ALM2_MICRO = 5;  // Pin To Activate LED and Piezo Alarm For Booster 2 Short Circuit Trip

// *** Below Should Not Need Touched ***
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "avdweb_AnalogReadFast.h"
#include "ARTWORK.h"
#include "CONFIG.h"
#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


// Variables
int PWM_COUNT = 0;
int PWM_DETECT = 1;
int PWM_LAST = 1;
unsigned long PWM_TIMER;
bool PWM_TIMER_ACTIVE = false;
bool BOOST1_ENABLED = false;
bool IS_POWER1_TRIPPED = false;
bool IS_POWER1_INSTANT_PRE_TRIPPED = false;
bool IS_POWER1_THERMAL_PRE_TRIPPED = false;
bool IS_POWER1_DRV8874_OK = false;
bool IS_POWER1_DRV8874_TRIPPED = false;
bool IS_POWER1_THERMAL_TRIPPED = false;
bool BOOST2_ENABLED = false;
bool IS_POWER2_TRIPPED = false;
bool IS_POWER2_INSTANT_PRE_TRIPPED = false;
bool IS_POWER2_THERMAL_PRE_TRIPPED = false;
bool IS_POWER2_DRV8874_OK = false;
bool IS_POWER2_DRV8874_TRIPPED = false;
bool IS_POWER2_THERMAL_TRIPPED = false;
float BOOST1_AMPS = 0;
float BOOST2_AMPS = 0;
float LAST_PRINT_DISPLAY_TIME = 0;
float LAST_DEBUG_REFRESH_TIME = 0;
unsigned long POWER1_INSTANT_PRE_TIME = 0;
unsigned long POWER1_THERMAL_PRE_TIME = 0;
unsigned long POWER1_DRV8874_NFAULT_TIME = 0;
unsigned long POWER2_INSTANT_PRE_TIME = 0;
unsigned long POWER2_THERMAL_PRE_TIME = 0;
unsigned long POWER2_DRV8874_NFAULT_TIME = 0;
unsigned long BOOSTER1_SHUTDOWN_TIME = 0;
unsigned long BOOSTER2_SHUTDOWN_TIME = 0;
unsigned long CURRENT_CYCLE_TIME = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Start Setup
void setup() {

  if (PRINT_DEBUG){
    Serial.begin(9600);
  }

  // Setup IO Pins
  pinMode(EN1_MICRO, OUTPUT); 
  pinMode(EN2_MICRO, OUTPUT);  
  pinMode(LN_TX_MICRO, OUTPUT);
  pinMode(ALM1_MICRO, OUTPUT);
  pinMode(ALM2_MICRO, OUTPUT);
  pinMode(N_FAULT1_MICRO, INPUT_PULLUP);
  pinMode(N_FAULT2_MICRO, INPUT_PULLUP);

  
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
  display.println("V2.1.10");
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
  IS_POWER1_THERMAL_TRIPPED = false;
}
void turnPower2On() {
  digitalWrite(EN2_MICRO, HIGH);
  BOOST2_ENABLED = true;
  IS_POWER2_TRIPPED = false;
  IS_POWER2_THERMAL_TRIPPED = false;
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
  
  // Set Current Millis As Time
  CURRENT_CYCLE_TIME = millis();
  
  // Check Current Of Boosters 1 and 2
    
    // Booster 1

  // Current Sense Math and ADC Read In
  BOOST1_AMPS = ((analogReadFast(C_SENSE1_MICRO) * 0.00488) - BOOST1_CS_OFFSET) / 0.4352; // Each ADC step represents approx 4.88mV. .4352 = .000455 x 956.5 where .000455 is uA/A and 1K + 22K = 956.5.
  BOOST1_AMPS = constrain(BOOST1_AMPS, 0, 9.9); // Filter Below 0A and above 9.9A..

  // Over current fault trip routine
  if (BOOST1_AMPS >= OC_LIMIT1 && !IS_POWER1_TRIPPED && !IS_POWER1_INSTANT_PRE_TRIPPED) {
      POWER1_INSTANT_PRE_TIME = CURRENT_CYCLE_TIME;
      IS_POWER1_INSTANT_PRE_TRIPPED = true;
    }
    else if (IS_POWER1_INSTANT_PRE_TRIPPED && BOOST1_AMPS < OC_LIMIT1 && (CURRENT_CYCLE_TIME - POWER1_INSTANT_PRE_TIME) >= OC_BOOST1_INRUSH) {
      IS_POWER1_INSTANT_PRE_TRIPPED = false;
    }

  if (IS_POWER1_INSTANT_PRE_TRIPPED && !IS_POWER1_TRIPPED && (CURRENT_CYCLE_TIME - POWER1_INSTANT_PRE_TIME) >= OC_BOOST1_INRUSH) {
    IS_POWER1_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
    turnPower1Off();
  }

  // DRV8874 NFAULT over current fault routine
  IS_POWER1_DRV8874_OK = digitalRead(N_FAULT1_MICRO);

  if (!IS_POWER1_DRV8874_OK && !IS_POWER1_DRV8874_TRIPPED) {
    POWER1_DRV8874_NFAULT_TIME = CURRENT_CYCLE_TIME;
    IS_POWER1_DRV8874_TRIPPED = true;
  }
    else if(IS_POWER1_DRV8874_OK && IS_POWER1_DRV8874_TRIPPED && (CURRENT_CYCLE_TIME - POWER1_DRV8874_NFAULT_TIME) >= OC_BOOST1_INRUSH) {
      IS_POWER1_DRV8874_TRIPPED = false;
    } 

  if (!IS_POWER1_DRV8874_OK && !IS_POWER1_TRIPPED && (CURRENT_CYCLE_TIME - POWER1_DRV8874_NFAULT_TIME) >= OC_BOOST1_INRUSH) {
    IS_POWER1_TRIPPED = true;
    BOOSTER1_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
    turnPower1Off();
  }

    // Thermal over current fault trip routine
  if (ENABLE_THERMAL_PROTECTION) {
    if (BOOST1_AMPS >= OC_THERM_LIMIT1 && !IS_POWER1_TRIPPED && !IS_POWER1_THERMAL_PRE_TRIPPED) {
        POWER1_THERMAL_PRE_TIME = CURRENT_CYCLE_TIME;
        IS_POWER1_THERMAL_PRE_TRIPPED = true;
      }
      else if (IS_POWER1_THERMAL_PRE_TRIPPED && BOOST1_AMPS < OC_THERM_LIMIT1 && (CURRENT_CYCLE_TIME - POWER1_THERMAL_PRE_TIME) >= OC_BOOST1_INRUSH) {
        IS_POWER1_THERMAL_PRE_TRIPPED = false;
      }

    if (IS_POWER1_THERMAL_PRE_TRIPPED && !IS_POWER1_TRIPPED && (CURRENT_CYCLE_TIME - POWER1_THERMAL_PRE_TIME) >= OC_BOOST1_THERM) {
      IS_POWER1_TRIPPED = true;
      IS_POWER1_THERMAL_TRIPPED = true;
      BOOSTER1_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
      turnPower1Off();
    }
  }

  // Booster 2

  // Current Sense Math and ADC Read In
  BOOST2_AMPS = ((analogReadFast(C_SENSE2_MICRO) * 0.00488) - BOOST2_CS_OFFSET) / 0.4352; // Each ADC step represents approx 4.88mV. .4352 = .000455 x 956.5 where .000455 is uA/A and 1K + 22K = 956.5.
  BOOST2_AMPS = constrain(BOOST2_AMPS, 0, 9.99); // Filter Below 0A and above 9.9A.

  // Over current fault trip routine  
  if (BOOST2_AMPS >= OC_LIMIT2 && !IS_POWER2_TRIPPED && !IS_POWER2_INSTANT_PRE_TRIPPED) {
      POWER2_INSTANT_PRE_TIME = CURRENT_CYCLE_TIME;
      IS_POWER2_INSTANT_PRE_TRIPPED = true;
    }
    else if (IS_POWER2_INSTANT_PRE_TRIPPED && BOOST2_AMPS < OC_LIMIT2 && (CURRENT_CYCLE_TIME - POWER2_INSTANT_PRE_TIME) >= OC_BOOST2_INRUSH) {
      IS_POWER2_INSTANT_PRE_TRIPPED = false;
    }

  if (IS_POWER2_INSTANT_PRE_TRIPPED && !IS_POWER2_TRIPPED && (CURRENT_CYCLE_TIME - POWER2_INSTANT_PRE_TIME) >= OC_BOOST2_INRUSH) {
    IS_POWER2_TRIPPED = true;
    BOOSTER2_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
    turnPower2Off();
  }

    // DRV8874 2 NFAULT over current fault routine
  IS_POWER2_DRV8874_OK = digitalRead(N_FAULT2_MICRO);

  if (!IS_POWER2_DRV8874_OK && !IS_POWER2_DRV8874_TRIPPED) {
    POWER2_DRV8874_NFAULT_TIME = CURRENT_CYCLE_TIME;
    IS_POWER2_DRV8874_TRIPPED = true;
  }
    else if(IS_POWER2_DRV8874_OK && IS_POWER2_DRV8874_TRIPPED && (CURRENT_CYCLE_TIME - POWER2_DRV8874_NFAULT_TIME) >= OC_BOOST2_INRUSH) {
      IS_POWER1_DRV8874_TRIPPED = false;
    } 

  if (!IS_POWER2_DRV8874_OK && !IS_POWER2_TRIPPED && (CURRENT_CYCLE_TIME - POWER2_DRV8874_NFAULT_TIME) >= OC_BOOST2_INRUSH) {
    IS_POWER2_TRIPPED = true;
    BOOSTER2_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
    turnPower2Off();
  }

    // Thermal over current fault trip routine
  if (ENABLE_THERMAL_PROTECTION) {
    if (BOOST2_AMPS >= OC_THERM_LIMIT2 && !IS_POWER2_TRIPPED && !IS_POWER2_THERMAL_PRE_TRIPPED) {
        POWER2_THERMAL_PRE_TIME = CURRENT_CYCLE_TIME;
        IS_POWER2_THERMAL_PRE_TRIPPED = true;
      }
      else if (IS_POWER2_THERMAL_PRE_TRIPPED && BOOST2_AMPS < OC_THERM_LIMIT2 && (CURRENT_CYCLE_TIME - POWER2_THERMAL_PRE_TIME) >= OC_BOOST2_INRUSH) {
        IS_POWER2_THERMAL_PRE_TRIPPED = false;
      }

    if (IS_POWER2_THERMAL_PRE_TRIPPED && !IS_POWER2_TRIPPED && (CURRENT_CYCLE_TIME - POWER2_THERMAL_PRE_TIME) >= OC_BOOST2_THERM) {
      IS_POWER2_TRIPPED = true;
      IS_POWER2_THERMAL_TRIPPED = true;
      BOOSTER2_SHUTDOWN_TIME = CURRENT_CYCLE_TIME;
      turnPower2Off();
    }
  }

  // Attempt To Repower The Boosters

  // Booster 1 Repower Procedure
  if ((IS_POWER1_THERMAL_TRIPPED) && (IS_POWER1_TRIPPED) && ((CURRENT_CYCLE_TIME - BOOSTER1_SHUTDOWN_TIME) >= OC_THERM_TIMEOUT1)) {
    turnPower1On();
  }
    else if ((!IS_POWER1_THERMAL_TRIPPED) && (IS_POWER1_TRIPPED) && ((CURRENT_CYCLE_TIME - BOOSTER1_SHUTDOWN_TIME) >= OC_TIMEOUT1)) { // Retry if timer resets.
      turnPower1On();
    }
  // Booster 2 Repower Procedure
  if ((IS_POWER2_THERMAL_TRIPPED) && (IS_POWER2_TRIPPED) && ((CURRENT_CYCLE_TIME - BOOSTER2_SHUTDOWN_TIME) >= OC_THERM_TIMEOUT2)) {
    turnPower2On();
  }
    else if ((!IS_POWER2_THERMAL_TRIPPED) && (IS_POWER2_TRIPPED) && ((CURRENT_CYCLE_TIME - BOOSTER2_SHUTDOWN_TIME) >= OC_TIMEOUT2)) { // Retry if timer resets.
      turnPower2On();
    }

  // Draw Display
  if (millis() - LAST_PRINT_DISPLAY_TIME >= PRINT_DISPLAY_DELAY_TIME) {
    display.clearDisplay(); // Clear Previous Display

    // Display Icons In Yellow Bar    
    if (BOOST1_ENABLED) {     // Booster 1 Enabled Icon
      display.drawBitmap(0, 0, POWERICON, 16, 16, WHITE);
      digitalWrite(ALM1_MICRO, LOW);  // Deactivate The Short Circuit Alarm   
    }
    if (BOOST2_ENABLED) {     // Booster 2 Enabled Icon
      display.drawBitmap(64, 0, POWERICON, 16, 16, WHITE);   
      digitalWrite(ALM2_MICRO, LOW);  // Deactivate The Short Circuit Alarm   
    }
    if (IS_POWER1_TRIPPED) {     // Booster 1 Overload Icon
      display.drawBitmap(16, 0, OVERLOADICON, 16, 16, WHITE); 
      digitalWrite(ALM1_MICRO, HIGH); // Activate The Short Circuit alarm 
    }
    if (IS_POWER2_TRIPPED) {     // Booster 2 Overload Icon
      display.drawBitmap(80, 0, OVERLOADICON, 16, 16, WHITE);   
      digitalWrite(ALM2_MICRO, HIGH); // Activate The Short Circuit alarm 
    }
    if (IS_POWER1_DRV8874_TRIPPED) {     
      display.drawBitmap(32, 0, ALARM1ICON, 16, 16, WHITE);   
    }
    if (IS_POWER2_DRV8874_TRIPPED) {     
      display.drawBitmap(96, 0, ALARM1ICON, 16, 16, WHITE);   
    }
    if (IS_POWER1_INSTANT_PRE_TRIPPED || IS_POWER1_THERMAL_PRE_TRIPPED) {     
      display.drawBitmap(48, 0, ALARM2ICON, 16, 16, WHITE);   
    }
    if (IS_POWER2_INSTANT_PRE_TRIPPED || IS_POWER2_THERMAL_PRE_TRIPPED) {     
      display.drawBitmap(112, 0, ALARM2ICON, 16, 16, WHITE);   
    }
    
    // Display Booster District Name and INFO Box
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setTextWrap(false);
    display.setCursor(2, 20);
    display.println("5A X2 LOCONET BOOSTER");
    
    // Display Booster 1 Load In Amps
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setTextWrap(false);
    display.setCursor(5, 32);
    display.println("B1 AMPS B2");
    display.setCursor(5, 50);
    display.println(BOOST1_AMPS, 1);

    // Display Booster 2 Load In Amps
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setTextWrap(false);
    display.setCursor(90, 50);
    display.println(BOOST2_AMPS, 1);

    // Finally Draw the Screen
    display.display();

    LAST_PRINT_DISPLAY_TIME = millis();
  }

  // Check For Railsync Activity And Turn Off Or ON Track Power Depending On Activity After Alloted Time
  if (!PWM_TIMER_ACTIVE){
    PWM_TIMER = millis();
    PWM_COUNT = 0;
    PWM_TIMER_ACTIVE = true;
  } 
  PWM_DETECT = digitalRead(PWM_DETECT_MICRO);
  if ((millis() - PWM_TIMER) <= PWM_TIMER_LIMIT) {
    if (PWM_LAST != PWM_DETECT) {
      PWM_COUNT = ++PWM_COUNT;
    }
  }  
  if ((millis() - PWM_TIMER) > (PWM_TIMER_LIMIT)) {  
    if ((PWM_COUNT >= PWM_SIG_EDGES) && (!IS_POWER1_TRIPPED)) {
      turnPower1On();
    }
    if ((PWM_COUNT >= PWM_SIG_EDGES) && (!IS_POWER2_TRIPPED)) {
      turnPower2On();
    }
    if (PWM_COUNT <= PWM_SIG_EDGES) {
      turnPowerOff();
    } 
    PWM_TIMER_ACTIVE = false;   
  }
  PWM_LAST = PWM_DETECT;

  // Debug Code
   if ((PRINT_DEBUG) && (CURRENT_CYCLE_TIME - LAST_DEBUG_REFRESH_TIME >= DEBUG_REFRESH_TIME)) {
    LAST_DEBUG_REFRESH_TIME = CURRENT_CYCLE_TIME;
    Serial.println();
    Serial.println();
    Serial.println("OMRA Booster Debug Report");
    Serial.println();   
    Serial.println(CURRENT_CYCLE_TIME);
    Serial.println();
    Serial.println("Booster 1 Enabled");
    Serial.println(BOOST1_ENABLED);
    Serial.println(); 
    Serial.println("Booster 1 Tripped");
    Serial.println(IS_POWER1_TRIPPED);
    Serial.println();  
    Serial.println("Booster 1 AMPS");
    Serial.println(BOOST1_AMPS, 2);
    Serial.println();
    Serial.println();
    Serial.println("Booster 2 Enabled");
    Serial.println(BOOST2_ENABLED);
    Serial.println(); 
    Serial.println("Booster 2 Tripped");
    Serial.println(IS_POWER2_TRIPPED);
    Serial.println();
    Serial.println("Booster 2 AMPS");
    Serial.println(BOOST2_AMPS, 2);
   }

}
