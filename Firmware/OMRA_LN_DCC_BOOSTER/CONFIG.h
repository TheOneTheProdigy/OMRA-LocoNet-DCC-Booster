//  DCC / LocoNet RailSync Booster - Dual 6A Booster Districts With Overload Protection, Overload Alarm Output, And OLED Screen With Both Booster's Current and Status

//  2024-2026 Lance Bradley  Ozarks Model Railroad Assocation
//  2024-2026 Kurt Clement   Ozarks Model Railroad Assocation

//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club In Springfield Missouri

// Debug
bool PRINT_DEBUG = false; // Print debug info to serial.
float DEBUG_REFRESH_TIME = 2500; // Milliseconds to wait before posting another debug report to serial.

// Zero Current Measurement Offset
float BOOST1_CS_OFFSET = 0.2174; // Zeros the offset for current measurements. This is a voltage on the current sense pin.  
float BOOST2_CS_OFFSET = 0.2174; // + Decreases - Increases current readings. If VREF 5.0V / 0.2174V = 0A.

// Instant Overcurrent Protection
float OC_LIMIT1 = 4.0; // Limit to trigger the over current protection in amps. Anything above this will blow after the inrush delay.
float OC_LIMIT2 = 4.0; // Limit to trigger the over current protection in amps. Anything above this will blow after the inrush delay.
float OC_TIMEOUT1 = 2000; // Milliseconds to wait to energize the track again between short circuits.
float OC_TIMEOUT2 = 2000; // Milliseconds to wait to energize the track again between short circuits.
unsigned long OC_BOOST1_INRUSH = 150; // Milliseconds to allow a inrush of current before tripping instant overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.
unsigned long OC_BOOST2_INRUSH = 150; // Milliseconds to allow a inrush of current before tripping instant overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.

// Thermal Overcurrent Protection
bool ENABLE_THERMAL_PROTECTION = true; // Enable or disable the thermal overcurrent protection.
float OC_THERM_LIMIT1 = 3.25; // Limit to trigger the thermal over current protection in amps. Anything above this will blow after the OC_BOOST_THERM delay.
float OC_THERM_LIMIT2 = 3.25; // Limit to trigger the thermal over current protection in amps. Anything above this will blow after the OC_BOOST_THERM delay.
float OC_THERM_TIMEOUT1 = 4000; // Milliseconds to wait to energize the track again between thermal overloads.
float OC_THERM_TIMEOUT2 = 4000; // Milliseconds to wait to energize the track again between thermal overloads.
unsigned long OC_BOOST1_THERM = 8000; // Milliseconds to allow a inrush of current before tripping thermal overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.
unsigned long OC_BOOST2_THERM = 8000; // Milliseconds to allow a inrush of current before tripping thermal overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.

// Railsync Monitoring / Booster Enable
float PWM_TIMER_LIMIT = 250; // Milliseconds to go without valid railsync commands before boosters shutdown. 
int PWM_SIG_EDGES = 2; // Edges to trigger railsync active or not within PWM_TRIGGER_LIMIT timeframe.

// OLED HMI Display
float PRINT_DISPLAY_DELAY_TIME = 100; // Refresh OLED screen 10 times a second.