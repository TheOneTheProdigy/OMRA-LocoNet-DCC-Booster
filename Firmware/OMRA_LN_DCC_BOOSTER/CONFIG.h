//  DCC / LocoNet RailSync Booster - Dual 5A Booster Districts With Overload Protection, Overload Alarm Output, And OLED Screen With Both Booster's Current and Status

//  2024-2026 Lance Bradley  Ozarks Model Railroad Assocation
//  2024 Kurt Clement   Ozarks Model Railroad Assocation

//  This Is The Firmware For The 10A-X2 Booster For The OMRA Club In Springfield Missouri

// Debug
bool PRINT_DEBUG = false; // Print debug info to serial.
float DEBUG_REFRESH_TIME = 2500; // Milliseconds to wait before posting another debug report to serial.

// Zero Current Measurement Offset
float BOOST1_CS_OFFSET = 0.2174; // Zeros the offset for current measurements. This is a voltage on the current sense pin.  
float BOOST2_CS_OFFSET = 0.2174; // + Decreases - Increases current readings. If VREF 5.0V / 0.2174V = 0A.

// Overcurrent Protection
float OC_LIMIT = 4; // Limit to trigger the over current protection in amps. Anything above this will blow after the inrush delay.
float OC_TIMEOUT = 2000; // Milliseconds to wait to energize the track again between short circuits.
unsigned long OC_BOOST1_INRUSH = 150; // Milliseconds to allow a inrush of current before tripping instant overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.
unsigned long OC_BOOST2_INRUSH = 150; // Milliseconds to allow a inrush of current before tripping instant overcurrent protection. Useful for allowing large keepalives to be powered on and not detect as a short.

// Railsync Monitoring / Booster Enable
float PWM_TIMER_LIMIT = 250; // Milliseconds to go without valid railsync commands before boosters shutdown. 
int PWM_SIG_EDGES = 2; // Edges to trigger railsync active or not within PWM_TRIGGER_LIMIT timeframe.

// OLED HMI Display
float PRINT_DISPLAY_DELAY_TIME = 100; // Refresh OLED screen 10 times a second.

// Micro Pin Assignments
int C_SENSE1_MICRO = A1; // Booster1 current sensing pin.
int C_SENSE2_MICRO = A2; // Booster2 current sensing pin.
int EN1_MICRO = 21; // Booster1 enable pin.
int EN2_MICRO = 18; // Booster2 enable pin
int PWM_DETECT_MICRO = 4; // Pin to detect valid railsync packets and shutdown track power if no signal.
int LN_RX_MICRO = 8; // Pin To Receive LocoNet Packets From The LocoNet Buss
int LN_TX_MICRO = 9; // Pin To Send LocoNet Packets To The LocoNet Buss
int ALM1_MICRO = 10; // Pin To Activate LED and Piezo Alarm For Booster 1 Short Circuit Trip
int ALM2_MICRO = 5;  // Pin To Activate LED and Piezo Alarm For Booster 2 Short Circuit Trip