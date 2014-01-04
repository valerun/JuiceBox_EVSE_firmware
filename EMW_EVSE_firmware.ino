/*
EMotorWerks JuiceBox - an open-source 15kW EVSE

Micro-Controller: Arduino Pro Mini 5V, (based on a ATmega328P-PU microcontroller)

this version is matching V8.0-8.3 boards

Basic code structure:
Startup:
* initialize pins
* set output power level based on trimpot
* set duty cycle to 0

In endless loop:
* check for J1772 state
* check for EV & diode presence
* check for EV requesting power (state C)
* close relay to provide power (this is optional and code will work if no relay is present)
* run loop with power until non-C state detected or a button pressed
*     measure current and increment energy meter
*     display major params on the screen (this is optional and code will work if no LCD is present)

Created Jan 2013 by Electric Motor Werks, Inc. / Valery Miftakhov, Copyright 2013+

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License

In a nutshell, it says if you wish to modify and distribute any derivation of this code, 
you must also distribute your modifications, and you MUST make the complete source code available.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: http://www.gnu.org/licenses/
*/

//------------------------------ MAIN SWITCHES -----------------------------------
#define DEBUG 1 // this results in many additional printouts
// the following results in much more frequent reporting of data by JuiceBox to EmotorWerks servers
// PLEASE DO NOT USE in your JuiceBox UNLESS AUTHORIZED BY EMotorWerks - this overloads our servers
// and slows down the system for everyone. JuiceBoxes that consistently report more frequently than 
// every ~1 minute will be permanently banned from our network
// #define DEBUG_WIFI 
#define AC1075
const int R_C=120; // this is the value of the shunting resistor. see datasheet for the right value. 
const int V_AC_threshold=120; // normally 164
// #define CT_8349-1500
// #define CT_8420-1000
// #define CT_3100
// #define JB_WiFi // is WiFi installed & we are using WiFlyHQ library?
 #define JB_WiFi_simple // is WiFi installed and we are just pushing data?
// #define LCD_SGC // old version of the u144 LCD - used in some early JuiceBoxes
//#define PCB_81 // 8.1 version of the PCB
 #define PCB_83 // 8.3+ version of the PCB, includes 8.6, 8.7 versions
 #define VerStr "V8.6.5" // detailed exact version of firmware (thanks Gregg!)
 #define GFI // need to be uncommented for GFI functionality
// #define trim120current
//------------------------------- END MAIN SWITCHES ------------------------------

#include <Arduino.h>
#include <avr/interrupt.h>

// EEPROM handler
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"

// WiFi library mega slon
#include <SoftwareSerial.h>
#include <WiFlyHQ.h>

//-------------------- WiFi UDP settings --------------------------------------------------------------------
// in JB_WiFI_simple mode, the IP settings are not required as all IP settings are pre-loaded on WiFly module
// const char *serverIP="50.21.181.240";
// const int UDPport=8042;
const char UDPpacketEndSig[6]="\n"; // what is the signature of the packet end (should match the WiFly setting)

// need this to remap PWM frequency
#include <TimerOne.h>

// our LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
// 4D Systems in its infinite wisdom decided to completely change the command set in its new 
// release of the LCDs so we (and other countless developers) had to completely rewrite our
// LCD libraries
#ifdef LCD_SGC
  #include <uLCD_144.h>
  uLCD_144 *myLCD;
#else
  #include <uLCD_144_SPE.h>
  uLCD_144_SPE *myLCD;
#endif

byte LCD_on=0; // this defines base vs. premium versions

//------------------ current sensor calibration - only for AC1075 for now -----------------------------------------------
// the current sensing is done using a simple diode rectifier. As such, there is natural non-linearity to readings
// this lookup table takes care of that. Alternative approach is a precision rectifier using an op amp but 
// that adds a bit in parts and cost so taking a software approach here
// one entry per 0.1V in observed voltage on A1 pin. Since we have a 3.3v zener on that pin, need only 32-element array
// array contains current value in 0.1A units
// if the sensing was purely linear, we'd expect ~18A/volt but the forward drop on diode varies from 0.4V to 1V 
// in JuiceBox operating range
const unsigned int AC1075_calibration[32]={0,0,5,28,50,70,95,120,140,
                                           165,188,210,230,252,275,298,320,
                                           342,365,385,409,434,458,484,510,
                                           535,559,585,610,635,660,685};
//------------------ END current sensor calibration ---------------------------------------------------------------------

//---------------- savings consts for displays 
const int gascost=350; // in cents per gallon
const int mpg=25; // mpg of the gasoline car
const int ecost=12; // in cents per kWhr
const int whpermile=300; // energy efficiency of the ecar
int savingsPerKWH; // will be recalced later
//---------------- end of the energy constants

//---------------- pin-out constants ----------------
//---------------- analog inputs
const byte pin_pV=0; // pilot signal sensed through a 3-element divider 
const byte pin_V=1; // input voltage 
const byte pin_C=2; // AC current - as measured by the current transformer
const byte pin_throttle=3; // wired to a trimpot on a board
// pins A4 / A5 reserved for SPI comms to RTC chip
#ifdef trim120current
  const byte pin_throttle120=5; // when RTC is not used, this is an input used to set 120V target current (0-30A range)
#endif

//---------------- digital inputs / outputs
const byte pin_sRX=2; // SoftSerial RX - used for LCD or WiFi (default)
const byte pin_sTX=4; // SoftSerial TX - used for LCD or WiFi (default)
// GFI trip pin - goes high on GFI fault, driven by the specialized circuit based on LM1851 
// has to be pin 3 as only pin 2 and 3 are available for interrupts on Pro Mini
const byte pin_GFI=3; 
const byte pin_inRelay=5; 
const byte pin_ctrlBtn=6; // control button, pulled down to GND
const byte pin_ctrlBtn2=8; // control button, pulled down to GND
const byte pin_PWM=9; // J pilot PWM pin

// pulling this pin high will trigger WPS application on the wifi module - on premium units, 
// this is also tied to one of the buttons of the remote so no Arduino action is needed
const byte pin_WPS=10; 

const byte pin_GFItest=12; // pin wired to a GFCI-tripping relay - for the periodic testing of the GFCI circuit & stuck relay detection
// rest of digital pins - 7,11,13 - open and not connected
//---------------- END PINOUTS -----------------------

//============= NON-VOLATILE INFO  =====
struct config_t {
  unsigned long energy; // total energy during lifetime, in kWHr
  // sensor config
  float Vcal;
  float Vcal_k;
  float Ccal;
  int day;
  int hour;
  int mins;
  unsigned long clock_offset; // poor man's RTC - reference point
  // IDs for linking the JB to customer's account - a bunch of random ints
  unsigned int IDstamp[16]; 
} configuration;
//=======================================

//==================================== calibration constants etc
const float Aref=5.; // should be close
float pV_min=-12.;
float V_J1772_pin_=0; // global pilot voltage
const float upper_pV_R=100.; // 100k
const float lower_pV_R=27.; // 27k
float V_Ard_pin_0;
//===============================================================

//========== define J1772 states ===============================
// defaults
const float def_state_A_Vmin=10.5, def_state_A_Vmax=14; 
const float def_state_B_Vmin=7.5, def_state_B_Vmax=10.5; 
const float def_state_C_Vmin=4.5, def_state_C_Vmax=7.5; 
const float def_state_D_Vmin=1.5, def_state_D_Vmax=4.5; 
const float def_state_E_Vmin=-1.5, def_state_E_Vmax=1.5; 
const float def_state_F_Vmin=-14., def_state_F_Vmax=-10.; 
// now adjusted for the actual voltages
float state_A_Vmin, state_A_Vmax; 
float state_B_Vmin, state_B_Vmax; 
float state_C_Vmin, state_C_Vmax; 
float state_D_Vmin, state_D_Vmax; 
float state_E_Vmin, state_E_Vmax; 
float state_F_Vmin, state_F_Vmax; 
#define STATE_INVALID -1
#define STATE_A 0
#define STATE_B 1 
#define STATE_C 2
#define STATE_D 3
#define STATE_E 4
#define STATE_F 5
//=========== end definition of J1772 states ===================

// these should be global vars  -----------------------------
unsigned int duty=0;
const unsigned int PWM_res=1024;
unsigned int MAXDUTY=970; // <97% to stay in AC charging zone for J1772 standard

const float maxC=60; // max rated current
const float inV_AC_nominal=240; // default line voltage
float inV_AC=0; // this will be measured
const float nominal_outC_240V=30; // 30A by default from a 240VAC line
const float nominal_outC_120V=15; // 15A by default from a 120VAC line
float outC=nominal_outC_240V; 
float power=0;
float energy=0; // how much energy went through - in kWHrs 

char str[64]; // main temp str buffer - do not expand beyond 64 - may run out of memory
char tempstr[16], hstr[3], mstr[3]; // scratchpad for text operations

byte GFI_tripped=0;
byte GFI_trip_count=0;

int cycleVar=0;
int n=0;
// ------------- end global vars ---------------------------

//------------- timing parameters --------------------------
unsigned long timer=0, timer0=0, timer_now=0, clock_offset=0, timer_chstart=0;
int GFIblankingtime=20; // mask GFI trips for this many milliseconds from relay's closing - anti-noise
unsigned int delta=0;
char *daysStr[7]={"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
unsigned int starttime[7]={0, 0, 0, 0, 0, 0, 0};
unsigned int endtime[7]={7, 7, 7, 7, 7, 17, 17}; // start and end times by weekday
// sensor timings
const int meas_cycle_delay=100; // in ms
// how often to report on status
// report in every cycle if in DEBUG mode
#ifdef DEBUG_WIFI
  const int type1_reportMask=10; // in standby mode, every 10 minutes
  const int type2_reportMask=1; // in run mode, every 1 minute
#else
  const int type1_reportMask=600; // in standby mode, every 10 minutes
  const int type2_reportMask=60; // in run mode, every 1 minute
#endif
//------------ end timing params ---------------------------


#ifdef JB_WiFi_simple
  SoftwareSerial wifiSerial(pin_sRX, pin_sTX);
#endif

#ifdef JB_WiFi
  SoftwareSerial wifiSerial(pin_sRX, pin_sTX);
  WiFly wifly;
#endif

void setup() {
  // set digital input pins
  pinMode(pin_GFI, INPUT);
  pinMode(pin_ctrlBtn, INPUT);
  pinMode(pin_ctrlBtn2, INPUT);

  // set digital output pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_inRelay, OUTPUT);
  // pinMode(pin_WPS, OUTPUT); // do NOT do this if there is a remote installed!
  pinMode(pin_GFItest, OUTPUT);

  // use Timer1 library to set PWM frequency 
  // 10-bit PWM resolution
  Timer1.initialize(1000); // 1kHz for J1772
  Timer1.pwm(pin_PWM, 0); 
  
  // initialize the clock - assume no RTC and that we are getting turned on at the hour
  int day=5, hour=12, tenmins=0, timeout=5000; // default day is Sat, time is noon, 0 min, timeout is 5s

  //================= initialize the display ===========================================
#ifdef LCD_SGC
  *myLCD=uLCD_144(9600);
#else
  *myLCD=uLCD_144_SPE(9600);
#endif
  //================= finish display init ==============================================

  // check if the display started / is present
  // if not present, we will assume this is the Base edition
  LCD_on=myLCD->isAlive();
  
#ifdef JB_WiFi_simple
  wifiSerial.begin(9600);
#endif

  // the time settings only valid in the PREMIUM edition
  // load day/hour from the configuration (EEPROM)
  EEPROM_readAnything(0, configuration);
  // take care of some first-time-read stuff
  if(configuration.day>-1) day=configuration.day; 
  if(configuration.hour>-1) hour=configuration.hour; 
  if(configuration.mins>-1) tenmins=configuration.mins/10; 
  if(int(configuration.energy)<0) {
    configuration.energy=0;
  }
  if(int(configuration.IDstamp[0])<0) {
    randomSeed(analogRead(6)+int(micros())); // this should be random enough
    for(int iii=0; iii<10; iii++) {
      configuration.IDstamp[iii]=random(9999);
    }
  }
  
  day=limit(day, 0, 6);
  hour=limit(hour, 0, 23);
  tenmins=limit(tenmins, 0, 6);
  
  if(LCD_on) { // this is a PREMIUM edition
    myLCD->clrScreen();
    myLCD->setOpacity(1);
    myLCD->setMode(1); // reverse landscape

    printClrMsg(F("Thank You for\nchoosing \nJ.u.i.c.e B.o.x !!!"), 5000, 0, 0x3f, 0);
    
    // will need to add pull of the true RTC time from a WiFi module here 
#ifdef JB_WiFi_simple
    // enter command mode, run 'get time'
#endif

    timer=millis();
    // user setup of the day - 0=Monday
    printClrMsg(F("Press button to\nselect the current\nday of week"), 50, 0, 0x3f, 0);
    myLCD->printStr(0, 6, 2, 0x1f, 0x1f, 0, daysStr[day]); // print starting point
    while(1) {
      if(isBtnPressed(pin_ctrlBtn)) {
        day++;
        if(day==7) day=0;
        timer=millis(); // reset timer
        myLCD->printStr(0, 6, 2, 0x1f, 0x1f, 0, daysStr[day]);
        delay(100); // avoid double-button-press
      }
      if(millis()-timer>timeout) {
        break; 
      }
      delay(50);
    }
    sprintf(str, "Selected: %s.\nWaiting for 2 sec...", daysStr[day]); 
    printClrMsg(str, 2000, 0, 0x3f, 0);
  
    // user setup of the time - in 24 hour clock
    printClrMsg(F("Press button to\nselect hour\n(24-hour day)"), 50, 0, 0x3f, 0);
    timeStr(hstr, hour);
    sprintf(str, "%s:00   ", hstr); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
    timer=millis();  // reset timer
    while(1) {
      if(isBtnPressed(pin_ctrlBtn)) {
        hour++;
        if(hour==24) hour=0;
        timer=millis();
        timeStr(hstr, hour);
        sprintf(str, "%s:00   ", hstr); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str);
        delay(100); // avoid double-button-press
      }
      if(millis()-timer>timeout) {
        break; // exit on timeout
      }
      delay(50);
    }
    sprintf(str, "Selected: %s:00.\nWaiting for 2 sec...", hstr); 
    printClrMsg(str, 2000, 0, 0x3f, 0);
    
    // user setup of the time - minutes
    printClrMsg(F("Press button to\nselect closest\n10 minutes"), 50, 0, 0x3f, 0);
    sprintf(str, "%s:%d0   ", hstr, tenmins); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
    timer=millis();  // reset timer
    while(1) {
      if(isBtnPressed(pin_ctrlBtn)) {
        tenmins++;
        if(tenmins==6) tenmins=0; // 60 minutes in an hour
        timer=millis();
        sprintf(str, "%s:%d0   ", hstr, tenmins); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str);
        delay(100); // avoid double-button-press
      }
      if(millis()-timer>timeout) {
        break; // exit on timeout
      }
      delay(50);
    }
    sprintf(str, "Selected: %s:%d0.\nWaiting for 2 sec...", hstr, tenmins); 
    printClrMsg(str, 2000, 0, 0x3f, 0);
    
    // set the clock offset; later in code, #of sec from midnight can be calculated as
    //     millis()/1000-clock_offset
    clock_offset=millis()/1000-long(day*24+hour)*3600-tenmins*600; 
    
    // store in EEPROM - only in Premium
    configuration.day=day;
    configuration.hour=hour;
    configuration.mins=tenmins*10;

  } // end [if PREMIUM_EDITION]

  EEPROM_writeAnything(0, configuration);

  //---------------------------- calibrate state boundaries ---------------------------------------------
  // first, need to record a minimum value of the wave - needed for pilot voltage measurement later on
  // set output pin to negative rail
  Timer1.setPwmDuty(pin_PWM, 0); // this should produce a steady -12V signal on a pilot pin
  pV_min=read_pV(); // this is supposed to be -12V

  // now calibrate the pilot voltage thresholds based on the actual voltage of positive rail 
  // calibration is done at every power-up
  Timer1.setPwmDuty(pin_PWM, PWM_res); // this should produce a steady +12V signal on a pilot pin
  float pVcal=read_pV();
#ifdef DEBUG
  sprintf(str, "pV: %d", int(pVcal*1000)); 
  printJBstr(0, 9, 2, 0x1f, 0, 0, str);      
#endif  

  // set default thresholds for pilot signal levels
  state_A_Vmin=def_state_A_Vmin; state_A_Vmax=def_state_A_Vmax; 
  state_B_Vmin=def_state_B_Vmin; state_B_Vmax=def_state_B_Vmax; 
  state_C_Vmin=def_state_C_Vmin; state_C_Vmax=def_state_C_Vmax; 
  state_D_Vmin=def_state_D_Vmin; state_D_Vmax=def_state_D_Vmax; 
  state_E_Vmin=-1.5, state_E_Vmax=1.5; 
  state_F_Vmin=-14., state_F_Vmax=-10.; 
  
  // recalibrate the pilot sensing code. helps fight any possible temperature / aging drifts
  // but only do it if it's not too far off - this will prevent recalibration in case the power 
  // cycles while the JuiceBox is plugged into the car
  // note that this will mean that the JuiceBox would not be able to recalibrate if the pilot is more than 
  // 10% off (unlikely with a precision 12V regulator used and R-R op amp)
  if(pVcal>def_state_B_Vmax) {  
    pVcal/=12.; // calibration constant
    // now adjust boundaries for top being not 12V
    state_A_Vmin=def_state_A_Vmin*pVcal;  state_A_Vmax=def_state_A_Vmax*pVcal; 
    state_B_Vmin=def_state_B_Vmin*pVcal;  state_B_Vmax=def_state_B_Vmax*pVcal; 
    state_C_Vmin=def_state_C_Vmin*pVcal;  state_C_Vmax=def_state_C_Vmax*pVcal; 
    state_D_Vmin=def_state_D_Vmin*pVcal;  state_D_Vmax=def_state_D_Vmax*pVcal; 
    state_E_Vmin=-1.5, state_E_Vmax=1.5; 
    state_F_Vmin=-14., state_F_Vmax=-10.; 
  }
  
  //-------------------- ONE-TIME: determine input / output AC voltage
  // this has to run before attaching interrupt to the GFI break pin
  // set the baseline 
  V_Ard_pin_0=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin_0=0; // DEBUG - override for now
  // now measure
#ifndef PCB_81
  // first, turn on the relay
  setRelay(HIGH);
  // second, force the GFI pin
  digitalWrite(pin_GFItest, HIGH);
  // wait for settling (RC on the pin is 0.1s so need to wait at least for 0.3s
  // but not too long or we will burn out the resistor...
  delay(300);
#endif
  inV_AC=read_V();    
  digitalWrite(pin_GFItest, LOW);
  setRelay(LOW);  
  
  // attach interrupt on pin 3 (GFI)
#ifdef GFI
  attachInterrupt(1, GFI_break, RISING);
#endif

  // prep for calc of the savings
  getSavingsPerKWH(gascost, mpg, ecost, whpermile);
  
  // send a power-on message to server
#ifdef JB_WiFi_simple
  sendWiFiMsg("Power ON");
#endif  
  
  myclrScreen();
}


//============================================= MAIN LOOP ============================================
void loop() {
  // load configuration - holding total energy and calibration constants
  // need to load here in loop() function as things will be written at end of each charge
  EEPROM_readAnything(0, configuration);

  // print JuiceBox ID if requested - required to associate the JuiceBox with online account
  // this is button D on the remote!
  if(isBtnPressed(pin_ctrlBtn2) && LCD_on) { // only for Premium version - need something to hold this pin down if button not pressed and there is nothing to do this in the Base edition 
    myLCD->clrScreen();
    for(int iii=0; iii<10; iii++) {
      sprintf(str, "%u", configuration.IDstamp[iii]);
      printJBstr(0, iii, 1, 0x1f, 0x3f, 0x1f, str);
    }
    while(!isBtnPressed(pin_ctrlBtn2)); // wait for another press of the same button
    myclrScreen();
  } 
  
  // reset energy counter for this cycle
  energy=0;
  
  // set the output current - can be changed between the charges without a restart
  setOutC();
  
  n++; // cycle counter
  
  int min2nextrun=-1; // no time of run restriction by default
  int savings=int(configuration.energy*savingsPerKWH/100);
  
  printTime();
  if(LCD_on) {
    switch(cycleVar) {
      case 4:
        cycleVar=0;
      case 0:
        myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY -"));
        break;
      case 1:
        myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY \\"));
        break;
      case 2:
        myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY |"));
        break;
      case 3:
        myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY /"));
        break;
      default: break;
    }
    cycleVar++;

    // output some key standby info - this is the default screen before charging commences
    sprintf(str, "Life: %d KWH", int(configuration.energy)); myLCD->printStr(0, 4, 2, 0, 0x3f, 0x1f, str);
    sprintf(str, "Saved: %d$", savings); myLCD->printStr(0, 5, 2, 0, 0x3f, 0x1f, str);
    sprintf(str, "Last: %s KWH", ftoa(tempstr, energy, 1)); myLCD->printStr(0, 7, 2, 0x1f, 0, 0x1f, str);
    sprintf(str, "Set: %dV, %dA    ", int(inV_AC), int(outC)); myLCD->printStr(0, 8, 2, 0x1f, 0x3f, 0x1f, str);
#ifdef DEBUG
    int reading=analogRead(pin_C);
    delay(8);
    reading+=analogRead(pin_C);
    sprintf(str, "A2:%d=%dA  ", int(reading*5./2/10.24), int(read_C()*10)); myLCD->printStr(0, 9, 2, 0x1f, 0x3f, 0x1f, str);
#endif
    min2nextrun=timeToNextRun();

  } else {
    
    // no LCD
    sprintf(str, "%d KWH, %d$, %s KWH, %dV, %dA", int(configuration.energy), savings, ftoa(tempstr, energy, 1), int(inV_AC), int(outC));
    Serial.println(str);
#ifdef DEBUG
    // print ID - only in non-LCD mode so not to clutter anything
    for(int iii=0; iii<10; iii++) {
      Serial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
    }
    Serial.println();
    sprintf(str, "    pilot=%d, inACpin=%d", int(V_J1772_pin_*1000), int(analogRead(pin_V)*Aref));
    Serial.println(str);
#endif

  }
  
  // send out a report to MotherShip via WiFi if WiFi is enabled
#ifdef JB_WiFi_simple
// report every cycle if DEBUG mode
  if(n > type1_reportMask)
  {
    n=0;
    sprintf(str, "V%d,L%d,S%d", int(inV_AC), configuration.energy, savings);
    sendWiFiMsg(str);
  }
#endif

  if(min2nextrun>0 && !isBtnPressed(pin_ctrlBtn)) { // check if we are ok to run
    sprintf(str, "Wait %d min.\nPlug & hold BTN to\nforce", min2nextrun); 
    printJBstr(0, 10, 1, 0x1f, 0x3f, 0x1f, str);    
  } else {
    // here, the run is enabled by our poor man's RTC timer
    
    //--------------- run J1772 logic
    // set output pin to +12V
    Timer1.setPwmDuty(pin_PWM, PWM_res); // this should produce a steady +12V signal on a pilot pin
    
    // check if the car is there and requesting power
    int state=getState(0);  
    
    // until we see state B or C, nothing happens
    if(state==STATE_B || state==STATE_C) {
      // vehicle is connected and may be requesting power
      printClrMsg("Something is connected", 5, 0x1f, 0x3f, 0);
      
      // check that this is not some kid's fingers
      Timer1.setPwmDuty(pin_PWM, PWM_res/2); // output 50% duty cycle so that average is zero if there is no diode
      // example: if we have 9V top and -12V bottom, we will get -1.5V average
      // therefore set 'safe' threshold at -1V - anything higher than that, we trip

      if(read_pV()<-1.0) {
        // -12V is still there - it's a car, allowed to turn on the power
        printClrMsg(F("Safety check PASSED"), 500, 0x1f, 0x3f, 0);

        // reset the duty to the right value
        Timer1.setPwmDuty(pin_PWM, duty); 
        // now we should be in state B again (before the car requests power)
  
#ifdef PCB_83
#ifdef GFI
        GFI_tripped=0;
        // check for stuck relay - the GFCI circuit should NOT be energized at this point 
        //    (as the main relay is still open)
        // attempt to trip - if it does trip, the relay is stuck ON 
        digitalWrite(pin_GFItest, HIGH);
        delay(100); 
        digitalWrite(pin_GFItest, LOW);
        // by now, if the trip occurred, the GFI trip flag should be set
        if(GFI_tripped==1) {
          // we have a stuck relay, throw an error
          printErrorMsg(F("STUCK RELAY! \nContact us\nExiting..."), 30000);
          return; // break from loop() which will be called back a moment later
        }
#endif
#endif 
  
        //----------------------- clear for charging - loop until we have to bail
        while(1) {
          // set the output current - can be changed DURING the charges without a restart
          // by turning the pot
          setOutC();
          // reset the duty to the right value
          Timer1.setPwmDuty(pin_PWM, duty); 
  
          // get the J1772 state (states B & C will keep the system in this endless loop)
          state=getState(1);

          GFI_tripped=0; // reset GFI trip status so we can retry after GFI timeout
          
          printClrMsg(F("Vehicle connected"), 5, 0x1f, 0x3f, 0); 
          // show energy moved so far in the cycle
          sprintf(str, "In: %s KWH  ", ftoa(tempstr, energy, 1)); 
          printJBstr(0, 5, 2, 0x1f, 0x3f, 0, str);   
          delay(500);

          if(state==STATE_A) {
            // vehicle disconnected 
            printClrMsg(F("Vehicle\nDisconnected!\nExiting..."), 1000, 0x1f, 0x3f, 0);
            break;
          }
          if(state==STATE_D) {
            // vehicle requested VENTILATED power - break 
            printClrMsg(F("Vehicle requested\nVENTILATED power!\nExiting..."), 1000, 0x1f, 0x3f, 0);
            break;
          }
          if(state==STATE_E || state==STATE_F || state==STATE_INVALID) {
            // abnormal state - break 
            printClrMsg(F("Abnormal State!\nExiting..."), 1000, 0x1f, 0x3f, 0);
            break;
          }
  
          if(state==STATE_C) {
            // vehicle requested NON-VENTILATED power - this is our MONEY state
            int ctrlBtn_cnt=0;
  
            // commence charging
            timer_chstart=millis(); // this will be used in blanking GFI spike for the first few milliseconds
            setRelay(HIGH);
            printClrMsg(F("Starting Charge!"), 1000, 0x1f, 0x3f, 0);
            myclrScreen();
  
            timer=millis(); // start timer
            timer0=timer;
            
            n=0;
            
            //+++++++++++++++++++++++++++++++++++++++++++ inner CHARGING loop!
            while(getState(1)==STATE_C) { // break on anything else
              // set the output current - can be changed DURING the charges without a restart
              // by turning the pot
              setOutC();
              // reset the duty to the right value
              Timer1.setPwmDuty(pin_PWM, duty); 
      
              n++; // cycle counter
          
              delay(meas_cycle_delay); // reasonable delay for screen refresh

              // check if the control button is held for more than 10 cycles
              if(LCD_on) {
                if(digitalRead(pin_ctrlBtn)==0) ctrlBtn_cnt=0;
                ctrlBtn_cnt++;
                if(ctrlBtn_cnt>10) {
                  printClrMsg(F("Breaking per user\nrequest! Unplug\nfrom car"), 3000, 0x1f, 0x3f, 0);
                  // attempt to set output pin back to +12V
                  Timer1.setPwmDuty(pin_PWM, PWM_res); // this should produce a steady +12V signal on a pilot pin
                  // wait until user unplugs
                  while(getState(0)!=STATE_A);
                  break;
                }
              }
              
#ifdef GFI
              // check GFI
              if(GFI_tripped) {
                printClrMsg(F("GFI tripped!\nRetrying in 15 min..."), 300, 0x1f, 0x3f, 0);
                GFI_trip_count++; // allowed max of 4; if more than 4, need 2 user inputs to override
                if(GFI_trip_count>4) {
                  // wait for user to unplug; since the user then will have to re-plug to re-energize the system, this can be considered 2 actions
                  printClrMsg(F("4th GFI trip!\nUnplug / re-plug\nto resume"), 1000, 0x1f, 0x3f, 0);
                  while(getState(0)!=STATE_A);
                } else {
#ifndef DEBUG                 
                  delaySecs(900); // 15 min
#endif
                }  
                break;  
              }
#endif
              
              // process energy metering
              float outC_meas=read_C();
              power=outC_meas*inV_AC/1000; // in KW

              timer_now=millis();
              delta=int(timer_now-timer);
              energy+=power*delta/1000/3600; 
              timer=timer_now;

              // print real-time stats
              printTime();    
              sprintf(str, "Pwr: %s KW  ", ftoa(tempstr, power, 1)); 
                  printJBstr(0, 2, 2, 0x1f, 0x3f, 0, str);   
              sprintf(str, "Time: %s min  ", ftoa(tempstr, float(timer-timer0)/1000/60, 1)); 
                  printJBstr(0, 3, 2, 0x1f, 0x3f, 0, str);   
              sprintf(str, "In: %s KWH  ", ftoa(tempstr, energy, 1)); 
                  printJBstr(0, 5, 2, 0x1f, 0x3f, 0, str);   
              sprintf(str, "%dV, %sA ", int(inV_AC), ftoa(tempstr, outC_meas, 1)); 
                  printJBstr(0, 7, 2, 0x1f, 0x3f, 0, str);   

              // send out a report to MotherShip via WiFi if on
#ifdef JB_WiFi_simple
              if(n > type2_reportMask)
              {
                n=0;
                sprintf(str, "V%d,L%d,E%d,A%d,P%d", int(inV_AC), int(configuration.energy+energy), int(energy*10), int(outC_meas*10), int(power*10));
                sendWiFiMsg(str);
              }
#endif    
            } //------------------------- end main charging loop
  
  
            setRelay(LOW); // open the input relay
  
            // store things in EEPROM so we can track total lifetime energy / savings and 
            // also are immune to short power interruptions
            configuration.energy+=energy; // keep track of the total energy transmitted through the EVSE
            configuration.day=dayOfWeek();
            configuration.hour=hourOfDay();
            configuration.mins=minsOfHour();
            EEPROM_writeAnything(0, configuration);
          } // end inner CHARGING loop
  
        } // end 'clear for charging' loop
  
      } else {// end state F after B/C (checking for the diode)
        printClrMsg(F("Safety check FAILED"), 2000, 0x1f, 0x3f, 0);
      }
      
    } // end state B/C entry
  
  } // end time-of-day lockout
  
  delay(meas_cycle_delay); // reasonable delay for screen refresh

} // end loop()


//=================================================== FUNCTIONS ==============================================
// interrupt - break on GFI trigger - breaks on RISING signal on pin 3 (transistor end of the relay)
void GFI_break() {
  // mask if within certain time from closing the main relay
  if(millis()-timer_chstart<GFIblankingtime) return;
  
  // check every 200uS for 20ms - if not a single additional trips, ignore
  for(int i=0; i<100; i++) {
    delayMicroseconds(200);
    if(digitalRead(pin_GFI)==HIGH) {
      GFI_tripped=1;
      // before tripping relay, kill pilot - maybe the onboard charger will drop current quickly and help save our relay contacts...
      Timer1.setPwmDuty(pin_PWM, 0); // -12V on pilot - ABNORMAL STATE. Compliant chargers should stop
      setRelay(LOW); // kill power NOW. generally, relay will take ~20-25ms to open
      break;
    }
  }
}

// operate output relay
void setRelay(int state) {
  digitalWrite(pin_inRelay, state);
}


void setOutC() {
  // set the output current based on the pot
  // use default current setting if pin is grounded
  float minThrottle=0.05;
  float throttle=0;

  // different trimpot depending on voltage
  if(inV_AC==120) {
#ifdef trim120current
    throttle=analogRead(pin_throttle120)/1024.;
    if(throttle>minThrottle) { // if something is set on the throttle pot, use that instead of the default outC
      outC=throttle*nominal_outC_120V*2;
    }
#else
    outC=min(nominal_outC_120V, outC);
#endif
  } else {
    throttle=analogRead(pin_throttle)/1024.;
    if(throttle>minThrottle) { // if something is set on the throttle pot, use that instead of the default outC
      outC=throttle*maxC;
    }
  }

  // per J1772 standard:
  // 1% duty = 0.6A until 85% duty cycle
  // after that, 1% = 2.5A up to 96%
  if(outC<51) {
    duty=PWM_res*outC/60.;
  } else {
    duty=PWM_res*(0.64+outC/250.);
  }
  
  if(duty>MAXDUTY) duty=MAXDUTY;
}


// this will block for ~200ms due to read_pV()
int getState(int mode) {
  float pV=read_pV();
  
#ifdef DEBUG
//  sprintf(str, "raw pV=%d, ", int(pV*1000));
//  printJBstr(0, 10, 2, 0x1f, 0, 0, str);      
#endif

  // in mode=1, the state is measured while pilot is oscillating so need to recalc
  // pV=pV_min*(1-duty)+pV_max*duty
  // so pV_max=(pV-pV_min*(1-duty))/duty
  if(mode==1) pV=((pV-pV_min)*PWM_res+pV_min*duty)/duty;
  
#ifdef DEBUG
//  sprintf(str, "calc pV=%d", int(pV*1000));
//  printJBstr(0, 11, 2, 0x1f, 0, 0, str);      
#endif

  if(pV>state_A_Vmin && pV<=state_A_Vmax) return STATE_A;
  if(pV>state_B_Vmin && pV<=state_B_Vmax) return STATE_B;
  if(pV>state_C_Vmin && pV<=state_C_Vmax) return STATE_C;
  if(pV>state_D_Vmin && pV<=state_D_Vmax) return STATE_D;
  if(pV>state_E_Vmin && pV<=state_E_Vmax) return STATE_E;
  if(pV>state_F_Vmin && pV<=state_F_Vmax) return STATE_F;

  return STATE_INVALID;
}


// read the average pilot voltage - this is a BLOCKING CALL (200ms)
// time constant of the RC filter: 27k/2k * 3.3uF = ~0.04s - enough to smooth 1kHz signal
float read_pV() {
  // ensure settling of the signal before measurement
  delay(200); // this is ~5 time constants of the RC filter on this pin - measured value should be within 2% of its settled value
  int reading=analogRead(pin_pV); // this takes 100uS
  delayMicroseconds(2500); // another opinion - anti-noise, read 180 degree off the prev reading (integer number of milliseconds + 500 uS (half-PWM-period) - ADC conversion time)
  reading+=analogRead(pin_pV);
  float V_Ard_pin=reading*Aref/1024./2;

  V_J1772_pin_=(2*V_Ard_pin-5)*upper_pV_R/lower_pV_R+V_Ard_pin;

  return V_J1772_pin_;
}

// read the average input AC voltage 
// this function should ONLY BE CALLED in setup()
// time constant of the RC filter: 27k * 3.3uF = ~0.09s - enough to smooth 60Hz signal
float read_V() {
  float V_AC=240; // default is 240
  
  float V_Ard_pin=analogRead(pin_V)*Aref/1024.;
  delay(8); // measure 180 degrees away by AC phase to smooth out any remaining ripple on A1 pin
  V_Ard_pin+=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin/=2;
  
#ifdef PCB_81
  // for PCB versions before 8.3, 
  // THIS FEATURE IS IN BETA AND MAY NOT WORK ON THE FIRST VERSION OF THE BASE BOARDS WITHOUT TWEAKING FIRMWARE
  // specifically, you may need to tweak the voltage threshold between 120V and 240V input voltage 
  //   (line starting with 'if(V_Ard_pin >' below). (1) connect JuiceBox to 120V, measure the voltage on pin A1 of the Arduino
  //   (2) connect JuiceBox to 240V, measure the voltage on pin A1. Set the threshold to the voltage in the middle between 
  //   these two values
  // with 200k resistor from AC rectified line, we have 0.8mA peak primary current at 120VAC
  //     and 1.6mA at 240VAC
  // according to PC817X1 opto's datasheet, CTR is 80-160% at 5mA
  // typical curve suggests 80% of that at 2.5mA, 50% at 1mA
  // therefore, we have a secondary peak current of 0.3-0.6mA at 120VAC, 1-2mA at 240VAC
  // this corresponds to a secondary voltage drop: 0.3-0.6V or 1-2V per 1k of secondary resistance
  //               (actually clipped to 5V since we are using 5v supply)
  // also, need to take into account that we see the significant current only at the positive peak of AC wave
  // generally, for ~1/4 of the period for 120VAC and 1/3rd for 240VAC
  // finally, the average drop within over the drop time is ~1/2 of the peak drop
  // putting it all together, we expect average drop of 0.04-0.08V per 1k for 120VAC and 0.16-0.32V per 1k for 240VAC 
  //               (with some clipping starting at 3-5k - really becoming visible at 5-7k)  
  // Example: 4.7k secondary - 0.2-0.4V and 0.8-1.5V drops
  // Example: 10k secondary - 0.4-0.8V and 1.2-2.2V drops
  // Using 10k secondary, place mid-point at 1V drop, or 4V threshold
  // cap at 4.9V to prevent from defaulting to 120V in case when no PC817 installed at all
  if(V_Ard_pin > 3.5 && V_Ard_pin<4.9) V_AC=120;
#endif

#ifdef PCB_83
  // in 8.3 and later, the implementation changed to measurement using the GFCI current sensor
  // ~200x division factor for voltage (total gain of test loop is ~1.2e-2)
  //     (306 from RMS voltage on sensor = 680x on the opamp, 0.5x due to half-wave rectification, 0.9x for converting to average from RMS)
  //     (3.9e-5x from 0.39V/A sensor sensitivity with 390R shunt resistor, 0.0001A/V voltage-to-current conversion on a 10k resistor)
  //

  // if no GFI installed, cannot measure voltage - default to 240V 
#ifdef GFI
  V_AC=150*(V_Ard_pin-V_Ard_pin_0);
#else
  V_AC=240;
#endif

#ifdef DEBUG
  sprintf(str, "V_AC: %d", int(V_AC));
  printJBstr(0, 9, 2, 0x1f, 0, 0, str);   
#endif
  
  if(V_AC < V_AC_threshold) { // midpoint between 120 and 208V
    V_AC=120;
  } else if(V_AC < 224) { // midpoint between 208V and 240V
    V_AC=208;
  } else {
    V_AC=240; // default fall-back value
  }
  
#endif

  return V_AC; 
}

// read the AC current via the current transformer
// in the absense of the current transformer this will return zero
// RC constant defined by R11 C5 = 27k * 3.3uF = 90ms, or >5 line periods
float read_C() {
 const float V_dDrop=0.2; // diode drop at very low current (<10mA)

#ifdef AC1075
  const int Te=1000; // # of turns
#endif
#ifdef CT_8349-1500
  const int Te=1500; // # of turns
#endif
#ifdef CT_8420-1000
  // assume 8420-1000 current transformer (50A max current, 20-1000 Hz working range)
  const int Te=1018; // # of turns
#endif
#ifdef CT_3100
  // assume 3100 current transformer (75A max current, 20-1000 Hz working range)
  const int Te=3100; // # of turns
#endif

  // read the rectified voltage of the half-wave from the transformer
  // average between 2 readings 180 degree off each other
  int reading=analogRead(pin_C);
  delay(8);
  reading+=analogRead(pin_C);
  // this assumes an RC filter before Arduino pon with time constant >> line period and impedance >> R
  float V_C=reading*Aref/2/1024; 

#ifdef AC1075
  // use a table lookup
  int index=floor(V_C*10);
  float remainder=V_C*10-index;
  if(index>31) return 75.;
  float coeff=float(AC1075_calibration[index+1]-AC1075_calibration[index]); // 0.1V step
  return float(AC1075_calibration[index]+remainder*coeff)/10.;
#else
  // use a crude linear approximation
  if(V_C>0.2) {
    V_C+=V_dDrop+V_C/6;
  } else V_C=0;

  // *2 for half-wave rectification, 1.11 for conversion of average into RMS
  // for AC1050-1075 this corresponds to ~18A/V
  return V_C*Te/R_C*2.22;  
#endif
}


//------------------------------ printing help functions -------------------------
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const __FlashStringHelper *fstr) {
  if(LCD_on) {
    myLCD->printStr(col, row, font, c1, c2, c3, fstr);
  } else {
    Serial.print("    ");
    Serial.println(fstr);
  }
}
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const char *sstr) {
  if(LCD_on) {
    myLCD->printStr(col, row, font, c1, c2, c3, sstr);
  } else {
    Serial.print("    ");
    Serial.println(sstr);
  }
}
void printClrMsg(const __FlashStringHelper *fstr, const int del, const byte red, const byte green, const byte blue) {
  myclrScreen();
  printJBstr(0, 2, 2, red, green, blue, fstr);      
  delay(del);
}
void printClrMsg(const char *str, const int del, const byte red, const byte green, const byte blue) {
  myclrScreen();
  printJBstr(0, 2, 2, red, green, blue, str);      
  delay(del);
}
void printErrorMsg(const __FlashStringHelper *fstr, const int del) {
  printClrMsg(fstr, 30000, 0x1f, 0x3f, 0);
  // also send a message to server if WiFI is enabled
#ifdef JB_WiFi_simple
  sendWiFiMsg(fstr, 1);
#endif
}


// custom clear screen function. prints some header info
void myclrScreen() {
  if(LCD_on) {
    myLCD->clrScreen();
  } else {
    Serial.println("");
  }
  printTime();
}
// print time etc on the first line
void printTime() {
  char hstr[3], mstr[3]; // define here to mask the global variables
  int hours=(millis()-clock_offset)/1000/3600;
  timeStr(hstr, hourOfDay());
  timeStr(mstr, minsOfHour());
  sprintf(tempstr, "JB %s %s:%s  ", VerStr, hstr, mstr); 
  printJBstr(0, 0, 2, 0x1f, 0, 0x1f, tempstr);     
}
// convert time (min, hour) into a displayable string
// expects a properly allocated char string as input
void timeStr(char *a, int time) {
  if(time<10) {
    sprintf(a, "0%d", time);
  } else {
    sprintf(a, "%d", time);
  }    
}


#ifdef JB_WiFi_simple
//==================== WIFI messaging functions ===============================================
void sendWiFiMsg(char *str) {
  // print out the packet
  // ID first
  for(int iii=0; iii<10; iii++) {
    wifiSerial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
  }
  wifiSerial.print(":");
  // print data now
  wifiSerial.print(str);
  wifiSerial.print(":");
  wifiSerial.println(UDPpacketEndSig);
}
void sendWiFiMsg(const __FlashStringHelper *fstr, int dummy) {
  // print out the packet
  // ID first
  for(int iii=0; iii<10; iii++) {
    wifiSerial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
  }
  wifiSerial.print(":");
  // print data now
  wifiSerial.print(fstr);
  wifiSerial.print(":");
  wifiSerial.println(UDPpacketEndSig);
}
//===================== END WiFi messaging functions ===========================================
#endif


// fixed-precision float print
char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}
//---------------------------- end printing help functions ------------------------

//---------------------------- input control functions ----------------------------
// this takes max of 50ms if the button is pressed
int isBtnPressed(int pin) {
  if(digitalRead(pin)==HIGH) {
    // check if noise
    for(int zz=0; zz<10; zz++) {
      if(digitalRead(pin)==LOW) return 0;
      delay(5);
    }
    return 1;
  } else {
    return 0;
  }
}

//---------------- timing functions -----------------------------------------------
// time in minutes to the next run
int timeToNextRun() {
  int day=dayOfWeek();
  int nextDay=day+1; 
  if(nextDay==7) nextDay=0; // wrap around
  int hour=hourOfDay();
  
  int timeToStart=starttime[day]-hour; 
  int timeToEnd=endtime[day]-hour; 

  // example: now=0200 Sat; alowed Sat times: 0-17, sun times 2-17
  // timeToStart=-2, timeToEnd=15 -> timeToStart*timeToEnd=-30 or <0
  // therefore, returning -1
  if(timeToStart*timeToEnd<0) return -1; // in window so can go right now

  return (starttime[nextDay]-hour+24)*60-minsOfHour();
}

// determine day of week
int dayOfWeek() {
  int day=(millis()/1000-clock_offset)/24/3600;
  day=day-int(day/7)*7; // day=0 = Monday
  
  return day;
}
// determine hour of day
int hourOfDay() {
  int hour=(millis()/1000-clock_offset)/3600;
  hour=hour-int(hour/24)*24; 
  
  return hour;
}
// determine minutes of hour
int minsOfHour() {
  int mins=(millis()/1000-clock_offset)/60;
  mins=mins-int(mins/60)*60; 
  
  return mins;
}

// need some RTC functions here...

//---------------- END timing functions -----------------------------------------------

int limit(int value, int minimum, int maximum) {
  int retval=value;
  
  if(value<minimum) value=minimum;
  if(value>maximum) value=maximum;
  
  return retval;
}

void getSavingsPerKWH(int gascost, int mpg, int ecost, int whpermile) {
  int gCostPerMile=gascost/mpg;
  int gCostPerKWH=gCostPerMile*1000/whpermile;
  
  savingsPerKWH=gCostPerKWH-ecost;
}

// long delays
void delaySecs(int secs) {
  for(int si=0; si<secs; si++) delay(1000);
}


