//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                *
// http://www.bausano.net                                                                    *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                 *
//                                                                                           *
//********************************************************************************************

//********************************************************************************************
//                                                                                           *
// This software is distributed as an example, "AS IS", in the hope that it could            *
// be useful, WITHOUT ANY WARRANTY of any kind, express or implied, included, but            *
// not limited,  to the warranties of merchantability, fitness for a particular              *
// purpose, and non infringiment. In no event shall the authors be liable for any            *
// claim, damages or other liability, arising from, or in connection with this software.     *
//                                                                                           *
//********************************************************************************************



//---- AB&T EasyCAT shield custom application example  ---------------------------------------


// This is the legacy "TestEasyCAT" example but the variables have been
// customized using the Easy Configuration tool.
// To understand how to do this please see the Easy Configurator user manual.
//
//
// The input variables used in this example are:
//
//		uint16_t    Analog_0                The first analog input
//		uint16_t    Analog_1                The second analog input
//		uint8_t     DipSwitches             The four dip switches
//		uint8_t     Bit8_FallingTestRamp    A falling test ramp
//		uint16_t    Bit16_RisingTestRamp    A rising test ramp
//
// And the output:
//
//		uint8_t     Leds;                   The four leds


// Code oraganization

// define
// include libraries
// constants
// global variables
// user defined functions
// setup()
// loop()

// EASYCAT.BufferIn.Cust.Temperature = RawTemp.Byte[0];
// EASYCAT.BufferIn.Cust.Analog = AnalogValue;
// LedStatus = EASYCAT.BufferOut.Cust.Leds;
// PwmValue = EASYCAT.BufferOut.Cust.Pwm;


//---- defines- --------------------------------------------------------------------------------------

#define CUSTOM  // Custom mode
#define DEBUG
#define spiChipSelect 9  // Chip select pin


#define DIGITALINPUTS0  //Use Arduino IO's as Inputs. Define how many Inputs you want in total and then which Pins you want to be Inputs.
#ifdef DIGITALINPUTS0
const int cDigitalInputs0 = 8;  //PDO inDigitalSet0: up to 32 digital inputs using internal Pullup resistor. (short to ground to trigger)
int inDigitalSet0[] = { 20, 21, 22, 23, 24, 25, 26, 27 };
#endif

#define DIGITALINPUTS1  //Use Arduino IO's as Inputs. Define how many Inputs you want in total and then which Pins you want to be Inputs.
#ifdef DIGITALINPUTS1
const int cDigitalInputs1 = 8;  //PDO inDigitalSet1: up to 32 digital inputs using internal Pullup resistor. (short to ground to trigger)
int inDigitalSet1[] = { 30, 31, 32, 33, 34, 35, 36, 37 };
#endif

//Use Arduino IO's as Toggle Inputs, which means Inputs (Buttons for example) keep HIGH State after Release and Send LOW only after beeing Pressed again.
#define TOGGLEINPUTS0  //Define how many Toggle Inputs you want in total and then which Pins you want to be Toggle Inputs.
#ifdef TOGGLEINPUTS0
const int cToggleInputs0 = 8;  //PDO inToggleSet0: up to 32 digital toggle inputs using internal Pullup resistor. (short to ground to trigger)
int inToggleSet0[] = { 40, 41, 42, 43, 44, 45, 46, 47 };
#endif

#define TOGGLEINPUTS1  //Define how many Toggle Inputs you want in total and then which Pins you want to be Toggle Inputs.
#ifdef TOGGLEINPUTS1
const int cToggleInputs1 = 6;  //PDO inToggleSet1: up to 32 digital toggle inputs using internal Pullup resistor. (short to ground to trigger)
int inToggleSet1[] = { 28, 29, 38, 29, 48, 49 };
#endif

#define OUTPUTS0  //Use Arduino IO's as Outputs. Define how many Outputs you want in total and then which Pins you want to be Outputs.
#ifdef OUTPUTS0
const int Outputs0 = 4;  //PDO outDigitalSet0: up to 32 digital outputs
int outDigitalSet0[] = { 4, 5, 6, 7 };
#endif

#define OUTPUTS1  //Use Arduino IO's as Outputs. Define how many Outputs you want in total and then which Pins you want to be Outputs.
#ifdef OUTPUTS1
const int Outputs1 = 4;  //PDO outDigitalSet1: up to 32 digital outputs
int outDigitalSet1[] = { 14, 15, 16, 17 };
#endif

#define QUADENC
//Support for Quadrature Encoders. Define Pins for A and B Signals for your encoders. Visit https://www.pjrc.com/teensy/td_libs_Encoder.html for further explanation.
// Download Zip from here: https://github.com/PaulStoffregen/Encoder and import as Library to your Arduino IDE.
#ifdef QUADENC
#include <Encoder.h>
#define QUADENCS 1  //how many Rotary Encoders do you want?

// Encoders have 2 signals, which must be connected to 2 pins. There are three options.

//Best Performance: Both signals connect to interrupt pins.
//Good Performance: First signal connects to an interrupt pin, second to a non-interrupt pin.
//Low Performance: Both signals connect to non-interrupt pins, details below.

//Board	            Interrupt Pins	            LED Pin(do not use)
//Teensy 4.0 - 4.1	All Digital Pins	          13
//Teensy 3.0 - 3.6	All Digital Pins	          13
//Teensy LC	        2 - 12, 14, 15, 20 - 23	    13
//Teensy 2.0	      5, 6, 7, 8	                11
//Teensy 1.0	      0, 1, 2, 3, 4, 6, 7, 16
//Teensy++ 2.0	    0, 1, 2, 3, 18, 19, 36, 37  6
//Teensy++ 1.0	    0, 1, 2, 3, 18, 19, 36, 37
//Arduino Due	      All Digital Pins	          13
//Arduino Uno	      2, 3	                      13
//Arduino Leonardo	0, 1, 2, 3	                13
//Arduino Mega	    2, 3, 18, 19, 20, 21	      13
//Sanguino	        2, 10, 11	                  0

Encoder Encoder0(2, 3);  //A,B Pin
//Encoder Encoder1(31, 33);         //A,B Pin
//Encoder Encoder2(A,B);
//Encoder Encoder3(A,B);
//Encoder Encoder4(A,B);
const int QuadEncMp[] = { 4 };   //some Rotary encoders send multiple Electronical Impulses per mechanical pulse. How many Electrical impulses are send for each mechanical Latch?

#endif

#define STATUSLED
#ifdef STATUSLED
const int constLedPin = 13;  //Pin for Status LED
#endif

//---- includes --------------------------------------------------------------------------------------

#include "3DWorksEasyCAT.h"  // This file has been created by the Easy Configurator
                             // and must be located in the Arduino project folder
#include "EasyCAT.h"         // EasyCAT library to interface the LAN9252
#include <SPI.h>             // SPI library


EasyCAT EASYCAT(spiChipSelect);  // EasyCAT istantiation

// The constructor allow us to choose the pin used for the EasyCAT SPI chip select
// Without any parameter pin 9 will be used

// We can choose between:
// 8, 9, 10, A5, 6, 7
// On the EasyCAT board the SPI chip select is selected through a bank of jumpers
// (The EasyCAT board REV_A allows only pins 8, 9, 10 through 0 ohm resistors)

//---- pins declaration ------------------------------------------------------------------------------

const int debounceDelay = 50;

#ifdef QUADENC
const int QuadEncs = QUADENCS;
#endif

//---- global variables ---------------------------------------------------------------------------

unsigned long Millis = 0;
unsigned long PreviousMillis = 0;

unsigned long curMillis = 0;
unsigned long lastPrint = 0;

#ifdef DIGITALINPUTS0
int InDigitalState0[cDigitalInputs0];
unsigned long lastInputDebounce0[cDigitalInputs0];
#endif

#ifdef DIGITALINPUTS1
int InDigitalState1[cDigitalInputs1];
unsigned long lastInputDebounce1[cDigitalInputs1];
#endif

#ifdef TOGGLEINPUTS0
int InToggleState0[cToggleInputs0];
int oldInToggleState0[cToggleInputs0];
int toggleinputs0[cToggleInputs0];
unsigned long lastsInputDebounce0[cToggleInputs0];
#endif

#ifdef TOGGLEINPUTS1
int InToggleState1[cToggleInputs1];
int oldInToggleState1[cToggleInputs1];
int toggleinputs1[cToggleInputs1];
unsigned long lastsInputDebounce1[cToggleInputs1];
#endif

#ifdef OUTPUTS0
int OutState0[Outputs0];
int oldOutState0[Outputs0];
#endif

#ifdef OUTPUTS1
int OutState1[Outputs1];
int oldOutState1[Outputs1];
#endif

#ifdef QUADENC
long EncCount[QuadEncs];
long OldEncCount[QuadEncs];
#endif

//---- user defined functions ---------------------------------------------------------------------
void Application();
void DebugMsg(const char msg[]);
void DebugData(char sig, int pin, int state);
void printArray(const char* name, const int arr[], int size);
void printVars();

#ifdef DIGITALINPUTS0
void readDigitalInputs0();
#endif
#ifdef DIGITALINPUTS1
void readDigitalInputs1();
#endif
#ifdef TOGGLEINPUTS0
void readToggleInputs0();
#endif
#ifdef TOGGLEINPUTS1
void readToggleInputs1();
#endif
#ifdef OUTPUTS0
void writeOutputs0();
#endif
#ifdef OUTPUTS1
void writeOutputs1();
#endif
#ifdef QUADENC
void readEncoders();
#endif
#ifdef STATUSLED
void StatLedErr(int offtime, int ontime);
#endif
//---- setup ---------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);  // serial line initialization
                       // (used only for debug)
  DebugMsg("\nEasyCAT - Generic EtherCAT slave\n");  // print the banner

  #ifdef DIGITALINPUTS0
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < cDigitalInputs0; i++) {
      pinMode(inDigitalSet0[i], INPUT_PULLUP);
    }
  #endif

  #ifdef DIGITALINPUTS1
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < cDigitalInputs1; i++) {
      pinMode(inDigitalSet1[i], INPUT_PULLUP);
    }
  #endif

  #ifdef TOGGLEINPUTS0
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < cToggleInputs0; i++) {
      pinMode(inToggleSet0[i], INPUT_PULLUP);
      oldInToggleState0[i] = -1;
      toggleinputs0[i] = 0;
    }
  #endif

  #ifdef TOGGLEINPUTS1
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < cToggleInputs1; i++) {
      pinMode(inToggleSet1[i], INPUT_PULLUP);
      oldInToggleState1[i] = -1;
      toggleinputs1[i] = 0;
    }
  #endif

  #ifdef OUTPUTS0
    for (int o = 0; o < Outputs0; o++) {
      digitalWrite(outDigitalSet0[o], LOW); //put all outputs to low state
      pinMode(outDigitalSet0[o], OUTPUT);
      oldOutState0[o] = 0;
    }
  #endif

  #ifdef OUTPUTS1
    for (int o = 0; o < Outputs1; o++) {
      digitalWrite(outDigitalSet1[o], LOW); //put all outputs to low state
      pinMode(outDigitalSet1[o], OUTPUT);
      oldOutState1[o] = 0;
    }
  #endif
                                        //---- initialize the EasyCAT board -----
  if (EASYCAT.Init() == true) {         // initialization
    DebugMsg("initialized");            // succesfully completed
  }                                     //
  else {                                // initialization failed
    DebugMsg("initialization failed");  // the EasyCAT board was not recognized
                                        // The most common reason is that the SPI
                                        // chip select choosen on the board doesn't
                                        // match the one choosen by the firmware
    #ifdef STATUSLED
    pinMode(constLedPin, OUTPUT);
    #endif                   //
                             //
    while (1) {              // stay in loop for ever
      StatLedErr(500, 500);  // with the Arduino led blinking
    }                        //
  }
}


//---- main loop ----------------------------------------------------------------------------------------

void loop()  // In the main loop we must call ciclically the
{            // EasyCAT task and our application
             //
             // This allows the bidirectional exachange of the data
             // between the EtherCAT master and our application
             //
             // The EasyCAT cycle and the Master cycle are asynchronous
             //

  EASYCAT.MainTask();  // execute the EasyCAT task
  Application();       // user applications
}



//---- user application ------------------------------------------------------------------------------

void Application() {
  Millis = millis();                                   // As an example for this application
  if (Millis - PreviousMillis >= 10) {                 // we choose a cycle time of 10 mS
    PreviousMillis = Millis;                           //

    #ifdef DIGITALINPUTS0
    readDigitalInputs0();                              //read Inputs & send data
    #endif
    #ifdef DIGITALINPUTS1
    readDigitalInputs1();                              //read Inputs & send data
    #endif
    #ifdef TOGGLEINPUTS0
    readToggleInputs0();                               //read Inputs & send data
    #endif
    #ifdef TOGGLEINPUTS1
    readToggleInputs1();                               //read Inputs & send data
    #endif
    #ifdef OUTPUTS0
    writeOutputs0();                                   //read Inputs & send data
    #endif
    #ifdef OUTPUTS1
    writeOutputs1();                                   //read Inputs & send data
    #endif
    #ifdef QUADENC
    readEncoders();                                    //read Encoders & send data
    #endif
    #ifdef DEBUG
    printVars();
    #endif
  }
}

void DebugMsg(const char* msg) {
#ifdef DEBUG
  Serial.println(msg);
#endif
}
void DebugData(char sig, int pin, int state) {
#ifdef DEBUG
  Serial.print(sig);
  Serial.print(pin);
  Serial.print(":");
  Serial.println(state);
#endif
}

void printArray(const char* name, const int arr[], int size) {
  Serial.print(name);
  Serial.print(" = [");
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i]);
    if (i < size - 1) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}

void printVars(){
  curMillis = millis();
  if (curMillis - lastPrint >= 5000) {                 // we choose a cycle time of 10 mS
    lastPrint = curMillis;
    //printArray("sInPinmap",sInPinmap,1);
  }
}

#ifdef DIGITALINPUTS0
void readDigitalInputs0() {
  for (int i = 0; i < cDigitalInputs0; i++) {
    int State = digitalRead(inDigitalSet0[i]);
    if (InDigitalState0[i] != State && millis() - lastInputDebounce0[i] > debounceDelay) {
      InDigitalState0[i] = State;
      lastInputDebounce0[i] = millis();
   
      DebugData('I', inDigitalSet0[i], InDigitalState0[i]);
      if (State) {
        EASYCAT.BufferIn.Cust.inDigitalSet0 |= (1 << i);
      }
      else {
        EASYCAT.BufferIn.Cust.inDigitalSet0 &= ~(1 << i);
      }
    }
  }
}
#endif

#ifdef DIGITALINPUTS1
void readDigitalInputs1() {
  for (int i = 0; i < cDigitalInputs1; i++) {
    int State = digitalRead(inDigitalSet1[i]);
    if (InDigitalState1[i] != State && millis() - lastInputDebounce1[i] > debounceDelay) {
      InDigitalState1[i] = State;
      lastInputDebounce1[i] = millis();
   
      DebugData('I', inDigitalSet1[i], InDigitalState1[i]);
      if (State) {
        EASYCAT.BufferIn.Cust.inDigitalSet1 |= (1 << i);
      }
      else {
        EASYCAT.BufferIn.Cust.inDigitalSet1 &= ~(1 << i);
      }
    }
  }
}
#endif

#ifdef TOGGLEINPUTS0
void readToggleInputs0() {
  for (int i = 0; i < cToggleInputs0; i++) {
    InToggleState0[i] = digitalRead(inToggleSet0[i]);
    if (InToggleState0[i] != oldInToggleState0[i] && millis() - lastsInputDebounce0[i] > debounceDelay) {
      // Button state has changed and debounce delay has passed

      if (InToggleState0[i] == LOW || oldInToggleState0[i] == -1) {  // Stuff after || is only there to send States at Startup
        // Button has been pressed
        toggleinputs0[i] = !toggleinputs0[i];  // Toggle the input state

        if (toggleinputs0[i]) {
          EASYCAT.BufferIn.Cust.inToggleSet0 |= (1 << i);
          DebugData('I', inToggleSet0[i], toggleinputs0[i]);  // Turn the input on
        }
        else {
          EASYCAT.BufferIn.Cust.inToggleSet0 &= ~(1 << i);
          DebugData('I', inToggleSet0[i], toggleinputs0[i]);  // Turn the input off
        }
      }
      oldInToggleState0[i] = InToggleState0[i];
      lastsInputDebounce0[i] = millis();
    }
  }
}
#endif

#ifdef TOGGLEINPUTS1
void readToggleInputs1() {
  for (int i = 0; i < cToggleInputs1; i++) {
    InToggleState1[i] = digitalRead(inToggleSet1[i]);
    if (InToggleState1[i] != oldInToggleState1[i] && millis() - lastsInputDebounce1[i] > debounceDelay) {
      // Button state has changed and debounce delay has passed

      if (InToggleState1[i] == LOW || oldInToggleState1[i] == -1) {  // Stuff after || is only there to send States at Startup
        // Button has been pressed
        toggleinputs1[i] = !toggleinputs1[i];  // Toggle the input state

        if (toggleinputs1[i]) {
          EASYCAT.BufferIn.Cust.inToggleSet1 |= (1 << i);
          DebugData('I', inToggleSet1[i], toggleinputs1[i]);  // Turn the input on
        }
        else {
          EASYCAT.BufferIn.Cust.inToggleSet1 &= ~(1 << i);
          DebugData('I', inToggleSet1[i], toggleinputs1[i]);  // Turn the input off
        }
      }
      oldInToggleState1[i] = InToggleState1[i];
      lastsInputDebounce1[i] = millis();
    }
  }
}
#endif

#ifdef OUTPUTS0
void writeOutputs0() {
  for (int o = 0; o < Outputs0; o++)
  if (EASYCAT.BufferOut.Cust.outDigitalSet0 & (1 << o))  //
    digitalWrite(outDigitalSet0[o], HIGH);               //
  else                                                   //
    digitalWrite(outDigitalSet0[o], LOW);                //
}
#endif

#ifdef OUTPUTS1
void writeOutputs1() {
  for (int o = 0; o < Outputs1; o++)
  if (EASYCAT.BufferOut.Cust.outDigitalSet1 & (1 << o))  //
    digitalWrite(outDigitalSet1[o], HIGH);               //
  else                                                   //
    digitalWrite(outDigitalSet1[o], LOW);                //
}
#endif

#ifdef QUADENC
void readEncoders() {
  if (QuadEncs >= 1) {
#if QUADENCS >= 1
    EncCount[0] = Encoder0.read() / QuadEncMp[0];
#endif
  }
  if (QuadEncs >= 2) {
#if QUADENCS >= 2
    EncCount[1] = Encoder1.read() / QuadEncMp[1];
#endif
  }
  if (QuadEncs >= 3) {
#if QUADENCS >= 3
    EncCount[2] = Encoder2.read() / QuadEncMp[2];
#endif
  }
  if (QuadEncs >= 4) {
#if QUADENCS >= 4
    EncCount[3] = Encoder3.read() / QuadEncMp[3];
#endif
  }
  if (QuadEncs >= 5) {
#if QUADENCS >= 5
    EncCount[4] = Encoder4.read() / QuadEncMp[4];
#endif
  }

  for (int i = 0; i < QuadEncs; i++) {
    if (OldEncCount[i] != EncCount[i]) {
      DebugData('R', i, EncCount[i]);  //send exact Counter
      EASYCAT.BufferIn.Cust.mpg0 = EncCount[i];
      OldEncCount[i] = EncCount[i];
    }
  }
}
#endif

#ifdef STATUSLED
void StatLedErr(int offtime, int ontime) {
  unsigned long NewMillis = millis();

  if (NewMillis - PreviousMillis >= offtime) {
    digitalWrite(constLedPin, HIGH);
  }
  if (NewMillis - PreviousMillis >= offtime + ontime) {
    digitalWrite(constLedPin, LOW);
    PreviousMillis = NewMillis;
  }
}
#endif