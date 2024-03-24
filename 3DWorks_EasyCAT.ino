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


#define INPUTS  //Use Arduino IO's as Inputs. Define how many Inputs you want in total and then which Pins you want to be Inputs.
#ifdef INPUTS
const int Inputs = 1;  //number of inputs using internal Pullup resistor. (short to ground to trigger)
int InPinmap[] = { 53 };
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

const int timeout = 10000;  // timeout after 10 sec not receiving Stuff
const int debounceDelay = 50;

const int BitOut0 = A2;  // digital output bit 0
const int BitOut1 = A3;  // digital output bit 1
const int BitOut2 = A4;  // digital output bit 2
const int BitOut3 = A5;  // digital output bit 3

const int BitIn[] = {0,1,2,3,4,5,6,7};

#ifdef QUADENC
const int QuadEncs = QUADENCS;
#endif

//---- global variables ---------------------------------------------------------------------------

unsigned long Millis = 0;
unsigned long PreviousMillis = 0;

unsigned long curMillis = 0;
unsigned long lastPrint = 0;

#ifdef INPUTS
int InState[Inputs];
unsigned long lastInputDebounce[Inputs];
#endif

#ifdef QUADENC
long EncCount[QuadEncs];
long OldEncCount[QuadEncs];
#endif

//---- user defined functions ---------------------------------------------------------------------
void Application();
void DebugMsg(const char msg[]);
void DebugData(char sig, int pin, int state);
void readEncoders();
void readInputs();

//---- setup ---------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);  // serial line initialization
                       // (used only for debug)

  DebugMsg("\nEasyCAT - Generic EtherCAT slave\n");  // print the banner

  #ifdef INPUTS
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < Inputs; i++) {
      pinMode(InPinmap[i], INPUT_PULLUP);
    }
  #endif

  pinMode(BitOut0, OUTPUT);  // digital output pins setting
  pinMode(BitOut1, OUTPUT);  //
  pinMode(BitOut2, OUTPUT);  //
  pinMode(BitOut3, OUTPUT);  //

  for (int i = 0; i < 8; i++) {
    pinMode(BitIn[i], INPUT_PULLUP); // digital input pins setting
  }

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
#endif                       // stay in loop for ever \
                             // with the Arduino led blinking
    while (1) {              //
      StatLedErr(500, 500);  //
    }                        //
  }

  PreviousMillis = millis();
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

  Application();  // user applications
#ifdef INPUTS
  readInputs();  //read Inputs & send data
#endif
  printVars();
}



//---- user application ------------------------------------------------------------------------------

void Application()

{
  Millis = millis();                                   // As an example for this application
  if (Millis - PreviousMillis >= 10) {                 // we choose a cycle time of 10 mS
    PreviousMillis = Millis;                           //
                                                       // --- four output bits management ----
                                                       //
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1 << 0))  // the four output bits are mapped to the
      digitalWrite(BitOut0, HIGH);                     // lower nibble of output Byte 0
    else                                               //
      digitalWrite(BitOut0, LOW);                      // we read each bit and write it
                                                       // to the corrisponding pin
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1 << 1))  //
      digitalWrite(BitOut1, HIGH);                     //
    else                                               //
      digitalWrite(BitOut1, LOW);                      //
                                                       //
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1 << 2))  //
      digitalWrite(BitOut2, HIGH);                     //
    else                                               //
      digitalWrite(BitOut2, LOW);                      //
                                                       //
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1 << 3))  //
      digitalWrite(BitOut3, HIGH);                     //
    else                                               //
      digitalWrite(BitOut3, LOW);                      //

    //--- four input bits management ---
    //

    for (int i=0; i<8; i++){
      if (digitalRead(BitIn[i]))                        // the four input pins are mapped to the
        EASYCAT.BufferIn.Cust.inputSet0 |= (1 << i);   // lower nibble of input Byte 6
      else                                             //
        EASYCAT.BufferIn.Cust.inputSet0 &= ~(1 << i);  // we read each pin and write it
    }
  }
#ifdef QUADENC
  readEncoders();  //read Encoders & send data
#endif
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

#ifdef INPUTS
void readInputs() {
  for (int i = 0; i < Inputs; i++) {
    int State = digitalRead(InPinmap[i]);
    if (InState[i] != State && millis() - lastInputDebounce[i] > debounceDelay) {
      InState[i] = State;
      lastInputDebounce[i] = millis();
   
      DebugData('I', InPinmap[i], InState[i]);
      if (State) {
        EASYCAT.BufferIn.Cust.inputSet1 |= (1 << i);
      }
      else {
        EASYCAT.BufferIn.Cust.inputSet1 &= ~(1 << i);
      }
    }
  }
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
    printArray("InPinmap",InPinmap,23);
    printArray("InState",InState,23);
  }
}