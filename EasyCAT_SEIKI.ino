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



//###################################################IO's###################################################

// Use Arduino IO's as Inputs. Define how many Inputs you want in total and then which Pins you want to be Inputs.
#define INPUTS
#ifdef INPUTS
const int Inputs = 17;  //number of inputs using internal Pullup resistor. (short to ground to trigger)
int InPinmap[] = { 6, 7, 8, 9, 11, 12, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49 };
#endif

// Use Arduino IO's as Toggle Inputs, which means Inputs (Buttons for example) keep HIGH State after Release and Send LOW only after beeing Pressed again.
//#define SINPUTS  //Define how many Toggle Inputs you want in total and then which Pins you want to be Toggle Inputs.
#ifdef SINPUTS
const int sInputs = 2;  //number of inputs using internal Pullup resistor. (short to ground to trigger)
int sInPinmap[] = { 4, 5 };
#endif

// Support for Quadrature Encoders. Define Pins for A and B Signals for your encoders. Visit https://www.pjrc.com/teensy/td_libs_Encoder.html for further explanation.
// Download Zip from here: https://github.com/PaulStoffregen/Encoder and import as Library to your Arduino IDE.
#define QUADENC
#ifdef QUADENC
#include <Encoder.h>
const int QuadEncs = 1; //number of encoders

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

Encoder Encoder0(2,3);              //A,B Pin
//Encoder Encoder1(31, 33);         //A,B Pin
                                    //Encoder Encoder2(A,B);
                                    //Encoder Encoder3(A,B);
                                    //Encoder Encoder4(A,B);
const int QuadEncSig[] = { 2 };  //define wich kind of Signal you want to generate.
                                    //1= send up or down signal (typical use for selecting modes in hal)
                                    //2= send position signal (typical use for MPG wheel)
const int QuadEncMp[] = { 4 };   //some Rotary encoders send multiple Electronical Impulses per mechanical pulse. How many Electrical impulses are send for each mechanical Latch?

#endif

//The Software will detect if there is an communication issue. When you power on your machine, the Buttons etc won't work, till LinuxCNC is running. THe StatusLED will inform you about the State of Communication.
// Slow Flash = Not Connected
// Steady on = connected
// short Flash = connection lost.

// if connection is lost, something happened. (Linuxcnc was closed for example or USB Connection failed.) It will recover when Linuxcnc is restartet. (you could also run "unloadusr arduino", "loadusr arduino" in Hal)
// Define an Pin you want to connect the LED to. it will be set as Output indipendand of the OUTPUTS function, so don't use Pins twice.
// If you use Digital LED's such as WS2812 or PL9823 (only works if you set up the DLED settings below) you can also define a position of the LED. In this case StatLedPin will set the number of the Digital LED Chain.

#define STATUSLED
#ifdef STATUSLED
const int StatLedPin = 13;                 //Pin for Status LED
const int StatLedErrDel[] = { 1000, 10 };  //Blink Timing for Status LED Error (no connection)
const int DLEDSTATUSLED = 0;               //set to 1 to use Digital LED instead. set StatLedPin to the according LED number in the chain.
#endif

//#define DEBUG
//#######################################   END OF CONFIG     ###########################




//---- AB&T EasyCAT shield application 3DWorks SEIKI Panel------------------------------------------  
#define CUSTOM
#include "3DWorksEasyCAT.h"         // Custom 3DWorks EtherCAT

//
//  EASYCAT.BufferIn.Cust.Temperature = RawTemp.Byte[0];
//  EASYCAT.BufferIn.Cust.Analog = AnalogValue;
//  LedStatus = EASYCAT.BufferOut.Cust.Leds;
//  PwmValue = EASYCAT.BufferOut.Cust.Pwm;
//

#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library

EasyCAT EASYCAT;                    // EasyCAT istantiation

                                    // The constructor allow us to choose the pin used for the EasyCAT SPI chip select 
                                    // Without any parameter pin 9 will be used 
                   
                                    // We can choose between:
                                    // 8, 9, 10, A5, 6, 7                                    

                                    // On the EasyCAT board the SPI chip select is selected through a bank of jumpers              

                                    // (The EasyCAT board REV_A allows only pins 8, 9, 10 through 0 ohm resistors)

 //EasyCAT EASYCAT(8);              // example:                                  
                                    // pin 8 will be used as SPI chip select
                                    // The chip select chosen by the firmware must match the setting on the board  

const int debounceDelay = 50;
//---- pins declaration ------------------------------------------------------------------------------


//Variables for Saving States
#ifdef INPUTS
int InState[Inputs];
int oldInState[Inputs];
unsigned long lastInputDebounce[Inputs];
#endif

#ifdef SINPUTS
int sInState[sInputs];
int soldInState[sInputs];
int togglesinputs[sInputs];
unsigned long lastsInputDebounce[sInputs];
#endif

#ifdef QUADENC
long EncCount[QuadEncs];
long OldEncCount[QuadEncs];
#endif

//---- global variables ---------------------------------------------------------------------------


UWORD ContaUp;                      // used for sawthoot test generation
UWORD ContaDown;                    //

unsigned long Millis = 0;
unsigned long PreviousMillis = 0;
unsigned long PreviousSaw = 0;
unsigned long PreviousCycle = 0;


//---- setup ---------------------------------------------------------------------------------------
 
void setup()
{
  Serial.begin(9600);                                             // serial line initialization
                                                                  //(used only for debug)
           
  Serial.print ("\nEasyCAT - Generic EtherCAT slave\n");          // print the banner

  #ifdef INPUTS
    //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < Inputs; i++) {
      pinMode(InPinmap[i], INPUT_PULLUP);
      oldInState[i] = -1;
    }
  #endif

  #ifdef SINPUTS
  //setting Inputs with internal Pullup Resistors
    for (int i = 0; i < sInputs; i++) {
      pinMode(sInPinmap[i], INPUT_PULLUP);
      soldInState[i] = -1;
      togglesinputs[i] = 0;
    }
  #endif



  ContaUp.Word = 0x0000;                                          //
  ContaDown.Word = 0x0000;                                        //
 
                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    Serial.print ("initialized");                                 //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
    Serial.print ("initialization failed");                       //     
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                  // match the one choosen by the firmware
    #ifdef STATUSLED
      pinMode(StatLedPin, OUTPUT);
    #endif                                                        // stay in loop for ever
                                                                  // with the Arduino led blinking
    while(1)                                                      //
    {                                                             //   
      StatLedErr(500, 200);                                       // 
    }                                                             // 
  } 
  
  PreviousMillis = millis();
}


//---- main loop ----------------------------------------------------------------------------------------
 
void loop()                                             // In the main loop we must call ciclically the 
{                                                       // EasyCAT task and our application
                                                        //
                                                        // This allows the bidirectional exachange of the data
                                                        // between the EtherCAT master and our application
                                                        //
                                                        // The EasyCAT cycle and the Master cycle are asynchronous
                                                        //   

  EASYCAT.MainTask();                                   // execute the EasyCAT task
  
  Application();                                        // user applications

  #ifdef INPUTS
    readInputs();  //read Inputs & send data
  #endif

  #ifdef SINPUTS
    readsInputs();  //read Inputs & send data
  #endif

  #ifdef QUADENC
    readEncoders();  //read Encoders & send data
  #endif
}



//---- user application ------------------------------------------------------------------------------

void Application ()                                        

{

  Millis = millis();                                    // As an example for this application 
  if (Millis - PreviousMillis >= 10)                    // we choose a cycle time of 10 mS 
  {                                                     // 
    PreviousMillis = Millis;                            //
                       
                                                        // --- test sawtooth generation --- 
                                                        //
    Millis = millis();                                  // each 100 mS
    if (Millis - PreviousSaw >= 100)                    // 
    {                                                   // 
      PreviousSaw = Millis;                             //
                                                        //
      ContaUp.Word++;                                   // we increment the variable ContaUp  
      ContaDown.Word--;                                 // and dec\wp-content\uploads\2021\03\Mega-tutorial-de-Arduino-Pinout.pngrement ContaDown
    }                                                   //

                                                        // we use these variables to create sawtooth,
                                                        // with different slopes and periods, for
                                                        // test pourpose, in input Bytes 2,3,4,5,30,31
  }   
}

#ifdef INPUTS
void readInputs() {
  for (int i = 0; i < Inputs; i++) {
    int State = digitalRead(InPinmap[i]);
    if (InState[i] != State && millis() - lastInputDebounce[i] > debounceDelay) {
      InState[i] = State;
      //sendData('I', InPinmap[i], InState[i]);

      lastInputDebounce[i] = millis();
    }
  }
}
#endif

#ifdef SINPUTS
void readsInputs() {
  for (int i = 0; i < sInputs; i++) {
    sInState[i] = digitalRead(sInPinmap[i]);
    if (sInState[i] != soldInState[i] && millis() - lastsInputDebounce[i] > debounceDelay) {
      // Button state has changed and debounce delay has passed

      if (sInState[i] == LOW || soldInState[i] == -1) {  // Stuff after || is only there to send States at Startup
        // Button has been pressed
        togglesinputs[i] = !togglesinputs[i];  // Toggle the LED state

        if (togglesinputs[i]) {
          //sendData('I', sInPinmap[i], togglesinputs[i]);  // Turn the LED on
        } else {
          //sendData('I', sInPinmap[i], togglesinputs[i]);  // Turn the LED off
        }
      }
      soldInState[i] = sInState[i];
      lastsInputDebounce[i] = millis();
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
    if (QuadEncSig[i] == 2) {
      if (OldEncCount[i] != EncCount[i]) {
        //sendData('R', i, EncCount[i]);  //send Counter
        EASYCAT.BufferIn.Cust.mpgEncoder1 = EncCount[i];
        OldEncCount[i] = EncCount[i];
      }
    }
    if (QuadEncSig[i] == 1) {
      if (OldEncCount[i] < EncCount[i]) {
        //sendData('R', i, 1);  //send Increase by 1 Signal
        OldEncCount[i] = EncCount[i];
      }
      if (OldEncCount[i] > EncCount[i]) {
        //sendData('R', i, 0);  //send Increase by 1 Signal
        OldEncCount[i] = EncCount[i];
      }
    }
  }
}
#endif

#ifdef STATUSLED
void StatLedErr(int offtime, int ontime) {
  unsigned long newMillis = millis();

  if (newMillis - PreviousMillis >= offtime) {
#ifdef DLED
    if (DLEDSTATUSLED == 1) {
      controlDLED(StatLedPin, 1);
    }
#endif
    if (DLEDSTATUSLED == 0) { digitalWrite(StatLedPin, HIGH); }
  }
  if (newMillis - PreviousMillis >= offtime + ontime) {
    {
#ifdef DLED
      if (DLEDSTATUSLED == 1) {
        controlDLED(StatLedPin, 0);
      }
#endif
      if (DLEDSTATUSLED == 0) { digitalWrite(StatLedPin, LOW); }

      PreviousMillis = newMillis;
    }
  }
}
#endif