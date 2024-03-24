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

#define CUSTOM                      // Custom mode
#define DEBUG
#define spiChipSelect 9             // Chip select pin

//---- includes --------------------------------------------------------------------------------------

#include "3DWorksEasyCAT.h"         // This file has been created by the Easy Configurator
                                    // and must be located in the Arduino project folder
#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library


EasyCAT EASYCAT(spiChipSelect);     // EasyCAT istantiation

                                    // The constructor allow us to choose the pin used for the EasyCAT SPI chip select
                                    // Without any parameter pin 9 will be used 

                                    // We can choose between:
                                    // 8, 9, 10, A5, 6, 7                                    
                                    // On the EasyCAT board the SPI chip select is selected through a bank of jumpers
                                    // (The EasyCAT board REV_A allows only pins 8, 9, 10 through 0 ohm resistors)

//---- pins declaration ------------------------------------------------------------------------------

const int BitOut0 = A2;             // digital output bit 0
const int BitOut1 = A3;             // digital output bit 1
const int BitOut2 = A4;             // digital output bit 2
const int BitOut3 = A5;             // digital output bit 3

const int BitIn00 = 0;              // digital input  bit 0
const int BitIn01 = 1;              // digital input  bit 1
const int BitIn02 = 2;              // digital input  bit 2
const int BitIn03 = 3;              // digital input  bit 3
const int BitIn04 = 4;              // digital input  bit 0
const int BitIn05 = 5;              // digital input  bit 1
const int BitIn06 = 6;              // digital input  bit 2
const int BitIn07 = 7;              // digital input  bit 3



//---- global variables ---------------------------------------------------------------------------

unsigned long Millis = 0;
unsigned long PreviousMillis = 0;

//---- user defined functions ---------------------------------------------------------------------
void Application();
void DebugMsg(const char msg[]);

//---- setup ---------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);                                             // serial line initialization
                                                                  // (used only for debug)
           
  DebugMsg("\nEasyCAT - Generic EtherCAT slave\n");               // print the banner

  pinMode(BitOut0, OUTPUT);                                       // digital output pins setting
  pinMode(BitOut1, OUTPUT);                                       //
  pinMode(BitOut2, OUTPUT);                                       //
  pinMode(BitOut3, OUTPUT);                                       //

  pinMode(BitIn00, INPUT_PULLUP);                                 // digital input pins setting
  pinMode(BitIn01, INPUT_PULLUP);                                 //
  pinMode(BitIn02, INPUT_PULLUP);                                 //
  pinMode(BitIn03, INPUT_PULLUP);                                 //
  pinMode(BitIn04, INPUT_PULLUP);                                 //
  pinMode(BitIn05, INPUT_PULLUP);                                 //
  pinMode(BitIn06, INPUT_PULLUP);                                 //
  pinMode(BitIn07, INPUT_PULLUP);                                 //
                                                                  //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true) {                                   // initialization
    DebugMsg("initialized");                                      // succesfully completed
  }                                                               //
  else {                                                          // initialization failed
    DebugMsg("initialization failed");                            // the EasyCAT board was not recognized
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                  // match the one choosen by the firmware
                                                                  
    pinMode(13, OUTPUT);                                          // stay in loop for ever
                                                                  // with the Arduino led blinking
    while(1)                                                      //
    {                                                             //   
      digitalWrite (13, LOW);                                     // 
      delay(500);                                                 //   
      digitalWrite (13, HIGH);                                    //  
      delay(500);                                                 // 
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

}



//---- user application ------------------------------------------------------------------------------

void Application()

{
  Millis = millis();                                    // As an example for this application 
  if (Millis - PreviousMillis >= 10)                    // we choose a cycle time of 10 mS 
  {                                                     // 
    PreviousMillis = Millis;                            //  
                                                        // --- four output bits management ----
                                                        //                          
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1<<0))     // the four output bits are mapped to the 
      digitalWrite (BitOut0, HIGH);                     // lower nibble of output Byte 0
    else                                                // 
      digitalWrite (BitOut0, LOW);                      // we read each bit and write it
                                                        // to the corrisponding pin
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1<<1))     // 
      digitalWrite (BitOut1, HIGH);                     //
    else                                                //
      digitalWrite (BitOut1, LOW);                      // 
                                                        //
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1<<2))     //   
      digitalWrite (BitOut2, HIGH);                     //
    else                                                //
      digitalWrite (BitOut2, LOW);                      // 
                                                        //  
    if (EASYCAT.BufferOut.Cust.outputSet0 & (1<<3))     // 
      digitalWrite (BitOut3, HIGH);                     //
    else                                                //
      digitalWrite (BitOut3, LOW);                      // 

                                                        //--- four input bits management ---
                                                        //  
    if (digitalRead(BitIn00))                           // the four input pins are mapped to the
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<0);        // lower nibble of input Byte 6
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<0);       // we read each pin and write it 
                                                        // to the corresponding bit
    if (digitalRead(BitIn01))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<1);        //  
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<1);       //
                                                        //
    if (digitalRead(BitIn02))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<2);        //   
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<2);       //  
                                                        //  
    if (digitalRead(BitIn03))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<3);        // 
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<3);       //

    if (digitalRead(BitIn04))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<4);        //
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<4);       //
                                                        //
    if (digitalRead(BitIn05))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<5);        //  
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<5);       //
                                                        //
    if (digitalRead(BitIn06))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<6);        //   
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<6);       //  
                                                        //  
    if (digitalRead(BitIn07))                           //
      EASYCAT.BufferIn.Cust.inputSet0 |= (1<<7);        // 
    else                                                //
      EASYCAT.BufferIn.Cust.inputSet0 &= ~(1<<7);       //
  }
}

void DebugMsg(const char* msg) {
  #ifdef DEBUG
    Serial.println(msg);
  #endif
}
