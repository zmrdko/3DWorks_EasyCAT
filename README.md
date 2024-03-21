Download Easy Configurator V4.2 (.zip) from here:

https://www.bausano.net/en/hardware/easycat.html

customize according to your needs and burn to eeprom

Download EasyCAT Library V2.0 (.zip) and import to Arduino IDE.
File -> Examples -> EasyCAT_V2_0 -> TestEasyCAT_Custom and replace include .h file with the one created in Easy Configurator.
```
#define CUSTOM
#include "3DWorksEasyCAT.h"         // Custom 3DWorks EtherCAT
```
Edit code to write inputs and outputs according to your needs:

```
EASYCAT.BufferIn.Cust.Temperature = RawTemp.Byte[0];
EASYCAT.BufferIn.Cust.Analog = AnalogValue;
LedStatus = EASYCAT.BufferOut.Cust.Leds;
PwmValue = EASYCAT.BufferOut.Cust.Pwm;
```
