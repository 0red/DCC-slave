# DCC-slave
DCC Arduino Nano script - I2C uplink to Master, DCC sniffing (NMRA-DCC)/executing and SPI mcp23s08 mcp23s17 port expander handling

You need to install the [jrArduinoLib library](/../../../jrArduinoLib) in Your Arduino\library folder 


#Actually 
* tested for Arduino Nano board
* pin 2 for DCC connection (DCC_PIN, DCC_IRQ)
* pin 10,11,12,13 SPI for MCP23S08 using 
* A4,A5 for I2C connection to Master  - [P82B715 I2C bus extender](http://www.nxp.com/documents/data_sheet/P82B715.pdf) is recommended 

Serial port Command
* basic jrCMD.h

#Configuration

## Board
```c
ArduinoU arduino={
  "1234567", //name [NAME_MAX-1]
   short(1), //board 
   short(73), //i2c number  
};
```

## Digital Pin
```c
DigitalPinU digitalPin[]={
  {"PinD1",0,3,0,0,300,power},
  {"PinD2",0,4,1,1,0,no_dcc},
  {"PinD3",0,5,0,0,301,sig2},
  {"PinD4",0,7,0,0,303,power}, 
  {"PinD5",0,8,1,1,0,no_dcc},
  {"PinD6",0,13,0,0,304,sig2}
  ...
  {"name",spi,pin,pull_up,in_port,dcc_num,dcc_type},
};

```
## Analog Pin
```c
AnalogPinU analogPin[]={
//  {"PinA1",short(0),uint8_t(600)},
  {"PinA2",short(1),uint8_t(602)},
  {"PinA3",short(2),uint8_t(603)},
  
  {"name",pin,threshold,dcc_type},
};
```

#Thanks for
* http://mrrwa.org/  - You can now install or update this library from within the Arduino IDE Library Manager by searching for “dcc”
