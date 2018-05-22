#include <SPI.h>
#include <Wire.h>
#include <NmraDcc.h>
#include <avr/wdt.h>


// -------------------------------------- DEFINE ------------------------------

#define DEBUG 1
#define JR_DCC_BOARD
#define TLC5940_BOARDS 2
#define TIMER_PORTFAST1 50

#define I2C_TIMEOUT 200UL
//#define ARDUINO_MEGA 1
#define SPI_8
//#define SPI_16
#define DCC_PIN 2
#define DCC_IRQ 0

#ifdef SPI_8
//#include <gpio_expander.h>
//#include <mcp23s08.h>

//#include <mcp23s17.h>
//https://github.com/sumotoy/gpio_expander
#include <jr_mcp23s08.h>


// SPI number CS pin SPI 1 always will be 01 (1) , SPI 2 10 (2), SPI 2 11 (3)
#define SPI_8_0   7
#define SPI_8_1   7
  #ifdef JR_DCC_BOARD
  #define SPI_8_2   7
  #define SPI_8_3   7
  #endif
#endif

#ifdef SPI_16
#include <gpio_expander.h>
//#include <mcp23s08.h>
#include <mcp23s17.h>
//https://github.com/sumotoy/gpio_expander

//#define SPI_1_16 1
//#define SPI_2_16 2
//#define SPI_3_16 3
//#include <MCP23S17.h>  
//https://github.com/n0mjs710/MCP23S17 
//http://playground.arduino.cc/Main/MCP23S17
#endif

#ifdef TLC5940_BOARDS
 #include "Tlc5940.h"
 #include <jr_tlc5940.h>
 #include "jr_cd4051.h"
 
#endif


#include <jr.h>
#include <jrDCC.h>
#include <jrCMD.h>



#ifndef ARDUINO_MEGA 


/*
byte PIN[4][22]={
  {     //main Arduino Nsno - 255 not used
  255,  //RX  0
  255,  //  TX  1
  255,  //D2 2 DCC IN IRQ0 
  0,    //D3 3 IRQ1 
  1,    //D4  4
  2,    //D5  5
  3,    //D6  6
  4,    //D7  7
  5,    //D8  8
  6,    //D9  9
  7,    //D10 10
  8,    //D11 11 SPI MOSI
  9,    //D12 12 SPI MISO 
  255,  //D13 13 SPI SCK 
  10,   //A0 14
  11,   //A1 15
  12,   //A2 16
  13,   //A3 17
  255,  //A4 18 I2C
  255,  //A5 19 I2C
  14,   //A6 20
  15,   //A7 21
  },
  {16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,255,255,255,255,255,255}, //SPI1
  {32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,255,255,255,255,255,255}, //SPI2
  {48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,255,255,255,255,255,255}  //SPI3
  
};
*/

#endif  


// -------------------------------------- VARIABLES ------------------------------

byte Stan[get_byte( (1<<ANALOG_MAX) + (1<<DIGITAL_MAX) + ((1<<LED_MAX)*SIGNAL_TYPE_LEN) + 1)];
byte pin_send=0;
byte analog_info=0;
NmraDcc  Dcc ;
ardState aState=initial;
JRcmd jrcmd;
char msg_received_bufor [CMD_BUF+1];
char msg_toMaster_bufor [CMD_BUF+1];
byte blocking [bl_end*BLOCKMSG_NR_BITS*8];

#ifdef SPI_1_16
 mcp23s17 spi1(10,0x21);
#endif

#ifdef SPI_2_16
 mcp23s17 spi2(10,0x22);
#endif

#ifdef SPI_3_16
 mcp23s17 spi3(10,0x23);
#endif


#ifdef SPI_8
 JRmcp23s08 spi8_x(10,0,0);
#endif

#ifdef SPI_8_0
 JRmcp23s08 spi8_0(SPI_8_0,0,0);
#endif

#ifdef SPI_8_1
 JRmcp23s08 spi8_1(SPI_8_1,0,1);
#endif

#ifdef SPI_8_2
 JRmcp23s08 spi8_2(SPI_8_2,1,0);
#endif

#ifdef SPI_8_3
 JRmcp23s08 spi8_3(SPI_8_3,1,1);
#endif

#ifdef TLC5940_BOARDS
 JRtlc5940 jrtlc(3,0);
#endif


// -------------------------------------- Board Specific data ------------------------------

ArduinoU arduino={
  "1234567", //name
   short(1), //board
   short(73), //i2c   
};

DigitalPinU digitalPin[]={//name,nr,spi,port,pull_up,in_port,dcc,dcc_type
  {"PinD1",100,1,3,0,0,300,power},
  {"PinD2",101,1,4,1,1,0,no_dcc},
  {"PinD3",102,1,5,0,0,301,sig2},
  {"PinD4",103,1,7,0,0,303,power},
  {"PinD5",104,1,8,1,1,0,no_dcc},
  {"PinD6",105,1,13,0,0,304,sig2}
};

AnalogPinU analogPin[]={   //name,nr,port,tresh,typ
//  {"PinA1",50,short(0),uint8_t(600),proximity},
  {"PinA2",51,short(1),uint8_t(602),proximity},
  {"PinA3",52,short(2),uint8_t(603),proximity}
};

LedPinU ledPin[]={
    {"PinL1",2,short(16),uint8_t(700),sem3},
    {"PinA2",21,short(1),uint8_t(702),sem3}
//  {"PinA3",22,short(2),uint8_t(603)}
};

byte ledPinVal[sizeof(ledPin)];

int _setSignal(int i) {
  if (i>=(sizeof(ledPin))/sizeof(LedPinU)) return -1;
  int port=ledPin[i].a.port;
  int i2=arduino.a.analog+arduino.a.digital+1;
  short kolor=0;
  for (int i1=0; i1<SIGNAL_TYPE_LEN;i1++) 
        if (GetBit(Stan,i2+i1+i*SIGNAL_TYPE_LEN)) SetBite(kolor,i1);
        
  JR_PRINTDECV("_setSignal i",i);
  JR_PRINTDECV(" port",port);
  
  switch (ledPin[i].a.signal_type) {
    case sem4:
        switch (kolor) {
            case yellow:
            case green:
            case white:
            case red:
            default:
              break;
        };
        
      break;
    case sem3:
    case lin3:
        switch (kolor) {
            
            case yellow:          
              jrtlc.led_on(port+0,1);
              jrtlc.led_on(port+1,0);
              jrtlc.led_on(port+2,0);

              break;
            case green:
            case white:
              jrtlc.led_on(port+0,0);
              jrtlc.led_on(port+1,0);
              jrtlc.led_on(port+2,1);
              break;
            case red:
            default:
              jrtlc.led_on(port+0,0);
              jrtlc.led_on(port+1,1);
              jrtlc.led_on(port+2,0);
              break;
        };
        
      break;
    case man2:
        switch (kolor) {

            case yellow:
            case green:
            case white:
              jrtlc.led_on(port+0,1);
              jrtlc.led_on(port+1,0);
              break;
            case red:         
            default:
              jrtlc.led_on(port+0,0);
              jrtlc.led_on(port+1,1);
              break;
        }
      break;
  }
  jrtlc.led_loop();
}

void initSignals() {
  for (int i=0;i<(sizeof(ledPin))/sizeof(LedPinU);i++) _setSignal(i);
}

void setSignal(int _id,short kolor) {
  for (int i=0;i<(sizeof(ledPin))/sizeof(LedPinU);i++) {
    
   JR_PRINTLNF("setSignal"); Serial.print (i);
  JR_PRINTDECV("i",i);
  JR_PRINTDECV(" nr",ledPin[i].a.nr);
  JR_PRINTDECV(" port",ledPin[i].a.port);
  JR_LN;
  
  if (ledPin[i].a.nr==_id) {
      JR_PRINTLNF("setSignal"); 
      int i2=arduino.a.analog+arduino.a.digital+1;
      for (int i1=0; i1<SIGNAL_TYPE_LEN;i1++) 
        if (GetBite(kolor,i1)) { SetBit(Stan,i2+i1+i*SIGNAL_TYPE_LEN); } else {ClearBit(Stan,i2+i1+i*SIGNAL_TYPE_LEN); }
      //ledPinVal[i]=kolor;
      _setSignal(i);
    }
  }
}


bool jr_cmd_signal(ParserParam *p1){
//  Serial.print("dupa2");
  JR_PRINTLNF("jr_cmd_signal"); 
  ParserParam p=*p1;
  setSignal(p.i[1],p.i[2]);
  JR_PRINTV("sig_id",p.i[1]);
  JR_PRINTV("kolor",p.i[2]);
  JR_LN;
  
};


// -------------------------------------- CMD function ------------------------------

bool jr_cmd_stan(ParserParam *p1){
  JR_PRINTLNF("jr_cmd_stan"); 
  ParserParam p=*p1;
  for (int i=0;i<arduino.a.bytes;i++) { JR_PRINTBINV(i,Stan[i]);}

  int j=(1<<DIGITAL_MAX)+(1<<ANALOG_MAX)+1;
  JR_PRINTDECV("A",(1<<ANALOG_MAX));
  JR_PRINTDECV("A",(1<<DIGITAL_MAX));
  JR_PRINTDECV("S",j);
  JR_PRINTDECV("B",get_byte(j));  
  JR_PRINTF("TO_MASTER:");JR_PRINTDEC(msg_toMaster_bufor[0]);JR_PRINTLN(msg_toMaster_bufor);
  JR_LN;
  
}


bool jr_cmd_spi(ParserParam *p1){
  JR_PRINTLNF("jr_cmd_spi"); 
  ParserParam p=*p1;
  setDPort(p.i[1],p.i[2],(p.i[3]==0)?LOW:HIGH);
  JR_PRINTV("spi",p.i[1]);
  JR_PRINTV("port",p.i[2]);
  JR_PRINTV("val",(p.i[3]==0)?LOW:HIGH);
  JR_LN;
  
};

bool jr_cmd_update(ParserParam *p1){
  JR_PRINTLNF("jr_cmd_update  - getDPortFast_Update();"); 
  ParserParam p=*p1;
  getDPortFast_Update();
}


bool jr_cmd_state(ParserParam *p1){
  JR_PRINTLNF("jr_cmd_state"); 
  ParserParam p=*p1;
  
  getDPortFast_Update();
  
  for (int i=0;i<=8;i++) {
    JR_PRINTV(i,analogRead(i+A0));    
    //if (i%5==3) { JR_LN;}
  }
  JR_LN;


  for (int i=0;i<20;i++) {
    if (i%10==0) { JR_LN; JR_PRINTF("0: ");}
    if (i<10) { JR_PRINTF(" ");}
    JR_PRINTV(i,digitalRead(i));    
  }
  JR_LN;JR_LN;
  
  #ifdef SPI_16
  for (int i=1;i<4;i++) {
    #ifndef SPI_1_16
    if (i==1) continue;
    #endif
    #ifndef SPI_2_16
    if (i==2) continue;
    #endif
    #ifndef SPI_3_16
    if (i==3) continue;
    #endif
    JR_PRINT(i);JR_PRINT(": ");
    for (int j=0;j<16;j++) {
      if (j<10) { JR_PRINTF(" ");}
      JR_PRINTV(j,getDPort(i,j));   
    }
    JR_LN;
  }
  #endif

  #ifdef SPI_8
  for (int i=1;i<4;i++) {
    #ifndef SPI_8_0
    if (i==1) continue;
    #endif
    #ifndef SPI_8_1
    if (i==2) continue;
    #endif
    #ifndef SPI_8_2
    if (i==3) continue;
    #endif
    JR_PRINT(i);JR_PRINT(": ");
    for (int j=0;j<8;j++) {
      if (j<10) { JR_PRINTF(" ");}
      JR_PRINTV(j,getDPort(i,j));   
    }
    JR_LN;
  }
  #endif

  
};

bool jr_cmd_master_msg (ParserParam *p1){  // send message to i2c master
  JR_PRINTLNF("jr_cmd_master_msg"); 
  ParserParam p=*p1;
  strcpy(msg_toMaster_bufor,p.raw+3);
   JR_PRINTF("TO_MASTER:");JR_PRINTLN(msg_toMaster_bufor);
};

// -------------------------------------- Pin Initialize ------------------------------

void pin_initialize() {
  byte JRpullup[6]={0,0,0,0,0,0};
  uint16_t a2;
  uint8_t  a1;


  JR_PRINTLNF("pin_initialize start");
  #ifdef SPI_8
    spi8_x.initialize();  //for working only
  #endif
  
  #ifdef SPI_8_0
    spi8_0.initialize();
  #endif

  #ifdef SPI_8_1
    spi8_1.initialize();
  #endif

  #ifdef SPI_8_2
    spi8_2.initialize();
  #endif

  #ifdef SPI_8_3
    spi8_3.initialize();
  #endif

  aState=initial;
  pin_send=0;

// =================== digital pin initialization ==================
  for (int i=0;i<arduino.a.digital;i++) {
    if (digitalPin[i].a.dcc_type==sig3 || digitalPin[i].a.dcc_type==switch3 || digitalPin[i].a.dcc_type==xswitch) {
//        digitalPin[i].a.dcc=digitalPin[i].a.dcc>>1<<1; // convert to parity 298=>298 299->298
    }
    
    #ifdef ARDUINO_MEGA 
    if (digitalPin[i].a.spi==0 && digitalPin[i].a.port<54) {
    #endif
    #ifndef ARDUINO_MEGA 
    if (digitalPin[i].a.spi==0 && digitalPin[i].a.port<20) {
    #endif
      if (digitalPin[i].a.in_port ) {
        if (digitalPin[i].a.pull_up) { 
          pinMode(digitalPin[i].a.port,INPUT_PULLUP);
          digitalWrite(digitalPin[i].a.port,HIGH);
        } else {
          pinMode(digitalPin[i].a.port,INPUT);        
        }
      } else {
        pinMode(digitalPin[i].a.port,OUTPUT);
        digitalWrite(digitalPin[i].a.port,HIGH);
      }
    }
  #ifdef SPI_16
  JR_PRINTF("SPI16:");
    if (
      (digitalPin[i].a.spi==1 && digitalPin[i].a.port<16) ||
      (digitalPin[i].a.spi==2 && digitalPin[i].a.port<16) ||
      (digitalPin[i].a.spi==3 && digitalPin[i].a.port<16) ) {
          if (digitalPin[i].a.pull_up || digitalPin[i].a.in_port) {
            #ifdef SPI_1_16
              if (digitalPin[i].a.spi==1) {spi1.gpioPinMode(digitalPin[i].a.port,INPUT);}
            #endif          
            #ifdef SPI_2_16
              if (digitalPin[i].a.spi==2) {spi2.gpioPinMode(digitalPin[i].a.port,INPUT);}
            #endif          
            #ifdef SPI_3_16
              if (digitalPin[i].a.spi==3) {spi3.gpioPinMode(digitalPin[i].a.port,INPUT);}
            #endif          

            if (digitalPin[i].a.pull_up) {
              int bit=(int)digitalPin[i].a.port+((digitalPin[i].a.spi-1)*16)
              SetBit(JRpullup,bit);
            }
          }
      }
      JR_PRINTLNF("SPI16: end");
  #endif

    #ifdef SPI_8
    JR_PRINTF("SPI8:");
            #ifdef SPI_8_0
              JR_PRINTF("8_0:");
              if (digitalPin[i].a.spi==1 && digitalPin[i].a.in_port) { spi8_0.setIN    (digitalPin[i].a.port,true);}
              if (digitalPin[i].a.spi==1 && digitalPin[i].a.pull_up) { spi8_0.setPullUp(digitalPin[i].a.port,true);}
            #endif          
            #ifdef SPI_8_1
              JR_PRINTF("8_1:");
              if (digitalPin[i].a.spi==2 && digitalPin[i].a.in_port) { spi8_1.setIN    (digitalPin[i].a.port,true);}
              if (digitalPin[i].a.spi==2 && digitalPin[i].a.pull_up) { spi8_1.setPullUp(digitalPin[i].a.port,true);}
            #endif          
            #ifdef SPI_8_2
              JR_PRINTF("8_2:");
              if (digitalPin[i].a.spi==3 && digitalPin[i].a.in_port) { spi8_2.setIN    (digitalPin[i].a.port,true);}
              if (digitalPin[i].a.spi==3 && digitalPin[i].a.pull_up) { spi8_2.setPullUp(digitalPin[i].a.port,true);}
            #endif          
			#ifdef SPI_8_3
              JR_PRINTF("8_3:");
              if (digitalPin[i].a.spi==4 && digitalPin[i].a.in_port) { spi8_3.setIN    (digitalPin[i].a.port,true);}
              if (digitalPin[i].a.spi==4 && digitalPin[i].a.pull_up) { spi8_3.setPullUp(digitalPin[i].a.port,true);}
            #endif          

      JR_PRINTLNF("SPI8: end");
  #endif

    
  }


// =================== analog pin initialization ==================

 JR_PRINTLNF("pin_initialize analog"); 
  for (int i=0;i<arduino.a.analog;i++) {
      //analogPin
  }


  #ifdef SPI_1_16
   DEBUG_PRINT(F("spi1 port update"));
   spi1.gpioPortUpdate();
  #endif
  #ifdef SPI_2_16
     DEBUG_PRINT(F("spi2 port update"));
     spi2.gpioPortUpdate();
  #endif
  #ifdef SPI_3_16
     DEBUG_PRINT(F("spi3 port update"));
     spi3.gpioPortUpdate();
  #endif
  
  JR_PRINTLNF("pin_initialize end pin_init");
  
}



// -------------------------------------- setDPort ------------------------------

boolean setDPort(short spi,short port,short val) {
  val=((val)?HIGH:LOW);

  #ifndef ARDUINO_MEGA 
    if (spi==0 && port<20) {
  #endif
    #ifdef ARDUINO_MEGA 
    if (spi==0 && port<54) {
  #endif
//      digitalWrite(int(digitalPin[port].a.port),val ); //0 - HIGH, 1 - LOW
      digitalWrite(port,val ); //0 - HIGH, t1 - LOW
    } 
  
  #ifdef SPI_1_16
    if (spi==1 && port>=0 && port<16) { spi1.gpioDigitalWrite(port,((val==LOW) ? false : true )); }
  #endif
        
  #ifdef SPI_2_16
    if (spi==2 && port>=0 && port<16) { spi2.gpioDigitalWrite(port,((val==LOW) ? false : true )); }
  #endif

  #ifdef SPI_3_16
    if (spi==3 && port>=0 && port<16) { spi3.gpioDigitalWrite(port,((val==LOW) ? false : true )); }
  #endif

  #ifdef SPI_8_0
    if (spi==1 && port>=0 && port<8) { spi8_0.pinWrite(port,((val==LOW) ? false : true )); }
  #endif
        
  #ifdef SPI_8_1
    if (spi==2 && port>=0 && port<8) { spi8_1.pinWrite(port,((val==LOW) ? false : true )); }
  #endif

  #ifdef SPI_8_2
    if (spi==3 && port>=0 && port<8) { spi8_2.pinWrite(port,((val==LOW) ? false : true )); }
  #endif

  #ifdef SPI_8_3
    if (spi==4 && port>=0 && port<8) { spi8_3.pinWrite(port,((val==LOW) ? false : true )); }
  #endif
  
}

// -------------------------------------- getDPort FAST------------------------------
// need to call getDPortFastUpdate prior get

#ifdef SPI_16 
union SPIports16 {
  uint16_t spi[3];
  byte b[6];
};
SPIports16 spiPorts16;
#endif

#ifdef SPI_8 
union SPIports8 {
  uint8_t spi[5];
  byte b[4];
};
SPIports8 spiPorts8;
#endif


void getDPortFast_Update() {
  
//DEBUG_PRINT("");  
  #ifdef SPI_16 
    for (byte i=0;i<6;i++) spiPorts16.b[i]=0;
    #ifdef SPI_1_16 
      spiPorts16.spi[0]=spi1.readGpioPort();  
      JR_PRINTBINV(F("1A"),spiPorts8.b[0]);
      JR_PRINTBINV(F("1B"),spiPorts8.b[1]);
    #endif
    #ifdef SPI_2_16 
      spiPorts16.spi[1]=spi2.readGpioPort();  
      JR_PRINTBINV(F("2A"),spiPorts8.b[2]);
      JR_PRINTBINV(F("2B",spiPorts8.b[3]);
    #endif
    #ifdef SPI_3_16 
      spiPorts16.spi[2]=spi3.readGpioPort();  
      JR_PRINTBINV(F("3A"),spiPorts8.b[4]);
      JR_PRINTBINV(F("3B"),spiPorts8.b[5]);
    #endif
  #endif
 
  #ifdef SPI_8
    for (byte i=0;i<4;i++) spiPorts8.b[i]=0;
    #ifdef SPI_8_0
      spiPorts8.spi[0]=spi8_0.getR(9);  //get GPIO A ports
//      JR_PRINTBINV(F("1A"),spiPorts8.b[0]);
      _critical();
    #endif
    #ifdef SPI_8_1 
      spiPorts8.spi[1]=spi8_1.getR(9);  //get GPIO A ports
//      JR_PRINTBINV(F("2A"),spiPorts8.b[1]);
      _critical();
    #endif
    #ifdef SPI_8_2 
      spiPorts8.spi[2]=spi8_2.getR(9);  //get GPIO A ports
//      JR_PRINTBINV(F("3A"),spiPorts8.b[2]);
      _critical();
    #endif
    #ifdef SPI_8_3
      spiPorts8.spi[3]=spi8_3.getR(9);  //get GPIO A ports
//      JR_PRINTBINV(F("3A"),spiPorts8.b[3]);
      _critical();
    #endif

    //JR_LN;  
  #endif
    
}

short getDPortFast (short spi,short port) {
  //return LOW_HIGH
  if (spi==0) return getDPort(0,port);
  #ifdef SPI_16  
  if (spi<4) {
    return (TestBit(spiPorts16.b,(spi-1)*16+(port+1))?LOW:HIGH);
	}
  #endif
  #ifdef SPI_8
  if (spi<5) {
    return (TestBit(spiPorts8.b,(spi-1)*8+(port+1))?LOW:HIGH);
	}
  #endif

}

bool jr_cmd_rr(ParserParam *p1){
  JR_PRINTLNF("Read address spi8_0");   
  ParserParam p=*p1;
  #ifdef SPI_8_0
    byte b=spi8_0.getR(p.i[1]);
    JR_PRINTBINV(p.i[1], b   );
  #endif
};

bool jr_cmd_wr(ParserParam *p1){
  JR_PRINTLNF("Write address spi8_0"); 
  ParserParam p=*p1;
  #ifdef SPI_8_0
  spi8_0.setR(p.i[1],p.i[2]);
    JR_PRINTDECV(F("reg"),p.i[1]);
    JR_PRINTBINV(F("v"),p.i[2]);
  #endif
  
};


// -------------------------------------- getDPort ------------------------------

short getDPort(short spi,short port) {
  #ifndef ARDUINO_MEGA 
  if (spi==0 && port<20) { return digitalRead(port);
  }

  #endif
  #ifdef ARDUINO_MEGA 
  if (spi==0 && port<54) { return digitalRead(port);
  } 
  #endif

  #ifdef SPI_1_16
    if (spi==1 && port>=0 && port<16) { return((spi1.gpioDigitalRead(port)>0) ? HIGH : LOW ); }
  #endif

  #ifdef SPI_2_16
    if (spi==2 && port>=0 && port<16) { return((spi2.gpioDigitalRead(port)>0) ? HIGH : LOW ); }
  #endif

  #ifdef SPI_3_16
    if (spi==3 && port>=0 && port<16) { return((spi3.gpioDigitalRead(port)>0) ? HIGH : LOW ); }
  #endif
  
  #ifdef SPI_8_0
    //JR_PRINTLNF("ala");
    //JR_PRINTLN(spi8_0.pinRead(port));
    if (spi==1 && port>=0 && port<8) { return((spi8_0.pinRead(port)>0) ? HIGH : LOW ); }
  #endif

  #ifdef SPI_8_1
    if (spi==2 && port>=0 && port<8) { return((spi8_1.pinRead(port)>0) ? HIGH : LOW ); }
  #endif

  #ifdef SPI_8_2
    if (spi==3 && port>=0 && port<8) { return((spi8_2.pinRead(port)>0) ? HIGH : LOW ); }
  #endif

  #ifdef SPI_8_3
    if (spi==4 && port>=0 && port<8) { return((spi8_3.pinRead(port)>0) ? HIGH : LOW ); }
  #endif
  
  return 1;
}


unsigned long timer_portfast;

// -------------------------------------- setup ------------------------------

void setup() {
  wdt_disable();
  Serial.begin(115200);           // start serial for output
  delay(10);
  JR_PRINTLNF("Setup START");

  msg_received_bufor[0]=0;
  msg_toMaster_bufor[0]=0;

  
  sprintf_P(msg_toMaster_bufor,PSTR("DEL %u"),arduino.a.i2c);
  
  
  // pin initialization
  arduino.a.digital=short(sizeof (digitalPin)/ sizeof (DigitalPinU)),
  arduino.a.analog =short(sizeof (analogPin) / sizeof (AnalogPinU) ),
  arduino.a.led =short(sizeof (ledPin) / sizeof (LedPinU) ),
  arduino.a.bytes  =get_byte(arduino.a.analog+arduino.a.digital+1+arduino.a.led*SIGNAL_TYPE_LEN);
  // +1 for bit informing for pending message to MASTER 
  
  aState=initial;
  pin_initialize(); 
  for (int i=0; i<sizeof(Stan);i++) Stan[i]=0;

  // I2C initialization
  JR_PRINTLNF("I2C Wire Library START");
  Wire.begin(int(arduino.a.i2c));                // join i2c bus with address #8
  TWAR = ((int(arduino.a.i2c)) << 1) | 1;  // enable broadcasts to be received

  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  JR_PRINTDECV (F("I2C number"),byte(arduino.a.i2c));
  JR_LN;
  
  // DCC Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  JR_PRINTLNF("NMRA DCC Library Start");
  Dcc.pin(DCC_IRQ, DCC_PIN, 1);
  JR_PRINTDECV (F("irq"),byte(DCC_IRQ));
  JR_PRINTDECV (F("pin"),byte(DCC_PIN));
  JR_LN;
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );


  jrcmd.standard();
  jrcmd.add(F("SPI")    ,&jr_cmd_spi);
  jrcmd.add(F("UPDATE") ,&jr_cmd_update);
  jrcmd.add(F("STATE")  ,&jr_cmd_state);
  jrcmd.add(F("WR")     ,&jr_cmd_wr);
  jrcmd.add(F("RR")     ,&jr_cmd_rr);
  jrcmd.add(F("MA")     ,&jr_cmd_master_msg);
  jrcmd.add(F("STAN")   ,&jr_cmd_stan);
  jrcmd.add(F("AALL")   ,&JRcd4051_cmdAll);
  jrcmd.add(F("AGET")   ,&JRcd4051_getA);
  jrcmd.add(F("ATRE")   ,&JRcd4051_setTreshhold);

  jrcmd.add(F("LED")    ,&JRtlc5940_cmdShow);
  jrcmd.add(F("LEDON")  ,&JRtlc5940_cmdLed);
  jrcmd.add(F("LEDSET") ,&JRtlc5940_cmdLedVal);
  jrcmd.add(F("DEMO")   ,&JRtlc5940_cmdDemo);
  jrcmd.add(F("SIG")    ,&jr_cmd_signal);

  jrtlc.init();
  timer_portfast=millis();
  initSignals();
  JR_PRINTLNF("Setup END");
  
}

// -------------------------------------- loop ------------------------------


void _critical() {
    Dcc.process();
    #ifdef TLC5940_BOARDS
        //jrtlc.demo_loop(1000);
      jrtlc.led_loop();
    #endif  
}

void loop() {
   _critical(); 

  byte i;
  // put your main code here, to run repeatedly:
//    Serial.println("Loop");
  
  // I2C timeout - to do


  if (msg_received_bufor[0]) {
    jrcmd.parse(msg_received_bufor,strlen(msg_received_bufor));
    msg_received_bufor[0]=0;
  }
  
  jrcmd.proceed(&Serial);
  //delay(10);
  
//return;

  // prepare the state of the pins  

 if (millis()>timer_portfast+TIMER_PORTFAST1) {
      getDPortFast_Update();
      timer_portfast=millis();
    
  
  for (i=0;i<arduino.a.digital;i++) {
    // Reed (kontaktron) & analog optical proximity sensors show LOW in case of USE 
    // therefore 0 (HIGH) state is normally set after the data get by I2C
    // and 1 (LOW) in case of any sensor occupancy found
    // 
    // todo --> check the Occupancy board result and reaction --> 
    //          1 on I2C means Train is on the track 
    //          0 - possibly no train (NOW)
    
    if (getDPortFast(digitalPin[i].a.spi,digitalPin[i].a.port)==LOW) {SetBit(Stan,i);}
  }
  for (i=0;i<arduino.a.analog;i++) {
  #ifndef ARDUINO_MEGA 
    if (analogRead(analogPin[i].a.port+A0)<analogPin[i].a.threshold) {
      int j=int(i+arduino.a.digital);
      SetBit(Stan,j);
    }
  #endif
  #ifdef ARDUINO_MEGA 
    // to be check the Mapping !!!!
    if (analogRead(analogPin[i].a.port+A0)<analogPin[i].a.threshold) {
      int j=int(i+arduino.a.digital);
      SetBit(Stan,j);
    }  
  #endif
    for (i=0;i<arduino.a.led;i++) {  
      int j=int(arduino.a.digital+arduino.a.analog+1+i*SIGNAL_TYPE_LEN);
      for (byte i1=0;i1<SIGNAL_TYPE_LEN;i1++) {
        
      }
    }
  }

  // add info if any message pending
  if (msg_toMaster_bufor[0]) {
      int i=int(arduino.a.digital+arduino.a.analog);
      SetBit(Stan,i); 
      //JR_PRINT("+");      JR_PRINTDEC(arduino.a.digital+arduino.a.analog);
  } else {
      int i=int(arduino.a.digital+arduino.a.analog);
      ClearBit(Stan,i); 
      //JR_PRINT("-");
  }


 }
}


// -------------------------------------- I2C request ------------------------------

// function that executes whenever data is requested by I2C master 
// this function is registered as an event, see setup()
void requestEvent() {
  JR_PRINTDECV(F("REQ: aState"),aState);
  JR_PRINTDECV(F("pin_send"),pin_send);
   arduino.a.ts=millis();

  switch (aState) {
  // block DCC current (if on board) and initialize module
  case initial:
    JR_PRINTF(" initial");
    break;
    
  //check if connection still exists
  case keep_alive:
    JR_PRINTF(" keep_alive");
    //send Arduino board info and reset quere
    break;
    
  case hand_shake:
    JR_PRINTF(" hand_shake");
    JR_VF(PROTOCOL_VER);
    //reset Pin quere and send first Digital
    Wire.write (byte(PROTOCOL_VER));
    Wire.write (arduino.b,sizeof(ArduinoU));
    JR_PRINTDECV(F(" size"),sizeof(ArduinoU));
    pin_send=0;
    break;
    
  case reset_pin_info:
    JR_PRINTF(" reset_pin_info");
    pin_send=0;
//    Wire.write(byte(arduino.a.digital));
//    Wire.write(byte(arduino.a.analog));    
    break;
  
  case get_pin_info: 
    JR_PRINTF(" get_pin_info");

  // send NEXT Digital & after Analog Pin info or NULL if no more Pin Info
      if (pin_send<arduino.a.digital) {
        JR_PRINTF(" D");
        JR_PRINTDECV(F(" pin"),digitalPin[pin_send].a.port);
        JR_PRINTDECV(F(" spi"),digitalPin[pin_send].a.spi);
        //send digital PIN
        Wire.write(digitalPin[pin_send].b,sizeof(DigitalPinU));
        JR_PRINTDECV(F(" size"),sizeof(DigitalPinU));
        pin_send++;
      } else {
      //if all Digital send --> propagate analogPin
      
        if (pin_send<arduino.a.digital+arduino.a.analog) {
          //send analog PIN
          JR_PRINTF(" A");
          JR_PRINTDECV(F(" pin"),analogPin[pin_send-arduino.a.digital].a.port);
          Wire.write(analogPin[pin_send-arduino.a.digital].b,sizeof(AnalogPinU));
          JR_PRINTDECV(F(" size"),sizeof(AnalogPinU));
          pin_send++;  
        } else {
      //if all Digital send --> propagate analogPin
      
          if (pin_send<arduino.a.digital+arduino.a.analog+arduino.a.led) {
            //send analog PIN
            JR_PRINTF(" L");
            JR_PRINTDECV(F(" pin"),ledPin[pin_send-arduino.a.digital-arduino.a.analog].a.port);
            Wire.write(ledPin[pin_send-arduino.a.digital-arduino.a.analog].b,sizeof(LedPinU));
            JR_PRINTDECV(F(" size"),sizeof(LedPinU));
            pin_send++;  
          }   
        }
      }
      
    break;
    
  case get_states:
    JR_PRINTF(" get_states ");
        //send state
        Wire.write(Stan,arduino.a.bytes);
        {
          //clear state
          for (int i=0; i<arduino.a.digital+arduino.a.analog+1;i++) ClearBit(Stan,i);
          /*
          for (int i=0; i<sizeof(Stan);i++) {
            JR_PRINTBINV(i,Stan[i]);
            Stan[i]=0;
          }
          */
            JR_PRINTF(" clean Stan ");
//            while(1);
        }
    break;

  
  case msg_to_master:  
    JR_PRINTF(" msg_to_master ");
    JR_PRINT(CMD_BUF);
    Wire.write(msg_toMaster_bufor,CMD_BUF);
    msg_toMaster_bufor[0]=0;msg_toMaster_bufor[CMD_BUF]=0;  //clear the bufer
    
    break;
    
  case get_Apin:
    //not tested  !!!!
    JR_PRINTF(" get_Apin");
    JR_VDF(analog_info);
    {
     int res=-1;
     int tre=-1;
    
   
     if (analog_info>=A0) {
        res=analogRead(analog_info);
        #ifndef ARDUINO_MEGA 
          for (int i=0;i<arduino.a.analog;i++) {
            if (analogPin[i].a.port==(analog_info-A0)) tre=analogPin[i].a.threshold;
          };
        #endif  
     } else {
      JR_PRINTDECV(F(" threshold"),analogPin[analog_info].a.threshold);
      if (analog_info<arduino.a.analog) {
        res=analogRead(A0+analogPin[analog_info].a.port);
        tre=analogPin[analog_info].a.threshold;
      }
     } 
     wire_writeword(res);
     wire_writeword(tre);
    }
  
  // return 2bytes Analog value of the PIN + 2 bytes of threshold
    break;

  // set the threshold
  case set_Atreshold:
      //not tested  !!!!
    break;
  default:
    break;
  JR_PRINTF(" DEFAULT");
  };

  JR_PRINTLNF(" :REQ");
}



boolean wait_bytes(byte ile,int _ts) {
  unsigned long  _init=millis();

  while (Wire.available()<ile) {
          _critical(); 
          if (_init+_ts>millis()) {
            JR_LN;JR_PRINTF("!!! wait_bytes fails: ");JR_V(_init);JR_V(_ts);JR_PRINT(millis());JR_LN;
            return false;
          }
        }
  return true;
}

#ifdef I2C_TIMEOUT
boolean wait_bytes(byte ile) {
  return wait_bytes(ile,I2C_TIMEOUT);
}
#endif

// -------------------------------------- I2C receive ------------------------------

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {

  JR_PRINTDECV(F("REC: howMany"),howMany);
  JR_PRINTDECV(F("pin_send"),pin_send);
  arduino.a.ts=millis();
  if (!howMany) return ;
  unsigned short command=Wire.read();
  aState=ardState(command);
  JR_PRINTDECV(F(" state"),aState);
  
  switch (ardState(command)) {
    case initial:
//      jr_soft_reset();
      pin_initialize(); 
      sprintf_P(msg_toMaster_bufor,PSTR("DEL %u"),arduino.a.i2c);
      break;
    case reset_pin_info:
      msg_toMaster_bufor[0]=0;
      pin_send=0;
      JR_VF(pin_send);
      break;

    case get_Apin:
      wait_bytes (1);
      analog_info=Wire.read();
      JR_VF(analog_info);
        break;
        
    case set_Atreshold:
      {
        wait_bytes (3);
                
        unsigned short port=Wire.read(); 
        int wire_readword(tre);
        JR_VF(port);
        JR_VF(tre);
        
        if (port<arduino.a.analog) {
          analogPin[port].a.threshold=tre & (B00000011 * 256 + B11111111); // 10 bits value
          JR_PRINTDECV(F(" set"),analogPin[port].a.threshold);
        }
      }
      break;
    case setD:
      {
        wait_bytes (3);
        
        short spi =Wire.read(); 
        short port=Wire.read(); 
        short val =Wire.read();
        JR_VF(spi);
        JR_VF(port);
        JR_VF(val);
        if (spi<4) {
                             //"normal" ports
            setDPort (spi,port,val);
            JR_PRINTDECV(F(" HW Pin set"),getDPort(spi,port));
        } else {            
                            //ports from digital array  --> SPI will be ommited
          if (port<arduino.a.digital) {
            setDPort (digitalPin[port].a.spi,digitalPin[port].a.port,val);
            JR_PRINTDECV(F(" digitalPin set"),getDPort(digitalPin[port].a.spi,digitalPin[port].a.port));
          }
        }
      }
      break;

    case msg_to_slave:
      {
        JR_PRINTF(" msg ");
        wait_bytes (1);
        short ile =Wire.read();
        
        wait_bytes (ile);
        for (int i=0;i<ile;i++) {
          if (i<CMD_BUF) msg_received_bufor[i]=Wire.read();  //in case of ile longer then CMD_BUF
        }
        msg_received_bufor[ile]=0;msg_received_bufor[CMD_BUF]=0; // for security
        JR_VF(ile);JR_PRINT(msg_received_bufor);
      }
      break;
    case block_msg:
      JR_PRINTF(" block msg ");
      wait_bytes (sizeof(BlockMsg));
      {
        BlockMsgU msg;  
        for (int i=0;i<sizeof(BlockMsg);i++) msg.b[i]=Wire.read();
        JR_PRINTDECV(F(" block?"),msg.a.block);
        JR_PRINTDECV(F(" typ"),msg.a.typ);
        JR_PRINTDECV(F(" nr"),msg.a.nr);
      }
      break;
    default:
      break;
  }
  /*short buf[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  while (1 < Wire.available()) { // loop through all but the last
    short s = Wire.read(); // receive byte as a character
//    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
  */
  JR_PRINTLNF(" :REC");
}


// ------------------------------------------- DCC ------------------------------
// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  JR_LN;JR_PRINTDECV(F("DCC Addr"),Addr);
  JR_VDF(Direction);
  JR_VDF(OutputPower);
   
  for (int i=0;i<arduino.a.digital;i++) {
    if (digitalPin[i].a.dcc && ((digitalPin[i].a.dcc==Addr) || 
        ((digitalPin[i].a.dcc==Addr+1) && (digitalPin[i].a.dcc_type==sig3 || digitalPin[i].a.dcc_type==switch3 || digitalPin[i].a.dcc_type==xswitch)))
       ) {
          bool parzysty=digitalPin[i].a.dcc & 1;
          JR_VDF(parzysty);
          JR_VDF(i);
           
        // procedura DCC

        switch ( digitalPin[i].a.dcc_type ) {
          case power:
          case light:
          case sig2:
                setDPort (digitalPin[i].a.spi,digitalPin[i].a.port, !Direction);
            break;
          case switch2:
                setDPort (digitalPin[i].a.spi,digitalPin[i].a.port, Direction);
            break;
          case sig3:
            if(digitalPin[i].a.dcc & 1) {
              //niep yellow
                setDPort (digitalPin[i].a.spi,0+digitalPin[i].a.port>>1<<1, LOW);
                setDPort (digitalPin[i].a.spi,1+digitalPin[i].a.port>>1<<1, HIGH);
            } else {
              if (Direction) {
                setDPort (digitalPin[i].a.spi,0+digitalPin[i].a.port>>1<<1, LOW);
                setDPort (digitalPin[i].a.spi,1+digitalPin[i].a.port>>1<<1, LOW);
                // zlelony
              } else {
                setDPort (digitalPin[i].a.spi,0+digitalPin[i].a.port>>1<<1, HIGH);
                setDPort (digitalPin[i].a.spi,1+digitalPin[i].a.port>>1<<1, HIGH);                
              }
            }
            break;
          case switch3:
          case xswitch:
          break;
        };


        
        break;
    }
  }
  
  DEBUG_PRINT(F("notifyDccAccTurnoutOutput: ")) ;
  JR_PRINTDECV(F("Addr"),Addr) ;
  JR_PRINTDECV(F("Direction"),Direction) ;
  JR_PRINTDECV(F("OutputPower"),OutputPower) ;
  JR_LN;
}


