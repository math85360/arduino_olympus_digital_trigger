//##main.pde
// Olympus RM-1 Infrared Transmitter for Arduino Diecimila
//
// The RM-1 remote works with a number of Olympus cameras. This program emulates the RM-1
// remote on an Arduino Diecimila. 
//
// The remote only has five buttons:
//
// SHUTTER - activates the camera shutter 
// W - zoom in (camera in record mode) or index display (camera in display mode)
// T - zoom out (camera in record mode) or close-up display (camera in display mode)
// MINUS - select picture (display mode W) or select section (display mode T)
// PLUS - select picture (display mode W) or select section (display mode T)
//
// The RM-1 also can send one of two codes to set the channel used:
//
// CHAN1 - use code set for channel 1
// CHAN2 - use code set for channel 2
//
// User interface:
//
// Macros are defined to implement all of the above commands using single keywords:
//
// RM1_SHUTTER
// RM1_W
// RM1_T
// RM1_PLUS
// RM1_MINUS
// RM1_CHAN1
// RM1_CHAN2
//
// This program as distributed sets the channel in setup() and simply executes the
// RM1_SHUTTER command the first time through loop().
//
// Note that these commands return before the IR code is finished transmitting, 
// so successive commands should be separated by at least some delay, e.g., 100 msec.
//
// Implementation details:
//
// Two timers are used, TIMER2 and TIMER1. TIMER1 is used as the RC5 state machine clock,
// with tick length of irparams.t (e.g., 889 microseconds), the basic RC5 time unit.
// The on/off state of the IR LED carrier (pulse clock) may change on any given tick of
// this clock.
//
// TIMER2 is used to generate a square wave on OC2A (pin 11) at the frequency of the SIRCS
// pulse clock (e.g., 40 kHz). The pulse clock is modulated (turned on and off) by setting
// or clearing the COM2A0 bit of register TCCR2A. Therefore, an IR LED connected between
// pin 11 and ground is the only external circuitry needed to implement the basic IR
// transmitter.
// 
// Using a high power IR LED (Vishay TSAL6100) connected this way (without a current-
// limiting resistor) results in an "on" current of about 80ma. This is within spec of
// that device (100ma max). The Arduino output pins are specced at 40ma max, so the
// current of 80ma exceeds the Arduino spec, but the duty cycle is low (50% max
// while pulsing), so it's probably OK.
//
// Joe Knapp   jmknapp AT gmail DOT com   30APR08

#include "irparams.h"

#define BLINKLED 13
#define IROUT 11    // pin 11 is OC2A output from TIMER2

#define TICKSPERUSEC ((SYSCLOCK/1000000.)/PRESCALE)
#define mark() (irparams.irledstate = 1) 
#define space() (irparams.irledstate = 0)

// xmitter states
#define START       1
#define STOP        2
#define IDLE        3
#define REXMIT      4
#define CODE        5
#define SENDONE     6
#define SENDZERO    7
#define STARTSPACE  8
#define REMARK      9
#define RESPACE     10
#define REBIT       11

// Camera-control macros
#define RM1_SHUTTER (button(SHUTTER,1))
#define RM1_W (button(W,1))
#define RM1_T (button(T,1))
#define RM1_PLUS (button(PLUS,1))
#define RM1_MINUS (button(MINUS,1))
#define RM1_CHAN1 (setchannel(CHAN1))
#define RM1_CHAN2 (setchannel(CHAN2))

// Pulse clock interrupt uses 16-bit TIMER1
#define INIT_TIMER_COUNT1 (65536 - (int)(TICKSPERUSEC*irparams.t))
#define RESET_TIMER1 TCNT1 = INIT_TIMER_COUNT1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// function prototypes
void button(byte buttonid, int n) ;
void brand(byte brandcode) ;
uint8_t timer2top(unsigned int freq) ;
unsigned long buttoncode(byte buttonid) ;
void setchannel(byte chan) ;

// state machine variables
struct rbutton {
  byte id ;
  unsigned long code ;   // button codes can be up to 32 bits 
} ;
struct {
  byte xmitstate ;
  byte returnstate ;
  byte timer ; ;
  byte sendflag ;
  byte retransmit ;
  byte codelen ;
  byte bitcounter ;
  byte nbuttons ;
  byte blinkstate ;
  byte irledstate ;
  byte chan ;
  byte click ;
  unsigned long button ;
  unsigned long code1 ;
  unsigned long mask32 ;
  unsigned int t ;
  unsigned int unit ;
  struct rbutton buttons[MAXBUTTONS] ;
} irparams ;

void setup() {
  Serial.begin(9600) ;
  Serial.println("XMITIR 0.0.0.1a") ;
 
  cbi(TCCR2A,COM2A1) ; // disconnect OC2A for now (COM2A0 = 0)
  cbi(TCCR2A,COM2A0) ;
  
  cbi(TCCR2B,WGM22) ;  // CTC mode for TIMER2
  sbi(TCCR2A,WGM21) ;
  cbi(TCCR2A,WGM20) ;
  
  TCNT2 = 0 ;
  
  cbi(ASSR,AS2) ;  // use system clock for timer 2
  
  OCR2A = 255 ;   // set TOP to 255 for now
  
  cbi(TCCR2B,CS22) ;  // TIMER2 prescale = 1
  cbi(TCCR2B,CS21) ;
  sbi(TCCR2B,CS20) ;
  
  cbi(TCCR2B,FOC2A) ;  // clear forced output compare bits
  cbi(TCCR2B,FOC2B) ;

  pinMode(IROUT, OUTPUT) ;  // set OC2A to OUPUT 
  pinMode(BLINKLED, OUTPUT) ;
  digitalWrite(BLINKLED, HIGH) ;
  
  DDRD |= B01000000 ;  // set pin 6 as output

  // setup pulse clock timer interrupt
  TCCR1A = 0;  // normal mode
 
  //Prescale /8 (16M/8 = 0.5 microseconds per tick)
  // Therefore, the timer interval can range from 0.5 to 128 microseconds
  // depending on the reset value (255 to 0)
  cbi(TCCR1B,CS12) ;
  sbi(TCCR1B,CS11) ;
  cbi(TCCR1B,CS10) ;
          
  //Timer1 Overflow Interrupt Enable
  sbi(TIMSK1,TOIE1) ;
          
  RESET_TIMER1;        
        
  sei();  // enable interrupts        

  // initialize some state machine variables
  irparams.sendflag = 0 ;
  irparams.blinkstate = HIGH ;
  irparams.xmitstate = IDLE ;
  
  brand(OLYMPUSRM1) ;  // set device params
  setchannel(irparams.chan) ;  // send set channel code to camera
}

// main loop
void loop() {
  static byte first = 1 ;
  if (first) {
    digitalWrite(BLINKLED, HIGH) ;  // flash the LED briefly
    delay(100) ;
    digitalWrite(BLINKLED, LOW) ;
    RM1_SHUTTER ;
    delay(2900);
//    first = 0 ;
  }
}

// xmit state machine 
// Extended NEC protocol: START + UNIT + BUTTON + CHAN + ^BUTTON + ^CHAN + STOP
ISR(TIMER1_OVF_vect) {
  RESET_TIMER1;
  
  irparams.click++ ;
  
  switch(irparams.xmitstate) {
    case START:
      if (irparams.timer == HEADERMARK) {   // if first time through
        irparams.code1 = (unsigned long)irparams.chan << BUTTONLENGTH | (unsigned long)irparams.button ;  // CHAN/BUTTON
        irparams.code1 ^= 0xff ;    // ^CHAN/^BUTTON
        irparams.code1 <<= (BUTTONLENGTH + CHANLENGTH) ;
        irparams.code1 |= (unsigned long)irparams.chan << BUTTONLENGTH | (unsigned long)irparams.button ;  // ^CHAN/^BUTTON/CHAN/BUTTON
        irparams.code1 <<= UNITLENGTH ;
        irparams.code1 |= irparams.unit ;   // ^CHAN/^BUTTON/CHAN/BUTTON/UNIT
//printf("0x%x\n",irparams.code1) ;
        
        irparams.mask32 = 0x1 ;    // point to first bit (LSB)
        irparams.codelen = CODELENGTH ;
        mark() ;   // send START mark
      }
      
      irparams.timer-- ;
      if (irparams.timer == 0) { 
        irparams.xmitstate = STARTSPACE ;
        irparams.timer = HEADERSPACE ;
        space() ;   // send START space
      }
      break ;
    case STARTSPACE:
       irparams.timer-- ;
       if (irparams.timer == 0) {
        irparams.xmitstate = CODE ;  // done sending START space, go to CODE
        irparams.bitcounter = 0 ;
       }
       break ;
    case CODE:
      if (irparams.bitcounter == irparams.codelen) {  // if done xmitting code bits
        mark() ;    // stop bit
        irparams.xmitstate = STOP ;
      }
      else {  // more bits to send
        irparams.returnstate = CODE ;
        if (irparams.code1 & irparams.mask32) {  // send ONE bit
          irparams.timer = ONELENGTH ;
          irparams.xmitstate = SENDONE ;
          mark() ;
        }
        else {   // send ZERO bit
          irparams.timer = ZEROLENGTH ;
          irparams.xmitstate = SENDZERO ;
          mark() ;
        }
        irparams.bitcounter++ ;
        irparams.mask32 <<= 1 ;  // point to next bit
      } 
      break ;
    case STOP:
      space() ;   // end STOP bit
      if (irparams.retransmit) {  // check for retransmit
        irparams.xmitstate = REXMIT ;
        irparams.sendflag = 0 ;
      }
      else {  // no more retransmissions, go back to IDLE
        irparams.xmitstate = IDLE ;
        irparams.sendflag = 0 ;
        space() ;
      }
      break ;
    case IDLE:
      // check sendflag
      if (irparams.sendflag) {
        irparams.xmitstate = START ;
        irparams.timer = HEADERMARK ;
        irparams.click = 0 ;
      }
      break ;
    case SENDONE:
      if (irparams.timer == ONELENGTH)   // if first time through, terminate MARK
        space() ;
      irparams.timer-- ;
      if (irparams.timer == 1)     // back to state CODE
        irparams.xmitstate = irparams.returnstate ;
     break ;
   case SENDZERO:
     if (irparams.timer == ZEROLENGTH)  // if first time through, terminate MARK
       space() ;
     irparams.timer-- ;
      if (irparams.timer == 1)
        irparams.xmitstate = irparams.returnstate ;
     break ;
   case REXMIT:
     if (irparams.click >= RETRANSGAP) {  // wait for interrupt click counter to equal RETRANSGAP
       irparams.sendflag = 1 ;
       irparams.xmitstate = REMARK ;
       irparams.timer = RETRANSMARK ;
       irparams.retransmit-- ;
       mark() ;   // begin retansmission start pulse
       irparams.click = 0 ;
     }
     break ;
   case REMARK:
     irparams.timer-- ;
     if (irparams.timer == 0) {
       irparams.xmitstate = RESPACE ;
       irparams.timer = RETRANSSPACE ;
       space() ;  // begin retransmission space
     }
     break ;
   case RESPACE:
     irparams.timer-- ;
     if (irparams.timer == 0) {
       mark() ;   // start retransmission bit
       irparams.xmitstate = REBIT ;
     }
     break ;
   case REBIT:
     space() ;  // end retransmission bit
     irparams.xmitstate = STOP ;  
     break ;
  } 
  
  // update LEDs
  if (irparams.irledstate) {
    digitalWrite(BLINKLED, HIGH) ;
    sbi(TCCR2A,COM2A0) ;   // connect pulse clock
  }
  else {
    digitalWrite(BLINKLED, LOW) ;
    cbi(TCCR2A,COM2A0) ;   // disconnect pulse clock
  }
}
// end SIRCS state machine

// send code for given buttonid, repeat n times
void button(byte buttonid, int n)
{
  irparams.button = buttoncode(buttonid) ;  // get code
  irparams.retransmit = n - 1;
  irparams.sendflag = 1;     // flag for the ISR to send irparams.button
}

// return ir code for given buttonid
unsigned long buttoncode(byte buttonid)
{
  int i ;
  byte found ;
  
  i = 0 ;
  found = 0 ;
  while (!found && (i < irparams.nbuttons)) {
    if (buttonid == irparams.buttons[i].id) {
      found = 1 ;
    }
    else
      i++ ;
  }
  if (found)
    return(irparams.buttons[i].code) ;
  else
    return(0) ;   // ERROR
}

// set parameters for given brand
void brand(byte brandcode)
{
  int i ;
  switch(brandcode) {
    case OLYMPUSRM1:
      OCR2A = timer2top(OLYMPUSRM1_PULSECLOCK) ;  // sets TOP value for TIMER2
      irparams.unit = OLYMPUSRM1_UNIT ;
      irparams.t = OLYMPUSRM1_T ;
      irparams.chan = OLYMPUSRM1_CHANNEL ;
      
      // buttons
      irparams.nbuttons = 0 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B01 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C01 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B02 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C02 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B03 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C03 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B04 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C04 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B05 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C05 ;
      irparams.buttons[irparams.nbuttons].id = OLYMPUSRM1_B06 ;
      irparams.buttons[irparams.nbuttons++].code = OLYMPUSRM1_C06 ;
      break ;
    default:
      break ;
  }
}

// return TIMER2 TOP value per given desired frequency (Hz)
uint8_t timer2top(unsigned int freq)
{
  return((byte)((unsigned long)SYSCLOCK/2/freq) - 1) ;
}

void setchannel(byte chan)
{
  irparams.chan = CHAN1 ;  // channel-set codes are always on channel 1
  switch(chan) {
    case CHAN1:
      button(SETCHAN1,1) ;
      break ;
    case CHAN2:
      button(SETCHAN2,1) ;
      break ;
  }
  delay(200) ;   // wait for code to be sent
  irparams.chan = chan ;  // set new channel in irparams
}
