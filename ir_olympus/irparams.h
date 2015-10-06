// brands
#define OLYMPUSRM1 1

#define MAXBUTTONS 8   // note: each button takes up sizeof(rbutton) bytes RAM 

#define CHAN1 0x0
#define CHAN2 0x1

// Basic timing paramters (probably don't need to change)
#define SYSCLOCK 16000000  // main system clock (Hz)
#define PRESCALE 8       // TIMER1 prescale value (state machine clock)
#define OLYMPUSRM1_PULSECLOCK 38000  // Hz
#define OLYMPUSRM1_T           560   // microseconds
#define OLYMPUSRM1_CHANNEL     CHAN1
// end timing parameters

// extended NEC protocol
// unit is 16 bits in length
// button command is 16 bits, 8 data and 8 complemented dsta
// The 8 data bits are split up into 4 command bits and 4 channel bits

#define HEADERMARK  16  // T units
#define HEADERSPACE 8
#define HEADERLENGTH 24
#define ONELENGTH 4
#define ZEROLENGTH 2
#define RETRANSGAP 191
#define RETRANSMARK 16
#define RETRANSSPACE 4
#define CODELENGTH 32    // bits

#define BUTTONLENGTH 7   // bits
#define CHANLENGTH 1
#define UNITLENGTH 16

// unit code:
#define OLYMPUSRM1_UNIT 0x3b86 // OLYMPUS RM-1 remote

// buttons
#define OLYMPUSRM1_B01 SHUTTER     // button id (defined below)
#define OLYMPUSRM1_C01 0x1         // RM-1 button code
#define OLYMPUSRM1_B02 W
#define OLYMPUSRM1_C02 0x2
#define OLYMPUSRM1_B03 T
#define OLYMPUSRM1_C03 0x3
#define OLYMPUSRM1_B04 MINUS
#define OLYMPUSRM1_C04 0x4
#define OLYMPUSRM1_B05 PLUS
#define OLYMPUSRM1_C05 0x5
#define OLYMPUSRM1_B06 SETCHAN1
#define OLYMPUSRM1_C06 0x7e
#define OLYMPUSRM1_B07 SETCHAN2
#define OLYMPUSRM1_C07 0x7f

// button labels
#define SHUTTER    0
#define W          1
#define T          2
#define MINUS      3
#define PLUS       4
#define SETCHAN1   5
#define SETCHAN2   6
