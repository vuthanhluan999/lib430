/**********************************************************************************************\
* Filename:     Main.c                                                                         *
* Description:  PC Starter for HTESYNCSRV                                                      *
* Push S2 to change state and S1 to set. S2 toggles thru the different modes which are         *
* indicated by the LEDs.                                                                       *
* LED: 7 6 5 4 3 2 1 0                                                                         *
*      o o o o o o o o                                                                         *
*                0 0 0 ; 0 = Off                                                               *
*                0 0 1 ; 1 = Alarm Active                                                      *
*                0 1 0 ; 2 = Set Day of Week Mo=1, Tu=2, We=3, Th=4, Fr=5, Sa=6, Su=7          *
*                0 1 1 ; 3 = Set Hour 24h Format                                               *
*                1 0 0 ; 4 = Set Minute Tenth                                                  *
*                1 0 1 ; 5 = Set Minute Ones                                                   *
*                1 1 0 ; 6 = Set Alarm Day                                                     *
*                1 1 1 ; 7 = Set Alarm Hours 24h Format                                        *
*                                                                                              *
* P1.7    6 5     4 3 2 1 0                               R V                                  *
*    |    | |     u | | | |                               S C                                  *
*    | S2 \ \ S1  n                                       T C  RELAY                           *
*    |    | |     u    P1.0 to P1.3                       o o o-o-o-o                          *
*    |    _ _     s    Charlieplexed LEDs                 o o o-o-o-o                          *
*  RELAY          e                                       T G  RELAY                           *
*    |            d                                       S N                                  *
*    |                                                    T D                                  *
*    |                                                                                         *
*    _                                                                                         *
*                                                                                              *
*                                                                                              *
* 32.768kHz Quartz Crystal on XIN/XOUT                                                         *
*                                                                                              *
* Device:   MSP430F2012                                                                        *
* Version:  1.0.3                                                                              *
* Compiler: IAR Embedded Workbench IDE V.5.51 (TI: V6.40)                                      *
*                                                                                              *
* COPYRIGHT:                                                                                   *
* Author:   Gerald Gradl                                                                       *
* Date:     01/2013                                                                            *
*                                                                                              *
* This program is free software: you can redistribute it and/or modify it under the terms of   *
* the GNU General Public License as published by the Free Software Foundation, either          *
* version 3 of the License, or (at your option) any later version.                             *
*                                                                                              *
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;    *
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    *
* See the GNU General Public License for more details.                                         *
*                                                                                              *
* You should have received a copy of the GNU General Public License along with this program.   *
* If not, see <http://www.gnu.org/licenses/>.                                                  *
\**********************************************************************************************/

#include  "msp430x20x2.h"

//#define ZEIT 1000 		// Timer delay timeout count, 1000 msec

// Multiplexed LEDs
#define LED_MASK 0x0F // 7 LEDs, 4 IO lines, n = 4, number of LEDS = n x (n-1)
#define LEDs 8
// 1 - 0x01
// 2 - 0x02
// 3 - 0x04
// 4 - 0x08
// Port Direction
// Example: LED A: Output from P1.0 to P1.1, P1DIR = BIT0 = H and BIT1 = L, both output direction
const char led_dir[LEDs] = {
(0x08 | 0x04), //H
(0x01 | 0x04), //E
(0x04 | 0x01), //F
(0x02 | 0x01), //B
(0x01 | 0x02), //A
(0x04 | 0x02), //D
(0x02 | 0x04), //C
(0x04 | 0x08), //G
/*
(0x02 | 0x08), //I
(0x08 | 0x02), //J
(0x01 | 0x08), //K
(0x08 | 0x01)  //L
*/
};

// Port Output
// Example: LED A, Output P1.0 = H, P1.1 = L
const char led_out[LEDs] = {
 0x08, //H
 0x01, //E
 0x04, //F
 0x02, //B
 0x01, //A
 0x04, //D
 0x02, //C
 0x04, //G
 /*
 0x02, //I
 0x08, //J
 0x01, //K
 0x08, //L
*/
};

unsigned long i;
unsigned int g;
unsigned char LED;
unsigned int day;
unsigned int hour;
unsigned int minuteT;
unsigned int minuteO;
unsigned int second;
unsigned char state;             // Off, Active, D, H, MT, MO, SAD, SAH
unsigned char view;
unsigned char alarm;
unsigned int alarmday;
unsigned int alarmhour;
unsigned int viewpointer;
enum states {OFF, ACTIVE, SET_DAY, SET_HOUR, SET_MINUTET, SET_MINUTEO, SET_ALARM_DAY, SET_ALARM_HOUR};
enum alarms {ALARM_OFF, ALARM_ON};
void configureClocks();
void delay(unsigned int k);
void delays(unsigned long k);
void sleep(unsigned int time);
void update_view(char state);
void LEDOn();
void LEDOff();

#define ON 1
#define OFF 0

#define Version 0x0103
#define VersionLocation 0xFFFF-RESET_VECTOR-3

__root static const int x@VersionLocation = Version;

void main(void)
{
/******************************************************************************/
// Init
/******************************************************************************/
  WDTCTL = WDTPW + WDTHOLD;     // watchdog timer off
  configureClocks();
  P1OUT = BIT5 + BIT6;          // mov BIT5&6 P1OUT = pullup
  P1REN = BIT5 + BIT6;          // mov P1REN = pullup/down enable
  P1DIR = BIT4 + BIT7;          // mov P1DIR = outup for relay and unused pin as output
  P1IFG = 0;
  P1IES = BIT5 + BIT6;          // set high to low transition for P1.5&6 interrupt
  P1IE  = BIT5 + BIT6;          // enable P1.5&6 interrupt

  //P1DIR = BIT0 + BIT6;        // mov BIT1 and BIT6 to P1DIR for output
  //P1OUT &= ~BIT0 + ~BIT6;
  //P2SEL &= ~BIT6 + ~BIT7;     // bic SEL of P2.6
  //P2OUT = BIT6 + BIT7;        // mov BIT6 P2OUT = pullup
  //P2REN = BIT6 + BIT7;        // mov BIT6 P2REN = pullup/down enable
  //P2IFG = 0;                  // Clear Interrupt Flags of P2.x
  //P2IES = BIT6 + BIT7;        // set high to low transition for P2.6 interrupt
  //P2IE  = BIT6 + BIT7;        // enable P2.6 interrupt

  WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL; // watchdog counter mode, ACLK, /32768
  IFG1 &= ~WDTIFG;              // Clear WDT interrupt flag
  IE1 |= WDTIE;                 // WDT interrupt enable

    day = 1;
    hour = 0;
    minuteO = 0;
    minuteT = 0;
    second = 0;
    alarmday = 3;
    alarmhour = 1;
    viewpointer = 1;
    LED = 0;
    state = ACTIVE;
    alarm = ON;

/******************************************************************************/
// Mainloop
/******************************************************************************/
  while(1) // endless loop
  {
    //__bis_SR_register(LPM3_bits+GIE);
    // P1OUT ^= BIT0;  // xor BIT0

     __enable_interrupt();

     if (alarm == ALARM_ON)
     {
      if (day == alarmday && hour == alarmhour && minuteT == 0 && minuteO == 0 && second == 0)
      {
      P1OUT |= BIT7;
      delays(250000);
      P1OUT &= ~BIT7;
      }
     }
     update_view(state);

     // LED On or Off acc. it's state in "view". The viewpointer is going from BIT0 to BIT5
     if(view & viewpointer)     // Is the Bit where viewpointer is pointing at set?
     {                          // Yes? Then switch on corresponding LED.
      LEDOn();
     //LED = (LED+1)%LEDs;
     }
     else if (!(view & viewpointer))
     {                          // No? Then switch off corresponding LED.
      LEDOff();
      //LED = (LED+1)%LEDs;
     }
     delays(50);

     // All LEDs Off
     P1DIR &= (~LED_MASK);
     P1OUT &= (~LED_MASK);
     delays(10);

     viewpointer<<=1;
     if (viewpointer==256) viewpointer=1;
     //LED = LED+1;
     if (++LED==LEDs) LED = 0;

  } // while()
} // main()

/******************************************************************************/
// Subroutines
/******************************************************************************/
void configureClocks()
{
// Set system DCO to 8MHz
BCSCTL1 = CALBC1_8MHZ;
DCOCTL = CALDCO_8MHZ;
// LFXT1 = VLO
// BCSCTL3 |= LFXT1S_2;                      // Select VLO as source for ACLK
BCSCTL3 |= LFXT1S_0;                      // 32kHz Crystal as source for ACLK
//BCSCTL3 |= XCAP_0;
//P1DIR |= BIT0;                            // Debug: ACLK @ P1.0
//P1SEL |= BIT0;                            // Debug: ACLK @ P1.0
IFG1 &= ~OFIFG;                           // Clear OSCFault flag
}
/******************************************************************************/
void delay(unsigned int k)
{
  unsigned int i,j;
  for(i=0;i<k;i++)
  {
    j=0;
    do
    j++;
    while(j<65000);
  }
}
/******************************************************************************/
void delays(unsigned long k)
{
  unsigned long i;
  unsigned long Freq;
  Freq = 8000000;
  Freq = Freq / 1000000;
  k = k*Freq;
  k = (((k-14)/3)+1);

  for(i=0;i<k;i++);
}
/******************************************************************************/
void sleep(unsigned int time)
{
    time = time * 10;
    TA0CCR0 = time;
    TA0CTL = TASSEL_1+MC_1+TACLR;
    TA0CCTL0 &= ~CCIFG;
    TA0CCTL0 |= CCIE;

    __bis_SR_register(LPM3_bits+GIE);
    __no_operation();
}

/******************************************************************************/
void clock(void)
{
  if (second == 60)
  {
    minuteO++;
    second = 0;
  }
  else if (minuteO == 10)
  {
    minuteT++;
    minuteO = 0;
    second = 0;
  }
  else if (minuteT == 6)
  {
    hour++;
    minuteT = 0;
    minuteO = 0;
    second = 0;
  }
  else if (hour == 24)
  {
    day++;
    hour = 0;
    minuteT = 0;
    minuteO = 0;
    second = 0;
  }
  else if (day > 7)
  {
    day = 1;
  }
  else
  {}
}

void update_view(char state)
{
  switch(state)
  {
    case OFF:
      view = 0;
      alarm = 0;
      break;
    case ACTIVE:
      view = state;
      alarm = 1;
      break;
    case SET_DAY:
      view = state+(day<<3);
      break;
    case SET_HOUR:
      view = state+(hour<<3);
      break;
    case SET_MINUTET:
      view = state+(minuteT<<3);
      break;
    case SET_MINUTEO:
      view = state+(minuteO<<3);
      break;
    case SET_ALARM_DAY:
      view = state+(alarmday<<3);
      break;
    case SET_ALARM_HOUR:
      view = state+(alarmhour<<3);
      break;
  }
}

void LEDOn()
{
      P1DIR |= led_dir[LED];
      P1OUT |= led_out[LED];
}

void LEDOff()
{

     P1DIR &= (~LED_MASK);
     P1OUT &= (~LED_MASK);
}

/******************************************************************************/
// Interrupt Routines
/******************************************************************************/

/******************************************************************************/
// Timer0_A0 Interrupt Service Routine: Disables the timer and exists LPM3
/******************************************************************************/
#pragma vector=TIMERA0_VECTOR
__interrupt void ISR_Timer0_A0(void)
{
  TA0CTL &= ~(MC_1);
  TA0CCTL0 &= ~(CCIE);

  __bic_SR_register_on_exit(LPM3_bits+GIE);
}
#pragma vector=WDT_VECTOR
__interrupt void ISR_WDT(void)
{
  second++;
  clock();
  __bic_SR_register_on_exit(LPM3_bits+GIE);
}
#pragma vector=PORT1_VECTOR
__interrupt void ISR_Port1(void)
{
  unsigned int PIF;
     P1DIR &= (~LED_MASK);
     P1OUT &= (~LED_MASK);
  delays(200000);     // 200ms debounce
  PIF = P1IFG;
  P1IFG &= ~BIT5 & ~BIT6;
    if (PIF & BIT5)
    {
      if (state == OFF)
      {
        alarm = 0;
      }
      else if (state == ACTIVE)
      {
        alarm = 1;
      }
      else if (state == SET_DAY)
      {
      alarm = 0;
      day++;
        if (day > 7)
        {
        day = 1;
        }
      }
      else if (state == SET_HOUR)
      {
      alarm = 0;
      hour++;
        if (hour >= 24)
        {
        hour = 0;
        }
      }
      else if (state == SET_MINUTET)
      {
      alarm = 0;
      minuteT++;
        if (minuteT >= 6)
        {
        minuteT = 0;
        }
      // second = 0;
      }
      else if (state == SET_MINUTEO)
      {
      alarm = 0;
      minuteO++;
        if (minuteO >= 10)
        {
        minuteO = 0;
        }
      second = 0;
      }
      else if (state == SET_ALARM_DAY)
      {
      alarm = 0;
      alarmday++;
        if (alarmday > 7)
        {
        alarmday = 1;
        }
      }
      else if (state == SET_ALARM_HOUR)
      {
      alarm = 0;
      alarmhour++;
        if (alarmhour >= 24)
        {
        alarmhour = 0;
        }
      }
      else {}
    }
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if (PIF & BIT6)
    {
      state = (state+1)%8;     // auto increment state and check if state is >7 then automatically reset to 0
      update_view(state);
    }
    PIF = 0;
}
