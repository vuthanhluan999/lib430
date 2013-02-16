/**********************************************************************************************\
* Filename:    Main.c                                                                          *
* Description: Boggle Timer                                                                    *
* 3 Minute Timer for the game Boggle. The green LED blinks during the 3 minutes. The red LED   *
* blinks and the buzzer beeps three times after the three minutes are over.                    *
* The timer is started by a small ball that acts as a switch and grounds P1.7.                 *
* The second tick is derived from the calibrated 1MHz clock. The accuracy is good enough for   *
* this purpose.                                                                                *
*                                                                                              *
*                                      MSP430G2230                                             *
*                              /|\  -----------------                                          *
*                               |  |                 |                                         *
*                               ---|DVCC         DVSS|--|                                      *
*                                  |                 |          /|\                            *
*                      |--BUZ------|P1.2          TST|-          |                             *
*                                  |                 |           |                             *
*                      |--|<|------|P1.5          RST|----R56k----                             *
*                  |---|           |                 |                                         *
*                      |--|<|------|P1.6         P1.7|------  ---|                             *
*                                  |                 |     |   |                               *
*                                   -----------------      | O |                               *
*                                                                                              *
* Device:   MSP430G2230                                                                        *
* Version:  1.0.1                                                                              *
* Compiler: IAR Embedded Workbench IDE V.5.40 (TI: V6.10)                                      *
*                                                                                              *
* COPYRIGHT:                                                                                   *
* Author:   Gerald Gradl                                                                       *
* Date:     02/2012                                                                            *
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

#include "msp430g2230.h"

unsigned int second;
unsigned int counter;
unsigned int repeat;
void ToggleLED();
void delay_us();
void init_PWM_TimerA();

#define LEDgreen 0x40
#define LEDred 0x20
#define Speaker 0x04
#define TIME 180
#define Version 0x0101
#define VersionLocation 0xFFFF-RESET_VECTOR-3

__root static const int x@VersionLocation = Version;


/******************************************************************************/
// Mainloop
/******************************************************************************/
void main(void)
{
/******************************************************************************/
// Init
/******************************************************************************/
   WDTCTL = WDTPW + WDTHOLD;                                                    // Stop watchdog timer
   P1OUT &= ~0x7B;                                                              // Set P1.x low
   P1DIR |= 0x7B;                                                               // Set P1.0-6 to output direction
   P1OUT |= 0x80;                                                               // Set P1.7 pullup
   P1REN = BIT7;                                                                // Set P1.7 = pullup/down enable
   P1IFG = 0;
   P1IES = BIT7;                                                                // Set high to low transition for P1.7 interrupt
   P1IE  = BIT7;                                                                // Enable P1.7 interrupt
   P2OUT &= ~0xC0;                                                              // Set P2.6/7 low
   P2DIR |= 0xC0;                                                               // Set P2.6/7 to output direction
   P2SEL &= ~0xC0;                                                              // Set P2.6/7 to pinmode

   if (BCSCTL1!=0xFF)
   {
   BCSCTL1 = CALBC1_1MHZ;
   DCOCTL = CALDCO_1MHZ;
   }

   IFG1 &= ~WDTIFG;                                                             // Clear WDT interrupt flag
   IE1 |= WDTIE;                                                                // WDT interrupt enable
   init_PWM_TimerA();

   __enable_interrupt();

while(1)
  {
   __low_power_mode_4();

   WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTIS0 + WDTIS1;                      // Start Watchdog counter mode, MCLK, /64
   P1OUT |= LEDgreen;
   second=0;
   counter=0;
   for(;second<TIME;);

   WDTCTL = WDTPW + WDTHOLD;                                                    // Stop watchdog timer
   P1OUT &= ~LEDgreen;
   ToggleLED(LEDred,3);

  }
}


/******************************************************************************/
// Subroutines
/******************************************************************************/
void ToggleLED(unsigned int LED,unsigned int repeat)
{
  for(int i=0;i<repeat;i++)
  {
      P1DIR |= Speaker;
      P1SEL |= Speaker;
      P1OUT |= LED;
      TACTL |= MC0;                                                             // Start Timer_A in Up Mode
      delay_us(400000);
      P1OUT &= ~LED;
      P1SEL &= ~Speaker;
      P1DIR &= ~Speaker;
      TACTL &= ~MC0;                                                            // Stop Timer_A
      delay_us(400000);
  }
}

/******************************************************************************/
void delay_us(unsigned long k)
{
  unsigned long i;
  unsigned long Freq;
  Freq = 1000000;
  Freq = Freq / 1000000;
  k = k*Freq;
  k = (((k-14)/3)+1);

  for(i=0;i<k;i++);
}

/******************************************************************************/
void init_PWM_TimerA(void)
{
  TACTL = TASSEL1 + TACLR;                                                      // SMCLK, Clear Tar
  CCR0 = 512-1;                                                                 // PWM Period
  CCR1 = 256;                                                                   // CCR1 PWM duty cycle
  CCTL1 = OUTMOD_7;                                                             // CCR1 reset/set
}

/******************************************************************************/


/******************************************************************************/
// Interrupt Routines
/******************************************************************************/

#pragma vector=WDT_VECTOR
__interrupt void ISR_WDT(void)
{
  counter++;
  if(counter==15625)                                                            // 1MHz / 64 = 15625
  {
    second++;
    counter=0;
    P1OUT ^= LEDgreen;
  };

}

#pragma vector=PORT1_VECTOR
__interrupt void ISR_Port1(void)
{
  //delay_us(100000);                                                             // 100ms debounce
  P1IFG &= ~BIT7;
  second=0;
  counter=0;
  __low_power_mode_off_on_exit();
}