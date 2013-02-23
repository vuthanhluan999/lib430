/**********************************************************************************************\
* Filename:    Main.c                                                                          *
* Description: Duktig                                                                          *
*                                                                                              *
*                                      MSP430F2012                                             *
*                              /|\  -----------------                                          *
*                               |  |                 |                                         *
*                               ---|DVCC         DVSS|--|                                      *
*                                  |                 |          /|\                            *
*                                  |              TST|-          |                             *
*                                  |                 |           |                             *
*                                  |              RST|----R47k----                             *
*                                  |                 |                                         *
*                        LED2------|P1.4         P1.7|------KEY2                               *
*                                  |                 |                                         *
*                        LED1------|P1.5         P1.6|------KEY1                               *
*                                  |                 |                                         *
*                                   -----------------                                          *
*                                                                                              *
* Device:   MSP430F2012                                                                        *
* Version:  1.0.2                                                                              *
* Compiler: IAR Embedded Workbench IDE V.5.51.3                                                *
*                                                                                              *
* COPYRIGHT:                                                                                   *
* Author:   Gerald Gradl                                                                       *
* Date:     02/2013                                                                            *
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

#include <msp430f2012.h>

#define LED1 BIT4
#define LED2 BIT5
#define KEY1 BIT7
#define KEY2 BIT6
#define ON   1
#define OFF  0

unsigned char LEDstate1;
unsigned char LEDstate2;
unsigned int time;

#define Version 0x0102
#define VersionLocation 0x0FFFF-RESET_VECTOR-3
__root static const int x@VersionLocation = Version;

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;

        P1OUT = 0;
        P1DIR = BIT0+BIT1+BIT2+BIT3+LED1+LED2;
        P1IES = KEY1+KEY2;
        P1IFG = 0;
        P1IE = KEY1+KEY2;
        P2SEL = 0;
        P2REN = BIT6+BIT7;

        if(CALBC1_1MHZ != 0xFF)
        {
        BCSCTL1 = 0;
        BCSCTL1 = CALBC1_1MHZ;
        DCOCTL = CALDCO_1MHZ;
        }
        BCSCTL3 |= LFXT1S_2; //VLO 2 ACLK

        IFG1 &= ~WDTIFG;
        IE1 |= WDTIE;
        __enable_interrupt();

        while(1)
        {
        __low_power_mode_4();
        __no_operation();
            while(1)
            {
              if(LEDstate1 == ON)
              {
                P1OUT |= LED1;
              }
              if(LEDstate2 == ON)
              {
                P1OUT |= LED2;
              }
              for(unsigned int j=0;j<0x250;j++){}
              if(LEDstate1 == ON)
              {
                P1OUT &= ~LED1;
              }
              if(LEDstate2 == ON)
              {
                P1OUT &= ~LED2;
              }
              for(unsigned int j=0;j<0x500;j++){}
              if((time>=100) || ((LEDstate1 == OFF) && (LEDstate2 == OFF)))                 // Actual seconds is roughly time x 3
              {
              time=0;
              WDTCTL = WDTPW + WDTHOLD;
              LEDstate1 = OFF;
              LEDstate2 = OFF;
              P1OUT &= ~(LED1+LED2);
              break;
              }
            }
        }
}

/******************************************************************************/
// Interrupt Routines
/******************************************************************************/

#pragma vector=WDT_VECTOR
__interrupt void ISR_WDT(void)
{
        IFG1 &= ~WDTIFG;              // Clear WDT interrupt flag
        time++;
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
        
        P1OUT &= ~(LED1 + LED2);
        for(unsigned int i=0;i<2;i++){for(unsigned int j=0;j<0xFFFF;j++){}}  //Debounce
        if(P1IFG & KEY1)
        {
        LEDstate1 ^= 1;
        }
        if(P1IFG & KEY2)
        {
        LEDstate2 ^= 1;
        }
        P1IFG = 0;
        time=0;
        if(1 == LEDstate1 | LEDstate2)
        {
        WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL; // watchdog counter mode, ACLK, /32768
        _BIC_SR_IRQ(OSCOFF+CPUOFF+SCG1+SCG0);
        }
}