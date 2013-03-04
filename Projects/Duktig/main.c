/**********************************************************************************************\
* Filename:    Main.c                                                                          *
* Description: Duktig - This code requires R1 and R8 to be changed to 560 Ohms                 *
*                                                                                              *
*                                      MSP430F2011                                             *
*                              /|\  -----------------                                          *
*                               |  |                 |                                         *
*                               ---|DVCC         DVSS|--|                                      *
*                                  |                 |          /|\                            *
*                                  |              TST|-          |                             *
*                                  |                 |           |                             *
*                                  |              RST|----R47k----                             *
*                                  |                 |                                         *
*                        LED2------|P1.4         P1.7|------KEY1                               *
*                                  |                 |                                         *
*                        LED1------|P1.5         P1.6|------KEY2                               *
*                                  |                 |                                         *
*                                   -----------------                                          *
*                                                                                              *
* Device:   MSP430F2011                                                                        *
* Version:  1.0.3                                                                              *
* Compiler: IAR Embedded Workbench IDE V.5.51.3                                                *
*                                                                                              *
* COPYRIGHT:                                                                                   *
* Author:   Gerald Gradl                                                                       *
* Date:     03/2013                                                                            *
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
#define KEY2 BIT6
#define KEY1 BIT7

unsigned int time;

#define Version 0x0103
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
        if(time>=20)                 // Actual seconds is roughly time x 3
        {
        time=0;
        _BIS_SR_IRQ(OSCOFF);
        WDTCTL = WDTPW + WDTHOLD;
        P1OUT &= ~(LED1+LED2);
        }

}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
        for(unsigned int i=0;i<2;i++){for(unsigned int j=0;j<0xA000;j++){}}  //Debounce
        if(P1IFG == KEY1)
        {
        P1OUT ^= LED1;
        }
        else
        {
        P1OUT ^= LED2;
        }
        P1IFG = 0;

        if(P1OUT == LED1 | LED2)
        {
        WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL; // watchdog counter mode, ACLK, /32768
        _BIC_SR_IRQ(OSCOFF);
        }
}