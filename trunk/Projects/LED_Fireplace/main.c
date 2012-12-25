/**********************************************************************************************\
* Filename:    Main.c                                                                          *
* Description: LED Fireplace                                                                   *
* The LED flickering is done using a Galois linear feedback shift register to produce          *
* pseudo random numbers. http://en.wikipedia.org/wiki/Linear_feedback_shift_register           *
* The code is based on the flickering candle from SarahC (sarah@untamed.co.uk)                 *
*                                                                                              *
*                                      MSP430G2230                                             *
*                              /|\  -----------------                                          *
*                               |  |                 |                                         *
*                               ---|DVCC         DVSS|--|                                      *
*                                  |                 |          /|\                            *
*                      |--|<|------|P1.2          TST|-          |                             *
*                                  |                 |           |                             *
*                      |--|<|------|P1.5          RST|----R56k----                             *
*                                  |                 |                                         *
*                      |--|<|------|P1.6         P1.7|------|>|--|                             *
*                                  |                 |                                         *
*                                   -----------------                                          *
*                                                                                              *
* Device:   MSP430G2230                                                                        *
* Version:  1.1.0                                                                              *
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

#include "msp430f2012.h"

#define Version 0x0110
#define VersionLocation 0xFFFF-RESET_VECTOR-3

__root static const int x@VersionLocation = Version;


void delay(volatile unsigned int counter)
   {
     do;
     while(counter--);
   }

unsigned long LEDRun(unsigned int duration,unsigned int brightness)
   {
      do
      {
        P1OUT |= BIT5+BIT6+BIT7;
        delay(brightness);

        P1OUT &= ~BIT5+BIT6+BIT7;
        delay(255-brightness);
      } while(duration--);

   return(brightness);
   }

void LEDRun1(unsigned int duration,unsigned int brightness)
   {
      do
      {
        P1OUT &= ~0x20;                 // P1.5 RED
        P1OUT &= ~0x80;                 // P1.7 RED
        delay(brightness+255);

        P1OUT |= 0x20;                  // P1.5 RED
        P1OUT |= 0x80;                  // P1.7 RED
        delay(255-brightness);
      } while(duration--);
   }
void LEDRun2(unsigned int duration,unsigned int brightness)
   {
     do
     {
        P1OUT &= ~0x04;                 // P1.2 YELLOW
        P1OUT &= ~0x40;                 // P1.6 YELLOW
        delay(brightness+255);
        
        P1OUT |= 0x04;                  // P1.2 YELLOW
        P1OUT |= 0x40;                  // P1.6 YELLOW
        delay(255-brightness);
      } while(duration--);
   }

void main(void)
   {
   unsigned int lfsr1=0xACE1u;
   unsigned int lfsr2=0x1;
   unsigned long ctr=0u;

   WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog timer
   P1DIR |= 0xFF;                       // Set P1.x to output direction

   BCSCTL1 = CALBC1_8MHZ;
   DCOCTL = CALDCO_8MHZ;

   P1OUT &=~ 0xFF;                      // Turn off the LEDs and wait awhile after power-on
   for(; ++ctr < 100000 ;);
   
   ctr=0;
   for(;LEDRun(7,ctr++) < 255;);        // Increase LED brightness fairly rapidly. This gives the effect of a candle lighting.
   
   for(;;)                              // Loop animation forever
      {

      unsigned int bit;

      /* taps: 16 14 13 11; characteristic polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
	  bit  = ((lfsr1 >> 0) ^ (lfsr1 >> 2) ^ (lfsr1 >> 3) ^ (lfsr1 >> 5) ) & 1;
	  lfsr1 =  (lfsr1 >> 1) | (bit << 15);

      LEDRun1((unsigned char)(lfsr1 >> 3) & 255          // Take the 3rd to 10th bit for the duration.
             ,(unsigned char)(lfsr1 >> 7) & 255);        // and 7th to 15th bit for the brightness.

      /* taps: 16 14 13 11; characteristic polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
	  bit  = ((lfsr2 >> 0) ^ (lfsr2 >> 2) ^ (lfsr2 >> 3) ^ (lfsr2 >> 5) ) & 1;
	  lfsr2 =  (lfsr2 >> 1) | (bit << 15);

      LEDRun2((unsigned char)(lfsr2 >> 3) & 255          // Take the 3rd to 10th bit for the duration.
             ,(unsigned char)(lfsr2 >> 7) & 255);        // and 7th to 15th bit for the brightness.
      }
    }