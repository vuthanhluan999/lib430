/**********************************************************************************************\
* Filename:    Main.c                                                                          *
* Description: LED Fireplace                                                                   *
* The LED flickering is done using a Fibonacci linear feedback shift register to produce       *
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

#include "msp430f2012.h"

unsigned long LEDRun(unsigned long duration,unsigned char brightness)
   {
      do
      {
        P1OUT |= 0xE4;
        delay(brightness);

        P1OUT &= ~0xE4;
        delay(255-brightness);
      } while(duration--);

   return(brightness);
   }

unsigned long LEDRun1(unsigned long duration,unsigned char brightness)
   {
      do
      {
        P1OUT &= ~0x20;                 // P1.5 RED
        P1OUT &= ~0x80;                 // P1.7 RED
        delay(brightness+127);

        P1OUT |= 0x20;                  // P1.5 RED
        P1OUT |= 0x80;                  // P1.7 RED
        delay(255-brightness);
      } while(duration--);

   return(brightness);
   }

unsigned long LEDRun2(unsigned long duration,unsigned char brightness)
   {
     do
     {
        P1OUT |= 0x04;                  // P1.2 YELLOW
        P1OUT |= 0x40;                  // P1.6 YELLOW
        delay(brightness);

        P1OUT &= ~0x04;                 // P1.2 YELLOW
        P1OUT &= ~0x40;                 // P1.6 YELLOW
        delay(511-brightness);
      } while(duration--);

   return(brightness);
   }

void delay(volatile unsigned int counter)
   {
     do;
     while(counter--);
   }

void main(void)
   {

   unsigned long lfsr=0;                // Seed value for pseudo random number
   unsigned char ctr=0;

   WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog timer
   P1DIR |= 0xFF;                       // Set P1.x to output direction

   BCSCTL1 = CALBC1_1MHZ;
   DCOCTL = CALDCO_1MHZ;

   P1OUT &=~ 0xFF;                      // Turn off the LEDs and wait awhile after power-on
   for(; ++lfsr < 100000 ;);

   for(; LEDRun(1,ctr++) < 255 ;);      // Increase LED brightness fairly rapidly. This gives the effect of a candle lighting.

                                        // Loop animation forever
   for(;;)
      {
      lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xd0000001u); // Taps 32 31 29 1
      LEDRun1(   (unsigned char)(lfsr >> 20) & 31        // Take the 20th to 24th bit for the duration. Range: 0-31
            ,(unsigned char)(lfsr >> 24) & 255);         // and 24th to 29th bit for the brightness.    Range: 0-63

      lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xd0000001u); // Taps 32 31 29 1
      LEDRun2(   (unsigned char)(lfsr >> 20) & 31        // Take the 20th to 24th bit for the duration. Range: 0-31
            ,(unsigned char)(lfsr >> 24) & 255);         // and 24th to 29th bit for the brightness.    Range: 0-63
      }
    }