/**********************************************************************************************
* Additional register and bit definitions for the Texas Instruments MSP430 Microcontroller.
* This file supports assembler and C development within the IAR-Embedded-Workbench environment.
* Enclose all #define statements with parentheses
*
* Gerald Gradl, Version 0.1
**********************************************************************************************/

//#define F2012
#define F2132

//**********************************************************************************************
Global_Include_Files                            ; Include files
//**********************************************************************************************
#ifdef F2132
//#include        "msp430x21x2.h"
#include        "msp430F6638.h"
#endif
#ifdef F2012
#include        "msp430x20x2.h"
#endif

//**********************************************************************************************

/**********************************************************************************************
* REGISTER DEFINITIONS
**********************************************************************************************/
#define         Counter R13
#define         RTEMP1  R14				/* Temp register 1 */
#define         RTEMP2  R15				/* Temp register 2 */
#define         FIRE    BIT6
#define         RELAY   BIT5
#define         LCDBUF  R12
#define         LCDSVL  R12
#define         LCDADD  R11

/**********************************************************************************************
* Memory Definitions
**********************************************************************************************/
#ifdef F2012
#define		BOR     00200h
#define		EOR     0027Fh
#define		BOI     01000h
#define		EOI     010FFh
#define		BOF     0F800h
#define		EOF     0FFC0h
#endif
#ifdef F2132
#define		BOR     02600h
#define		EOR     063FFh
#define		BOI     01800h
#define		EOI     019FFh
#define		BOF     08000h
#define		EOF     0FFC0h
#endif
#define         REV     0FFFFh-RESET_VECTOR-3

/**********************************************************************************************
* Project Specific Definitions
**********************************************************************************************/
#define         RED     BIT0+BIT1
#define         GREEN   BIT2+BIT3
#define         BLUE    BIT4+BIT5

#define         VCC1    BIT4	// P3.4
#define         TSIC1   BIT1	// P1.1
//#define         VSS1    BIT0

#define         VCC2    BIT3	// P3.3
#define         TSIC2   BIT2	// P1.2
//#define         VSS2    BIT0

#define         VCC3    BIT2	// P3.2
#define         TSIC3   BIT3	// P1.3
//#define         VSS3    BIT0

#define         VCC4    BIT1	// P3.1
#define         TSIC4   BIT4	// P1.4
//#define         VSS5    BIT0

#define         VCC5    BIT0	// P3.0
#define         TSIC5   BIT5	// P1.5
//#define         VSS5    BIT0

#define         VSS     BIT0

/**********************************************************************************************
* EA DOG LCD Definitions
**********************************************************************************************/
//#define LCD

#ifdef LCD

#define   VCCLCD    BIT2
#define   VSSLCD    BIT0
#define   SIMO      BIT1
#define   SE        BIT2
#define   SCLK      BIT3
#define   RS        BIT4
#define   RST       BIT5

#define   C0              (0x0001)
#define   C1              (0x0002)
#define   C2              (0x0004)
#define   C3              (0x0008)
#define   C4              (0x0001)
#define   C5              (0x0002)
#define   LCDPWRCNTL      (0x0054)
#define   LCDCTRCNTL      (0x0070)

#define   EADOGM163_33V
//#define   EADOGM162_33V

#ifdef    EADOGM163_33V
#define   FunctionSet1    00111001b           // 39h
#define   BiasSet         00010101b           // 15h
#define   PowerControl    01010101b           // 55h
#define   FollowerControl 01101110b           // 6Eh
#define   ContrastSet     01110010b           // 72h
#define   FunctionSet0    00111000b           // 38h
#define   DisplayOn       00001111b           // 0Fh
#define   ClearDisplay    00000001b           // 01h
#define   EntryModeSet    00000110b           // 06h
#endif

#ifdef    EADOGM162_33V
#define   FunctionSet1    00111001b           // 39h
#define   BiasSet         00010100b           // 14h
#define   PowerControl    01010101b           // 55h
#define   FollowerControl 01101101b           // 6Dh
#define   ContrastSet     01111000b           // 78h
#define   FunctionSet0    00111000b           // 38h
#define   DisplayOn       00001111b           // 0Fh
#define   ClearDisplay    00000001b           // 01h
#define   EntryModeSet    00000110b           // 06h
#endif
#endif
/**********************************************************************************************
* END OF INCLUDEFILE
**********************************************************************************************/