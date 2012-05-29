//(c)2010 by Texas Instruments Incorporated, All Rights Reserved.
/*
 * ======== main.c ========
 * Local Echo Demo:
 *
 * This USB demo example is to be used with a PC application (e.g. HyperTerminal)
 * The demo application demonstartes a local echo; Characters that are
 * transmitted from PC are echoed back.
 *
 * NOTE: In order to see the local echo in Hyper Terminal goto File -> Propoerties
 * -> Settings -> ASCII Setup and check the "Echo Typed Characters Locally"
 *
 * ----------------------------------------------------------------------------+
 * Please refer to the MSP430 USB API Stack Programmer's Guide,located
 * in the root directory of this installation for more details.
 * ---------------------------------------------------------------------------*/
#include <intrinsics.h>
#include <string.h>

#include "USB_config/descriptors.h"

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"               //Basic Type declarations
#include "USB_API/USB_Common/usb.h"                 //USB-specific functions

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"

#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbConstructs.h"

VOID Init_Ports (VOID);
VOID Init_Clock (VOID);

void delay(unsigned int k);
void delays(unsigned long k);
volatile BYTE bCDCDataReceived_event = FALSE;   //Flag set by event handler to indicate data has been received into USB buffer

unsigned char SecondCounter;
unsigned int Temperature1;
unsigned int Temperature2;
unsigned int Temperature3;
unsigned int Temperature4;
unsigned int Temperature5;
extern unsigned short ZACTemp1(void);
extern unsigned short ZACTemp2(void);
extern unsigned short ZACTemp3(void);
extern unsigned short ZACTemp4(void);
extern unsigned short ZACTemp5(void);

#define BUFFER_SIZE 256
char nl[2] = "\n";
char dataBuffer[BUFFER_SIZE] = "";
//char GGData[7] = {'G','e','r','a','l','d','\n'};
//char GGData[15] = {'$','1',';','1',';',';','T','T','T','.','T',';','0',13,10};
//char GGData[21] = {'$','1',';','1',';',';','T','T','T','.','T',';','T','T','T','.','T',';','0',13,10};
char GGData[39] = {'$','1',';','1',';',';','T','T','T','.','T',';','T','T','T','.','T',';','T','T','T','.','T',';','T','T','T','.','T',';','T','T','T','.','T',';','0',13,10};
WORD count;

/*----------------------------------------------------------------------------+
 | Main Routine                                                                |
 +----------------------------------------------------------------------------*/
VOID main (VOID)
{
    WDTCTL = WDTPW + WDTHOLD;                                   //Stop watchdog timer

  //  Init_Ports();                                               //Init ports (do first ports because clocks do change ports)
    SetVCore(3);
    Init_Clock();                                               //Init clocks

    USB_init();                 //Init USB

    //Enable various USB event handling routines

    USB_setEnabledEvents(
        kUSB_VbusOnEvent + kUSB_VbusOffEvent + kUSB_receiveCompletedEvent
        + kUSB_dataReceivedEvent + kUSB_UsbSuspendEvent + kUSB_UsbResumeEvent +
        kUSB_UsbResetEvent);

    //See if we're already attached physically to USB, and if so, connect to it
    //Normally applications don't invoke the event handlers, but this is an exception.
    if (USB_connectionInfo() & kUSB_vbusPresent){
        if (USB_enable() == kUSB_succeed){
            USB_reset();
            USB_connect();
        }
    }

    __enable_interrupt();                           //Enable interrupts globally

    WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL_1 + WDTIS_4; // watchdog counter mode, ACLK, /32768
    SFRIFG1 &= ~WDTIFG;                                // Clear WDT interrupt flag
    SFRIE1 |= WDTIE;                                   // WDT interrupt enable
    
    while (1)
    {
        BYTE ReceiveError = 0, SendError = 0;
    //    WORD count;
        //Check the USB state and loop accordingly
        switch (USB_connectionState())
        {
            case ST_USB_DISCONNECTED:
                __bis_SR_register(LPM3_bits + GIE);                             //Enter LPM3 until USB is connected
                __no_operation();
                break;

            case ST_USB_CONNECTED_NO_ENUM:
                break;

            case ST_ENUM_ACTIVE:

                __bis_SR_register(LPM0_bits + GIE);                             //Enter LPM0 until awakened by an event handler
                __no_operation();
                
              Temperature1 = ZACTemp1();
              Temperature2 = ZACTemp2();
              Temperature3 = ZACTemp3();
              Temperature4 = ZACTemp4();
              Temperature5 = ZACTemp5();
  
              for (int i=0;i<=20;i++)
              {dataBuffer[i] = GGData[i];};
              
              dataBuffer[10] = (Temperature1 & 15)+48;
              dataBuffer[8] = ((Temperature1 >> 4) & 15)+48;
              dataBuffer[7] = ((Temperature1 >> 8) & 15)+48;
              dataBuffer[6] = ((Temperature1 >> 12) & 15)+48;
              
              dataBuffer[16] = (Temperature2 & 15)+48;
              dataBuffer[14] = ((Temperature2 >> 4) & 15)+48;
              dataBuffer[13] = ((Temperature2 >> 8) & 15)+48;
              dataBuffer[12] = ((Temperature2 >> 12) & 15)+48;
              
              dataBuffer[22] = (Temperature3 & 15)+48;
              dataBuffer[20] = ((Temperature3 >> 4) & 15)+48;
              dataBuffer[19] = ((Temperature3 >> 8) & 15)+48;
              dataBuffer[18] = ((Temperature3 >> 12) & 15)+48;
              
              dataBuffer[28] = (Temperature4 & 15)+48;
              dataBuffer[26] = ((Temperature4 >> 4) & 15)+48;
              dataBuffer[25] = ((Temperature4 >> 8) & 15)+48;
              dataBuffer[24] = ((Temperature4 >> 12) & 15)+48;
              
              dataBuffer[34] = (Temperature5 & 15)+48;
              dataBuffer[32] = ((Temperature5 >> 4) & 15)+48;
              dataBuffer[31] = ((Temperature5 >> 8) & 15)+48;
              dataBuffer[30] = ((Temperature5 >> 12) & 15)+48;
              
              _NOP();
                count = 21;
                cdcSendDataInBackground((BYTE*)dataBuffer,count,CDC0_INTFNUM,1);
                break;

            case ST_ENUM_SUSPENDED:
                __bis_SR_register(LPM3_bits + GIE);             //Enter LPM3 until a resume or VBUS-off event
                break;

            case ST_ENUM_IN_PROGRESS:
                break;

            case ST_NOENUM_SUSPENDED:
                __bis_SR_register(LPM3_bits + GIE);
                break;

            case ST_ERROR:
                _NOP();
                break;

            default:;
        }
        if (ReceiveError || SendError){
            //TO DO: User can place code here to handle error
        }
       // delays(280000);
    }  //while(1)
}                               //main()

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

void delays(unsigned long k)
{
  unsigned long Freq;
  Freq = 8000000;
  Freq = Freq / 1000000;
  k = k*Freq;
  k = (((k-14)/3)+1);

  for(;k>0;k--);
}

/*
 * ======== Init_Clock ========
 */
VOID Init_Clock (VOID)
{
    //Initialization of clock module
    if (USB_PLL_XT == 2){
		#if defined (__MSP430F552x) || defined (__MSP430F550x)
			P5SEL |= 0x0C;                                      //enable XT2 pins for F5529
		#elif defined (__MSP430F563x_F663x)
			P7SEL |= 0x0C;
		#endif

        //use REFO for FLL and ACLK
        UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
        UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

        //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
        Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //Start the FLL, at the freq indicated by the config
                                                                        //constant USB_MCLK_FREQ
        XT2_Start(XT2DRIVE_0);                                          //Start the "USB crystal"
    }
	else {
		#if defined (__MSP430F552x) || defined (__MSP430F550x)
			P5SEL |= 0x10;                                      //enable XT1 pins
		#endif
        //Use the REFO oscillator to source the FLL and ACLK
        UCSCTL3 = SELREF__REFOCLK;
        UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

        //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
        Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //set FLL (DCOCLK)

        XT1_Start(XT1DRIVE_0);                                          //Start the "USB crystal"
    }
}

/*
 * ======== Init_Ports ========
 */
VOID Init_Ports (VOID)
{
    //Initialization of ports (all unused pins as outputs with low-level
    P1OUT = 0x00;
    P1DIR = 0xFF;
    P2OUT = 0x00;
    P2DIR = 0xFF;
    P3OUT = 0x00;
    P3DIR = 0xFF;
    P4OUT = 0x00;
    P4DIR = 0xFF;
    P5OUT = 0x00;
    P5DIR = 0xFF;
    P6OUT = 0x00;
    P6DIR = 0xFF;
    P7OUT = 0x00;
    P7DIR = 0xFF;
    P8OUT = 0x00;
    P8DIR = 0xFF;
    #if defined (__MSP430F563x_F663x)
		P9OUT = 0x00;
		P9DIR = 0xFF;
    #endif
}



/*
 * ======== UNMI_ISR ========
 */
#pragma vector = UNMI_VECTOR
__interrupt VOID UNMI_ISR (VOID)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG); //Clear OSC flaut Flags fault flags
            SFRIFG1 &= ~OFIFG;                          //Clear OFIFG fault flag
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            SYSBERRIV = 0;                                      //clear bus error flag
            USB_disable();                                      //Disable
    }
}



#pragma vector=WDT_VECTOR
__interrupt void ISR_WDT(void)
{       /*
        SecondCounter++;
        if (SecondCounter == 2)
        {
          SecondCounter = 0;
        }
        */
        __bic_SR_register_on_exit(LPM0_bits);
        
              
}