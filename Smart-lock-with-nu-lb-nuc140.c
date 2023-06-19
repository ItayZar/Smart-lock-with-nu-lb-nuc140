//
// Smpl_Timer_SR04 : using one SR04 Ultrasound Sensors
//
// Timer Capture :
// GPB2 / RTS0 / T2EX (NUC140 pin34)
// GPB4 / RX1         (NUC140 pin19)

// SR04 Ultrasound Sensor 
// pin1 Vcc : to Vcc		5v
// pin2 Trig: to GPB4      (NUC140VE3xN pin19)
// pin3 ECHO: to GPB2/T2EX (NUC140VE3xN pin34)
// pin4 Gnd : to GND

// HC05 Bluetooth module
// pin1 : KEY   N.C
// pin2 : VCC   to Vcc +5V
// pin3 : GND   to GND
// pin4 : TXD   to NUC140 UART0-RX (GPB0)
// pin5 : RXD   to NUC140 UART0-TX (GPB1)
// pin6 : STATE N.C.


#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "LCD_Driver.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "scankey.h"


// Global definition
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number
#define  OTP_LENGTH 4
#define RGB_BLUE			12
#define RGB_GREEN			13
#define RGB_RED				14

unsigned char DisplayBuf [128*8];
char TEXT[16];
volatile uint8_t comRbuf[9];
volatile uint8_t comRbytes = 0;

// Global variables
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;

char	TEXT0[17] = "                 ";
char	TEXT1[17] = "                 ";
char	TEXT2[17] = "                 ";
char	TEXT3[17] = "                 ";
char 	otp[OTP_LENGTH + 1];
char 	inputOTP[OTP_LENGTH + 1];

uint32_t time_s =0;
uint32_t distance_mm;



void InitTIMER0(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR0_S = 0;	//Select 12Mhz for Timer0 clock source 
  	SYSCLK->APBCLK.TMR0_EN = 1;	//Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER0->TCSR.MODE=PERIODIC;		//Select PERIODIC mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER0->TCSR.PRESCALE=255;	// Set Prescale [0~255]
	TIMER0->TCMPR = 46875;		// Set TCMPR [0~16777215]
								// (1/12000000)*(255+1)* 46875 = 1 sec / 1 Hz

	/* Step 4. Enable interrupt */
	TIMER0->TCSR.IE = 1;
	TIMER0->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR0_IRQn);	//Enable Timer0 Interrupt

	/* Step 5. Enable Timer module */
	TIMER0->TCSR.CRST = 1;		//Reset up counter
	//TIMER0->TCSR.CEN = 1;		//Enable Timer0

//  	TIMER0->TCSR.TDR_EN=1;		// Enable TDR function
}

void TMR0_IRQHandler(void) // Timer0 interrupt subroutine 
{
	time_s += 1;
	//sprintf(TEXT3+8, "%2d sec  ", time_s);	
	//print_lcd(3, TEXT3);	        //Line 2: distance [mm]

	//if (time_s ==10)
	//{
	//	time_s = 0;
	//	TIMER0->TCSR.CEN = 0;		//Disable Timer0
	//}

 	TIMER0->TISR.TIF =1;	   
}


void Init_TMR2(void)
{	
	//Step 1: T2EX pin Enable (PB.2, Pin34)
	SYS->GPBMFP.UART0_nRTS_nWRL = 1;	
	SYS->ALTMFP.PB2_T2EX = 1;
	
  	//Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR2_RST = 1;          //Timer Controller: Reset
	SYS->IPRSTC2.TMR2_RST = 0;          //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR2_S = 0;	        //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;         //Timer C lock Enable

	//Step 3: Timer Controller Setting
	//  TMR0_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
	TIMER2->TCMPR = 0xffffff;           //Timer Compare Value:  [0~16777215]
	TIMER2->TCSR.PRESCALE = 11;         //Timer Prescale:       [0~255]
	TIMER2->TCSR.MODE = ONESHOT;        //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER2->TEXCON.TEXEN = 1;	          //External Capture Function Enable
	TIMER2->TEXCON.RSTCAPSEL = 0;	      //Capture Mode Select: Capture Mode
	TIMER2->TEXCON.TEX_EDGE = 2;	      //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
//	TIMER2->TCSR.IE = 1;				      //Timeout Interrupt Enable
//	TIMER2->u32TISR |= 0x01;		      //Clear Timeout Flag (TIF)
	TIMER2->TEXCON.TEXIEN = 1;		      //Capture Interrupt Enable
	TIMER2->u32TEXISR |= 0x01;		      //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR2_IRQn);		      //Timer NVIC IRQ Enable

	//Step 6: Start Timer Capture (Set by Ultrasonic_Trigger() Function)
// 	TIMER2->TCSR.CRST = 1;			      //Timer Counter Reset
// 	TIMER2->TCSR.CEN = 1;				      //Timer Start
}

// TMR2 Interrupt Handler
void TMR2_IRQHandler(void)
{
	TIMER2->TEXCON.RSTCAPSEL = 0;       // set back for falling edge to capture
	TIMER2->TCSR.CEN = 1;					      //Timer Start

	if(TIMER2->TEXISR.TEXIF == 1)	      //Capture Flag (TEXIF)
	{
	 	TIMER2->u32TEXISR |= 0x01;				//Clear Capture Flag (TEXIF)
		SR04A_Echo_Width = TIMER2->TCAP;	//Load Capture Value (Unit: us)
		SR04A_Echo_Flag  = TRUE;
	}
}

// Ultrasonic Trigger
void SR04_Trigger(void)
{
	//Trigger of Ultrasonic Sensor
	_SR04A_TRIG_High;
	DrvSYS_Delay(10);							// 10us for TRIG width
	_SR04A_TRIG_Low;
	
	DrvGPIO_Open(E_GPB, 11, E_IO_OUTPUT); // initial GPIO pin GPB11 for controlling Buzzer

  	TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
}



int DistMeasure(void)
{
	SR04_Trigger();                 // Trigger Ultrasound Sensor for 10us   		
	DrvSYS_Delay(40000);            // Wait 40ms for Echo to trigger interrupt
	
	if(SR04A_Echo_Flag==TRUE)
	{
		SR04A_Echo_Flag = FALSE;			
		distance_mm = SR04A_Echo_Width * (340/2) / 1000;
		sprintf(TEXT2+6, " %d mm  ", distance_mm);	
		print_lcd(2, TEXT2);	        //Line 2: distance [mm]
 	}   
	DrvSYS_Delay(10000);           // 10ms from Echo to next Trigger
	return distance_mm; 
}


void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input						
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
  	_SR04A_TRIG_Low;                  // set Trig output to Low
}

void UART_INT_HANDLE(void)
{
	uint8_t TxString[9] = "igotchu\r\n";

	while(UART0->ISR.RDA_IF==1) 
	{
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;
		//sprintf(TEXT,"%d,%d,%d",comRbuf[0],comRbuf[1],comRbuf[2]);
		//print_lcd(0,TEXT);		
		if (comRbytes==3) {
			DrvUART_Write(UART_PORT0 , TxString , 9);
			sprintf(TEXT,"cmd: %s",comRbuf);
			print_lcd(0,TEXT);

			if (comRbuf[0] == 'p' && comRbuf[1] == 'r' && comRbuf[2] == 'n')	//prn
			{
				print_lcd(1,"Chai            ");
				print_lcd(2,"Itay            ");
				print_lcd(3,"Kirill          ");	
			}
			if (comRbuf[0] == 'l' && comRbuf[1] == 'e' && comRbuf[2] == 'd')	//led
			{
				DrvGPIO_ClrBit(E_GPC, 15); 		
				DrvGPIO_ClrBit(E_GPC, 14); 		
				DrvGPIO_ClrBit(E_GPC, 13); 		
				DrvGPIO_ClrBit(E_GPC, 12); 
			}
			if (comRbuf[0] == 'r' && comRbuf[1] == 'g' && comRbuf[2] == 'b')	//rgb
			{
				DrvGPIO_ClrBit(E_GPA, RGB_BLUE); 						
				DrvGPIO_ClrBit(E_GPA, RGB_GREEN); 						
				DrvGPIO_ClrBit(E_GPA, RGB_RED);	
			}
			if (comRbuf[0] == 's' && comRbuf[1] == 'm' && comRbuf[2] == 'l')	//sml
			{
				print_lcd(1,"       :)       ");
				print_lcd(2,"     Smile      ");
				print_lcd(3,"                ");
			}
			if (comRbuf[0] == 'c' && comRbuf[1] == 'l' && comRbuf[2] == 'r')	//clr
			{
				clr_all_panel();
				DrvGPIO_SetBit(E_GPA, RGB_BLUE); 						
				DrvGPIO_SetBit(E_GPA, RGB_GREEN); 						
				DrvGPIO_SetBit(E_GPA, RGB_RED);
				DrvGPIO_SetBit(E_GPC, 12); 
				DrvGPIO_SetBit(E_GPC, 13); 
				DrvGPIO_SetBit(E_GPC, 14);
				DrvGPIO_SetBit(E_GPC, 15);
			}				
			comRbytes=0;
		}
	}
}


void generateOTP()
{
	// Function to generate a random 4-digit OTP
	int i;
	srand(DistMeasure());

    for (i = 0; i < OTP_LENGTH; i++) {
        otp[i] = '1' + (rand() % 9);   // from 1 to 0 ' because no zero on keypad
    }
    otp[OTP_LENGTH] = '\0';  // Add null terminator	
}


void sendOTP()
{
	// Function to send the OTP to a Bluetooth terminal
	DrvUART_Write(UART_PORT0 , otp , OTP_LENGTH);
}


//------------------------------
// MAIN function
//------------------------------
int main (void)
{	
	STR_UART_T sParam;
	int flag;
	int i,attemps,tmp;
	
	//System Clock Initial
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, ENABLE);
	while(DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
	DrvSYS_Open(50000000);
	LOCKREG();

	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins

		/* UART Setting */
    sParam.u32BaudRate 		  = 9600;
    sParam.u8cDataBits 		  = DRVUART_DATABITS_8;
    sParam.u8cStopBits 		  = DRVUART_STOPBITS_1;
    sParam.u8cParity 		    = DRVUART_PARITY_NONE;
    sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;

	/* Set UART Configuration */
 	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);
	
	Initial_panel();                  // initialize LCD
	clr_all_panel();                  // clear LCD display
	print_lcd(0, "  Gate Control  "); // Line 0 display
	sprintf(TEXT3+8, "IDLE   ");
	                          
	InitTIMER0();
	Init_TMR2();
	Init_GPIO_SR04();
	

	while(1) {
		DistMeasure();
		if(distance_mm <= 100)
		{
			TIMER0->TCSR.CEN = 1;		// Enable Timer0
		}
		if(distance_mm <= 100 &&  time_s == 3)
		{
				//print_lcd(1, "Objet Identified");
				print_lcd(2, "                ");
				//print_lcd(3, "                ");
				//TIMER0->TCSR.CEN = 0;		// Disable Timer0
				generateOTP();
				sendOTP();
				print_lcd(1,otp);
				time_s = 0;
				while(time_s <= 30)
				{
					for(i=0;i<OTP_LENGTH;i++)
					{
						tmp=0;
						while(tmp==0){
							tmp=Scankey();
						}
						if(tmp!=0)
						{ 
							inputOTP[i] = 48 + tmp; // 48 in dec = '0' ASCII
						}
						while(tmp!=0){
							tmp=Scankey();
						}
					}
					inputOTP[OTP_LENGTH] = '\0';

					// 3 attemps -> if coorect or not  -> appropriate LED & SOUND ; 
					//sprintf(TEXT2, "%d sec", time_s);
					//clr_all_panel();  
				 	print_lcd(3,inputOTP); //Keypad
				}
				TIMER0->TCSR.CEN = 0;		// Disable Timer0
		}
	}
	
}
