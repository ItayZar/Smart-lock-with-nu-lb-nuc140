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

#include <stdio.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "LCD_Driver.h"

// Global definition
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number

// Global variables
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;

char	TEXT2[17] = "Dist: ";
char	TEXT3[17] = "P.Time: ";

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
	sprintf(TEXT3+8, "%2d sec  ", time_s);	
	print_lcd(3, TEXT3);	        //Line 2: distance [mm]

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



void DistMeasure(void)
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
}


void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input						
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
  	_SR04A_TRIG_Low;                  // set Trig output to Low
}

//------------------------------
// MAIN function
//------------------------------
int main (void)
{	
	int flag;
	//System Clock Initial
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, ENABLE);
	while(DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
	DrvSYS_Open(50000000);
	LOCKREG();
	
	Initial_panel();                  // initialize LCD
	clr_all_panel();                  // clear LCD display
	print_lcd(0, "  Gate Control  "); // Line 0 display
	sprintf(TEXT3+8, "IDLE   ");
	print_lcd(3, TEXT3);
	                          
	InitTIMER0();
	Init_TMR2();
	Init_GPIO_SR04();
  
	while(1) {
		DistMeasure();
		if(distance_mm <= 100)
		{
			TIMER0->TCSR.CEN = 1;		// Enable Timer0
		}
		if(distance_mm <= 30 && time_s > 0 && time_s < 10)
		{
				print_lcd(1, "Car Parked!");
				print_lcd(2, "                ");
				print_lcd(3, "                ");
				TIMER0->TCSR.CEN = 0;		// Disable Timer0
				time_s = 0;
				while(distance_mm < 100)
				{
					 DistMeasure();
				}
		}
		if (distance_mm > 30 && distance_mm < 100 && time_s == 10)
		{
			print_lcd(1, "Timed Out!");
			print_lcd(2, "                ");
			print_lcd(3, "                ");

			DrvGPIO_ClrBit(E_GPB,11); 	// GPB11 = 0 to turn on Buzzer
			DrvSYS_Delay(100000);	    // Delay 
			DrvGPIO_SetBit(E_GPB,11); 	// GPB11 = 1 to turn off Buzzer	

			TIMER0->TCSR.CEN = 0;		// Disable Timer0
			time_s = 0;
			flag = 1;
			while(distance_mm < 100 && flag ==1)
				{
					 DistMeasure();
					 if( distance_mm > 100) 
					 {
					 	flag = 0;
					 }
				}
		}
		if(distance_mm > 100)
		{
			time_s = 0;
			print_lcd(1, "                ");
			sprintf(TEXT3+8, "IDLE   ");
			print_lcd(3, TEXT3);
			TIMER0->TISR.TIF =1;		// Reset Timer0
			TIMER0->TCSR.CEN = 0;		// Disable Timer0
		}
	}
	
}
