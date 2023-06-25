//
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

// PWM0 	DC Servo 
// DATA		PWM3/GPA12 		[orange]	   
// VCC		Vcc +5V			[red]
// GND		GND				[brown]

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
#include "Driver_PWM_Servo.h"

// Global definition
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

//TIMERS
#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number

//OTP
#define  OTP_LENGTH 4

//LEDS
#define RGB_BLUE			12
#define RGB_GREEN			13
#define RGB_RED				14

//PWM
#define HITIME_MIN 		30  // was 0.17ms [17]
#define HITIME_MAX 		128 // was 1.2ms  [120]

unsigned char DisplayBuf [128*8];
char TEXT[16];
volatile uint8_t comRbuf[9];
volatile uint8_t comRbytes = 0;

volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;


char	TEXT0[17] = "                 ";
char	TEXT1[17] = "                 ";
char	TEXT2[17] = "                 ";
char	TEXT3[17] = "                 ";


char 	otp[OTP_LENGTH + 1];
char 	input_otp[OTP_LENGTH + 1];
char	local_password[OTP_LENGTH + 1]="1987" ;
char 	input_local_password[OTP_LENGTH + 1];

uint32_t time_s =0;
uint32_t distance_mm;
uint32_t hitime;

typedef enum {
    IDLE,	  
    USR_PSWD, 
    OTP_AUTH, 
    DR_OPEN	  
} State;

State current_state = IDLE;


/*uint8_t calc_deg(uint8_t value) {
	return ((value-HITIME_MIN)*90)/(HITIME_MAX-HITIME_MIN); 	// 0 to 90 degrees!
} */

void servo_close(void) {
	 for (hitime=HITIME_MIN; hitime<=HITIME_MAX; hitime++) {
		PWM_Servo(0, hitime);
		DrvSYS_Delay(20000);
	}
}

void servo_open(void) {
	 for (hitime=HITIME_MAX; hitime>=HITIME_MIN; hitime--) {
		PWM_Servo(0, hitime);
		DrvSYS_Delay(20000);
	}
}

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
	uint8_t TxString[9] = "ACK\r\n";

	while(UART0->ISR.RDA_IF==1) 
	{
		// Data recieved to  comRbuf[] array
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;		
		if (comRbytes==3) {
			DrvUART_Write(UART_PORT0 , TxString , 9);
			sprintf(TEXT,"cmd: %s",comRbuf);
			print_lcd(0,TEXT);
				
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
	DrvUART_Write(UART_PORT0 , "OTP:\t" , strlen("OTP:\t"));
	DrvUART_Write(UART_PORT0 , otp , OTP_LENGTH);
	DrvUART_Write(UART_PORT0 , "\n\r" , 2); //new line and return cursor for next message

}

void display_status(int locked)
{
	clr_all_panel();
 	if (locked){
		print_lcd(0, "Status: Locked"); 	
	}
	else{
		print_lcd(0, "Status: Unlocked"); 	
	}
}

void delay_sec(int sec)
{
	int t=0;
	for(t=0;t<sec;t++)
	{
	 	DrvSYS_Delay(335000);
		DrvSYS_Delay(335000);
		DrvSYS_Delay(335000);
	}
}

void clearText(char* text) {
    strcpy(text, "                 ");
}




//------------------------------
// MAIN function
//------------------------------
int main (void)
{	
	STR_UART_T sParam;
	int flag;
	int i,tmp,attemps=1 ;
	
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
	display_status(1);
	OpenKeyPad();                          
	InitTIMER0();
	Init_TMR2();
	Init_GPIO_SR04();
	InitPWM(0);   // initialize PWM0
	PWM_Servo(0, HITIME_MAX); //Make sure gate is closed at the beginning
	time_s = 0;
		

	while(1) {
		switch(current_state){
			case(IDLE):
				clr_all_panel();
				display_status(1);
				print_lcd(1,"IDLE");	//prints current state
				DistMeasure();
				if(distance_mm <= 100)
				{
					TIMER0->TCSR.CEN = 1;		// Enable Timer0
				}
				if(distance_mm <= 100 &&  time_s == 3)
				{
				 	current_state = USR_PSWD;
					TIMER0->TCSR.CEN = 0;		// Disable Timer0
					time_s = 0;	
				}
				break;
			case(USR_PSWD):
				clr_all_panel();
				display_status(1);
				print_lcd(1, "Enter Your Code");
				for(i=0;i<OTP_LENGTH;i++)
				{
					tmp=0;
					while(tmp==0){
						tmp=Scankey(); 
					}
					input_local_password[i] = 48 + tmp; // 48 in dec = '0' ASCII
					sprintf(TEXT2+i,"%c",input_local_password[i]);
					print_lcd(2, TEXT2);
					while(tmp!=0){
						tmp=Scankey();
					}
					DrvSYS_Delay(335000);
				}
				input_local_password[OTP_LENGTH] = '\0';
				clr_all_panel();
				display_status(1);
				if(0==strcmp(input_local_password,local_password))
				{
					print_lcd(2, "Correct !");
					delay_sec(1);
				 	current_state = OTP_AUTH;
					generateOTP();
					sendOTP();
					attemps=1;
				}
				else
				{
					print_lcd(2, "Incorrect !");
					sprintf(TEXT3,"%d attemps left", 3-attemps);
					print_lcd(3,TEXT3);
					delay_sec(2);
				 	attemps++;
				}
				if(attemps==4)
				{
					clr_all_panel();
				 	print_lcd(2, "Reached max attemps");
					current_state = IDLE;		
				}
				break;
			case(OTP_AUTH):
				clr_all_panel();
				clearText(TEXT2);
				display_status(1);
				TIMER0->TCSR.CEN = 1;		// Enable Timer0
				print_lcd(2, "OTP sent via BT");
				delay_sec(1);
				clr_all_panel();
				while(time_s <= 30)
				{
					clr_all_panel();
					print_lcd(1, "Enter Your OTP");
					for(i=0;i<OTP_LENGTH;i++)
					{
						tmp=0;
						while(tmp==0){
							tmp=Scankey(); 
						}
						input_otp[i] = 48 + tmp; // 48 in dec = '0' ASCII
						sprintf(TEXT2+i,"%c",input_otp[i]);
						print_lcd(2, TEXT2);
						while(tmp!=0){
							tmp=Scankey();
						}
						DrvSYS_Delay(335000);
					}
					input_otp[OTP_LENGTH] = '\0';
					delay_sec(1);
					clr_all_panel();
					display_status(1);;
					if(0==strcmp(input_otp,otp))
					{
						print_lcd(2, "Correct !");
						delay_sec(2);
					 	current_state = DR_OPEN;;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;
						attemps=1;
						break;
					}
					else
					{
						print_lcd(2, "Incorrect !");
						sprintf(TEXT3,"%d attemps left", 3-attemps);
						print_lcd(3,TEXT3);
						delay_sec(2);
					 	attemps++;
					}
					if(attemps==4)
					{
						clr_all_panel();
					 	print_lcd(2, "Reached Max attemps");
						current_state = IDLE;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;
						break;		
					}
					if(time_s==30)
					{
					 	print_lcd(1, "Timed out!");
						current_state = IDLE;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;
						break;		
					}
				}
				break;
			case(DR_OPEN):
				clr_all_panel();
				print_lcd(1, "Opening door");
				servo_open();
				clr_all_panel();
				display_status(0);
				delay_sec(3);
				print_lcd(1, "Closing door");
				servo_close();
				current_state = IDLE;
				break;
		}
	}
}
		
