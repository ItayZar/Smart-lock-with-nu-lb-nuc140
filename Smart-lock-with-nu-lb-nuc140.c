/* Wired connections */

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

//SR04 definition
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

//PWM
#define HITIME_MIN 		77  
#define HITIME_MAX 		128 

//Users
#define MAX_USERS 8

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
char 	input_local_password[OTP_LENGTH + 1];

uint32_t time_s =0;
uint32_t distance_mm;
uint32_t hitime;

/* States of the system */
typedef enum {
    IDLE,	  
    USR_PSWD, 
    OTP_AUTH, 
    DR_OPEN	  
} State;

State current_state = IDLE;

/* Struct of a single user */
typedef struct{
	int id; //Must be corresponding to the one on the server.py
	char password[OTP_LENGTH + 1];
}user;

 void Init_LED()
{
	// initialize GPIO pins
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT); // GPA13 pin set to output mode
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT); // GPA14 pin set to output mode
	// set GPIO pins output Hi to disable LEDs
	DrvGPIO_SetBit(E_GPA, 13); // GPA13 pin output Hi to turn off Green LED
	DrvGPIO_SetBit(E_GPA, 14); // GPA14 pin output Hi to turn off Red   LED
}   

/* Function to close the door */
void servo_close(void) {
	 for (hitime=HITIME_MIN; hitime<=HITIME_MAX; hitime++) {
		PWM_Servo(0, hitime);
		DrvSYS_Delay(20000);
	}
}

/* Function to open the door */
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

void UART_INT_HANDLE(void)
{
	int i;

	while(UART0->ISR.RDA_IF==1) 
	{
		// Data recieved to  comRbuf[] array
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;
		if(comRbytes==OTP_LENGTH){
		    for (i = 0; i < OTP_LENGTH; i++) {
	        otp[i] = comRbuf[i]; 
	    	}
		}
	}
}
void sendID(int id)
{
   	// Function to send the ID to the server using BT
	char id_ascii[1];
	id_ascii[0] = 48 + id;	//48 dec is '0' in ASCII
	id_ascii[1] = '\0';
	DrvUART_Write(UART_PORT0 , id_ascii , strlen(id_ascii));

}

void display_status(int locked)
{
	// Displays status on LCD and lights appropriate color
	clr_all_panel();
 	if (locked){
		print_lcd(0, "Status: Locked"); 
		// set RGBled to Red
	    	DrvGPIO_SetBit(E_GPA,13); 
	    	DrvGPIO_ClrBit(E_GPA,14); // GPA14 = Red,   0 : on, 1 : off	
	}
	else{
		print_lcd(0, "Status: Unlocked"); 
		// set RGBled to Green
	    	DrvGPIO_ClrBit(E_GPA,13); // GPA13 = Green, 0 : on, 1 : off
		DrvGPIO_SetBit(E_GPA,14); 	
	}
}

void buzzer(int times)
{
	// Single buzzer sound
	int i;
	for(i=0;i<times;i++)
	{
		DrvGPIO_ClrBit(E_GPB,11); // GPB11 = 0 to turn on Buzzer
		DrvSYS_Delay(100000);	    // Delay 
		DrvGPIO_SetBit(E_GPB,11); // GPB11 = 1 to turn off Buzzer	
		DrvSYS_Delay(100000);	    // Delay 
	}
}
void delay_sec(int sec)
{
	//Delay approximation in seconds 0.335 sec * 3 = 1.005 sec -> +/- 5ms for each second
	int t=0;
	for(t=0;t<sec;t++)
	{
	 	DrvSYS_Delay(335000);
		DrvSYS_Delay(335000);
		DrvSYS_Delay(335000);
	}
}

void clearText(char* text)
{
	// Clears given string
	strcpy(text, "                 ");
}

//------------------------------
// MAIN function
//------------------------------
int main (void)
{	
	STR_UART_T sParam;
	int flag;
	int i,tmp,attemps=1,correct ;

	/* Up to 9 users can be defined  */
	/* Password is set to be 4 digits */
	user users[MAX_USERS];
	users[0].id = 0; // This ID should be matched to the one configured in the server.py
    	snprintf(users[0].password, sizeof(users[0].password), "1987");

    	users[1].id = 1;
    	snprintf(users[1].password, sizeof(users[1].password), "1234");

	
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

	/* Initialization */
	Initial_panel();                  // initialize LCD
	clr_all_panel();                  // clear LCD display
	display_status(1);
	OpenKeyPad();                          
	InitTIMER0();
	Init_LED();
	Init_TMR2();
	Init_GPIO_SR04();
	InitPWM(0);   // initialize PWM0
	PWM_Servo(0, HITIME_MAX); //Make sure gate is closed at the beginning
	DrvGPIO_Open(E_GPB, 11, E_IO_OUTPUT); // initial GPIO pin GPB11 for controlling Buzzer
	time_s = 0;

	while(1) {
		/* The OS is based on a state machine  */
		switch(current_state){
			case(IDLE):
				clr_all_panel();
				display_status(1);
				print_lcd(1,"IDLE");	//prints current state
				DistMeasure();
				if(distance_mm <= 100)
				{
					/* If an object is detected for 3 seconds within 100mm the system will change its state to the first password authentication */
					TIMER0->TCSR.CEN = 1;		// Enable Timer0
					if(distance_mm <= 100 &&  time_s == 3) //
					{
					 	current_state = USR_PSWD;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;	
					}
				}
				else{time_s=0;}
				//DistMeasure();
				break;
			case(USR_PSWD):
				/* First tier of authentication */
				/* User has 3 authentication attempts, otherwise will return to idle state  */
				clr_all_panel();
				display_status(1);
				print_lcd(1, "Enter Your Code");
				for(i=0;i<OTP_LENGTH;i++)
				{
					/* Code input using the onboard keypad */
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
				for(i=0;i<MAX_USERS;i++)
				{
					/* Validation attempt of the entered code against all the users' codes */
					/* When matches the system will send the matched ID to the server and changes its state to the second authentication */
					if(0==strcmp(input_local_password,users[i].password))
					{
						print_lcd(2, "Correct !");
						delay_sec(1);
					 	current_state = OTP_AUTH;
						sendID(users[i].id);
						attemps=1;
						correct=1;
						break;
					}
				}
				if(!correct)
				{
					/* When there is no matched code to any of the users */
					print_lcd(2, "Incorrect !");
					sprintf(TEXT3,"%d attempts left", 3-attemps);
					print_lcd(3,TEXT3);
					buzzer(3);
					delay_sec(2);
				 	attemps++;
				}
				if(attemps==4)
				{
					/* After 3 mismatches */
					clr_all_panel();
				 	print_lcd(2, "Reached max attemps");
					current_state = IDLE;		
				}
				break;
			case(OTP_AUTH):
				/* Second tier of authentication */
				/* The system receives the same OTP that was sent to the user's email address and validates it  */
				/* The user has 3 attempts and 30 seconds to validate his OTP */
				/* When time out has reached, the state will be switched to IDLE */
				clr_all_panel();
				clearText(TEXT2);
				display_status(1);
				TIMER0->TCSR.CEN = 1;		// Enable Timer0
				time_s=0;
				print_lcd(2, "OTP sent to your");
				print_lcd(3, "Email");
				delay_sec(1);
				clr_all_panel();
				while(time_s <= 30)
				{
					/* 30 seconds window for authentication begins */
					clr_all_panel();
					print_lcd(1, "Enter Your OTP");
					for(i=0;i<OTP_LENGTH;i++)
					{
						/* Code input using the onboard keypad */
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
						/* Authorized */
						/* System state changes to unlock and open the door */
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
						/* Incorrect OTP */
						print_lcd(2, "Incorrect !");
						sprintf(TEXT3,"%d attemps left", 3-attemps);
						print_lcd(3,TEXT3);
						buzzer(3);
						delay_sec(2);
					 	attemps++;
					}
					if(attemps==4)
					{
						/* Reached maximum number of attempts */
						clr_all_panel();
					 	print_lcd(2, "Reached Max attempts");
						current_state = IDLE;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;
						break;		
					}
					if(time_s==30)
					{
						/* 30 seconds time out has been reached */
					 	print_lcd(1, "Timed out!");
						current_state = IDLE;
						TIMER0->TCSR.CEN = 0;		// Disable Timer0
						time_s = 0;
						break;		
					}
				}
				break;
			case(DR_OPEN):
				/* This state opens the door for 3 seconds and closes it back */
				/* Green light will be visible to the user as well as LCD output that indicates the door is unlocked */
				/* When door closes the system goes back to IDLE */
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
		
