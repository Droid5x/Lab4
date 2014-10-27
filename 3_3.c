/* Aaron Clippinger, Lance Gerday
   Section 2 Side A Seat 5
   October 23, 2014
   Lab 3-3 */

#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define PW_MIN 2030
#define PW_MAX 3500
#define PW_NEUT 2760
//-----------------------------------------------------------------------------
// 8051 Initialization Functions
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
void Drive_Motor(unsigned int);
void Interrupt_Init(void);
void SMB_Init(void);
unsigned char read_ranger(void);

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
unsigned int MOTOR_PW = 0;
unsigned int c = 0;
unsigned char getRange = 1;
unsigned char rWait = 0;
unsigned int range_val = 0;
unsigned char Data[2];
unsigned char addr=0xE0; 	// the address of the ranger is 0xE0
unsigned int motorPW;

__sbit __at 0xB6 SS0;		// Assign P3.6 to SS (Slide Switch)
//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------

void main(void) 
{
    Sys_Init();		// initialize board
    putchar(' '); 	//the quotes in this line may not format correctly
    Port_Init();
    PCA_Init();
	SMB_Init();

    //print beginning message
    printf("Embedded Control Drive Motor Control\r\n");
    // Initialize motor in neutral and wait for 1 second
    MOTOR_PW = PW_NEUT;
	motorPW = 0xFFFF-MOTOR_PW;
    PCA0CPL2 = motorPW;
    PCA0CPH2 = motorPW>>8;
	printf("Pulse Width = %d\r\n",MOTOR_PW);
    c = 0;
    while (c < 50);								//wait 1 second in neutral
	printf("end wait \r\n");
    while (1)
	{
		if(getRange)
		{
			getRange=0;							// Reset 80 ms flag
			range_val = read_ranger();			// Read the distance from the ranger
			printf("Range:			%d cm \r\n", range_val);
			printf("Pulse Width:	%d \r\n", MOTOR_PW);

			// Start a new ping
			Data[0] = 0x51; 					// write 0x51 to reg 0 of the ranger:
			i2c_write_data(addr, 0, Data, 1);  // write one byte of data to reg 0 at addr
		}

		if(SS0) Drive_Motor(range_val);
		else Drive_Motor(45);					// Hold the motor in neutral if the slide switch is off
	}
}
//-----------------------------------------------------------------------------
// Drive_Motor
//-----------------------------------------------------------------------------
//
// Vary the pulsewidth based on the user input to change the speed 
// of the drive motor.
//

void Drive_Motor(unsigned int input) 
{
	unsigned int motorPW;										// Declare local variable
    
    if (input <= 10)	MOTOR_PW = PW_MAX;						// Motor at full forward

	else if (input >= 90) MOTOR_PW = PW_MIN;  					// Motor in full reverse

	else if (input >= 40 && input <= 50) MOTOR_PW = PW_NEUT;	// Motor in neutral

	else if (input > 10 && input < 40) MOTOR_PW = ((PW_NEUT - PW_MAX)/30) * (input - 10) + PW_MAX;

	else if (input > 50 && input < 90) MOTOR_PW = ((PW_MIN - PW_NEUT)/40) * (input - 50) + PW_NEUT;

	//printf("Pulse Width = %d\r\n",MOTOR_PW);
	motorPW = 0xFFFF-MOTOR_PW;
    PCA0CPL2 = motorPW;					// Set High and low byte for motor speed
    PCA0CPH2 = motorPW>>8;
}

//-----------------------------------------------------------------------------
// read_ranger
//-----------------------------------------------------------------------------
//
// Read the value of the ultrasonic ranger
// 

unsigned char read_ranger(void) 
{
	unsigned int range = 0;
	unsigned char slave_reg = 2;	//start at register 2
	unsigned char num_bytes = 2;	//read 2 bytes

	i2c_read_data(addr,slave_reg,Data,num_bytes); 		// read two bytes, starting at reg 2
	range = (((unsigned int)Data[0] << 8) | Data[1]);	// Store high and low bytes of Data in variable range
	return range;
}

//-----------------------------------------------------------------------------
// SMB Init
//-----------------------------------------------------------------------------
void SMB_Init(void)
{
	SMB0CR = 0x93;	// Set SCL to 100 kHz
	ENSMB = 1;		// Enable SMBus
}
//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//

void Port_Init(void) 
{
    P1MDOUT |= 0x04; 	// set output pin P1.2 for push-pull mode
	XBR0 = 0x27; 	 	// configure crossbar with UART, SPI, SMBus, and CEX channels 

	P3MDOUT &= ~0x40;	// Set P3.6 to an input
	P3 |= 0x40;
}
    
//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//

void PCA_Init(void) {
    // reference to the sample code in Example 4.5 - Pulse Width Modulation 
    // implemented using the PCA (Programmable Counter Array, p. 50 in Lab Manual.
    // Use a 16 bit = counter with SYSCLK/12.
    PCA0MD = 0x81; 	 	// enable CF interupt, use SYSCLK/12
    PCA0CPM2 = 0xC2; 	// select 16bit PWM, enable positive edge capture, enable pulse width modulation
    PCA0CN = 0x40; 	 	// enable PCA0 counter
	EIE1 |= 0x08;    	// enable PCA interrupts
    EA = 1;          	// enable all interrupts
}
//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//

void PCA_ISR(void) __interrupt 9 
{
    // reference to the sample code in Example 4.5 -Pulse Width Modulation 
    // implemented using the PCA (Programmable Counter Array), p. 50 in Lab Manual.
	if (CF) 
	{
        CF = 0; 						// clear overflow flag
        PCA0L = (unsigned char)28672; 	// set low byte for 20ms period pulse
        PCA0H = 28672>>8;   			// set high byte for 20 ms period pulse
		c++;							// counter for initial wait to initialize motor
		rWait++;						// counter to set 80ms flag
		if(rWait >= 4)
		{
			getRange=1;					// 80ms flag
			rWait=0;					// Reset counter
		}
    }
	PCA0CN &= 0xC0;						// Handle other PCA interrupts

}
