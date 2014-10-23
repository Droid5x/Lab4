/* Code for Lab 3.3
Bradley Jewett
Mark Blanco
10/20/14
*/

#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>
#define PROPORTION 0.417
#define HEADING 900


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init();
void Interrupt_Init(void);
void SMB_Init(void);
void PCA_ISR ( void ) __interrupt 9;
unsigned int Read_Compass();
void Steering_Servo(unsigned int current_heading);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char interrupts;
unsigned char take_heading;
unsigned int reading;
unsigned int PW_CENTER = 2905;	// Center PW value
unsigned int PW_MIN = 2385;		// Minimum left PW value
unsigned int PW_MAX = 3315;		// Maximum right PW value
unsigned int PW = 2905;			// Start PW at center
unsigned char final = 0;
__sbit __at 0xB7 SS;			// Slide switch input pin at P3.7

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    PCA_Init();
	SMB_Init();
	Interrupt_Init();
	printf("Starting\n\r");
	
	//Main Functionality
	while(1) {
		if (!SS){	// If the slide switch is active, set PW to center
			PW = PW_CENTER;
			PCA0CP0 = 0xFFFF - PW;		// Update comparator with new PW value
			while(!SS);					// Wait...
		}
		else if (take_heading) {		// Otherwise take a new heading
			reading = Read_Compass();	// Get current heading
			printf("%d\n\r", reading);
			Steering_Servo(reading);	// Change PW based on current heading
			PCA0CP0 = 0xFFFF - PW;		// Update comparator with new PW value
		}
	}
}


//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
//
unsigned int Read_Compass() 
{
	unsigned char addr = 0xC0; 	// Address the compass heading will be written to
	unsigned char Data[2];		// Data array to store heading data
	unsigned int heading;		// Variable to store heading data
	i2c_read_data(addr, 2, Data, 2);// Read data from compass registers, store it in Data buffer
	heading = (((unsigned int)Data[0] << 8) | Data[1]);	//Take high data byte, convert to int, 
														//shift left 8 bits, and copy lower compass byte to first half of int
	take_heading = 0;
	return heading;				// Return data heading between 0 and 3599 
}



//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
    P0MDOUT &= ~0xC0;   //(00XX XXXX) Set P0.6 and P0.7 Open Drain (Input)
	P0 |= 0xC0;			//(11XX XXXX) Set P0.6 and P0.7 to High-Impedence Mode
	P1MDOUT |= 0x01;  	//set output pin for CEX0 at P1.0 to push pull
	P3MDOUT &= ~0x80;	// Set input for slide switch on P3.7 Open Drain
	P3 |= ~0x80;
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{
    XBR0 = 0x27;  	//Configure Crossbar

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
	PCA0CN |= 0x40;		//Enable Counter (Bit 0)
	PCA0MD |= 0x01;		//Enable PCA overflow interrupt (bit 0)	
	PCA0MD &= ~0x0E;	//Timer uses SYSCLK/12 (Bits 1-3)
	PCA0CPM0 = 0xC2;	//Use 16bit counter (bit 7)
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up interrupts
//
void Interrupt_Init(void)
{
	
	EIE1 |= 0x08;		//Enable PCA0 Interrupt (bit 3) 
	EA = 1;				//Enable global interrupts


}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Configure Magnetic Compass Interface
//
void SMB_Init(void)
{
	SMB0CR = 0x93;		//Configure SCL frequency
	SMB0CN = 0x40;		//Enable SMBus
}



//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{	
	if (CF) {						//If an interrupt has occured
		interrupts++;
		if (interrupts >= 2) {		//If two interrupts have occured
				interrupts = 0;		//Reset interrupts
				take_heading = 1;	//It is appropriate to take a reading
		}
		CF = 0;						//Clear Interrupt Flag
		PCA0 = 0x7000;				//Jump timer ahead for given period
	}			
}

//-----------------------------------------------------------------------------
// Steering_Servo
//-----------------------------------------------------------------------------
//
void Steering_Servo(unsigned int current_heading)
{
	signed int error = 0;
	//signed int temp;
	error = HEADING - current_heading;		// Calculate signed error
	if ( error > 1800){						// If the error is greater than 1800
		error = 3600%error;					// or less than -1800, then the 
		error*=-1;							// conjugate angle needs to be generated
	} else if ( error < -1800 ){			// with opposite sign from the original
		error = 3600%abs(error);			// error
	}
	printf("\t%d\n\r",error);				// Commented out unless testing
	PW = PROPORTION * error + PW_CENTER;	// Update PW based on error

	if(PW > PW_MAX){ 		// check if pulsewidth maximum exceeded
       	PW = PW_MAX;		// set PW to a maximum value
   	}
	else if(PW < PW_MIN){ 	// check if less than pulsewidth minimum
       	PW = PW_MIN;		// set SERVO_PW to a minimum value
    }
}