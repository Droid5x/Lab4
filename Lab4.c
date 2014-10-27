/* Aaron Clippinger, Lance Gerday, Bradley Jewett, Mark Blanco
   Section 2 Side A Seat 5
   October 27, 2014
   Lab 4 */


#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define PROPORTION 0.417
#define HEADING 900
#define PW_MIN 2030
#define PW_MAX 3500
#define PW_NEUT 2760


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
void Drive_Motor(unsigned int);
unsigned int Read_Compass();
void Steering_Servo(unsigned int current_heading);
void Interrupt_Init(void);
void SMB_Init(void);
unsigned char read_ranger(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char interrupts;
unsigned char take_heading;
unsigned int reading;
unsigned int PWCENTER = 2905; // Center PW value
unsigned int PWMIN = 2385; // Minimum left PW value
unsigned int PWMAX = 3315; // Maximum right PW value
unsigned int PW = 2905; // Start PW at center
unsigned char final = 0;
__sbit __at 0xB7 SS1; // Slide switch input pin at P3.7

unsigned int MOTOR_PW = 0;
unsigned int c = 0;
unsigned char getRange = 1;
unsigned char rWait = 0;
unsigned int range_val = 0;
unsigned char Data[2];
unsigned char addr = 0xE0; // the address of the ranger is 0xE0
unsigned int motorPW;

__sbit __at 0xB6 SS0; // Assign P3.6 to SS (Slide Switch)

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------

void main(void) {
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    PCA_Init();
    SMB_Init();
    Interrupt_Init();
    printf("Starting\n\r");

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
    
    
    
    //Main Functionality
    while (1) {
        if (!SS1) { // If the slide switch is active, set PW to center
            PW = PWCENTER;
            PCA0CP0 = 0xFFFF - PW; // Update comparator with new PW value
            while (!SS1); // Wait...
        } else if (take_heading) { // Otherwise take a new heading
            reading = Read_Compass(); // Get current heading
            printf("%d\n\r", reading);
            Steering_Servo(reading); // Change PW based on current heading
            PCA0CP0 = 0xFFFF - PW; // Update comparator with new PW value
        }


        if (getRange) {
            getRange = 0; // Reset 80 ms flag
            range_val = read_ranger(); // Read the distance from the ranger
            printf("Range:			%d cm \r\n", range_val);
            printf("Pulse Width:	%d \r\n", MOTOR_PW);

            // Start a new ping
            Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
            i2c_write_data(addr, 0, Data, 1); // write one byte of data to reg 0 at addr
        }

        if (SS0) Drive_Motor(range_val);
        else Drive_Motor(45); // Hold the motor in neutral if the slide switch is off
    }
}


//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
//

unsigned int Read_Compass() {
    unsigned char addr = 0xC0; // Address the compass heading will be written to
    unsigned char Data[2]; // Data array to store heading data
    unsigned int heading; // Variable to store heading data
    i2c_read_data(addr, 2, Data, 2); // Read data from compass registers, store it in Data buffer
    heading = (((unsigned int) Data[0] << 8) | Data[1]); //Take high data byte, convert to int, 
    //shift left 8 bits, and copy lower compass byte to first half of int
    take_heading = 0;
    return heading; // Return data heading between 0 and 3599 
}


//-----------------------------------------------------------------------------
// Drive_Motor
//-----------------------------------------------------------------------------
//
// Vary the pulsewidth based on the user input to change the speed 
// of the drive motor.
//

void Drive_Motor(unsigned int input) {
    unsigned int motorPW; // Declare local variable

    if (input <= 10) MOTOR_PW = PW_MAX; // Motor at full forward

    else if (input >= 90) MOTOR_PW = PW_MIN; // Motor in full reverse

    else if (input >= 40 && input <= 50) MOTOR_PW = PW_NEUT; // Motor in neutral

    else if (input > 10 && input < 40) MOTOR_PW = ((PW_NEUT - PW_MAX) / 30) * (input - 10) + PW_MAX;

    else if (input > 50 && input < 90) MOTOR_PW = ((PW_MIN - PW_NEUT) / 40) * (input - 50) + PW_NEUT;

    //printf("Pulse Width = %d\r\n",MOTOR_PW);
    motorPW = 0xFFFF - MOTOR_PW;
    PCA0CPL2 = motorPW; // Set High and low byte for motor speed
    PCA0CPH2 = motorPW >> 8;
}

//-----------------------------------------------------------------------------
// read_ranger
//-----------------------------------------------------------------------------
//
// Read the value of the ultrasonic ranger
// 

unsigned char read_ranger(void) {
    unsigned int range = 0;
    unsigned char slave_reg = 2; //start at register 2
    unsigned char num_bytes = 2; //read 2 bytes

    i2c_read_data(addr, slave_reg, Data, num_bytes); // read two bytes, starting at reg 2
    range = (((unsigned int) Data[0] << 8) | Data[1]); // Store high and low bytes of Data in variable range
    return range;
}


//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//

void Port_Init() {
    P1MDOUT |= 0x04; // set output pin P1.2 for push-pull mode
    XBR0 = 0x27; // configure crossbar with UART, SPI, SMBus, and CEX channels 

    P3MDOUT &= ~0x40; // Set P3.6 to an input
    P3 |= 0x40;

    P0MDOUT &= ~0xC0; //(00XX XXXX) Set P0.6 and P0.7 Open Drain (Input)
    P0 |= 0xC0; //(11XX XXXX) Set P0.6 and P0.7 to High-Impedence Mode
    P1MDOUT |= 0x01; //set output pin for CEX0 at P1.0 to push pull
    P3MDOUT &= ~0x80; // Set input for slide switch on P3.7 Open Drain
    P3 |= ~0x80;
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
    PCA0MD = 0x81; // enable CF interupt, use SYSCLK/12
    PCA0CPM2 = 0xC2; // select 16bit PWM, enable positive edge capture, enable pulse width modulation
    PCA0CN = 0x40; // enable PCA0 counter



    PCA0MD |= 0x01; //Enable PCA overflow interrupt (bit 0)	
    PCA0MD &= ~0x0E; //Timer uses SYSCLK/12 (Bits 1-3)
    PCA0CPM0 = 0xC2; //Use 16bit counter (bit 7)
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up interrupts
//

void Interrupt_Init(void) {

    EIE1 |= 0x08; //Enable PCA0 Interrupt (bit 3) 
    EA = 1; //Enable global interrupts

}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Configure Magnetic Compass Interface
//

void SMB_Init(void) {
    SMB0CR = 0x93; //Configure SCL frequency
    SMB0CN = 0x40; //Enable SMBus



    ENSMB = 1; // Enable SMBus
}



//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//

void PCA_ISR(void) __interrupt 9 {//                                                          needs work
    if (CF) { //If an interrupt has occured
        interrupts++;
        if (interrupts >= 2) { //If two interrupts have occured
            interrupts = 0; //Reset interrupts
            take_heading = 1; //It is appropriate to take a reading
        }
        CF = 0; //Clear Interrupt Flag
        PCA0 = 0x7000; //Jump timer ahead for given period
    






        CF = 0; // clear overflow flag
        PCA0L = (unsigned char) 28672; // set low byte for 20ms period pulse
        PCA0H = 28672 >> 8; // set high byte for 20 ms period pulse
        c++; // counter for initial wait to initialize motor
        rWait++; // counter to set 80ms flag
        if (rWait >= 4) {
            getRange = 1; // 80ms flag
            rWait = 0; // Reset counter
        }
    }
    PCA0CN &= 0xC0; // Handle other PCA interrupts
}

//-----------------------------------------------------------------------------
// Steering_Servo
//-----------------------------------------------------------------------------
//

void Steering_Servo(unsigned int current_heading) {
    signed int error = 0;
    //signed int temp;
    error = HEADING - current_heading; // Calculate signed error
    if (error > 1800) { // If the error is greater than 1800
        error = 3600 % error; // or less than -1800, then the 
        error *= -1; // conjugate angle needs to be generated
    } else if (error < -1800) { // with opposite sign from the original
        error = 3600 % abs(error); // error
    }
    printf("\t%d\n\r", error); // Commented out unless testing
    PW = PROPORTION * error + PWCENTER; // Update PW based on error

    if (PW > PW_MAX) { // check if pulsewidth maximum exceeded
        PW = PW_MAX; // set PW to a maximum value
    } else if (PW < PWMIN) { // check if less than pulsewidth minimum
        PW = PWMIN; // set SERVO_PW to a minimum value
    }
}