/* Aaron Clippinger, Lance Gerday, Bradley Jewett, Mark Blanco
   Section 2 Side A Seat 5
   October 27, 2014
   Lab 4 */


#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define PROPORTION 0.417
//#define HEADING 900
#define MOTOR_PW_MIN 2030
#define MOTOR_PW_MAX 3500
#define MOTOR_PW_NEUT 2760


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
void ADC_Init(void); // Initialize A to D Conversion
unsigned char read_AD_input(void);
unsigned char read_ranger(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char interrupts;
unsigned char take_heading;
unsigned int reading;
unsigned int servo_PW_CENTER = 2905; // Center PW value
unsigned int servo_PW_MIN = 2385; // Minimum left PW value
unsigned int servo_PW_MAX = 3315; // Maximum right PW value
unsigned int servo_PW = 2905; // Start PW at center
unsigned int heading = 900;//set initial heading to 90 degrees
unsigned char final = 0;
__sbit __at 0xB7 SS_steer; // Slide switch input pin at P3.7

unsigned int MOTOR_PW = 0;
unsigned int c = 0;
unsigned char getRange = 1;
unsigned char rWait = 0;
unsigned int range_val = 0;
unsigned char Data[2];
unsigned char addr = 0xE0; // the address of the ranger is 0xE0
unsigned int motor_PW;

__sbit __at 0xB6 SS_range; // Assign P3.6 to SS (Slide Switch)

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
    MOTOR_PW = MOTOR_PW_NEUT;
    motor_PW = 0xFFFF - MOTOR_PW;
    PCA0CPL2 = motor_PW;
    PCA0CPH2 = motor_PW >> 8;
    printf("Pulse Width = %d\r\n", MOTOR_PW);
    c = 0;
    while (c < 50); //wait 1 second in neutral
    printf("end wait \r\n");



    //Main Functionality
    while (1) {
        if (!SS_steer) { // If the slide switch is active, set PW to center
            servo_PW_CENTER;
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
        } else if (take_heading) { // Otherwise take a new heading
            reading = Read_Compass(); // Get current heading
            printf("%d\n\r", reading);
            Steering_Servo(reading); // Change PW based on current heading
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
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

        if (!SS_range) Drive_Motor(range_val);
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

    if (input <= 10) MOTOR_PW = MOTOR_PW_MAX; // Motor at full forward

    else if (input >= 90) MOTOR_PW = MOTOR_PW_MIN; // Motor in full reverse

    else if (input >= 40 && input <= 50) MOTOR_PW = MOTOR_PW_NEUT; // Motor in neutral

    else if (input > 10 && input < 40) MOTOR_PW = ((MOTOR_PW_NEUT - MOTOR_PW_MAX) / 30) * (input - 10) + MOTOR_PW_MAX;

    else if (input > 50 && input < 90) MOTOR_PW = ((MOTOR_PW_MIN - MOTOR_PW_NEUT) / 40) * (input - 50) + MOTOR_PW_NEUT;

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
    P1MDOUT |= 0x05; // set output pin P1.2 and P1.0 for push-pull mode (CEX2 and CEX0)
    XBR0 = 0x27; // configure crossbar with UART, SPI, SMBus, and CEX channels 

    P3MDOUT &= ~0xC0; // Set P3.6 and 3.7 to inputs
    P3 |= 0xC0;

    P0MDOUT &= ~0xC0; //(00XX XXXX) Set P0.6 and P0.7 Open Drain (Input)
    P0 |= 0xC0; //(11XX XXXX) Set P0.6 and P0.7 to High-Impedence Mode
    //P3 |= ~0x80;
    
    
    
     // Port 1
    P1MDIN &= ~0x80;
    P1MDOUT &= ~0x80;
    P1 |= 0x80;
}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//
// initilize analog to digitial conversion
//
void ADC_Init(void) {
    REF0CN = 0x03; // Use internal reference voltage (2.4V)
    ADC1CN = 0x80; // Enable A/D conversion
    ADC1CF &= 0xFC; // Reset last two bits to 0
    ADC1CF |= 0x01; // Gain set to 1.0
}

//-----------------------------------------------------------------------------j'
// read_AD_input
//-----------------------------------------------------------------------------
//
// read analog input
//
unsigned char read_AD_input(void) {
    AMX1SL = 7; // Set pin 7 as the analog input
    ADC1CN &= ~0x20; // Clear 'conversion complete' flag
    ADC1CN |= 0x10; // Initiate A/D conversion

    while ((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete

    return ADC1; // Return digital conversion value
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



    //PCA0MD |= 0x01; //Enable PCA overflow interrupt (bit 0)	
    //PCA0MD &= ~0x0E; //Timer uses SYSCLK/12 (Bits 1-3)
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
        c++; // counter for initial wait to initialize motor
        rWait++; // counter to set 80ms flag
        if (rWait >= 4) {
            getRange = 1; // 80ms flag
            rWait = 0; // Reset counter
        }
        //Portion for compass 
        if (interrupts >= 2) { //If two interrupts have occured
            interrupts = 0; //Reset interrupts
            take_heading = 1; //It is appropriate to take a reading
        }
        CF = 0; //Clear Interrupt Flag
        PCA0 = 28672; //Jump timer ahead for given period
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
    error = heading - current_heading; // Calculate signed error
    if (error > 1800) { // If the error is greater than 1800
        error = 3600 % error; // or less than -1800, then the 
        error *= -1; // conjugate angle needs to be generated
    } else if (error < -1800) { // with opposite sign from the original
        error = 3600 % abs(error); // error
    }
    printf("\t%d\n\r", error); // Commented out unless testing
    servo_PW = PROPORTION * servo_PW_CENTER; // Update PW based on error

    if (servo_PW > MOTOR_PW_MAX) { // check if pulsewidth maximum exceeded
        servo_PW = MOTOR_PW_MAX; // set PW to a maximum value
    } else if (servo_PW < servo_PW_MIN) { // check if less than pulsewidth minimum
        servo_PW = servo_PW_MIN; // set SERVO_PW to a minimum value
    }
}
