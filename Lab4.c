/* Aaron Clippinger, Lance Gerday, Bradley Jewett, Mark Blanco
   Section 2 Side A Seat 5
   October 27, 2014
   Lab 4 */


#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define MAX_RANGE 60
#define MOTOR_PW_MIN 2030
#define MOTOR_PW_MAX 3500
#define MOTOR_PW_NEUT 2760
#define R_ADDR 0xE0
#define C_ADDR 0xC0
#define SPEED  6//half speed


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
void Check_Menu(void);
void Load_Menu(void);


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char interrupts;
unsigned char take_heading;
unsigned int servo_PW_CENTER = 2905; // Center PW value
unsigned int servo_PW_MIN = 2385; // Minimum left PW value
unsigned int servo_PW_MAX = 3315; // Maximum right PW value
unsigned int servo_PW = 2905; // Start PW at center

unsigned int desired_heading = 900; //set initial heading to 90 degrees

unsigned int compass_error;
unsigned int compass_val;
float compass_gain = 0.417;
float voltage;

unsigned int MOTOR_PW = 0;
unsigned int c = 0;
unsigned char getRange = 1;
unsigned int range_val = 0;
unsigned char Data[2];
unsigned int motorPW;

float range_gain = 15;
unsigned int range_adj;

__sbit __at 0xB6 SS_range; // Assign P3.6 to SS (Slide Switch)
__sbit __at 0xB7 SS_steer; // Slide switch input pin at P3.7

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
    ADC_Init();
    Interrupt_Init();
    printf("Starting\n\r");

    //print beginning message
    printf("Embedded Control Drive Motor Control\r\n");
    
    // Initialize motor in neutral and wait for 1 second
    Drive_Motor(0);
    c = 0;
    while (c < 50); //wait 1 second in neutral
	c = 0;
    printf("end wait \r\n");
	Load_Menu();

    //Main Functionality
    while (1) {
        if (SS_steer) { // If the slide switch is active, set PW to center
            servo_PW = servo_PW_CENTER;
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
        } else if (take_heading) { // Otherwise take a new heading
			compass_val = Read_Compass();
            Steering_Servo(compass_val); // Change PW based on current heading
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
        }


        if (getRange) {
            getRange = 0; // Reset 80 ms flag
            range_val = read_ranger(); // Read the distance from the ranger
            printf("Range:			%d cm \r\n", range_val);
            //printf("Pulse Width:	%d \r\n", MOTOR_PW);


            // range is the value from the ultrasonic ranger


            if (range_val > MAX_RANGE) range_adj = 0; //no obstacle in range, no change
            else range_adj = (int) (range_gain * (MAX_RANGE - range_val)); //find adjustment
      

            // compass_adj is the compass heading error multiplied by its error gain
            //servo_PW = servo_PW_CENTER + compass_adj + range_adj; //use both to adjust steering


            // Start a new ping
            Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
            i2c_write_data(R_ADDR, 0, Data, 1); // write one byte of data to reg 0 at R_ADDR
        }
        if (SS_range) Drive_Motor(0); // Hold the motor in neutral if the slide switch is active
        else Drive_Motor(SPEED);
		if (c >= 50){
			//Print Serial Output for data collection
			printf("Error: %d  Heading: %d  Steering PW: %d  Adjustment: %d", compass_error, compass_val, servo_PW, range_adj);

			// Print the battery voltage (from AD conversion);
			voltage = read_AD_input();
			voltage /= 256;
			voltage *= 15.6;
			printf_fast_f("Battery voltage is: %.2f\n\r", voltage);
			Load_Menu();
			c = 0;
		}
		while (SS_range && SS_steer){
			Drive_Motor(0);
			servo_PW = servo_PW_CENTER;
            PCA0CP0 = 0xFFFF - servo_PW;
			Check_Menu();
			c = 0;
			while(c < 5){}
			c = 0;
		}
	}
}


//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
//

unsigned int Read_Compass() {
    unsigned char c_Data[2]; 	// c_Data array to store heading data
    unsigned int heading; 		// Variable to store heading data
    i2c_read_data(C_ADDR, 2, c_Data, 2); // Read data from compass registers, store it in c_Data buffer
    heading = (((unsigned int) c_Data[0] << 8) | c_Data[1]); //Take high c_data byte, convert to int, 
    //shift left 8 bits, and copy lower compass byte to first half of int
    take_heading = 0;
    return heading; // Return C_Data heading between 0 and 3599 
}


void Check_Menu() {
	signed char menu_input = read_keypad();		//Determine pressed button on keypad (if any)
	unsigned int keypad_input;

	if ((menu_input - '0') == 1) {				//If compass gain is selected
		printf("Please enter a 5 digit gain constant (of the form: xx.xxx) \n\r");
		lcd_clear();
		lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
		while (read_keypad() != -1);
		keypad_input = kpd_input(1);
		compass_gain = keypad_input * 0.001;
		printf_fast_f("New gain is %f\n\r",compass_gain);
		Load_Menu();
	}
	else if ((menu_input - '0') == 2) {			//If ranger gain is selected
		printf("Please enter a 5 digit gain constant (of the form: xx.xxx) \n\r");
		lcd_clear();
		lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
		while (read_keypad() != -1);
		keypad_input = kpd_input(1);
		range_gain = keypad_input * 0.001;
		printf_fast_f("New gain is %f\n\r",range_gain);
		Load_Menu();
	}
	else if ((menu_input - '0') == 3) {			//If desired heading is selected
		printf("Please choose an option: \n\r");
		printf("1: 0 degrees\n\r2: 90 degrees\n\r3: 180 degrees\n\r4: 270 degrees\n\r5: Enter a value\n\r");	//Print menu on terminal output
		lcd_clear();
		lcd_print("\n1.0 deg   2.90 deg\n3.180 deg 4.270 deg\n5.Enter a value");		//Print menu on
		while(read_keypad()!=-1);
		menu_input = read_keypad();
		while (menu_input == -1) menu_input = read_keypad();
		if ((menu_input - '0') == 1) {			//For 0 degrees
			desired_heading = 0;
		}
		else if ((menu_input - '0') == 2) {		//For 90 degrees
			desired_heading = 900;
		}
		else if ((menu_input - '0') == 3) {		//For 180 degrees
			desired_heading = 1800;
		}
		else if ((menu_input - '0') == 4) {		//For 270 degrees
			desired_heading = 2700;
		}
		else if ((menu_input - '0') == 5) {		//For enter own value
			printf("Please enter a 5 digit compass heading (of the form: 0xxxx) \n\r");
			lcd_clear();
			lcd_print("\nEnter a 5 digit\nheading (0xxxx)\n\r");
			while(read_keypad() == -1);
			keypad_input = kpd_input(1);
			desired_heading = keypad_input%3600;
		}
		printf("New heading is %d\n\r",desired_heading);
		Load_Menu();
	}
}


void Load_Menu(void){
	lcd_clear();
	lcd_print("1. Compass Gain\n");
	lcd_print("2. Ranger Gain\n");
	lcd_print("3. Desired Heading\n");
	lcd_print("R:%3d H:%4d B:%2d\n", range_val, compass_val, (int)voltage);
}



//-----------------------------------------------------------------------------
// Drive_Motor
//-----------------------------------------------------------------------------
//
// Vary the pulsewidth based on the user input to change the speed 
// of the drive motor.
//

void Drive_Motor(unsigned int input) {

    MOTOR_PW = ((MOTOR_PW_MAX - MOTOR_PW_NEUT) / 10) * (input) + MOTOR_PW_NEUT;

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

    i2c_read_data(R_ADDR, slave_reg, Data, num_bytes); // read two bytes, starting at reg 2
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
    XBR0 = 0x27; // configure crossbar with UART, SPI, SMBus, and CEX channels 
    P1MDOUT |= 0x05; // set output pin P1.2 and P1.0 for push-pull mode (CEX2 and CEX0)

    // Port 1 ADC
    P1MDIN &= ~0x80; //set P1.7 to Analog input
    P1MDOUT &= ~0x80; //set P1.7 to open drain mode
    P1 |= 0x80; //set P1.7 to high impedance

    P3MDOUT &= ~0xC0; // Set P3.6 and 3.7 to inputs
    P3 |= 0xC0; //set P3.6 and 3.7 to high impedance

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
    PCA0MD = 0x81; // enable CF interrupt, use SYSCLK/12

    PCA0CN = 0x40; // enable PCA0 counter
    PCA0CPM2 = 0xC2; // select 16bit PWM, enable positive edge capture, enable pulse width modulation(ranger)
    PCA0CPM0 = 0xC2; // select 16bit PWM, enable positive edge capture, enable pulse width modulation(compass)
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
    SMB0CR = 0x93; //Configure SCL frequency to 100kHz
    ENSMB = 1; // Enable SMBus
}



//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//

void PCA_ISR(void) __interrupt 9 {
    if (CF) { //If an interrupt has occured
        interrupts++;
        c++; // counter for initial wait to initialize motor
		if (interrupts % 2 == 0) {
       		take_heading = 1; //It is appropriate to take a reading
        }
        if (interrupts >= 4) {
            getRange = 1; // 80ms flag
			interrupts = 0; // Reset counter
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
    compass_error = desired_heading - current_heading; // Calculate signed error
    if (compass_error > 1800) { // If the error is greater than 1800
        compass_error = 3600 % compass_error; // or less than -1800, then the 
        compass_error *= -1; // conjugate angle needs to be generated
    } else if (compass_error < -1800) { // with opposite sign from the original
        compass_error = 3600 % abs(compass_error); // error
    }
    //printf("%d\n\r",current_heading);
    //printf("\t%d\n\r", error); 					// Commented out unless testing
    servo_PW = compass_gain * compass_error + range_adj + servo_PW_CENTER; // Update PW based on error and distance to obstacle
    if (servo_PW > servo_PW_MAX) { // check if pulsewidth maximum exceeded
        servo_PW = servo_PW_MAX; // set PW to a maximum value
    } else if (servo_PW < servo_PW_MIN) { // check if less than pulsewidth minimum
        servo_PW = servo_PW_MIN; // set SERVO_PW to a minimum value
    }
}
