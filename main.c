#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "SysTickInitialization.h"
#include "LCDinitialization.h"

/**
 * main.c
 * Summary: This code creates a home security system for a model home. It uses various
 *          electronic elements such as a liquid crystal display, a speaker, a motor and
 *          multiple light emitting diodes. The microcontroller used was a TI MSP 432.
 * Author: Wesley Luna, Even Bauer, with the assistance of Scott Zuidema
 * Date: November 1st, 2018
 */

/*
LED PINS
BLUE  P7.6
RED   P7.5
GREEN P7.4

Speaker
P5.7

*/
//////////////////////////////////
/// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet

void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
void setupPorts(); // Sets up P1.0 as an output to drive the on board LED
void setupSerial(); // Sets up serial for use and enables interrupts

void changeLED(char color, int percent);
void redBrightness(int dutyCycle);
void blueBrightness(int dutyCycle);
void greenBrightness(int dutyCycle);
void initializePWM();
///////////////////////////////


void menu(); //Goes to menu

void initializePWM(); //Initializes timer and pins related to pwm


//LED
//void setBlueBrightness();  //Sets blue LED brightness
//void setRedBrightness();    //Sets red LED brightness
//void setGreenBrightness();  //Sets green LED brightness
void lightInterrupt();  //function to create a light switch


//SPEAKER
void alarm(int on); //sounds alarm
void TimerA2config(void); // initializes timer A2
void SpeakerConfig(void); // initializes the speaker


void initT32();  //initializes TIMER32_1

//INTERRUPTS

void initInterruptPins();  //enables interrupts for pins


//Global variables
int blueBright = 100;  //blue LED's brightness
int redBright = 100;   //red LED's brightness
int greenBright = 100; //green LED's brightness
int armed = 0;  //flag to check if the door is armed
int open = 0; //flag to check if the door is open
int savedPin = -1;  //holds the saved pin value
int lightOn = 0; //flag to check if the lights are on

/*
 * Initializes everything
 */
/*
void main(void)

{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    initSysTick();
    initPins();
    initInterruptPins();
    delayMicro(100);
    LCDInit();
    delayMicro(100);
    initializePWM();
    initT32();
    TimerA2config();
    SpeakerConfig();
    NVIC_EnableIRQ(PORT2_IRQn);
    NVIC_EnableIRQ(PORT3_IRQn);
    __enable_interrupt();
    menu(); //main menu
}*/

int main(void)
{
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
    WDT_A->CTL = WDT_A_CTL_PW |  WDT_A_CTL_HOLD; // Stop watchdog timer

    setupPorts();
    setupSerial();
    initializePWM();
    INPUT_BUFFER[0]= '\0';  // Sets the global buffer to initial condition of empty

    __enable_irq();  // Enable all interrupts (serial)

    int valid = 0;
    int val1, val2, val3;
    char c;
    while(1) {
        val1 = 0;
        val2 = 0;
        val3 = 0;
        readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
        if(string[0] != '\0'){ // if string is not empty, check the inputted data.
            if(!(strcmp(string,"ON"))) // If command is "ON", turn on LED
            {
                P1->OUT |= BIT0;
                writeOutput("Turned P1.0 On\n");
            }
            if(!(strcmp(string,"OFF"))) // If command is "OFF", turn off LED
            {
                P1->OUT &= ~BIT0;
                writeOutput("Turned P1.`0 Off\n");
            }
            valid = 0;
            if(string[0] == 'R' || string[0] == 'B' || string[0] == 'G'){

                c=string[0];

                //convert character to integer
                val1 = string[1] - '0';
                val2 = string[2] - '0';
                val3 = string[3] - '0';

                //converts to percentage
                val1*=100;
                val2*=10;
                val3+= val1 + val2;
                if(val3 >= 0 && val3 <=100)
                {
                    valid = 1;
                }
            }
            if(valid){
                writeOutput("Valid ");
                changeLED(c,val3);
                writeOutput(string);
            }else{
                writeOutput("Invalid ");
                writeOutput(string);
            }
        }
   }
}

/*----------------------------------------------------------------
 * void menu()
 *
 * Description: The main menu. Allows users to pick which functions
 *              they want to run
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void menu()
{
    //variables to print strings
    char line0[] = "      Menu";
    char line1[] = "1. Door";
    char line2[] = "2. Motor";
    char line3[] = "3. Lights";
    char line4[] = "4. arm/disarm";
    char line5[] = "5. Set password";
    char line6[] = "6. Screen saver";
    char line7[] = "*. Continue";

    //other variables
    int i, key, flag, flag2;

    //prints info
    resetLCD();
    for(i = 0; i < strlen(line0); i++)
    {
        dataWrite(line0[i]);
    }
    delaySeconds(1);
}

//
///*----------------------------------------------------------------
// * void setBlueBrightness()
// *
// * Description: Saves duty cycle to blueBright and changes the
// *              LED brightness
// * Inputs: None
// * Outputs: None
//----------------------------------------------------------------*/
//void setBlueBrightness()
//{
//    unsigned int dutyCycle;
//    if(dutyCycle > 100)
//    {
//        dutyCycle = 100;
//    }
//    blueBright = dutyCycle;
//    TIMER_A1->CCR[2] = (int)((dutyCycle /100.00) * 60000);
//}
//
///*----------------------------------------------------------------
// * void setRedBrightness()
// *
// * Description: Saves duty cycle to redBright and changes the
// *              LED brightness
// * Inputs: None
// * Outputs: None
//----------------------------------------------------------------*/
//void setRedBrightness()
//{
//    unsigned int dutyCycle;
//    if(dutyCycle > 100)
//    {
//        dutyCycle = 100;
//    }
//    redBright = dutyCycle;
//    TIMER_A1->CCR[3] = (int)((dutyCycle /100.00) * 60000);
//}
//
///*----------------------------------------------------------------
// * void setGreenBrightness()
// *
// * Description: Saves duty cycle to greenBright and changes the
// *              LED brightness
// * Inputs: None
// * Outputs: None
//----------------------------------------------------------------*/
//void setGreenBrightness()
//{
//    unsigned int dutyCycle;
//    if(dutyCycle > 100)
//    {
//        dutyCycle = 100;
//    }
//    greenBright = dutyCycle;
//    TIMER_A1->CCR[4] = (int)((dutyCycle /100.00) * 60000);
//}
//

/*----------------------------------------------------------------
 * void initializePWM()
 *
 * Description: initializes the PWM pins and timers
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
/*
void initializePWM()
{
    P2->SEL0 |= BIT4|BIT5;
    P2->SEL1 &= ~BIT4|BIT5;  // Setting SEL0 = 1 and SEL1 = 0 will activate the secondary function of these pins.
    P2->DIR |=BIT4|BIT5;    // Set pins as outputs.  Also required for PWM output.
    P2->OUT &= ~BIT4|BIT5;   // output.

    TIMER_A0->CCR[0] = 37500;  //Running at 40 Hz

    TIMER_A0->CCTL[1] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCR[1] = 36500;//100 % duty cycle
    TIMER_A0->CCTL[2] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCR[2] = 36500;//100 % duty cycle
    TIMER_A0->CTL = 0b0000001001010100;//input divider by /2

    P7->SEL0 |= (BIT4|BIT5|BIT6|BIT7);
    P7->SEL1 &= ~(BIT4|BIT5|BIT6|BIT7);;  // Setting SEL0 = 1 and SEL1 = 0 will activate the secondary function of these pins.
    P7->DIR |=(BIT4|BIT5|BIT6|BIT7);;    // Set pins as outputs.  Also required for PWM output.
    P7->OUT &= ~(BIT4|BIT5|BIT6|BIT7);;   // output.

    TIMER_A1->CCR[0] = 60000;  //Running at 50 Hz
    TIMER_A1->CCTL[1] = 0b0000000011100000;  //reset / set
    TIMER_A1->CCTL[2] = 0b0000000011100000;  //reset / set
    TIMER_A1->CCTL[3] = 0b0000000011100000;  //reset / set
    TIMER_A1->CCTL[4] = 0b0000000011100000;  //reset / set
    TIMER_A1->CCR[1] = 60000 * (1/20);//closed
    TIMER_A1->CCR[2] = 60000;//Blue LED on
    TIMER_A1->CCR[3] = 60000;//Red LED on
    TIMER_A1->CCR[4] = 60000;//Green LED on
    TIMER_A1->CTL = 0b0000001000010100;
}
*/

/*----------------------------------------------------------------
 * void alarm(int on)
 *
 * Description: Displays a message on the LCD and toggles/sounds alarm
 * Inputs: an integer flag that determines whether the alarm should sound
 * Outputs: Sound to speaker
----------------------------------------------------------------*/
void alarm(int on)
{
    //variables to print strings
    int i;
    char str[] = "Alarm on";
    char str2[] = "Alarm off";
    if(on)
    {
        //sound alarm
        TIMER_A2->CCR[2]    =    5700;
        resetLCD();
        for(i = 0; i < strlen(str); i++)
        {
            dataWrite(str[i]);
        }
        delaySeconds(1);
    }else{
        //turn off alarm
        TIMER_A2->CCR[2]    =    0;
        resetLCD();
        for(i = 0; i < strlen(str2); i++)
        {
            dataWrite(str2[i]);
        }
        delaySeconds(1);
    }
}

/*----------------------------------------------------------------
 * void PORT3_IRQHandler(void)
 *
 * Description: interrupt handler for the light switch button
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void PORT3_IRQHandler(void)
{
    if(P3 -> IFG & BIT2)
    {
        lightInterrupt(); //light switch
        P3 -> IFG &= ~BIT2;
    }
    return;
}

/*----------------------------------------------------------------
 * void PORT2_IRQHandler(void)
 *
 * Description: interrupt handler for the motor button
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void PORT2_IRQHandler(void)
{
    if(P2 -> IFG & BIT6)
    {
        P2 -> IFG &= ~BIT6;
    }
    return;
}

/*----------------------------------------------------------------
 * void initInterruptPins()
 *
 * Description: initializes button interrupts for the motor and lights
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void initInterruptPins()
{
    //LIGHT SWITCH
    P3 -> SEL0 &= ~BIT2;
    P3 -> SEL1 &= ~BIT2;
    P3 -> DIR &= ~BIT2;
    P3 -> REN |= BIT2;
    P3 -> OUT |= BIT2;
    P3 -> IE |= BIT2;
    P3 -> IES |= BIT2;
    P3 -> IFG &= ~BIT2;

    //EMERGENCY STOP FOR MOTOR
    P2 -> SEL0 &= ~BIT6;
    P2 -> SEL1 &= ~BIT6;
    P2 -> DIR &= ~BIT6;
    P2 -> REN |= BIT6;
    P2 -> OUT |= BIT6;
    P2 -> IE |= BIT6;
    P2 -> IES |= BIT6;
    P2 -> IFG &= ~BIT6;

}

/*----------------------------------------------------------------
 * void lightInterrupt()
 *
 * Description: Turns off the red, blue, and green LEDs if their on.
 *              Turn them on to their previous value if their off.
 * Inputs: None
 * Outputs: Sets LED to their corresponding values based on the conditions
----------------------------------------------------------------*/
void lightInterrupt()
{
    if(lightOn)
    {
        //turns lights off
        TIMER_A1->CCR[2] = 0;
        TIMER_A1->CCR[3] = 0;
        TIMER_A1->CCR[4] = 0;
        lightOn = 0;
    }else
    {
        //turns light on
        TIMER_A1->CCR[2] = (int)(blueBright/100.0)* 6000; //blue
        TIMER_A1->CCR[3] = (int)(redBright/100.0)* 6000;; //red
        TIMER_A1->CCR[4] = (int)(greenBright/100.0)* 6000;; //green
        lightOn = 1;
    }
}




/*----------------------------------------------------------------
 * void initT32()
 *
 * Description: Initializes TIMER32_1
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void initT32()
{
    TIMER32_1->CONTROL = 0b11000011;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
    TIMER32_1->LOAD = 30000000 - 1;   //10 seconds           //Set to a count down of 1 second on 3 MHz clock
}

/*----------------------------------------------------------------
 * void TimerA2config(void)
 *
 * Description: Initializes TIMER_A2
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void TimerA2config(void)
{
    TIMER_A2->CCR[0]    =    5714;                    // Initialize the period of TimerA0 PWM to the maximum.  This will change at the first interrupt.  Could be set to something else.
    TIMER_A2->CCR[2]    =    (0);                         // Will reset (set to 0) immediately, so it will always be off by default
    TIMER_A2->CCTL[2]   =    0b11100000;                // Set to Reset/set Compare Mode (BITs 7-5 set to 1)
    TIMER_A2->CTL       =    0b1000010100;              // Bits 9-8 = 10 to Set to SMCLK
                                                        // Bits 5-4 = 01 to Set to Count Up Mode
                                                        // Bit 2 to 1 to Clear and Load Settings.
}

/*----------------------------------------------------------------
 * void SpeakerConfig(void)
 *
 * Description: Set Speaker pin to PWM controlled by Timer_A0
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void SpeakerConfig(void)
{
    P5->SEL0            |=   BIT7;                      // Configure P2.4 to TIMER_A0 PWM Control
    P5->SEL1            &=  ~BIT7;                      // SEL = 01 sets to PWM Control
    P5->DIR             |=   BIT7;                      // Initialize speaker pin as an output
}

/*----------------------------------------------------------------
 * Description:
 * Example code of using the serial port on the MSP432.
 * Receives commands of either ON or OFF from the serial port to
 * turn on or off the P1.0 on board LED.
 * ---------------------------------------------------------------
 * Revision History:
 * ---------------------------------------------------------------
 * Initial Release
 * Scott Zuidema
 * 11/2/2018
 *
 * Update 1:
 * Scott Zuidema
 * 11/3/2018
 * Changed to 115200 8E2 to make Lab 10 more difficult.
 * Added writeOutput() function.
----------------------------------------------------------------*/





/*----------------------------------------------------------------
 * void writeOutput(char *string)
 *
 * Description:  This is a function similar to most serial port
 * functions like printf.  Written as a demonstration and not
 * production worthy due to limitations.
 * One limitation is poor memory management.
 * Inputs: Pointer to a string that has a string to send to the serial.
 * Outputs: Places the data on the serial output.
----------------------------------------------------------------*/
void writeOutput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to

    while(string[i] != '\0') {
        EUSCI_A0->TXBUF = string[i];
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
    }
}

/*----------------------------------------------------------------
 * void readInput(char *string)
 *
 * Description:  This is a function similar to most serial port
 * functions like ReadLine.  Written as a demonstration and not
 * production worthy due to limitations.
 * One of the limitations is that it is BLOCKING which means
 * it will wait in this function until there is a \n on the
 * serial input.
 * Another limitation is poor memory management.
 * Inputs: Pointer to a string that will have information stored
 * in it.
 * Outputs: Places the serial data in the string that was passed
 * to it.  Updates the global variables of locations in the
 * INPUT_BUFFER that have been already read.
----------------------------------------------------------------*/
void readInput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to

    // One of the few do/while loops I've written, but need to read a character before checking to see if a \n has been read
    do
    {
        // If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
        while(read_location == storage_location && INPUT_BUFFER[read_location] != '\n');
        string[i] = INPUT_BUFFER[read_location];  // Manual copy of valid character into "string"
        INPUT_BUFFER[read_location] = '\0';
        i++; // Increment the location in "string" for next piece of data
        read_location++; // Increment location in INPUT_BUFFER that has been read
        if(read_location == BUFFER_SIZE)  // If the end of INPUT_BUFFER has been reached, loop back to 0
            read_location = 0;
    }
    while(string[i-1] != '\n'); // If a \n was just read, break out of the while loop

    string[i-1] = '\0'; // Replace the \n with a \0 to end the string when returning this function
}

/*----------------------------------------------------------------
 * void EUSCIA0_IRQHandler(void)
 *
 * Description: Interrupt handler for serial communication on EUSCIA0.
 * Stores the data in the RXBUF into the INPUT_BUFFER global character
 * array for reading in the main application
 * Inputs: None (Interrupt)
 * Outputs: Data stored in the global INPUT_BUFFER. storage_location
 * in the INPUT_BUFFER updated.
----------------------------------------------------------------*/
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
    {
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF; // store the new piece of data at the present location in the buffer
        EUSCI_A0->IFG &= ~BIT0; // Clear the interrupt flag right away in case new data is ready
        storage_location++; // update to the next position in the buffer
        if(storage_location == BUFFER_SIZE) // if the end of the buffer was reached, loop back to the start
            storage_location = 0;
    }
}

/*----------------------------------------------------------------
 * void setupP1()
 * Sets up P1.0 as a GPIO output initialized to 0.
 *
 * Description:
 * Inputs: None
 * Outputs: Setup of P.1 to an output of a 0.
----------------------------------------------------------------*/
void setupPorts()
{
    P1->SEL0 &= ~BIT0; //GPIO
    P1->SEL1 &= ~BIT0;
    P1->DIR  |=  BIT0; //OUTPUT
    P1->OUT  &= ~BIT0; //Start as off
}

/*----------------------------------------------------------------
 * void setupSerial()
 * Sets up the serial port EUSCI_A0 as 115200 8E2 (8 bits, Even parity,
 * two stops bit.)  Enables the interrupt so that received data will
 * results in an interrupt.
 * Description:
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void setupSerial()
{
    P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
    P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.

    EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
    EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10
    EUSCI_A0->CTLW0 |= (BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT14 = Even, BIT11 = Two Stop Bits
    // Baud Rate Configuration
    // 3000000/(16*115200) = 1.628  (3 MHz at 115200 bps is fast enough to turn on over sampling (UCOS = /16))
    // UCOS16 = 1 (0ver sampling, /16 turned on)
    // UCBR  = 1 (Whole portion of the divide)
    // UCBRF = .628 * 16 = 10 (0x0A) (Remainder of the divide)
    // UCBRS = 3000000/115200 rOFFFemainder=0.04 -> 0x01 (look up table 22-4)
    EUSCI_A0->BRW = 1;  // UCBR Value from above
    EUSCI_A0->MCTLW = 0x01A1; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)

    EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
    EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
    EUSCI_A0->IE |= BIT0;      // Enable interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}

void changeLED(char color, int percentage)
{
    if(color == 'R')
    {
        redBrightness(percentage);
    }

    if(color == 'B')
    {
        blueBrightness(percentage);
    }

    if(color == 'G')
    {
        greenBrightness(percentage);
    }
}

void redBrightness(int dutyCycle){
    if(dutyCycle > 100){
        dutyCycle = 100;
    }else if(dutyCycle < 0){
        dutyCycle = 0;
    }
    TIMER_A0->CCR[2] = (int)((dutyCycle /100.00) * 999);
}

void blueBrightness(int dutyCycle){
   if(dutyCycle > 100){
           dutyCycle = 100;
   }else if(dutyCycle < 0){
       dutyCycle = 0;
   }
   TIMER_A0->CCR[1] = (int)((dutyCycle /100.00) * 999);
}

void greenBrightness(int dutyCycle){
   if(dutyCycle > 100){
       dutyCycle = 100;
   }else if(dutyCycle < 0){
       dutyCycle = 0;
   }
   TIMER_A0->CCR[3] = (int)((dutyCycle /100.00) * 999);
}

/*----------------------------------------------------------------
 * void initializePWM()
 *
 * Description: initializes the PWM pins and timers
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void initializePWM(){
    P2->SEL0 |= (BIT4|BIT5|BIT6);
    P2->SEL1 &= ~(BIT4|BIT5|BIT6);  // Setting SEL0 = 1 and SEL1 = 0 will activate the secondary function of these pins.
    P2->DIR |=(BIT4|BIT5|BIT6);    // Set pins as outputs.  Also required for PWM output.
    P2->OUT &= ~(BIT4|BIT5|BIT6);   // output.

    TIMER_A0->CTL = 0b0000001000010100;
    TIMER_A0->CCR[0] = 999;  //Running at 60 Hz
    TIMER_A0->CCTL[1] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCTL[2] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCTL[3] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCR[1] = 999*(1/2.0);//Blue LED on
    TIMER_A0->CCR[2] = 999*(1/2.0);//Red LED on
    TIMER_A0->CCR[3] = 999*(1/2.0);//Green LED on
}
