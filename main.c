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

void printTime();
void readAlarm();
void readTime();
void setAlarm(int hour, int minute);
void setTime(int hour, int minute, int second);
int getTemp();
///////////////////////////////
//alarm
int AHOUR, AMIN, alarm = 1;
//time
int HOUR, MIN, SEC;
uint16_t temp=0;
void menu(); //Goes to menu

void initializePWM(); //Initializes timer and pins related to pwm


//LED
//void setBlueBrightness();  //Sets blue LED brightness
//void setRedBrightness();    //Sets red LED brightness
//void setGreenBrightness();  //Sets green LED brightness
void lightInterrupt();  //function to create a light switch


//SPEAKER
//void alarm(int on); //sounds alarm
void TimerA2config(void); // initializes timer A2
void SpeakerConfig(void); // initializes the speaker
void ADC14init(void);

void initT32();  //initializes TIMER32_1

//INTERRUPTS

void initInterruptPins();  //enables interrupts for pins
void ADC14_IRQHandler(void);

//Global variables
int blueBright = 100;  //blue LED's brightness
int redBright = 100;   //red LED's brightness
int greenBright = 100; //green LED's brightness


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
    int hour, minute, second,i;
    char function[20];
    //remove while loop after testing
    while(1) {
        hour = 0;
        minute = 0;
        second = 0;
        readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
        if(string[0] != '\0'){ // if string is not empty, check the inputted data.
            if(!(strcmp(string,"READALARM")))
            {
                readAlarm();
            }else if(!(strcmp(string,"READTIME")))
            {
                readTime();
            }else{


                for(i=0; string[i] != ' '; i++)
                {
                    function[i] = string[i];
                }
                function[i] = '\0';

                //convert string to time
                i++;
                hour = string[i] - '0';
                hour*=10;
                i++;
                hour += string[i] - '0';
                i+=2;

                minute = string[i] - '0';
                minute*=10;
                i++;
                minute += string[i] - '0';
                i+=2;

                second = string[i] - '0';
                second*=10;
                i++;
                second += string[i] - '0';

                if(hour<=24 && hour>=0){
                    if(minute <= 60 && minute >=0)
                        if(second <= 60 && second >=0)
                            valid = 1;
                }
                if(valid){

                    writeOutput("Valid\n");

                    if(!(strcmp(function,"SETTIME")))
                    {
                        setTime(hour, minute, second);

                    }else if(!(strcmp(function,"SETALARM")))
                    {
                        setAlarm(hour, minute);
                    }

                }else{
                    writeOutput("Invalid\n");
                    //writeOutput(string);
                }
            }
        }
   }
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

void readAlarm(){
    printf("read alarm\n");
    char buff[20];
    writeOutput("Alarm is ");
    sprintf(buff, "2%d:2%d%c", AHOUR, AMIN, '/0');
    writeOutput(buff);
}

void readTime(){
    printf("read time\n");
    char buff[20];
    writeOutput("Time is ");
    sprintf(buff, "2%d:2%d:2%d%c", HOUR, MIN, SEC, '/0');
    writeOutput(buff);
}
void setAlarm(int hour, int minute){
    printf("set alarm\n");
    AHOUR = hour;
    AMIN = minute;
    char buff[20];
    writeOutput("Alarm set to ");
    sprintf(buff, "2%d:2%d%c", AHOUR, AMIN, '/0');
    writeOutput(buff);
}
void setTime(int hour, int minute, int second){
    printf("set time\n");
    HOUR = hour;
    MIN = minute;
    SEC = second;
    char buff[20];
    writeOutput("Time set to ");
    sprintf(buff, "2%d:2%d:2%d%c", HOUR, MIN, SEC, '/0');
    writeOutput(buff);
}

//
void printTime(){
   char line1[20];
   char line2[20];
   char line3[20];
   char line4[20];
   int i, n;
   resetLCD();
   if(HOUR < 12 || HOUR == 24)
   {
       sprintf(line1, "%d:2%d:2%d AM%c", HOUR, MIN, SEC, '/0');
   }else{
       sprintf(line1, "%d:2%d:2%d PM%c", HOUR, MIN, SEC, '/0');
   }

   if(alarm == 2)
  {
      sprintf(line2,"SNOOZE%c", '/0');
  }else if(alarm == 1){
      sprintf(line2,"ON%c", '/0');
  }else{
      sprintf(line2,"OFF%c", '/0');
  }

   if(AHOUR < 12 || AHOUR == 24)
   {
       sprintf(line3, "%d:2%d AM%c", AHOUR, AMIN, '/0');
   }else{
       sprintf(line3, "%d:2%d PM%c", AHOUR, AMIN, '/0');
   }

   sprintf(line4,"%.1d%c", getTemp(), '/0');
   n = strlen(line1);
   for(i=0;i<n;i++)
   {
       dataWrite(line1[i]);
   }

   commandWrite(0xC0);
   delayMicro(100);
   commandWrite(0x0F);
   n = strlen(line2);
   for(i=0;i<n;i++)
   {
       dataWrite(line2[i]);
   }

   commandWrite(0x90);
   delayMicro(100);
   commandWrite(0x0F);
   n = strlen(line3);
   for(i=0;i<n;i++)
   {
       dataWrite(line3[i]);
   }

  commandWrite(0xC0+0x10);
  delayMicro(100);
  commandWrite(0x0F);
  n = strlen(line4);
  for(i=0;i<n;i++)
  {
      dataWrite(line4[i]);
  }
  dataWrite(0b11011111);
  dataWrite('F');

}

//remember that this was done with a timer. fix later
int getTemp(){
    float result_temp;
     uint16_t result;

//    while(1)
//    {
        ADC14->CTL0 |=0b1;
        result = temp;
        result_temp = ((result*3.3)/16384);
        result_temp = (result_temp * 1000 - 500)/10;
        result_temp = 32 + (result_temp * 9.0/5.0);
        return result_temp;

//    }

}


/*----------------------------------------------------------------
 * void ADC14init(void)
 *
 * Description: Function will set up the ADC14 to run in single
 * measurement mode and to interrupt upon conversion.
 * Clock Source: SMCLK
 * Clock DIV:   32
 * Resolution: 10 bits
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void ADC14init(void)
{
    //For Analog Input 8
//    P4->SEL0            |=   BIT5;                      // Select ADC Operation
//    P4->SEL1            |=   BIT5;                      // SEL = 11 sets to ADC operation
    P5->SEL0            |=   BIT5;                      // Select ADC Operation
    P5->SEL1            |=   BIT5;                      // SEL = 11 sets to ADC operation

    ADC14->CTL0         =    0;                         // Disable ADC for setup

    // CTL0 Configuration
    // 31-30 = 10   to divide down input clock by 32X
    // 26    = 1    to sample based on the sample timer.  This enables the use of bits 11-8 below.
    // 21-19 = 100  for SMCLK
    // 18-17 = 01   for reading multiple channels at once
    // 11-8  = 0011 for 32 clk sample and hold time
    // 7     = 0    for one ADC channel being read
    // 4     = 1    to turn on ADC
    ADC14->CTL0         =    0b10000100001000100000001100010000;

    ADC14->CTL1         =    BIT5;         // Bits 5 = 11 to enable 14 bit conversion
                                                     // Bit 23 turns on Temperature Sensor
    ADC14->MCTL[0]      =    0|BIT7;                         // A0 on P4.5, BIT7 says stop converting after this ADC
    //ADC14->MCTL[1]      =    8;                         // A8 on P4.5
    //ADC14->MCTL[2]      =    22 | BIT7;                 // Internal Temperature Sensor on A22 WHICH IS P6.3
                                                        // BIT7 says to stop converting after this ADC.
    ADC14->IER0         |=   BIT0;            // Interrupt on for all three conversions

    ADC14->CTL0         |=   0b10;                      // Enable Conversion
    NVIC->ISER[0]       |=   1<<ADC14_IRQn;             // Turn on ADC Interrupts in NVIC.  Equivalent to "NVIC_EnableIRQ(ADC14_IRQn);"
}
//Interrupts
void ADC14_IRQHandler(void)
{
    if(ADC14->IFGR0 & BIT0)
    {
       temp = ADC14->MEM[0];
        ADC14->CLRIFGR0     &=  ~BIT0;                  // Clear MEM0 interrupt flag
    }

    ADC14->CLRIFGR1     &=    ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
}
