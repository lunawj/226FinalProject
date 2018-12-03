#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "SysTickInitialization.h"
#include "LCDinitialization.h"
//TODO test LED gradually increasing, alarm functionality, and LCD Blinking
//TODO clear setTimeFlag in button interrupt
/**
 * main.c
 * Summary: This code creates a home security system for a model home. It uses various
 *          electronic elements such as a liquid crystal display, a speaker, a motor and
 *          multiple light emitting diodes. The microcontroller used was a TI MSP 432.
 * Author: Wesley Luna, Even Bauer, with the assistance of Scott Zuidema
 * Date: November 1st, 2018
 */

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


void setBrightness(unsigned int dutyCycle);
void initializePWM();

void printTime();
void readAlarm();
void readTime();
void setAlarm(int hour, int minute);
void setTime(int hour, int minute, int second);
int getTemp();

void RTC_Init();
void P1_Init();
void Buttoninit();
void printSetTime(int blink, int spot);
void printSetAlarm(int blink, int spot);
// Time Update Flag to Show Time After an Update.
// Alarm Update Flag to Show Alarm Happened
int time_update = 0, alarm_update = 0;

// display_state 0=show time, 1= show alarm
int display_state = 0;
int setTimeFlag = 0;
// Hours, mins, and seconds all positive integers in the range 0-255
uint8_t hours, mins, secs;


///////////////////////////////
//alarm
int AHOUR = 0, AMIN = 0, alarm = 0;
uint16_t temp=0, LED = 0;


void initializePWM(); //Initializes timer and pins related to pwm


//SPEAKER
void Alarm(); //sounds alarm
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
//    initInterruptPins();
//    initializePWM();
//    initT32();
//    TimerA2config();
//    SpeakerConfig();
//    NVIC_EnableIRQ(PORT2_IRQn);
//    NVIC_EnableIRQ(PORT3_IRQn);
//    __enable_interrupt();
  // menu(); //main menu
}
*/
int main(void)
{
    initSysTick();
    Buttoninit();//Initalize buttons
    RTC_Init();  // Initialize the RTC
    P1_Init();   // Initialize the P1 Buttons as interrupts
    initPins();
    SpeakerConfig();
    TimerA2config();
    delayMicro(100);
    LCDInit();
    delayMicro(100);
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
    WDT_A->CTL = WDT_A_CTL_PW |  WDT_A_CTL_HOLD; // Stop watchdog timer

    setupPorts();
    setupSerial();
    initializePWM();
    NVIC_EnableIRQ(PORT4_IRQn);//SET time and Set alarm interrupt
    __enable_interrupt(); //enable button interrupts
    INPUT_BUFFER[0]= '\0';  // Sets the global buffer to initial condition of empty

    __enable_irq();  // Enable all interrupts (serial)

    int valid = 0;
    int hour, minute, second,i;
    char function[20];
    //remove while loop after testing
//    while(1) {
//        hour = 0;
//        minute = 0;
//        second = 0;
//
//        readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
//        if(string[0] != '\0'){ // if string is not empty, check the inputted data.
//            if(!(strcmp(string,"READALARM")))
//            {
//                readAlarm();
//            }else if(!(strcmp(string,"READTIME")))
//            {
//                readTime();
//            }else{
//
//
//                for(i=0; string[i] != ' '; i++)
//                {
//                    function[i] = string[i];
//                }
//                function[i] = '\0';
//                if(!(strcmp(function,"SETTIME")))
//                {
//
//                    //convert string to time
//                    i++;
//                    hour = string[i] - '0';
//                    hour*=10;
//                    i++;
//                    hour += string[i] - '0';
//                    i+=2;
//
//                    minute = string[i] - '0';
//                    minute*=10;
//                    i++;
//                    minute += string[i] - '0';
//                    i+=2;
//
//                    second = string[i] - '0';
//                    second*=10;
//                    i++;
//                    second += string[i] - '0';
//
//                    if(hour<=24 && hour>=0){
//                        if(minute <= 60 && minute >=0)
//                            if(second <= 60 && second >=0){
//                                writeOutput("Valid\n");
//                                setTime(hour, minute, second);
//                                valid = 1;
//                            }
//                    }
//                }else if(!(strcmp(function,"SETALARM")))
//                {
//                    //convert string to time
//                   i++;
//                   hour = string[i] - '0';
//                   hour*=10;
//                   i++;
//                   hour += string[i] - '0';
//                   i+=2;
//
//                   minute = string[i] - '0';
//                   minute*=10;
//                   i++;
//                   minute += string[i] - '0';
//                   i+=2;
//
//                   if(hour<=24 && hour>=0){
//                       if(minute <= 60 && minute >=0)
//                       {
//                           writeOutput("Valid\n");
//                           setAlarm(hour, minute);
//                           valid = 1;
//                       }
//                   }
//                }
//                else{
//                    writeOutput("Invalid\n");
//                }
//            }
//        }
    while(1){                                       // Main loop of program
       if(time_update){                            // Time Update Occurred (from interrupt handler)
           time_update = 0;                        // Reset Time Update Notification Flag
           AHOUR= (RTC_C->AMINHR & 0x7F00)>>8; //Sets the ALARM hour
           AMIN = (RTC_C->AMINHR & 0x007F); //Sets the ALARM minute
           //Alarm();
           delayMicro(100);
           printTime();
//                   if(display_state)
//                       printf("Alarm = %02d:%02d\n",(RTC_C->AMINHR & 0x7F00)>>8,RTC_C->AMINHR & 0x007F); // Print time with mandatory 2 digits  each for hours, mins, seconds
//                   else
//                       printTime();
//                       //printf("Time = %02d:%02d:%02d\n",hours,mins,secs); // Print time with mandatory 2 digits  each for hours, mins, seconds
//                  }
           if(alarm_update){                           // Alarm Update Occurred (from interrupt handler)
               printf("ALARM\n");                      // Display Alarm status to user
               alarm_update = 0;                       // Reset Alarm Update Notification Flag
           }
       }
    }
}


/*----------------------------------------------------------------
 * void setBrightness(unsigned int dutyCycle)
 *
 * Description: Changes the
 *              LED brightness
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void setBrightness(unsigned int dutyCycle)
{
    if(dutyCycle > 100)
    {
        dutyCycle = 100;
    }
    TIMER_A1->CCR[2] = (int)((dutyCycle /100.00) * 60000);
    TIMER_A1->CCR[1] = (int)((dutyCycle /100.00) * 60000);
}


/*----------------------------------------------------------------
 * void initializePWM()
 *
 * Description: initializes the PWM pins and timers
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/

void initializePWM()
{
    P2->SEL0 |= BIT4|BIT5;
    P2->SEL1 &= ~BIT4|BIT5;  // Setting SEL0 = 1 and SEL1 = 0 will activate the secondary function of these pins.
    P2->DIR |=BIT4|BIT5;    // Set pins as outputs.  Also required for PWM output.
    P2->OUT &= ~BIT4|BIT5;   // output.

//    P7->SEL0 |= (BIT4|BIT5|BIT6|BIT7);
//    P7->SEL1 &= ~(BIT4|BIT5|BIT6|BIT7);;  // Setting SEL0 = 1 and SEL1 = 0 will activate the secondary function of these pins.
//    P7->DIR |=(BIT4|BIT5|BIT6|BIT7);;    // Set pins as outputs.  Also required for PWM output.
//    P7->OUT &= ~(BIT4|BIT5|BIT6|BIT7);;   // output.
    //P7 is A1
    //P5 is A2

    TIMER_A0->CCR[0] = 50000;  //Running at 60 Hz
    TIMER_A0->CCTL[1] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCTL[2] = 0b0000000011100000;  //reset / set
    TIMER_A0->CCR[1] = 50000;// Timer LED
    TIMER_A0->CCR[2] = 50000;// Timer LED
    TIMER_A0->CTL = 0b0000001000010100;
}



///*----------------------------------------------------------------
// * void initT32()
// *
// * Description: Initializes TIMER32_1
// * Inputs: None
// * Outputs: None
//----------------------------------------------------------------*/
//void initT32()
//{
//    TIMER32_1->CONTROL = 0b11000011;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
//    TIMER32_1->LOAD = 30000000 - 1;   //10 seconds           //Set to a count down of 1 second on 3 MHz clock
//}
//
///*----------------------------------------------------------------
// * void TimerA2config(void)
// *
// * Description: Initializes TIMER_A2
// * Inputs: None
// * Outputs: None
//----------------------------------------------------------------*/
void TimerA2config(void)
{
    TIMER_A2->CCR[0]    =    5714;                    // Initialize the period of TimerA0 PWM to the maximum.  This will change at the first interrupt.  Could be set to something else.
    TIMER_A2->CCTL[2]   =    0b11100000;                // Set to Reset/set Compare Mode (BITs 7-5 set to 1)
    TIMER_A2->CCR[2]    =    (0);                         // Will reset (set to 0) immediately, so it will always be off by default
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
    P5->SEL0            |=   BIT7;                      // Configure P.7 to TIMER_A2 PWM Control
    P5->SEL1            &=  ~BIT7;                      // SEL = 01 sets to PWM Control
    P5->DIR             |=   BIT7;                      // Initialize speaker pin as an output
}


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


void readAlarm(){
    char buff[20];
    writeOutput("Alarm is ");
    sprintf(buff, "%02d:%02d%c", AHOUR, AMIN, '\0');
    writeOutput(buff);
}

void readTime(){
    char buff[20];
    writeOutput("Time is ");
    sprintf(buff, "%02d:%02d:%02d%c", hours, mins, secs, '\0');
    writeOutput(buff);
}
void setAlarm(int hour, int minute){
    AHOUR = hour;
    AMIN = minute;
    RTC_C->AMINHR = AHOUR<<8 | AMIN | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    alarm = 1;
    char buff[20];
    writeOutput("Alarm set to ");
    sprintf(buff, "%02d:%02d%c", AHOUR, AMIN, '\0');
    writeOutput(buff);
}
void setTime(int hour, int minute, int second){
    setTimeFlag = 1;
    hours = hour;
    mins = minute;
    secs = second;
    RTC_C->TIM0 = mins<<8 | secs;//min, secs
    RTC_C->TIM1 = (RTC_C->TIM1 & 0xFF00) | hours;  //day, hours
    char buff[20];
    writeOutput("Time set to ");
    sprintf(buff, "%02d:%02d:%02d%c", hours, mins, secs, '\0');
    writeOutput(buff);
    printTime();
    setTimeFlag = 0;
}

//
void printTime(){
   char line1[20];
   char line2[20];
   char line3[20];
   char line4[20];
   int i, n;
   if(hours < 12)
     {
         sprintf(line1, "%d:%02d:%02d AM %c", hours, mins, secs,'\0');
     }else if(hours == 24){
         sprintf(line1, "%d:%02d:%02d AM %c", 12, mins, secs, '\0');
     }else if(hours > 12){
         sprintf(line1, "%d:%02d:%02d PM %c", (hours-12), mins, secs, '\0');
     }else if(hours == 12){
         sprintf(line1, "%d:%02d:%02d PM %c", 12, mins, secs, '\0');
     }

   if(alarm == 2)
  {
      sprintf(line2,"ALARM SNOOZE %c", '\0');
  }else if(alarm == 1){
      sprintf(line2,"ALARM ON     %c", '\0');
  }else{
      sprintf(line2,"ALARM OFF    %c", '\0');
  }

   if(AHOUR < 12)
   {
       sprintf(line3, "%d:%02d AM %c", AHOUR, AMIN, '\0');
   }else if(AHOUR == 24){
       sprintf(line3, "%d:%02d AM %c", 12, AMIN, '\0');
   }else if(AHOUR > 12){
       sprintf(line3, "%d:%02d PM %c", AHOUR-12, AMIN, '\0');
   }else if(hours == 12){
       sprintf(line1, "%d:%02d PM %c", 12, AMIN, '\0');
   }

   ////sprintf(line4,"%02.1d%c", getTemp(), '\0');
   //sprintf(line4,"%.1d%c", getTemp(), '\0');

   commandWrite(0X2);
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

//  commandWrite(0xC0+0x10);
//  delayMicro(100);
//  commandWrite(0x0F);
//  n = strlen(line4);
//  for(i=0;i<n;i++)
//  {
//      dataWrite(line4[i]);
//  }
//  dataWrite(0b11011111);
//  dataWrite('F');


}

/*blink should be incremented in the interrupt handler, it will blink every other time the function is called
 * spot is where the time is beig changed.
 * 1 is for hours
 * 2 is for minutes
 * 3 is for seconds
 */
void printSetTime(int blink, int spot){
   char line1[20] = "SET TIME";
   char line2[20];
   setTimeFlag = 1;
   if(blink == 0){
         resetLCD();
   }
   int i, n;
   if(blink%2){
       if(hours < 12)
         {
             sprintf(line2, "%02d:%02d:%02d AM %c", hours, mins, secs,'\0');
         }else if(hours == 24){
             sprintf(line2, "%02d:%02d:%02d AM %c", 12, mins, secs, '\0');
         }else if(hours > 12){
             sprintf(line2, "%02d:%02d:%02d PM %c", (hours-12), mins, secs, '\0');
         }else if(hours == 12){
             sprintf(line2, "%02d:%02d:%02d PM %c", 12, mins, secs, '\0');
         }
   }else{
       if(spot == 1){
           if(hours < 12)
            {
                sprintf(line2, "  :%02d:%02d AM %c", mins, secs,'\0');
            }else if(hours == 24){
                sprintf(line2, "  :%02d:%02d AM %c", mins, secs, '\0');
            }else if(hours > 12){
                sprintf(line2, "  :%02d:%02d PM %c", mins, secs, '\0');
            }else if(hours == 12){
                sprintf(line2, "  :%02d:%02d PM %c", mins, secs, '\0');
            }
       }else if (spot == 2){
           if(hours < 12)
            {
                sprintf(line2, "%02d:  :%02d AM %c", hours, secs,'\0');
            }else if(hours == 24){
                sprintf(line2, "%02d:  :%02d AM %c", 12, secs, '\0');
            }else if(hours > 12){
                sprintf(line2, "%02d:  :%02d PM %c", (hours-12), secs, '\0');
            }else if(hours == 12){
                sprintf(line2, "%02d:  :%02d PM %c", 12, secs, '\0');
            }
       }else if (spot == 3){
           if(hours < 12)
            {
                sprintf(line2, "%02d:%02d:   AM %c", hours, mins, '\0');
            }else if(hours == 24){
                sprintf(line2, "%02d:%02d:   AM %c", 12, mins, '\0');
            }else if(hours > 12){
                sprintf(line2, "%02d:%02d:   PM %c", (hours-12), mins, '\0');
            }else if(hours == 12){
                sprintf(line2, "%02d:%02d:   PM %c", 12, mins, '\0');
            }
       }
   }


   commandWrite(0X2);
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
   delaySeconds(1);
}

/*blink should be incremented in the interrupt handler, it will blink every other time the function is called
 * spot is where the time is beig changed.
 * 1 is for hours
 * 2 is for minutes
 * Always set 0 to blink for the first time
 */
void printSetAlarm(int blink, int spot){
   char line1[20] = "SET ALARM";
   char line2[20];

   if(blink == 0){
      resetLCD();
   }
   int i, n;
   if(blink%2){
       if(AHOUR < 12)
          {
              sprintf(line2, "%02d:%02d AM %c", AHOUR, AMIN, '\0');
          }else if(AHOUR == 24){
              sprintf(line2, "%02d:%02d AM %c", 12, AMIN, '\0');
          }else if(AHOUR > 12){
              sprintf(line2, "%02d:%02d PM %c", AHOUR-12, AMIN, '\0');
          }else if(hours == 12){
              sprintf(line2, "%02d:%02d PM %c", 12, AMIN, '\0');
          }
   }else{
       if(spot == 1){
           if(AHOUR < 12)
            {
                sprintf(line2, "  :%02d AM %c", AMIN,'\0');
            }else if(AHOUR == 24){
                sprintf(line2, "  :%02d AM %c", AMIN, '\0');
            }else if(AHOUR > 12){
                sprintf(line2, "  :%02d PM %c", AMIN, '\0');
            }else if(AHOUR == 12){
                sprintf(line2, "  :%02d PM %c", AMIN, '\0');
            }
       }else if (spot == 2){
           if(AHOUR < 12)
            {
                sprintf(line2, "%02d:   AM %c", AHOUR,'\0');
            }else if(AHOUR == 24){
                sprintf(line2, "%02d:   AM %c", 12, '\0');
            }else if(AHOUR > 12){
                sprintf(line2, "%02d:   PM %c", AHOUR - 12, '\0');
            }else if(AHOUR == 12){
                sprintf(line2, "%02d:   PM %c", 12, '\0');
            }
       }
   }


   commandWrite(0X2);
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
   delaySeconds(1);
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

void Buttoninit()
{
    //Button for setting time Black button
    P4 -> SEL0 &= ~BIT0;
    P4 -> SEL1 &= ~BIT0;
    P4 -> DIR &= ~BIT0;
    P4 -> REN |= BIT0;
    P4 -> OUT |= BIT0;
    P4 -> IE |= BIT0;
    P4 -> IES |= BIT0;
    P4 -> IFG &= ~BIT0;

    //Button Set Alarm Green Button
    P4 -> SEL0 &= ~BIT1;
    P4 -> SEL1 &= ~BIT1;
    P4 -> DIR &= ~BIT1;
    P4 -> REN |= BIT1;
    P4 -> OUT |= BIT1;
    P4 -> IE |= BIT1;
    P4 -> IES |= BIT1;
    P4 -> IFG &= ~BIT1;

    //ON, OFF, Up button
    P4 -> SEL0 &= ~BIT2;
    P4 -> SEL1 &= ~BIT2;
    P4 -> DIR &= ~BIT2;
    P4 -> REN |= BIT2;
    P4 -> OUT |= BIT2;

    //Snooze, Down button
    P4 -> SEL0 &= ~BIT3;
    P4 -> SEL1 &= ~BIT3;
    P4 -> DIR &= ~BIT3;
    P4 -> REN |= BIT3;
    P4 -> OUT |= BIT3;

}

void PORT4_IRQHandler(void)//SET TIME/ALARM interrupt
{ int butcount = 0, hour,minute;
    if(P4 -> IFG & BIT0)// Set Time
    {  //Hours flash
        //Hours roll over and AM/PM update
        //Press Set Time button to move onto Minutes update
        //Minutes Flash
        //Minute roll over
        //Press Set Time button to finish setting the time
        if(!(P4 ->IN & BIT0)){
            butcount=butcount+1;
        }
     if(butcount==0)
     {
        if(!(P4->IN & BIT2)) //Up button
        {
            hour=hour+1;
        }
        if(!(P4->IN & BIT2))//Down button
        {
            hour=hour-1;
        }
     }
     if(butcount==1)
     {
         if(!(P4->IN & BIT2)) //Up button
         {
             minute=minute+1;
         }
         if(!(P4->IN & BIT2))//Down button
         {
             minute=minute-1;
         }
     }
     if(butcount==2)
     {
         butcount = 0;
         P4 -> IFG &= ~BIT0;//clear flag
     }

    }else if(P4 -> IFG & BIT1) //Set Alarm
    {
        if(!(P4 ->IN & BIT0)){
            butcount=butcount+1;
        }
     if(butcount==0)
     {
        if(!(P4->IN & BIT2)) //Up button
        {
            hour=hour+1;
        }
        if(!(P4->IN & BIT2))//Down button
        {
            hour=hour-1;
        }
     }
      if(butcount==1)
         {
             if(!(P4->IN & BIT2)) //Up button
             {
                 minute=minute+1;
             }
             if(!(P4->IN & BIT2))//Down button
             {
                 minute=minute-1;
             }
         }
         if(butcount==2)
         {
             butcount = 0;
             P4 -> IFG &= ~BIT1;//clear flag
         }
    }
     return;
}
/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void RTC_Init()
 *
 * Interrupt setup for RTC.
 *
 * This function sets up the RTC to operate, initializes the time and alarm to default values, and creates interrupts
 * for time updates and alarms.
 *
-------------------------------------------------------------------------------------------------------------------------------*/
void RTC_Init(){
    //Initialize time to 2:45:55 pm
//    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = 40<<8 | 50;//40 min, 50 secs
    RTC_C->TIM1 = 1<<8 | 14;  //Monday, 11 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = 15<<8 | 5 | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    alarm = 1;
    RTC_C->PS1CTL = 0b00010;  //1/64 second interrupt
    //RTC_C->PS1CTL = 0b11010;  //runs every second
    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    //TODO
    NVIC_EnableIRQ(RTC_C_IRQn);
}

/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void RTC_C_IRQHandler()
 *
 * Interrupt Handler for RTC.  The name of this function is set in startup_msp432p401r_ccs.c
 *
 * This handler checks for PS1 flags and Alarm flags.  Sets global variables time_update and alarm_update to 1 when
 * their respective interrupt occurs.
 *
-------------------------------------------------------------------------------------------------------------------------------*/
void RTC_C_IRQHandler()
{
    if(setTimeFlag){
        RTC_C->PS1CTL &= ~BIT0;                         // Reset interrupt flag
    }else{

        if(RTC_C->PS1CTL & BIT0){                           // PS1 Interrupt Happened
            hours = RTC_C->TIM1 & 0x00FF;                   // Record hours (from bottom 8 bits of TIM1)
            mins = (RTC_C->TIM0 & 0xFF00) >> 8;             // Record minutes (from top 8 bits of TIM0)
            secs = RTC_C->TIM0 & 0x00FF;                    // Record seconds (from bottom 8 bits of TIM0)
            // For increasing the number of seconds  every PS1 interrupt (to allow time travel)
            if(secs < 59){                                 // If not  59 seconds, add 1 (otherwise 59+1 = 60 which doesn't work)
                RTC_C->TIM0 = RTC_C->TIM0 + 1;
            }
            else {
                RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;  // Add a minute if at 59 seconds.  This also resets seconds.
                //RTC_C->TIM0 = 0;
                if(mins == 59 ) {
                       RTC_C->TIM0 = 0<<8;  // Add a minute if at 59 seconds.  This also resets seconds.
                       RTC_C->TIM1 = (RTC_C->TIM1 & 0x00FF) + 1;
                }
            }
            if(hours == 25)
            {
               RTC_C->TIM1 = 1;
               hours = (RTC_C->TIM1 & 0xFF00) | 1;
            }

                if(hours == AHOUR && mins == AMIN)
                {
                    //sound alarm function goes here
                    //printf("SOUND ALARM\n");    //TODO MUST REMOVE LATER, ONLY FOR TESTING
                }else if(LED==0 && alarm){
                    if(AHOUR == (hours + 1) && AMIN < 5 && mins >= 55){
                        //edge cases
                        switch(AMIN)
                        {
                        case 0:
                            if(mins == 55)
                            {
                                LED = 3;
                            }
                        case 1:
                            if(mins == 56)
                            {
                                LED = 3;
                            }
                            break;
                        case 2:
                            if(mins == 57)
                            {
                                LED = 3;
                            }
                            break;
                        case 3:
                            if(mins == 58)
                            {
                                LED = 3;
                            }
                            break;
                        case 4:
                            if(mins == 59)
                            {
                                LED = 3;
                            }
                            break;
                        default:
                            break;
                        }

                    }else if(hours == AHOUR && AMIN == (mins + 5)){
                        LED = 3;
                    }
                }
            if(LED > 0 && LED <= 100 && alarm){
                //printf("LED turned on\n");  //TODO MUST REMOVE LATER, ONLY FOR TESTING
                if(!(LED%3))
                {
                    setBrightness(LED/3);     //increase by LED/3
                }
                LED++;
            }
            RTC_C->PS1CTL &= ~BIT0;                         // Reset interrupt flag
            time_update = 1;                                     // Send flag to main program to notify a time update occurred.

        }
        if(RTC_C->CTL0 & BIT1)                              // Alarm happened!
        {
            Alarm();
            alarm_update = 1;                               // Send flag to main program to notify a time update occurred.
            RTC_C->CTL0 = (0xA500) | BIT5;                  // Resetting the alarm flag.  Need to also write the secret code
                                                            // and rewrite the entire register.
                                                            // TODO: It seems like there is a better way to preserve what was already
                                                            // there in case the setup of this register needs to change and this line
                                                            // is forgotten to be updated.
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void PORT1_IRQHandler(void)
 *
 * Interrupt Handler for P1.  The name of this function is set in startup_msp432p401r_ccs.c
 *
 * This handler checks for interrupts on P1.1 and P1.4 and sets a flag to change the output of the main program.
 *
-------------------------------------------------------------------------------------------------------------------------------*/
void PORT1_IRQHandler(void)
{
    if(P1->IFG & BIT1) {                                //If P1.1 had an interrupt
        display_state = 1;
    }
    if(P1->IFG & BIT4) {                                //If P1.4 had an interrupt
        display_state = 0;
    }
    P1->IFG = 0;                                        //Clear all flags
}

/*-------------------------------------------------------------------------------------------------------------------------------
 *
 * void P1_Init()
 *
 * P1 setup for P1.1 and P1.4 to be button inputs with pull ups and interrupts enabled.
 *
-------------------------------------------------------------------------------------------------------------------------------*/
void P1_Init() {
    P1->SEL0 &= ~(BIT1|BIT4);
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR  &= ~(BIT1|BIT4);
    P1->REN  |=  (BIT1|BIT4);
    P1->OUT  |=  (BIT1|BIT4);
    P1->IE   |=  (BIT1|BIT4);
    NVIC_EnableIRQ(PORT1_IRQn);
}

/*----------------------------------------------------------------
 * void alarm(int on)
 *
 * Description: Displays a message on the LCD and toggles/sounds alarm
 * Inputs: an integer flag that determines whether the alarm should sound
 * Outputs: Sound to speakerr
----------------------------------------------------------------*/
void Alarm()
{

    //sound alarm
    TIMER_A2->CCR[2]    =    5700;
    delaySeconds(1);
    TIMER_A2->CCR[2]    =    0;
    delaySeconds(1);
}
