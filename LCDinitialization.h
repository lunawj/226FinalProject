/*
 * LCDinitialization.h
 *
 *  Created on: Nov 14, 2018
 *      Author: Wesley
 */

#ifndef LCDINITIALIZATION_H_
#define LCDINITIALIZATION_H_

#include "SysTickInitialization.h"

/*
LCD PINS
DB7 P6.7
DB6 P6.6
DB5 P6.5
DB4 P6.4
E   P3.0
RS  P5.6

IMPORTANT comandWrite() ARGUMENTS
line 1          0x8
line 2          0xC0
line 3          0x90
line 4          0xC0+0x10
shifts left     0x18
shifts right    0x1F
write           0x0F

*/


/*----------------------------------------------------------------
 * void initPins()
 *
 * Description: initializes the pins for the LCD
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void initPins()
{
    //D7
    P6 -> SEL0 &= ~BIT7;
    P6 -> SEL1 &= ~BIT7;
    P6 -> DIR |= BIT7;

    //D6
    P6 -> SEL0 &= ~BIT6;
    P6 -> SEL1 &= ~BIT6;
    P6 -> DIR |= BIT6;

    //D5
    P6 -> SEL0 &= ~BIT5;
    P6 -> SEL1 &= ~BIT5;
    P6 -> DIR |= BIT5;

    //D4
    P6 -> SEL0 &= ~BIT4;
    P6 -> SEL1 &= ~BIT4;
    P6 -> DIR |= BIT4;

    //E
    P3 -> SEL0 &= ~BIT0;
    P3 -> SEL1 &= ~BIT0;
    P3 -> DIR |= BIT0;

    //RS
    P5 -> SEL0 &= ~BIT6;
    P5 -> SEL1 &= ~BIT6;
    P5 -> DIR |= BIT6;
}


/*----------------------------------------------------------------
 * void pulseEnablePin()
 *
 * Description: sets enable pin to high, then low
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void pulseEnablePin()
{
    P3 -> OUT &= ~BIT0;
    delayMilli(1);
    P3 -> OUT |= BIT0;
    delayMilli(1);
    P3 -> OUT &= ~BIT0;
    delayMilli(1);
}

/*----------------------------------------------------------------
 * void pushNibble()
 *
 * Description: pushes data to the pins
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void pushNibble(uint8_t nibble)
{
    //clears pin values
    P6 -> OUT &= ~(BIT7 | BIT6 | BIT5 | BIT4);

    //D7
    P6 -> OUT |= (nibble & BIT7);

    //D6
    P6 -> OUT |= (nibble & BIT6);

    //D5
    P6 -> OUT |= (nibble & BIT5);

    //D4
    P6 -> OUT |= (nibble & BIT4);
    pulseEnablePin();
    delayMicro(100);
}

/*----------------------------------------------------------------
 * void pushByte()
 *
 * Description: pushes data to pushNibble()
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void pushByte(uint8_t byte)
{
    uint8_t val = 0;
    val |= (byte & BIT7) | (byte & BIT6) | (byte & BIT5) | (byte & BIT4);
    pushNibble(val);
    val = 0;
    val |= (byte & BIT3) | (byte & BIT2) | (byte & BIT1) | (byte & BIT0);
    val<<= 4;
    pushNibble(val);
    delayMicro(100);
}

/*----------------------------------------------------------------
 * void commandWrite()
 *
 * Description: writes a command to the LCD
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void commandWrite(uint8_t command)
{
    P5 -> OUT &= ~BIT6; //RS
    //RW is connected to ground
    pushByte(command);   //put a byte in there
}

/*----------------------------------------------------------------
 * void dataWrite()
 *
 * Description: writes data to the LCD
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void dataWrite(uint8_t data)
{
    P5 -> OUT |= BIT6; //RS
    //RW is connected to ground
    pushByte(data);  //put a byte in there
}


/*----------------------------------------------------------------
 * void LCDInit()
 *
 * Description: initializes LCD
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void LCDInit()
{
    commandWrite(3);
    delayMilli(100);
    commandWrite(3);
    delayMicro(200);
    commandWrite(3);
    delayMilli(100);

    commandWrite(2);
    delayMicro(100);
    commandWrite(2);
    delayMicro(100);

    commandWrite(8);
    delayMicro(100);
    commandWrite(0x0F);
    delayMicro(100);
    commandWrite(1);
    delayMicro(100);
    commandWrite(6);
    delayMilli(10);
}


/*----------------------------------------------------------------
 * void resetLCD()
 *
 * Description: resets the LCD
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void resetLCD()
{
    commandWrite(1);
    delayMicro(100);
    commandWrite(0x0F);
}


#endif /* LCDINITIALIZATION_H_ */
