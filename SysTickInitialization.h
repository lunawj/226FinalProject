/*
 * SysTickInitialization.h
 *
 *  Created on: Nov 14, 2018
 *      Author: Wesley
 */

#ifndef SYSTICKINITIALIZATION_H_
#define SYSTICKINITIALIZATION_H_

/*----------------------------------------------------------------
 * void initSysTick()
 *
 * Description: initializes SysTick timer
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void initSysTick()
{
    SysTick-> CTRL = 0; //off
    SysTick -> LOAD = 0xBBB; //1 ms + 1 clock
    SysTick-> VAL = 0;//reset count;
    SysTick -> CTRL = 5;//BIT(16); //enables bits
}

/*----------------------------------------------------------------
 * void delayMicro()
 *
 * Description: delays the specified number of microseconds
 * Inputs: number of microseconds
 * Outputs: None
----------------------------------------------------------------*/
void delayMicro(unsigned micros)
{
    SysTick ->LOAD = (3 * micros)- 1;
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & BIT(16)) == 0){}
}

/*----------------------------------------------------------------
 * void delayMilli()
 *
 * Description: delays the specified number of milliseconds
 * Inputs: number of milliseconds
 * Outputs: None
----------------------------------------------------------------*/
void delayMilli(unsigned ms)
{
    SysTick ->LOAD = (3000 * ms)- 1;
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & BIT(16)) == 0){}
}

/*----------------------------------------------------------------
 * void delaySeconds()
 *
 * Description: delays the specified number of seconds
 * Inputs: number of seconds
 * Outputs: None
----------------------------------------------------------------*/
void delaySeconds(unsigned sec)
{
    SysTick ->LOAD = (3000000 * sec)- 1;
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & BIT(16)) == 0){}
}


#endif /* SYSTICKINITIALIZATION_H_ */
