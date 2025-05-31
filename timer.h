#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void delay_us(uint32_t us);       // Delay in microseconds
void Timer_Init(void);            // General-purpose timer initialization
void Timer_Start(void);           // Starts timing
uint32_t Timer_Stop(void);        // Stops timing and returns duration in microseconds

#endif // TIMER_H
