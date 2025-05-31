// adc_speed.h
#ifndef ADC_SPEED_H
#define ADC_SPEED_H

#include <stdint.h>

// Initializes ADC0 on PE1 (AIN2)
void ADC0_Init(void);

// Reads the analog value from PE1 (returns 0 - 4095)
uint32_t ADC0_Read(void);

// Converts ADC value to approximate speed (0 - 120 km/h)
float ConvertToSpeed(uint32_t adcValue);

#endif