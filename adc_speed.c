// adc_speed.c
#include "tm4c.h"
#include <stdint.h>

void ADC0_Init(void) {
    SYSCTL_RCGCADC_R |= 0x01;     // Enable clock for ADC0
    SYSCTL_RCGCGPIO_R |= 0x10;    // Enable clock for GPIOE
    while ((SYSCTL_PRGPIO_R & 0x10) == 0);  

    GPIO_PORTE_AFSEL_R |= 0x02;   // Enable alternate function on PE1
    GPIO_PORTE_DEN_R &= ~0x02;    // Disable digital function on PE1
    GPIO_PORTE_AMSEL_R |= 0x02;   // Enable analog mode on PE1

    ADC0_ACTSS_R &= ~0x08;        // Disable SS3 for config
    ADC0_EMUX_R &= ~0xF000;       // Trigger = software
    ADC0_SSMUX3_R = 2;            // Select channel AIN2 (PE1)
    ADC0_SSCTL3_R = 0x06;         // Flag + End
    ADC0_ACTSS_R |= 0x08;         // Enable SS3
	
	  ADC0_IM_R |= 0x08;          // Enable SS3 interrupts
		NVIC_EN0_R |= (1 << 14);     // Enable ADC0 SS3 interrupt in the NVIC (IRQ#14)
		NVIC_PRI4_R = (NVIC_PRI4_R & ~0x00FF0000) | (0x40 << 16);

}


uint32_t ADC0_Read(void) {
    ADC0_PSSI_R = 0x08;                    // Start sampling on SS3
    while ((ADC0_RIS_R & 0x08) == 0);      // Wait for conversion to complete
    uint32_t result = ADC0_SSFIFO3_R & 0xFFF; // Read 12-bit result
    ADC0_ISC_R = 0x08;                     // Clear interrupt flag
    return result;
}


float ConvertToSpeed(uint32_t adcValue) {
    // Assuming 0-4095 maps to 0 - 200 km/h
    return (adcValue * 200.0f) / 4095.0f;
}