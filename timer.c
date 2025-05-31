#include "tm4c.h"
#include "timer.h"

static volatile uint32_t start_time = 0;
static volatile uint32_t end_time = 0;

void delay_us(uint32_t us) {
    // Use Timer0A for delay
    SYSCTL_RCGCTIMER_R |= (1 << 0);  // Enable Timer0
    TIMER0_CTL_R = 0x00000000;       // Disable Timer0A
    TIMER0_CFG_R = 0x00000000;       // 32-bit mode
    TIMER0_TAMR_R = 0x00000002;      // Periodic mode
    TIMER0_TAILR_R = (16 * us) - 1;  // Load value for 1us steps (16 MHz)
    TIMER0_ICR_R = 0x00000001;       // Clear timeout flag
    TIMER0_CTL_R = 0x00000001;       // Enable Timer0A

    while ((TIMER0_RIS_R & 0x01) == 0);  // Wait until timeout
    TIMER0_ICR_R = 0x00000001;          // Clear flag
}

void Timer_Init(void) {
    // Use Timer1A in edge-time mode
    SYSCTL_RCGCTIMER_R |= (1 << 1);    // Enable Timer1
    TIMER1_CTL_R &= ~0x01;             // Disable Timer1A
    TIMER1_CFG_R = 0x04;               // 16-bit mode
    TIMER1_TAMR_R = 0x02;              // Periodic Timer mode
    TIMER1_TAILR_R = 0xFFFF;           // Max load
    TIMER1_ICR_R = 0x01;               // Clear timeout
    TIMER1_CTL_R |= 0x01;              // Enable
}

void Timer_Start(void) {
    start_time = TIMER1_TAR_R;
}

uint32_t Timer_Stop(void) {
    end_time = TIMER1_TAR_R;

    if (start_time >= end_time)
        return (start_time - end_time) / 16;  // Assuming 16MHz clock
    else
        return ((0xFFFF - end_time + start_time) & 0xFFFF) / 16;
}
