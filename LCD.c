#include "tm4c.h"
#include <stdint.h>

// Port D pins for data (D4-D7)
#define D4 0x01  // PD0
#define D5 0x02  // PD1
#define D6 0x04  // PD2
#define D7 0x08  // PD3

// Port E pins for control
#define RS 0x10  // PE4
#define EN 0x20  // PE5

void delayMs(int n) {
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++);
}

void LCD_EnablePulse(void) {
    GPIO_PORTE_DATA_R |= EN;
    delayMs(1);
    GPIO_PORTE_DATA_R &= ~EN;
    delayMs(1);
}

void LCD_SendNibble(uint8_t nibble) {
    // Clear data bits first
    GPIO_PORTD_DATA_R &= ~(D4 | D5 | D6 | D7);
    
    // Align nibble to lower 4 bits, then set PD0..PD3 accordingly
    GPIO_PORTD_DATA_R |= (nibble >> 4) & 0x0F;
    
    LCD_EnablePulse();
}

void LCD_SendByte(uint8_t data, uint8_t isData) {
    if (isData)
        GPIO_PORTE_DATA_R |= RS;
    else
        GPIO_PORTE_DATA_R &= ~RS;

    LCD_SendNibble(data & 0xF0);         // Send high nibble
    LCD_SendNibble((data << 4) & 0xF0);  // Send low nibble
}

void LCD_Cmd(uint8_t cmd) {
    LCD_SendByte(cmd, 0);
    delayMs(2);
}

void LCD_Char(char data) {
    LCD_SendByte(data, 1);
}

void LCD_Print(const char *str) {
    while (*str)
        LCD_Char(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    LCD_Cmd(0x80 | (addr + col));
}

void LCD_Clear(void) {
    LCD_Cmd(0x01);
    delayMs(2);
}

void LCD_Init(void) {
    // Enable clocks for Port D and E
    SYSCTL_RCGCGPIO_R |= (1 << 3) | (1 << 4);  // Port D = bit3, Port E = bit4
    while ((SYSCTL_PRGPIO_R & ((1 << 3) | (1 << 4))) != ((1 << 3) | (1 << 4)));

    // Set Port D pins 0-3 as outputs and digital enable
    GPIO_PORTD_DIR_R |= (D4 | D5 | D6 | D7);
    GPIO_PORTD_DEN_R |= (D4 | D5 | D6 | D7);

    // Set Port E pins 4-5 as outputs and digital enable
    GPIO_PORTE_DIR_R |= (RS | EN);
    GPIO_PORTE_DEN_R |= (RS | EN);

    delayMs(20);

    // Initialization sequence
    LCD_SendNibble(0x30); delayMs(5);
    LCD_SendNibble(0x30); delayMs(1);
    LCD_SendNibble(0x30); delayMs(1);

    LCD_SendNibble(0x20); // Set 4-bit mode
    delayMs(1);

    LCD_Cmd(0x28); // 4-bit, 2 lines, 5x8 font
    LCD_Cmd(0x0C); // Display ON, cursor OFF
    LCD_Cmd(0x06); // Entry mode
    LCD_Clear();
}