#include <stdint.h>
#include <stdio.h>
#include "tm4c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LCD.c"
#include "adc_speed.h"
#include "Doors.h"
#include "queue.h"
#include "ultrasonic.h"
#include "timer.h"
#include "DIO.h"
#include "bitwise_operations.h"
#include "semphr.h"
#include <stdbool.h>


float current_speed;
QueueHandle_t xDistanceQueue;
QueueHandle_t xTransmissionQueue;

static SemaphoreHandle_t xDoorLockSemaphore = NULL;
// Keep track of the last door state (0 = locked (PE0 low), 1 = unlocked (PE0 high)).
static uint8_t lastDoorStatus = 0xFF;

SemaphoreHandle_t xLCDMutex;

void Buzzer_Init(void) {
    // Configure PB5 as output
    dio_init('B', Pin5, Output);
}


void LED_Init(void) {
    SYSCTL_RCGCGPIO_R |= (1 << 5);  // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & (1 << 5)) == 0);  // Wait for activation
    
    GPIO_PORTF_DIR_R |= (1 << 1) | (1 << 2) | (1 << 3);  // Set PF1, PF2, PF3 as outputs
    GPIO_PORTF_DEN_R |= (1 << 1) | (1 << 2) | (1 << 3);  // Enable digital function
}


void vBuzzerTask(void *pvParameters) {
    float distance;
    uint32_t beepDelay;

    while (1) {
        if (xQueuePeek(xDistanceQueue, &distance, pdMS_TO_TICKS(10)) == pdPASS) {
            
					// Set the RGB LED color based on distance:
						if (distance > 30)
								GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x08;  // Green: PF3 on
						else if (distance > 20)
								GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x0A;  // Yellow: PF1 and PF3 on
						else
								GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x02;  // Red: PF1 on
					
					
					// Determine beep frequency based on distance
						if (distance > 40)
						{
							dio_writepin('B', Pin5, 0);
							vTaskDelay(pdMS_TO_TICKS(50));
							continue;
						}
						else if (distance > 30) {
                beepDelay = 1000;  // Very slow beeping at >100 cm
            } else if (distance > 20) {
                beepDelay = 500;   // Moderate beeping at 50–100 cm
            } else if (distance > 10) {
                beepDelay = 200;   // Fast beeping at 20–50 cm
            } else {
                beepDelay = 100;   // Rapid beeping at <20 cm
            }
            dio_writepin('B', Pin5, 1);
            vTaskDelay(pdMS_TO_TICKS(beepDelay));  // Beep duration
            dio_writepin('B', Pin5, 0);
            vTaskDelay(pdMS_TO_TICKS(beepDelay));  // Pause duration
        }
    }
}

void NVIC_EnablePeripherals(void) {
    NVIC_EN0_R |= (1 << 0)   // Port A
              | (1 << 2)   // Port C
              | (1 << 3)   // Port D
              | (1 << 4)   // Port E
              | (1 << 14); // ADC0 Sequence 0
}

// Initialize the ignition switch on PA4 (active-low)
void Ignition_Init(void) {
    // Configure PA4 as an input with pull-up
    dio_init('A', Pin4, Input);

    // Enable clock for Port A
    SYSCTL_RCGCGPIO_R |= (1U << 0);
    while ((SYSCTL_PRGPIO_R & (1U << 0)) == 0);

    // Configure PA4 for edge-triggered interrupts
    //GPIO_PORTA_IS_R &= ~Pin4;  // Set to edge-sensitive
    //GPIO_PORTA_IBE_R |= Pin4;  // Trigger on both edges
  //  GPIO_PORTA_ICR_R |= Pin4;  // Clear any previous interrupt
//    GPIO_PORTA_IM_R |= Pin4;   // Unmask interrupt

	
	GPIO_PORTA_DIR_R &= ~Pin4;    // PA4 as input
GPIO_PORTA_DEN_R |= Pin4;     // Enable digital function
GPIO_PORTA_PUR_R |= Pin4; 
	
	
    // Enable the NVIC interrupt for Port A (IRQ number 0 for Port A)
 //   NVIC_EN0_R |= (1U << 0);
}

// Initialize DIP-switch for transmission mode using PC6, PC7, and PD6
void Transmission_Init(void) {
    // Configure PC6, PC7, and PD6 as input pins with pull-ups
    dio_init('C', Pin6 | Pin7, Input);
    dio_init('D', Pin6, Input);

    // Enable clock for Port C and Port D
    SYSCTL_RCGCGPIO_R |= (1 << 2) | (1 << 3);
    while ((SYSCTL_PRGPIO_R & ((1 << 2) | (1 << 3))) == 0);

    // Configure PC6, PC7, and PD6 for edge-triggered interrupts
    GPIO_PORTC_IS_R &= ~(Pin6 | Pin7);  // Edge-sensitive
    GPIO_PORTC_IBE_R |= (Pin6 | Pin7);  // Both edges trigger
    GPIO_PORTC_ICR_R |= (Pin6 | Pin7);  // Clear any previous interrupt
    GPIO_PORTC_IM_R |= (Pin6 | Pin7);   // Unmask interrupt

    GPIO_PORTD_IS_R &= ~Pin6;           // Edge-sensitive
    GPIO_PORTD_IBE_R |= Pin6;           // Both edges trigger
    GPIO_PORTD_ICR_R |= Pin6;           // Clear any previous interrupt
    GPIO_PORTD_IM_R |= Pin6;            // Unmask interrupt

    // Enable NVIC interrupts for Port C and Port D (IRQ numbers vary by chip)
    NVIC_EN0_R |= (1 << 2) | (1 << 3);  
}

void vTransmissionTask(void *pvParameters) {
    char mode;
    char buf[7];

    while (1) {
        // Wait for a new mode update from the queue
        if (xQueueReceive(xTransmissionQueue, &mode, portMAX_DELAY)) {
						sprintf(buf, "   M: %c", mode);

            // Ensure only one task writes to LCD
            if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE) {
                LCD_SetCursor(1, 8);
                LCD_Print(buf);
                xSemaphoreGive(xLCDMutex);
            }
        }
    }
}




void GPIOC_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Clear the interrupt flags
    GPIO_PORTC_ICR_R |= (Pin6 | Pin7);  

    // Determine which switch is active
    char mode;
    if (!(GPIO_PORTC_DATA_R & Pin6)) {
        mode = 'P';
    } else if (!(GPIO_PORTC_DATA_R & Pin7)) {
        mode = 'D';
    } else {
        mode = '-';
    }

    // Send mode update to transmission task
    xQueueSendFromISR(xTransmissionQueue, &mode, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIOD_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the interrupt flag
    GPIO_PORTD_ICR_R |= Pin6;

    char mode;
		if (!(GPIO_PORTD_DATA_R & Pin6)) {
				mode = 'R';
		} else {
				mode = '-';
		}

    xQueueSendFromISR(xTransmissionQueue, &mode, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DoorLock_Init(void) {
    // --- Configure Port E for the mechanical end switch (PE0) ---
    SYSCTL_RCGCGPIO_R |= (1 << 4);              // Enable clock for Port E
    while ((SYSCTL_PRGPIO_R & (1 << 4)) == 0);    // Wait until ready
    GPIO_PORTE_DIR_R &= ~(DOOR_SWITCH_PIN);       // Set PE0 as input
    GPIO_PORTE_DEN_R |= DOOR_SWITCH_PIN;          // Digital enable
    GPIO_PORTE_PUR_R |= DOOR_SWITCH_PIN;          // Enable pull-up
    // No interrupts used; we will poll PE0.

    // --- Configure Port A for the DIP switch (PA4) ---
    SYSCTL_RCGCGPIO_R |= (1 << 0);                // Enable clock for Port A
    while ((SYSCTL_PRGPIO_R & (1 << 0)) == 0);      // Wait until ready
    GPIO_PORTA_DIR_R &= ~Pin4;                    // Set PA4 as input
    GPIO_PORTA_DEN_R |= Pin4;                     // Digital enable
    GPIO_PORTA_PUR_R |= Pin4;                     // Enable pull-up
}


void vDoorLockTask(void *pvParameters) {
    // Initialize hardware
    DoorLock_Init();

    // Determine initial state
    uint8_t baseState = ((GPIO_PORTE_DATA_R & DOOR_SWITCH_PIN) == 0) ? 0 : 1;
    uint8_t state = baseState;

    // Speed override
    if ((state == 1) && (current_speed > 20)) {
        state = 0;
    }

    // DIP override (active LOW)
    bool dip_override = !(GPIO_PORTA_DATA_R & Pin4);
    if ((state == 0) && dip_override) {
        state = 1;
    }

    lastDoorStatus = state;
    if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) {
        LCD_SetCursor(1, 0);
        LCD_Print(lastDoorStatus ? "DOR OPEN" : "DOR LOCK");
        xSemaphoreGive(xLCDMutex);
    }

    // Main task loop
    for (;;) {
        // 1. Read mechanical door switch
        baseState = ((GPIO_PORTE_DATA_R & DOOR_SWITCH_PIN) == 0) ? 0 : 1;
        state = baseState;

        // 2. Speed override
        if ((state == 1) && (current_speed > 20)) {
            state = 0;
        }

        // 3. DIP override
        dip_override = !(GPIO_PORTA_DATA_R & Pin4);  // Read DIP switch
        if ((state == 0) && dip_override) {
            vTaskDelay(pdMS_TO_TICKS(10)); // debounce
            if (!(GPIO_PORTA_DATA_R & Pin4)) { // confirm still active LOW
                state = 1;
            }
        }

        // 4. Update LCD if state changed
        if (state != lastDoorStatus) {
            lastDoorStatus = state;
            if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) {
                LCD_SetCursor(1, 0);
                LCD_Print(lastDoorStatus ? "DOR OPEN" : "DOR LOCK");
                xSemaphoreGive(xLCDMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // poll every 100ms
    }
}


void vUltrasonicTask(void *pvParameters) {
    float distance;
    char buf[17];  // Buffer for a 16-character LCD line plus null terminator

    while (1) {
        // Get the current distance from the ultrasonic sensor.
        distance = Ultrasonic_GetDistance();
				xQueueOverwrite(xDistanceQueue, &distance);

			
        
        // Take the LCD mutex before updating the display.
        if(xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE) {
            // Set the cursor to the desired display line and column.
            // For example, display the distance on row 0, column 0.
            LCD_SetCursor(0, 8);
            // Format the distance reading into the buffer.
            // For example: "Dist: 123.4 cm"
            sprintf(buf, "DST:%5.1f", distance);
            // Print the formatted string to the LCD.
            LCD_Print(buf);
            // Release the LCD mutex.
            xSemaphoreGive(xLCDMutex);
        }
        
        // Delay before taking the next measurement (e.g., 200ms).
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void ADC0SS0_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Retrieve the conversion result from SS3 FIFO.
    uint32_t val = ADC0_SSFIFO3_R;  
    // Clear the ADC0 SS3 interrupt flag.
    ADC0_ISC_R = 0x08;  
    
    current_speed = ConvertToSpeed(val);
    
    // Interrupt 1: If speed exceeds 20 and the door is open, force lock.
    if (current_speed > 20 && lastDoorStatus == 1) {
         lastDoorStatus = 0;  // 0 means locked
         xSemaphoreGiveFromISR(xDoorLockSemaphore, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vADCTask(void *pvParameters) {
    char buf[17];
    ADC0_Init();  // Ensure ADC is configured properly

    while (1) {
        // Trigger ADC0 conversion on Sample Sequence 3 (SS3)
        ADC0_PSSI_R |= 0x08;  // Bit 3 corresponds to SS3, initiating conversion

        // Wait for conversion to complete (poll the ADC0_RIS register)
        while ((ADC0_RIS_R & 0x08) == 0);  // Wait until SS3 conversion is complete

        // Read the conversion result (clears the interrupt flag)
        uint32_t val = ADC0_SSFIFO3_R;
        ADC0_ISC_R = 0x08;  // Clear SS3 interrupt flag

        // Convert and store speed globally
        current_speed = ConvertToSpeed(val);

        // Update LCD display
        sprintf(buf, "SPD:%3.0f", current_speed);
        if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdPASS) {
            LCD_SetCursor(0, 0);
            LCD_Print(buf);
            xSemaphoreGive(xLCDMutex);
        }

        // Delay before triggering next conversion
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



int main(void) {
	
	NVIC_EnablePeripherals();
	xLCDMutex = xSemaphoreCreateMutex();
	xTransmissionQueue = xQueueCreate(10, sizeof(char));
	xDistanceQueue = xQueueCreate(1, sizeof(float));

	
	LCD_Init();
	Ultrasonic_Init();
	Transmission_Init();
	Ignition_Init();
	Buzzer_Init();
	LED_Init();
	
	
	
	float ADCTask = xTaskCreate(vADCTask, "Speed Task", 240, NULL, 1, NULL);
	float DoorLockTask = xTaskCreate(vDoorLockTask, "Door Lock Task", 240, NULL, 1, NULL);
	float UltrasonicTask = xTaskCreate(vUltrasonicTask, "Ultrasonic Task", 240, NULL, 1, NULL);
	float TransmissionTask = xTaskCreate(vTransmissionTask, "Transmission Task", 240, NULL, 1, NULL);
	float BuzzerTask = xTaskCreate(vBuzzerTask, "Buzzer Task", 240, NULL, 1, NULL); 
	

	vTaskStartScheduler();
	
	while(1){}
	
}