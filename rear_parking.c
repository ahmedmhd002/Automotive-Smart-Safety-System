#include "rear_parking.h"
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "DIO.h"
#include "timer.h"
#include "ultrasonic.h"
#include <stdio.h>
#include "buzzer.h"
#include "lcd.h"
#include "gear.h" 


// Gear position setup
#define GEAR_PORT 'E'
#define REVERSE_PIN Pin2

// Beep duration
#define SHORT_BEEP_MS 100

// External shared queue and mutex (defined in main.c)
extern QueueHandle_t xDistanceQueue;
extern SemaphoreHandle_t xLCDMutex;

// Task to read distance and send via queue


// Task to display distance and handle buzzer alerts
void RearParkingTask(void *pvParameters) {
    float distance = 100.0f;  // Default distance
    char buffer[16];

    while (1) {
        if (isInReverse()) {
            // Wait for new distance from the queue
            if (xQueueReceive(xDistanceQueue, &distance, portMAX_DELAY) == pdTRUE) {
                // Lock LCD to display
                if (xSemaphoreTake(xLCDMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    LCD_Clear();
                    sprintf(buffer, "Dist: %.1f cm", distance);
                    LCD_Print(buffer);
                    xSemaphoreGive(xLCDMutex);
                }

                // Beeping logic based on distance
                if (distance < 10.0f) {
                    Buzzer_On();
                    if (xSemaphoreTake(xLCDMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        LCD_Clear();
                        LCD_Print("Doors Locked");
                        xSemaphoreGive(xLCDMutex);
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                else if (distance < 20.0f) {
                    Buzzer_On();
                    vTaskDelay(pdMS_TO_TICKS(SHORT_BEEP_MS));
                    Buzzer_Off();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                else if (distance < 30.0f) {
                    Buzzer_On();
                    vTaskDelay(pdMS_TO_TICKS(SHORT_BEEP_MS));
                    Buzzer_Off();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                else {
                    Buzzer_Off();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        } else {
            // Not in reverse gear
            Buzzer_Off();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

uint8_t isInReverse(void) {
    return Get_Gear() == GEAR_REVERSE;
}

// Reads gear state
//uint8_t isInReverse(void) {
  //  return dio_readpin(GEAR_PORT, REVERSE_PIN);
//}
