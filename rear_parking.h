#ifndef REAR_PARKING_H
#define REAR_PARKING_H

#include <stdint.h>

// Initializes the HC-SR04 ultrasonic sensor and timer
void HC_SR04_Init(void);

// Sends the 10us trigger pulse to the ultrasonic sensor
void HC_SR04_Trigger(void);

// Measures and returns the distance in centimeters
uint32_t HC_SR04_ReadDistanceCM(void);

// Checks if the vehicle is in reverse gear
uint8_t isInReverse(void);

// FreeRTOS task for rear parking assistance
void RearParkingTask(void *pvParameters);

#endif // REAR_PARKING_H
