#ifndef DOORS_H
#define DOORS_H

#include "tm4c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DOOR_SWITCH_PIN  (1U << 0)  // PE0

// Function prototypes
void DoorLock_Init(void);
void vDoorLockTask(void *pvParameters);

#endif