#include "ultrasonic.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>

#define TRIGGER_PIN Pin2  // Now PB2
#define ECHO_PIN    Pin3  // Now PB3


#include "DIO.h"
#include "timer.h"  // implement delay + timer read logic

void Ultrasonic_Init(void) {
    dio_init('B', TRIGGER_PIN, Output);
    dio_init('B', ECHO_PIN, Input);
    Timer_Init();
}


float Ultrasonic_GetDistance(void) {
    uint32_t duration;
    float distance;

    // Send 10us pulse
    dio_writepin('B', TRIGGER_PIN, 1);
    delay_us(10);
    dio_writepin('B', TRIGGER_PIN, 0);

    // Wait for echo to go high
    while (!dio_readpin('B', ECHO_PIN));

    // Measure how long echo pin stays high
    Timer_Start();
    while (dio_readpin('B', ECHO_PIN));
    duration = Timer_Stop();

    // Convert to cm (34300 cm/s)
    distance = (duration * 0.0343f) / 2;

    return distance;
}
