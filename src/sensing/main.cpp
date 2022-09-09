/*
 * main.cpp
 */

#include "stdArduino.hpp"
#include <Arduino_FreeRTOS.h>

#define LED 13

void xTaskCreate( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(sendCANBus, "sendCANBus", 128, NULL, 2, NULL);


}

void loop() {
  // Empty. Things are done in Tasks.
}

void sendCANBus(void *pvParameters) { // This is a task.  
  (void) pvParameters;

  for (;;) { // A Task shall never return or exit. 
    Serial.println('A');
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

