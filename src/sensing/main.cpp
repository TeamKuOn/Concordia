/*
 * main.cpp
 */

#include "stdArduino.hpp"
#include <Arduino_FreeRTOS.h>

#define LED 13

void TaskA( void *pvParameters );
void TaskB( void *pvParameters );

void TaskBlink(void *pvParameters);
void TaskChecker(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskA
    ,  "TaskA"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskB
    ,  "TaskB"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // xTaskCreate(
  //   TaskBlink, 
  //   "Blink", 
  //   128, 
  //   NULL, 
  //   2, 
  //   NULL);


  // xTaskCreate(
  //   TaskChecker, 
  //   "Checker", 
  //   128, 
  //   NULL, 
  //   1, 
  //   NULL);

}

void loop() {
  // Empty. Things are done in Tasks.
}

void TaskA(void *pvParameters) { // This is a task.  
  (void) pvParameters;

  for (;;) { // A Task shall never return or exit. 
    Serial.println('A');
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskB(void *pvParameters) {  // This is a task.
  (void) pvParameters;

  for (;;) {
    Serial.println('B');
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

unsigned char led_state;

void TaskBlink(void *pvParameters) {
  (void)pvParameters;

  pinMode(LED, OUTPUT);

  for (;;) {
    digitalWrite(LED, HIGH);
    led_state = HIGH;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    led_state = LOW;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskChecker(void *pvParameters) {
  (void)pvParameters;

  int value = analogRead(A0);
  int i = 0;
  for (;;) {

    value = analogRead(A0);

    Serial.println(value);
    delay(100);
    i++;

    if (i == 30) {
      if ((value < 700) && (led_state == HIGH)) {
        Serial.println("くらい");
        i = 0;

      } else if ((value > 900) && (led_state == HIGH)) {
        Serial.println("明るすぎ！");
        i = 0;

      } else {
        i = 0;
      }
    }

    vTaskDelay(1);

  }
}