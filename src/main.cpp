#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "MPU6050.h"
#include <SD.h>
#include <FS.h>


void GPS_TASK(void *pvParameters);
void IMU_TASK(void *pvParameters);
void SD_CARD(void *pvParameters);
void UART_TASK(void *pvParameters);



void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:
}



void GPS_TASK(void *pvParameters) {
  // GPS task code here
  while(true) {
    // Simulate some work
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void IMU_TASK(void *pvParameters) {
  // IMU task code here
  while(true) {
    // Simulate some work
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void SD_CARD(void *pvParameters) {
  // SD card task code here
  while(true) {
    // Simulate some work
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void UART_TASK(void *pvParameters) {
  // UART task code here
  while(true) {
    // Simulate some work
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

