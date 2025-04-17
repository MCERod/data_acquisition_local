#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SD.h>
#include <FS.h>
#include <SoftwareSerial.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Define this at the top of your file after includes
struct GpsData {
  bool isValid;
  double latitude;
  double longitude;
  int satellites;
  double speed;       // km/h
  double altitude;    // meters
  int year, month, day;
  int hour, minute, second;
  double course;      // degrees
  unsigned long lastUpdate;
};

// Define IMU data structure at the top near your GpsData struct
struct ImuData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

// Global variables
QueueHandle_t gpsQueue;
QueueHandle_t imuQueue;
SemaphoreHandle_t i2cSemaphore = NULL; // Protects I2C bus access
MPU6050 mpu;
Quaternion q;           // [w, x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]
int16_t gyro[3];        // [x, y, z]

// To these proper vector types:
VectorInt16 rawAccel;    // Raw acceleration including gravity
VectorInt16 linAccel;    // Linear acceleration (gravity removed)

// Add these globals
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // Expected DMP packet size
uint16_t fifoCount;     // Count of all bytes in FIFO

void GPS_TASK(void *pvParameters);
void IMU_TASK(void *pvParameters);
void SD_CARD(void *pvParameters);
void UART_TASK(void *pvParameters);

void setup() {
  gpsQueue = xQueueCreate(1, sizeof(GpsData));
}

void loop() {
  // put your main code here, to run repeatedly:
}



/***
 * @brief Task to read GPS data
 * @details This task reads data from the GPS module and updates the GpsData struct.
 *          It uses a SoftwareSerial object to communicate with the GPS module.
 *          The GPS data is updated every second and sent to a queue for other tasks to consume.
 *          The task also checks for GPS connection issues and updates the isValid flag accordingly.
 */


void GPS_TASK(void *pvParameters) {
  // Initialize GPS-specific objects inside the task
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial(18, 19); // RX, TX
  
  // Create a local struct to hold GPS data
  GpsData gpsData = {};
  
  // GPS initialization
  gpsSerial.begin(9600); // NEO-7M typically runs at 9600 baud
  
  // Task loop
  while(true) {
    // Read all available characters from GPS
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
    }
    
    // Update GPS data structure once per second
    if (millis() - gpsData.lastUpdate > 1000) {
      gpsData.lastUpdate = millis();
      
      // Store GPS information in struct
      gpsData.isValid = gps.location.isValid();
      
      if (gps.location.isValid()) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
      }
      
      if (gps.satellites.isValid()) {
        gpsData.satellites = gps.satellites.value();
      }
      
      if (gps.speed.isValid()) {
        gpsData.speed = gps.speed.kmph();
      }
      
      if (gps.altitude.isValid()) {
        gpsData.altitude = gps.altitude.meters();
      }
      
      if (gps.date.isValid() && gps.time.isValid()) {
        gpsData.year = gps.date.year();
        gpsData.month = gps.date.month();
        gpsData.day = gps.date.day();
        gpsData.hour = gps.time.hour();
        gpsData.minute = gps.time.minute();
        gpsData.second = gps.time.second();
      }
      
      if (gps.course.isValid()) {
        gpsData.course = gps.course.deg();
      }
      
      // Here you could do something with the gpsData struct
      // For example, pass it to other tasks if needed
    }
    
    // If no data is coming from GPS after 5 seconds
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      // GPS connection issue, you might want to set a flag in the struct
      gpsData.isValid = false;
    }
    
    xQueueOverwrite(gpsQueue, &gpsData);
    // Yield to other tasks
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Shorter delay for more responsive GPS reading
  }
}

void IMU_TASK(void *pvParameters) {
  // IMU task local variables
  ImuData imuData = {};
  
  // Wait for a moment to ensure the I2C bus is ready
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Initialize MPU6050 (your existing code)
  if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
    // Your existing initialization code...
    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    
    // Verify connection
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed");
    }
    
    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) {
      // Turn on the DMP
      mpu.setDMPEnabled(true);
      
      // Get expected DMP packet size
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // DMP Initialization failed
      Serial.print("DMP Initialization failed (code ");
      Serial.print(devStatus);
      Serial.println(")");
    }
    
    xSemaphoreGive(i2cSemaphore);
  }
  
  // Task loop
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
      fifoCount = mpu.getFIFOCount();
      
      if (fifoCount >= packetSize) {
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        // Process DMP data with correct vector types
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Get acceleration with gravity and convert to gravity-free
        mpu.dmpGetAccel(&rawAccel, fifoBuffer);
        mpu.dmpGetLinearAccel(&linAccel, &rawAccel, &gravity);
        
        // Now store the linear acceleration in your struct (converted to g units)
        imuData.accelX = (float)linAccel.x / 8192.0f;  // Convert to g
        imuData.accelY = (float)linAccel.y / 8192.0f;  // Convert to g
        imuData.accelZ = (float)linAccel.z / 8192.0f;  // Convert to g
        
        // Get gyro data
        mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);
        imuData.gyroX = (float)gyro[0] / 131.0f;  // Convert to degrees/s
        imuData.gyroY = (float)gyro[1] / 131.0f;  // Convert to degrees/s
        imuData.gyroZ = (float)gyro[2] / 131.0f;  // Convert to degrees/s
        
        // Update queue with newest IMU data
        xQueueOverwrite(imuQueue, &imuData);
      }
      
      xSemaphoreGive(i2cSemaphore);
    }
    
    vTaskDelay(2 / portTICK_PERIOD_MS);
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

