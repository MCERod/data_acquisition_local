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
#include <DFRobot_LIDAR07.h>  // Adicione após outras inclusões
#include <WiFi.h>
#include <ArduinoJson.h>

// Conditionally include the appropriate IMU library
#if NORMAL_IMU
  #include <MPU6050.h>
#else
  #include <MPU6050_6Axis_MotionApps20.h>
#endif

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

// Define LIDAR data structure
struct LidarData {
  float distance;           // Distance in cm
  float strength;           // Signal strength
  uint8_t status;           // Status code
  unsigned long timestamp;  // Time of measurement
};

// Define recording control structure
struct RecordingState {
  bool isRecording;
  bool newCommand;
  unsigned long startTime;
  unsigned long recordedSamples;
  char filename[32];
};

// Define event data structure
struct EventData {
  char eventType[10];  // "buraco" or "lomba"
  double latitude;
  double longitude;
  unsigned long timestamp;
  bool processed;  // Flag to track if event was written to CSV
};

// Global variables
QueueHandle_t gpsQueue;
QueueHandle_t imuQueue;
QueueHandle_t lidarQueue;
QueueHandle_t recordingQueue; // Global queue for recording state
QueueHandle_t eventQueue;
SemaphoreHandle_t i2cSemaphore = NULL; // Protects I2C bus access
bool lidarActive = false; // Add this with your other global variables

// Conditionally define MPU6050 and related variables
#if NORMAL_IMU
  MPU6050 mpu;
#else
  MPU6050 mpu;
  Quaternion q;           // [w, x, y, z]
  VectorFloat gravity;    // [x, y, z]
  float ypr[3];           // [yaw, pitch, roll]
  int16_t gyro[3];        // [x, y, z]
  VectorInt16 rawAccel;   // Raw acceleration including gravity
  VectorInt16 linAccel;   // Linear acceleration (gravity removed)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint16_t packetSize;    // Expected DMP packet size
  uint16_t fifoCount;     // Count of all bytes in FIFO
#endif

void GPS_TASK(void *pvParameters);
void IMU_TASK(void *pvParameters);
void SD_CARD(void *pvParameters);
void LIDAR_TASK(void *pvParameters);  // Adicione junto com outras declarações
void TCP_SERVER_TASK(void *pvParameters);


void setup() {
  gpsQueue = xQueueCreate(1, sizeof(GpsData));
  imuQueue = xQueueCreate(1, sizeof(ImuData));
  lidarQueue = xQueueCreate(1, sizeof(LidarData));
  recordingQueue = xQueueCreate(1, sizeof(RecordingState));
  eventQueue = xQueueCreate(5, sizeof(EventData));  // Store up to 5 events
  i2cSemaphore = xSemaphoreCreateMutex();

  // Initialize Serial for debugging
  Serial.begin(115200);

  // Create tasks
  xTaskCreate(GPS_TASK, "GPS_TASK", 2048, NULL, 2, NULL);
  xTaskCreate(IMU_TASK, "IMU_TASK", 4096, NULL, 3, NULL); // Double the stack size
  xTaskCreate(LIDAR_TASK, "LIDAR_TASK", 2048, NULL, 3, NULL);  // Nova task
  xTaskCreate(SD_CARD, "SD_CARD", 8192, NULL, 2, NULL);
  xTaskCreate(TCP_SERVER_TASK, "TCP_SERVER_TASK", 4096, NULL, 1, NULL); // Nova task
}

void loop() {
  vTaskDelete(NULL); // Delete the loop task
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
  SoftwareSerial gpsSerial(25, 26); // RX, TX
  
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
    if (millis() - gpsData.lastUpdate > 1) {
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
        gpsData.satellites = gps.satellites.value();
        gpsData.altitude = gps.altitude.meters();
        gpsData.speed = gps.speed.kmph();
        //Serial.println("GPS Date: " + String(gpsData.year) + "-" + String(gpsData.month) + "-" + String(gpsData.day));
        //Serial.println("GPS Time: " + String(gpsData.hour) + ":" + String(gpsData.minute) + ":" + String(gpsData.second));
        /*Serial.println("GPS Satellites: " + String(gps.satellites.value()));
        Serial.println("latitude: " + String(gpsData.latitude, 6));
        Serial.println("longitude: " + String(gpsData.longitude, 6));
        Serial.println("speed: " + String(gpsData.speed, 2) + " km/h");
        Serial.println("altitude: " + String(gpsData.altitude, 2) + " m");*/
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
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Shorter delay for more responsive GPS reading
  }
}

void IMU_TASK(void *pvParameters) {
  // IMU task local variables
  ImuData imuData = {};

  // Define the number of samples to average
  const int numSamples = 10;
  float accelXBuffer[numSamples];
  float accelYBuffer[numSamples];
  float accelZBuffer[numSamples];
  int sampleIndex = 0;

  float accelXSum = 0;
  float accelYSum = 0;
  float accelZSum = 0;

  // Complementary filter parameters
  float alpha = 0.98; // Adjust this value (0 < alpha < 1)

  // Initial gravity vector
  float gravityX = 0;
  float gravityY = 0;
  float gravityZ = 1;
  
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
    } else {
      Serial.println("MPU6050 connection successful");
    }
    
    // Configure MPU6050 settings
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // Set accelerometer range to ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // Set gyro range to ±250°/s
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);            // Set low pass filter
    
    // Calibrate sensors
    Serial.println("Calibrating sensors...");
    mpu.CalibrateGyro(7);   // Calibrate gyro with 7 loops
    mpu.CalibrateAccel(7);  // Calibrate accel with 7 loops
    mpu.PrintActiveOffsets();
    
    Serial.println("MPU6050 ready!");
    
    xSemaphoreGive(i2cSemaphore);
  }
  
  // Task loop
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
      // Read raw sensor data
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      xSemaphoreGive(i2cSemaphore);
      // Convert raw values to meaningful units
      float accelX = ax / 16384.0;  // Convert to g (assuming ±2g range)
      float accelY = ay / 16384.0;
      float accelZ = az / 16384.0;
      
      float gyroX = gx / 131.0;    // Convert to degrees/s (assuming ±250°/s range)
      float gyroY = gy / 131.0;
      float gyroZ = gz / 131.0;

      // Complementary filter to estimate gravity
      gravityX = alpha * gravityX + (1 - alpha) * accelX;
      gravityY = alpha * gravityY + (1 - alpha) * accelY;
      gravityZ = alpha * gravityZ + (1 - alpha) * accelZ;
      
      // Normalize gravity vector
      float norm = sqrt(gravityX * gravityX + gravityY * gravityY + gravityZ * gravityZ);
      gravityX /= norm;
      gravityY /= norm;
      gravityZ /= norm;

      // Remove gravity from raw accelerations
      accelX = accelX - gravityX;
      accelY = accelY - gravityY;
      accelZ = accelZ - gravityZ;
      
      // Apply moving average filter
      accelXSum -= accelXBuffer[sampleIndex];
      accelYSum -= accelYBuffer[sampleIndex];
      accelZSum -= accelZBuffer[sampleIndex];

      accelXBuffer[sampleIndex] = accelX;
      accelYBuffer[sampleIndex] = accelY;
      accelZBuffer[sampleIndex] = accelZ;

      accelXSum += accelXBuffer[sampleIndex];
      accelYSum += accelYBuffer[sampleIndex];
      accelZSum += accelZBuffer[sampleIndex];

      sampleIndex = (sampleIndex + 1) % numSamples;

      imuData.accelX = accelXSum / numSamples;
      imuData.accelY = accelYSum / numSamples;
      imuData.accelZ = accelZSum / numSamples;
      imuData.gyroX = gyroX;
      imuData.gyroY = gyroY;
      imuData.gyroZ = gyroZ;
      
      
      xQueueOverwrite(imuQueue, &imuData);
      
      
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void SD_CARD(void *pvParameters) {
  // Initialize SD card
  if (!SD.begin()) {
    Serial.println("SD Card initialization failed!");
    // Keep trying to initialize
    while (!SD.begin()) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  Serial.println("SD Card initialized successfully");
  
  // Local variables
  RecordingState recordingState = {false, false, 0, 0, ""};
  File dataFile;
  unsigned long lastWrite = 0;

  // Fixed-size buffers for CSV formatting
  char gpsDataStr[128];
  char imuDataStr[128];
  char lidarDataStr[64];
  char dataLine[512]; // Adjust size as needed
  
  // Task loop
  while (true) {
    EventData eventData;
    // Check for recording state changes
    if (xQueuePeek(recordingQueue, &recordingState, 0) == pdTRUE) {
      // If new command received
      if (recordingState.newCommand) {
        // Clear flag
        recordingState.newCommand = false;
        xQueueOverwrite(recordingQueue, &recordingState);
        
        // Handle START command
        if (recordingState.isRecording) {
          // Close any existing file
          if (dataFile) {
            dataFile.close();
          }
          
          // Open new file
          dataFile = SD.open(recordingState.filename, FILE_WRITE);
          
          // Write headers
          if (dataFile) {
            dataFile.println("timestamp,gps_valid,latitude,longitude,satellites,altitude,speed,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,lidar_distance,lidar_strength,event");
            dataFile.flush();
            Serial.println("New recording file created: " + String(recordingState.filename));
          } else {
            Serial.println("Failed to create file!");
          }
        } 
        // Handle STOP command
        else {
          if (dataFile) {
            dataFile.close();
            Serial.println("Recording file closed");
          }
        }
      }
    }
    
    // If recording, write data periodically
    if (recordingState.isRecording && dataFile) {
      lastWrite = millis();
      
      // Get latest sensor data
      GpsData gpsData;
      ImuData imuData;
      LidarData lidarData;
      
      bool gpsActive = xQueuePeek(gpsQueue, &gpsData, 0) == pdTRUE;
      bool imuActive = xQueuePeek(imuQueue, &imuData, 0) == pdTRUE;
      bool lidarActive = xQueuePeek(lidarQueue, &lidarData, 0) == pdTRUE;
      
      // Format CSV line using snprintf
      int dataLen = snprintf(dataLine, sizeof(dataLine), "%lu,", millis());

      // GPS data
      if (gpsActive) {
        dataLen += snprintf(gpsDataStr, sizeof(gpsDataStr), "%d,%.9f,%.9f,%d,%.1f,%.2f,",
                            gpsData.isValid ? 1 : 0, gpsData.latitude, gpsData.longitude,
                            gpsData.satellites, gpsData.altitude, gpsData.speed);
      } else {
        dataLen += snprintf(gpsDataStr, sizeof(gpsDataStr), "0,0,0,0,0,0,");  // Extra 0 for speed
      }

      // IMU data
      if (imuActive) {
        xQueuePeek(imuQueue, &imuData, 0); // Ensure you're getting the latest IMU data
        dataLen += snprintf(imuDataStr, sizeof(imuDataStr), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                            imuData.accelX, imuData.accelY, imuData.accelZ,
                            imuData.gyroX, imuData.gyroY, imuData.gyroZ);
      } else {
        dataLen += snprintf(imuDataStr, sizeof(imuDataStr), "0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,");
      }

      // LIDAR data
      if (lidarActive) {
        dataLen += snprintf(lidarDataStr, sizeof(lidarDataStr), "%.2f,%.2f",
                            lidarData.distance, lidarData.strength);
      } else {
        dataLen += snprintf(lidarDataStr, sizeof(lidarDataStr), "0,0");
      }

      // Add event data if available
      char eventStr[16] = "";
      
      
      // Check for unprocessed events
      if (xQueuePeek(eventQueue, &eventData, 0) == pdTRUE && !eventData.processed) {
        // If event timestamp is close to current time, include it
        if (abs((long)(millis() - eventData.timestamp)) < 500) {
          strncpy(eventStr, eventData.eventType, sizeof(eventStr) - 1);
          eventStr[sizeof(eventStr) - 1] = '\0';
          
          // Mark as processed and update queue
          eventData.processed = true;
          xQueueOverwrite(eventQueue, &eventData);
        }
      }
      
      // Add event column to CSV line
      dataLen += snprintf(dataLine + strlen(dataLine), 
                          sizeof(dataLine) - strlen(dataLine), 
                          ",%s", eventStr);

      // Check if the formatted data fits into the buffer
      if (dataLen < sizeof(dataLine) - 1) {
        // Concatenate the formatted strings
        strcat(dataLine, gpsDataStr);
        strcat(dataLine, imuDataStr);
        strcat(dataLine, lidarDataStr);

        // Write data
        dataFile.println(dataLine);
      } else {
        Serial.println("Data line too long!");
      }
      
      // Flush to SD every 10 records (approx. 1 second)
      static int flushCounter = 0;
      if (++flushCounter >= 10) {
        dataFile.flush();
        flushCounter = 0;
      }
      
      // Update samples count
      recordingState.recordedSamples++;
      xQueueOverwrite(recordingQueue, &recordingState);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



void LIDAR_TASK(void *pvParameters) {
  // Initialize LIDAR sensor
  DFRobot_LIDAR07_IIC lidar;
  LidarData lidarData = {};
  
  // Wait for I2C to be available
  vTaskDelay(200 / portTICK_PERIOD_MS);
  
  if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
    // Inicializa o sensor LIDAR com endereço I2C padrão 0x62
    int i = 0;
    while (!lidar.begin() && i < 5) {
      Serial.println("Falha ao inicializar o LIDAR07! Tentativa " + String(i+1) + "/5");
      i++;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (i >= 5) {
      Serial.println("Tentativas de inicialização do LIDAR07 esgotadas!");
      xSemaphoreGive(i2cSemaphore);
      lidarActive = false;
      // Don't delete task, just mark as inactive
      while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Sleep forever
      }
    }

    // Configuração para taxa máxima - velocidade I2C aumentada
    Wire.setClock(400000); // Aumenta para 400kHz (máximo para I2C padrão)
    lidar.setConMeasureFreq(10); // Configura a frequência de medição contínua para 2ms (500Hz)
    // Configura o modo de medição contínua
    lidar.setMeasureMode(DFRobot_LIDAR07::eLidar07Continuous);
    lidar.startMeasure();
    lidar.startFilter(); // Ativa o filtro leve do sensor
    Serial.println("LIDAR07 inicializado com sucesso em modo de alta velocidade");
    xSemaphoreGive(i2cSemaphore);
  }
  
  // After initialization
  Serial.print("LIDAR initialization status: ");
  Serial.println(lidarActive ? "SUCCESS" : "FAILED");
  
  // Loop da task
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, 5 / portTICK_PERIOD_MS) == pdTRUE) {
      // Lê os dados do LIDAR
      if(lidar.getValue()){
         uint16_t measurement = lidar.getDistanceMM();
      uint16_t strength = lidar.getSignalAmplitude();
      
      // Liberamos o semáforo imediatamente após a leitura
        xSemaphoreGive(i2cSemaphore);
        
        // Processamos os dados fora da seção crítica1
        lidarData.distance = measurement / 10.0f;  // Converte mm para cm
        lidarData.strength = strength;
        lidarData.status = (measurement == 0) ? 2 : 
                          (strength < 100) ? 1 : 0;
        lidarData.timestamp = millis();
        /*Serial.printf("LIDAR Distance: %.2f cm, Strength: %.2f, Status: %d\n", 
                      lidarData.distance, lidarData.strength, lidarData.status);*/
        // Atualiza a fila com os dados mais recentes
        xQueueOverwrite(lidarQueue, &lidarData);
        }
     
    }
    
    // Menor intervalo possível entre leituras 
    // Taxa de aproximadamente 100Hz (máximo prático do sensor)
    vTaskDelay(10); // Praticamente sem delay, apenas cede o processador
  }   
}

void TCP_SERVER_TASK(void *pvParameters) {
  // Network configuration
  const char* ssid = "AP_ESP32";
  const char* password = "MCE_ROD_2025";
  EventData eventData;
  // Static IP Configuration
  IPAddress local_IP(192,168,20,55);
  IPAddress gateway(192,168,20,1);
  IPAddress subnet(255,255,255,0);
  
  // TCP Server and Client
  WiFiServer server(1000);
  WiFiClient client;
  
  // Local recording state initialization
  RecordingState recordingState = {false, false, 0, 0, ""};
  
  // Initialize WiFi in AP mode
  Serial.println("Setting up Access Point...");
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.softAPIP());
  
  // Start server and update recording queue
  server.begin();
  xQueueOverwrite(recordingQueue, &recordingState);
  
  unsigned long lastStatusUpdate = 0;
  
  // Task loop
  while (true) {
    volatile int event = 0;
    // Accept new client if needed
    if (!client || !client.connected()) {
      client = server.available();
      if (client) {
        Serial.println("Client connected!");
      }
    }
    
    // Handle incoming commands if a client is connected
    if (client && client.connected() && client.available()) {
      String command = client.readStringUntil('\n');
      command.trim();
      Serial.println("Command received: " + command);
      
      bool stateChanged = false;
      
      if (command == "START" && !recordingState.isRecording) {
        recordingState.isRecording = true;
        recordingState.newCommand = true;
        recordingState.startTime = millis();
        recordingState.recordedSamples = 0;
        sprintf(recordingState.filename, "/data_%lu.csv", millis());
        stateChanged = true;
        Serial.println("Recording started: " + String(recordingState.filename));
      } 
      else if (command == "STOP" && recordingState.isRecording) {
        recordingState.isRecording = false;
        recordingState.newCommand = true;
        stateChanged = true;
        Serial.println("Recording stopped");
      }
      else if (command == "buraco" || command == "lomba") {
        // Create event record
        EventData eventData;
        strncpy(eventData.eventType, command.c_str(), sizeof(eventData.eventType) - 1);
        eventData.eventType[sizeof(eventData.eventType) - 1] = '\0';  // Ensure null termination
        eventData.timestamp = millis();
        eventData.processed = false;
        
        // Get current GPS coordinates
        GpsData gpsData;
        if (xQueuePeek(gpsQueue, &gpsData, 0) == pdTRUE) {
          eventData.latitude = gpsData.latitude;
          eventData.longitude = gpsData.longitude;
        } else {
          eventData.latitude = 0.0;
          eventData.longitude = 0.0;
        }
        
        // Send to queue
        xQueueSend(eventQueue, &eventData, 0);
        
        // Acknowledge
        event = 1;

      }
      
      // Update queue if a state change occurred
      if (stateChanged) {
        xQueueOverwrite(recordingQueue, &recordingState);
      }
      
      // Prepare acknowledgment JSON
      StaticJsonDocument<200> jsonDoc;
      jsonDoc["status"] = recordingState.isRecording ? 1 : 0;
      jsonDoc["command"] = command;
      jsonDoc["result"] = stateChanged ? "success" : "no change";
      if (recordingState.isRecording) {
        jsonDoc["filename"] = recordingState.filename;
        jsonDoc["elapsed"] = (millis() - recordingState.startTime) / 1000;
      }
      if (event) {
          jsonDoc["status"] = "event_recorded";
          jsonDoc["event"] = eventData.eventType;
          jsonDoc["lat"] = eventData.latitude;
          jsonDoc["lng"] = eventData.longitude;
          event = 0;
      }
      
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      client.print(jsonString);
    }
    
    // Periodic status update every 1 second
    if (millis() - lastStatusUpdate > 1000) {
      lastStatusUpdate = millis();
      
      // Add these declarations
      ImuData imuData;
      bool imuActive = xQueuePeek(imuQueue, &imuData, 0) == pdTRUE;
      
      // Now the rest of your code will work
      client.println();
      client.print("Recording: ");
      client.println(recordingState.isRecording ? "ON" : "OFF");
      client.print("File: ");
      client.println(recordingState.filename);
      if (imuActive) {
        client.print("Accel X: ");
        client.println(imuData.accelX, 4);
        client.print("Accel Y: ");
        client.println(imuData.accelY, 4);
        client.print("Accel Z: ");
        client.println(imuData.accelZ, 4);
      }
    }
    
    // Small delay to yield to other tasks
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



