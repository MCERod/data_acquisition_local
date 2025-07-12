# ESP32 Data Acquisition System - Flowchart

```mermaid
flowchart TD
    A[System Start] --> B[Initialize Serial & I2C]
    B --> C[Scan I2C Bus]
    C --> D[Create Queues & Semaphores]
    D --> E[Create FreeRTOS Tasks]
    E --> F[Delete Main Loop Task]
    
    %% Task Creation
    E --> G1[GPS_TASK]
    E --> G2[IMU_TASK]
    E --> G3[LIDAR_TASK]
    E --> G4[SD_CARD]
    E --> G5[TCP_SERVER_TASK]
    
    %% GPS Task Flow
    G1 --> H1[Initialize GPS Serial]
    H1 --> I1{GPS Data Available?}
    I1 -->|Yes| J1[Parse GPS Data]
    I1 -->|No| K1[Check Connection]
    J1 --> L1[Update GPS Struct]
    K1 --> M1{Connection OK?}
    M1 -->|No| N1[Set Invalid Flag]
    M1 -->|Yes| L1
    N1 --> L1
    L1 --> O1[Send to GPS Queue]
    O1 --> P1[Delay 100ms]
    P1 --> I1
    
    %% IMU Task Flow
    G2 --> H2[Wait for I2C Ready]
    H2 --> I2[Initialize MPU6050]
    I2 --> J2{MPU6050 Connected?}
    J2 -->|No| K2[Print Error]
    J2 -->|Yes| L2[Configure & Calibrate]
    K2 --> M2[Continue Loop]
    L2 --> M2[Read Motion Data]
    M2 --> N2[Apply Filters]
    N2 --> O2[Remove Gravity]
    O2 --> P2[Moving Average]
    P2 --> Q2[Send to IMU Queue]
    Q2 --> R2[Delay 10ms]
    R2 --> M2
    
    %% LIDAR Task Flow
    G3 --> H3[Wait for I2C Ready]
    H3 --> I3[Initialize LIDAR]
    I3 --> J3{LIDAR Connected?}
    J3 -->|No| K3[Retry Init]
    J3 -->|Yes| L3[Configure Continuous Mode]
    K3 --> I3
    L3 --> M3[Start Measurement]
    M3 --> N3{LIDAR Data Ready?}
    N3 -->|Yes| O3[Read Distance & Strength]
    N3 -->|No| P3[Delay 100ms]
    O3 --> Q3{Signal Strength > 80?}
    Q3 -->|Yes| R3[Send to LIDAR Queue]
    Q3 -->|No| P3
    R3 --> P3
    P3 --> N3
    
    %% SD Card Task Flow
    G4 --> H4[Initialize SD Card]
    H4 --> I4{SD Card Ready?}
    I4 -->|No| J4[Retry Init]
    I4 -->|Yes| K4[Main SD Loop]
    J4 --> I4
    K4 --> L4{Recording Command?}
    L4 -->|Start| M4[Create New File]
    L4 -->|Stop| N4[Close File]
    L4 -->|None| O4{Currently Recording?}
    M4 --> P4[Write CSV Header]
    N4 --> O4
    P4 --> O4
    O4 -->|Yes| Q4[Read Sensor Queues]
    O4 -->|No| R4[Delay 100ms]
    Q4 --> S4[Format CSV Line]
    S4 --> T4[Write to SD Card]
    T4 --> U4{Flush Counter = 10?}
    U4 -->|Yes| V4[Flush to SD]
    U4 -->|No| R4
    V4 --> R4
    R4 --> L4
    
    %% TCP Server Task Flow
    G5 --> H5[Setup WiFi AP]
    H5 --> I5[Start TCP Server]
    I5 --> J5[Main TCP Loop]
    J5 --> K5{Client Connected?}
    K5 -->|No| L5[Wait for Client]
    K5 -->|Yes| M5{Command Available?}
    L5 --> J5
    M5 -->|Yes| N5[Read Command]
    M5 -->|No| O5{Status Update Time?}
    N5 --> P5{Command Type?}
    P5 -->|START| Q5[Start Recording]
    P5 -->|STOP| R5[Stop Recording]
    P5 -->|buraco/lomba| S5[Create Event]
    Q5 --> T5[Send JSON Response]
    R5 --> T5
    S5 --> U5[Add to Event Queue]
    U5 --> T5
    T5 --> O5
    O5 -->|Yes| V5[Send Status Data]
    O5 -->|No| W5[Delay 100ms]
    V5 --> W5
    W5 --> J5
    
    %% Data Flow
    classDef queueClass fill:#e1f5fe
    classDef taskClass fill:#f3e5f5
    classDef actionClass fill:#e8f5e8
    
    class G1,G2,G3,G4,G5 taskClass
    class O1,Q2,R3,T4,U5 queueClass
    class A,B,C,D,M4,S4,Q5,R5 actionClass
```

## System Overview

This ESP32-based data acquisition system consists of 5 main FreeRTOS tasks running concurrently:

### 1. **GPS_TASK** (Priority: 2)
- Reads GPS data via SoftwareSerial (pins 25, 26)
- Parses NMEA data using TinyGPSPlus library
- Updates GPS data structure with position, speed, altitude, etc.
- Sends data to GPS queue every 100ms

### 2. **IMU_TASK** (Priority: 3)
- Manages MPU6050 accelerometer/gyroscope
- Applies complementary filter to estimate gravity
- Removes gravity from acceleration readings
- Applies moving average filter (10 samples)
- Sends filtered data to IMU queue every 10ms

### 3. **LIDAR_TASK** (Priority: 3)
- Controls DFRobot LIDAR07 sensor
- Operates in continuous measurement mode at 10Hz
- Filters readings based on signal strength (>80)
- Sends distance and strength data to LIDAR queue

### 4. **SD_CARD** (Priority: 2)
- Manages SD card file operations
- Monitors recording state from TCP commands
- Writes CSV data combining GPS, IMU, LIDAR, and event data
- Flushes data to SD card every 10 records

### 5. **TCP_SERVER_TASK** (Priority: 1)
- Creates WiFi Access Point (192.168.20.55)
- Listens for TCP connections on port 1000
- Handles commands: START, STOP, buraco, lomba
- Sends periodic status updates with sensor data
- Manages recording state and event logging

### Key Features:
- **Concurrent Operation**: All sensors read simultaneously using FreeRTOS
- **Thread-Safe**: Uses queues and semaphores for inter-task communication
- **Remote Control**: TCP server allows wireless control and monitoring
- **Event Logging**: Records road events (holes, speed bumps) with GPS coordinates
- **Real-time Data**: 10Hz IMU, 10Hz LIDAR, 1Hz GPS sampling rates
- **Data Storage**: CSV format with timestamps for all sensor readings

The system is designed for vehicle-based data acquisition, capturing motion, position, and distance measurements while allowing remote control and event marking during operation.
