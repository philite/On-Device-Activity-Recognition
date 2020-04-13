# Realtime On-Device Human Activity Recognition



**NOTE: For some reason my MPU accelerometer's XYZ values driftted every time I record so I use gyroscope instead.

Equipment: ESP8266, MPU6050 (IMU)
Arduino Libraries: 
* [I2cdevlibs for reading MPU6050 values](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050)
Python Libraries:
* [Emlearn for converting sklearn's model that can put inside ESP8266](https://github.com/emlearn/emlearn)
* Numpy, pandas for data manipulation
* Sklearn for Machine Learning modeling
* PySerial for serial monitoring

Instruction:

## Connecting

1. Connect ESP8266 to MPU6050 

* MPU6050/GY521 --> ESP8266/NodeMCU Connection
*  VCC  -->   VU (3.3V is enough)
*  GND  -->   G  (Ground)
*  SCL  -->   D1 (GPIO05 - I2C clock)
*  SDA  -->   D2 (GPIO04 - I2C data)   
*  INT  -->   D8 (GPIO15 - Interrupt pin, optional if not using interrupt)

[from MPU6050_DMP6_ESPWiFi](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6_ESPWiFi/MPU6050_DMP6_ESPWiFi.ino)

2. (Optional) If not sure of I2C address, use I2CScanner. 
3. (Optional) [Calibrate MPU6050 offset using IMU_ZERO](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/IMU_Zero/IMU_Zero.ino)

## Arduino Code

[for more information about MPU6050](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050)

4. Include libraries --> I2Cdev.h, MPU6050.h, Wire.h
5. Variables

* #define SCL_PIN 5 // SCL     D1 (GPIO05)   I2C clock
* #define SDA_PIN 4 // SDA     D2 (GPIO04)   I2C data
* int16_t ax, ay, az; // Accelerometer
* int16_t gx, gy, gz; // Gyroscope
* MPU6050 mpu; // MPU

6. Initialize Wire and MPU
* Wire.begin(SDA_PIN, SCL_PIN); // (SDA, SCL)
* mpu.initialize();

7. Read raw data
* mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

## Record Data in Time Window
ESP8266 record in 100Mhz, 100 data points per second

[Code adapted from "How to do gesture identification on arduino"](https://eloquentarduino.github.io/2019/12/how-to-do-gesture-identification-on-arduino/)

8. Variables
* #define WINDOW_SIZE 150 // 1.5 seconds of 1 row (100Hz) [for better performance from 2014 research "Window Size Impact in Human Activity Recognition"](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4029702/)
* #define NUM_AXES 3 // 6 if record both accelerometer and gyroscope
* float features[WINDOW_SIZE * NUM_AXES]; // 1 row (1 time window) = WINDOW_SIZE * NUM_AXES for example 1.5 seconds time window on 3 axes = 450 columns (150 X, 150 Y, 150 Z)

9. Function

void getIMU_Window() // For recording
void printFeatures_Window() // Print recorded values to serial monitor

10. Save values from serial monitor and convert to csv and use sklearn to model the machine learning algorithm



