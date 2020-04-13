// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"

// // ISR not in IRAM!
// void ICACHE_RAM_ATTR ISRoutine ();

// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//#include "lgb.h"
//#include "rf_model.h"
#include "rf150_running_model.h"

MPU6050 mpu;


// # For use with MPU6050_6Axis_MotionApps20.h

// MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// Pin value
#define SCL_PIN 5 // SCL     D1 (GPIO05)   I2C clock
#define SDA_PIN 4 // SDA     D2 (GPIO04)   I2C data

// Data acquisition
#define WINDOW_SIZE 150 // 1.5 seconds of 1 row (100Hz)
#define NUM_AXES 3 // acc doesn't accurate for some reason?

// # For Emlearn
float baseline[NUM_AXES]; // Calibration
float features[WINDOW_SIZE * NUM_AXES];

// # For M2Cgen
// double baseline[NUM_AXES]; // Calibration
// double features[WINDOW_SIZE * NUM_AXES];

// Accelerometer & Gyroscope vars
int16_t ax, ay, az;
int16_t gx, gy, gz;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// for initial calibration
//#define CALIBRATION

// Calibrated OUTPUT_READABLE_ACCELGYRO
//#define CALIBRATED_READABLE_ACCELGYRO

// For recording
//#define GET_DATA_Window
//#define PRINT_DATA_Window

// For acitivity recognition
#define GET_DATA_AND_CLASSIFY


void calibrate(){
    for (int i=0; i<10; i++){
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        delay(100);
    }
    baseline[0] = ax;
    baseline[1] = ay;
    baseline[2] = az;
    baseline[3] = gx;
    baseline[4] = gy;
    baseline[5] = gz;

    Serial.print("Baseline = ");
    Serial.print(baseline[0]); Serial.print(",");
    Serial.print(baseline[1]); Serial.print(",");
    Serial.print(baseline[2]); Serial.print(",");
    Serial.print(baseline[3]); Serial.print(",");
    Serial.print(baseline[4]); Serial.print(",");
    Serial.println(baseline[5]);
}



void getIMU_Window() {

    for (int i = 0; i < WINDOW_SIZE; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        features[i * NUM_AXES + 0] = gx;
        features[i * NUM_AXES + 1] = gy;
        features[i * NUM_AXES + 2] = gz;

    }
}

void printFeatures_Window() {
    const uint16_t numFeatures = sizeof(features) / sizeof(float);
    
    for (int i = 0; i < numFeatures; i++) {
        Serial.print(features[i]);
        // Serial.print(i == numFeatures - 1 ? 'n': ',');
        if (i == (numFeatures - 1)){
            Serial.println();
        }
        else {
            Serial.print(',');
        }
    }
}

void classify() {
    //double value;
    Serial.print("Detected gesture: ");
    Serial.println(rf150_running_model_predict_tree_0(features, 450));
    // score(features, &value);
    // Serial.println(value);
    
}

void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA_PIN, SCL_PIN); // (SDA, SCL)
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);
    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(210);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    /* Using IMU_ZERO
    -4110,   -2508,    1298,      34,      27,     -11

    [-4111,-4110] --> [0,6]    
    [-2500,-2499] --> [-4,8]        
    [1301,1302] --> [16363,16398]   
    [35,36] --> [-5,2]      
    [30,31] --> [-18,1]       
    [-12,-11] --> [0,2]
    
    */

    mpu.setXAccelOffset(-4111);
    mpu.setYAccelOffset(-2500);
    mpu.setZAccelOffset(1302);
    mpu.setXGyroOffset(36);
    mpu.setYGyroOffset(31);
    mpu.setZGyroOffset(-12);

    #ifdef CALIBRATION
        calibrate();
    #endif
}

void loop() {
    // read raw accel/gyro measurements from device

    #ifdef OUTPUT_READABLE_ACCELGYRO
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // display tab-separated accel/gyro x/y/z values
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
    #endif

    #ifdef CALIBRATED_READABLE_ACCELGYRO
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // display tab-separated accel/gyro x/y/z values
        ax = ax - baseline[0];
        ay = ay - baseline[1];
        az = az - baseline[2];
        gx = gx - baseline[3];
        gy = gy - baseline[4];
        gz = gz - baseline[5];

        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
    #endif

    #ifdef GET_DATA_Window
        getIMU_Window();
    #endif

    #ifdef PRINT_DATA_Window
        printFeatures_Window();
    #endif

    #ifdef GET_DATA_AND_CLASSIFY
        delay(275);
        getIMU_Window();
        classify();
    #endif

}
