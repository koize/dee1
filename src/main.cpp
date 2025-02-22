#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include "QmuTactile.h"
#include "sbus.h"
#include "math.h"
#include "types.h"
#include "SSD1306.h"
#include "oled_display.h"
#include "device_node.h"
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

using namespace std; 

// Function prototypes
void dmpDataReady();
void IRAM_ATTR onPpmTimer();
int getRcChannel_wrapper(uint8_t channel);
void outputSubtask();
void ioTaskHandler(void *pvParameters);
void imuSubtask();
void i2cResourceTaskHandler(void *pvParameters);
int angleToRcChannel(float angle);
int joystickToRcChannel(float angle);


/*
 * Choose Trainer output type. Uncommend correcty line
 */
//#define TRAINER_MODE_SBUS
#define TRAINER_MODE_PPM
#define MPU6050_UPDATE_TASK_MS 25
#define OUTPUT_UPDATE_TASK_MS 20
#define SERIAL_TASK_MS 50
#define SERIAL1_RX 25
#define SERIAL1_TX 14
#define I2C1_SDA_PIN 21
#define I2C1_SCL_PIN 22
int THR_VAL = 1000;
#define PIN_THR_DOWN 4
#define PIN_THR_UP 0
#define PIN_ARM 2
#define PIN_NUKE 13
#define PIN_CALIBRATE 15

bool armFlag = false;

//Adafruit_MPU6050 mpu;
//sensors_event_t acc, gyro, temp;

uint32_t nextSerialTaskMs = 0;

imuData_t imu;
dataOutput_t output;
thumb_joystick_t thumbJoystick;
DeviceNode device;

//MPU6050 mpu(Wire);
MPU6050 mpu;
int roll, pitch, yaw;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#ifdef TRAINER_MODE_SBUS
#define SBUS_UPDATE_TASK_MS 15
uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
uint32_t nextSbusTaskMs = 0;
#endif

//#ifdef TRAINER_MODE_PPM

#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 8

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum ppmState_e {
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};

void IRAM_ATTR onPpmTimer() {

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = LOW;
    static int usedFrameLength = 0;
    int currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE) {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
    }

    if (ppmState == PPM_STATE_PULSE) {
        ppmOutput = HIGH;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    } else if (ppmState == PPM_STATE_FILL) {
        ppmOutput = LOW;
        currentChannelValue = getRcChannel_wrapper(ppmChannel);
        
        ppmChannel++;
        ppmState = PPM_STATE_PULSE;

        if (ppmChannel > PPM_CHANNELS) {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        } else {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(SERIAL1_TX, ppmOutput);
}

//#endif

TaskHandle_t i2cResourceTask;
TaskHandle_t ioTask;

QmuTactile buttonThrDown(PIN_THR_DOWN);
QmuTactile buttonThrUp(PIN_THR_UP);
QmuTactile buttonArm(PIN_ARM);
QmuTactile buttonNuke(PIN_NUKE);
QmuTactile buttonCalibrate(PIN_CALIBRATE);


void setup()
{
    Serial.begin(115200);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setTimeout(3000); //timeout value in uSec. Used to fix Serial Freezing after few secs
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

     // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-83.9);
    mpu.setZAccelOffset(820); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    /*
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); // gyro and accelero
    Serial.println("Done!\n");*/

#ifdef TRAINER_MODE_PPM
    pinMode(SERIAL1_TX, OUTPUT);
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onPpmTimer, true);
    timerAlarmWrite(timer, 12000, true);
    timerAlarmEnable(timer);
#endif

#ifdef TRAINER_MODE_SBUS
    sbusSerial.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
#endif

    buttonThrDown.start();
    buttonThrUp.start();    
    buttonArm.start();
    buttonNuke.start();
    buttonCalibrate.start();
 

    xTaskCreatePinnedToCore(
        i2cResourceTaskHandler, /* Function to implement the task */
        "imuTask",              /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        0,                      /* Priority of the task */
        &i2cResourceTask,       /* Task handle. */
        0);

    xTaskCreatePinnedToCore(
        ioTaskHandler, /* Function to implement the task */
        "outputTask",  /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        0,             /* Priority of the task */
        &ioTask,       /* Task handle. */
        0);

    
   
}

int getRcChannel_wrapper(uint8_t channel)
{
    if (channel >= 0 && channel < SBUS_CHANNEL_COUNT)
    {
        return output.channels[channel];
    }
    else
    {
        return DEFAULT_CHANNEL_VALUE;
    }
}


void outputSubtask()
{
   
    output.channels[ROLL] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(roll);
    output.channels[PITCH] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(pitch);
    //output.channels[YAW] = DEFAULT_CHANNEL_VALUE;
    output.channels[YAW] = DEFAULT_CHANNEL_VALUE + (0.2 * angleToRcChannel(yaw));
    if (buttonThrDown.getFlags() & TACTILE_FLAG_EDGE_PRESSED)
    {
        if (THR_VAL - THROTTLE_BUTTON_STEP < 1000) // 1000 is the minimum value for throttle
        {
            output.channels[THROTTLE] = 1000;
            Serial.println("THR_DEC: 1000");
            THR_VAL = 1000;
        }
        else
        {
            output.channels[THROTTLE] = THR_VAL - THROTTLE_BUTTON_STEP;
            Serial.println("THR_DEC: " + (THR_VAL - THROTTLE_BUTTON_STEP));
            THR_VAL -= THROTTLE_BUTTON_STEP;
        }
   
    }
  
    if (buttonThrUp.getFlags() & TACTILE_FLAG_EDGE_PRESSED)
    {
        if (THR_VAL + THROTTLE_BUTTON_STEP > 2000) // 2000 is the maximum value for throttle
        {
            output.channels[THROTTLE] = 2000;
            Serial.println("THR_INC: 2000");
            THR_VAL = 2000;
        }
        else
        {
            output.channels[THROTTLE] = THR_VAL + THROTTLE_BUTTON_STEP;
            Serial.println("THR_INC: " + (THR_VAL + THROTTLE_BUTTON_STEP));
            THR_VAL += THROTTLE_BUTTON_STEP;
        }
    }

    if (buttonArm.getFlags() & TACTILE_FLAG_EDGE_PRESSED)
    {
        armFlag = !armFlag;
        Serial.println("arm button pressed");
        if (armFlag)
        {
            Serial.println("armed");
            output.channels[AUX1_ARM] = 2000;
        }
        else
        {
            Serial.println("disarmed");
            output.channels[AUX1_ARM] = 1000;
        }
    }

    if (buttonNuke.getFlags() & TACTILE_FLAG_EDGE_PRESSED)
    {
        output.channels[AUX2_NUKE] = 1000;
        delay(150);
        Serial.println("NUKE");
    }
    else {
        output.channels[AUX2_NUKE] = 2000;
    }

    if (buttonCalibrate.getState() == TACTILE_STATE_SHORT_PRESS)
    {
        Serial.println(F("recalibrating"));
        output.channels[ROLL] = DEFAULT_CHANNEL_VALUE;
        output.channels[PITCH] = DEFAULT_CHANNEL_VALUE;
        output.channels[YAW] = DEFAULT_CHANNEL_VALUE;

        dmpReady = false;
        //mpu.setDMPEnabled(false);
        //mpu.initialize();
        //mpu.resetSensors();
        mpu.CalibrateGyro(1);
        mpu.CalibrateAccel(1);
        dmpReady = true;
        //mpu.PrintActiveOffsets();
        //mpu.setDMPEnabled(true);
        //dmpReady = true;

        /*
        mpu.calcOffsets();
        mpu.resetAllAngles();
        Serial.println("Done!\n");
        */
    }

    for (uint8_t i = 0; i < 16; i++) {
        output.channels[i] = constrain(output.channels[i], 1000, 2000);
    }

    
}

void ioTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = OUTPUT_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        buttonThrDown.loop();
        buttonThrUp.loop();
        buttonArm.loop();
        buttonNuke.loop();
        buttonCalibrate.loop();
       

        outputSubtask();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

void imuSubtask()
{
    static uint32_t prevMicros = 0;
    float dT = (micros() - prevMicros) * 0.000001f;
    prevMicros = micros();

    if (prevMicros > 0)
    {
         if (!dmpReady) return;

         if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            roll = ypr[2] * 180/M_PI;
            pitch = ypr[1] * 180/M_PI;
            yaw = ypr[0] * 180/M_PI;
    }
        /*
        mpu.update();
        roll = mpu.getAngleX();
        pitch = mpu.getAngleY();
        yaw = mpu.getAngleZ();

      
        //Serial.println(mpu.getGyroZoffset());
        //Serial.println(mpu.getAccZoffset());
        */
        
        Serial.print("X : ");
        Serial.print(roll);
        Serial.print("\tY : ");
        Serial.print(pitch);
        Serial.print("\tZ : ");
        Serial.println(yaw);
        
    }
}

void i2cResourceTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = MPU6050_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        imuSubtask();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

int angleToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 5.0f), -45.0f, 45.0f); //5 deg deadband
    return (int)fscalef(value, -45.0f, 45.0f, -500, 500);
}

int joystickToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 0.02f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, -200, 200);
}

void loop()
{
    /* 
     * Send Trainer data in SBUS stream
     */
#ifdef TRAINER_MODE_SBUS
    if (millis() > nextSbusTaskMs)
    {
        sbusPreparePacket(sbusPacket, false, false, getRcChannel_wrapper);
        sbusSerial.write(sbusPacket, SBUS_PACKET_LENGTH);
        nextSbusTaskMs = millis() + SBUS_UPDATE_TASK_MS;
    }
#endif

    if (millis() > nextSerialTaskMs)
    {
        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}