

//za PID
#include "PID_v1.h" 
//varijable za PID
double CiljnaVrijednost ; 
double UlazSenzora; 
double IzlaznaVrijednost ; 

//PID parameters
//double Kp=12, Ki=40, Kd=0.2; somewhat success
double Kp = 13, Ki=100, Kd=0.4; //needs a lot of time to stabilize
//double Kp = 7, Ki=40, Kd=0.6;
//premalo Kp, robot uvijek više ide u jednom smjeru ili agresivno krene u tom smjeru, previše Kp usitni naprijed nazad pobudali
// previše Kd, malo se treska, dobar Kd oporavi se brzo od vanjskih uticaja, tj kad se gurne
// Ki povećavaš dok ne utrefiš da dobro radi

PID myPID(&UlazSenzora, &IzlaznaVrijednost, &CiljnaVrijednost, Kp, Ki, Kd, DIRECT);

 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//#define IzlaznaVrijednost_READABLE_QUATERNION

//#define IzlaznaVrijednost_READABLE_EULER

//#define IzlaznaVrijednost_READABLE_YAWPITCHROLL

//#define IzlaznaVrijednost_READABLE_REALACCEL

//#define IzlaznaVrijednost_READABLE_WORLDACCEL

//#define IzlaznaVrijednost_TEAPOT

#define in1 9 //PWM pin.
#define in2 8 //PWM pin
#define in3 7 //PWM pin
#define in4 6 //PWM pin
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
int MIN_ABS_SPEED = 30;
int _currentSpeed;

 /*Also, if your motors are too fast, you can adjust the motorSpeedFactorLeft and motorSpeedFactorRight values.*/

bool blinkState = false;
char BluetoothData = 'S';
bool leftFlag, rightFlag = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orietation/motion vars
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

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
   // CiljnaVrijednost = 0;
    myPID.SetOutputLimits(-255,255);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(20);
    myPID.SetTunings(Kp, Ki, Kd);
    pinMode (in1, IzlaznaVrijednost);
    pinMode (in2, IzlaznaVrijednost);
    pinMode (in3, IzlaznaVrijednost);
    pinMode (in4, IzlaznaVrijednost);
    /* 
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);*/
    
    Serial1.begin(9600);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
    #endif
  
    Serial.begin(115200); 
    
    mpu.initialize();
    
    pinMode(INTERRUPT_PIN, INPUT);
  
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(-289);
    mpu.setYAccelOffset(209);
    mpu.setZAccelOffset(987);
    mpu.setXGyroOffset(56);
    mpu.setYGyroOffset(9);
    mpu.setZGyroOffset(-18);
    
    if (devStatus == 0) {
     
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    pinMode(LED_PIN, IzlaznaVrijednost);  
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        } 
       
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if(fifoCount < packetSize){}
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
          mpu.resetFIFO();
      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while(fifoCount >= packetSize){ 
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  
  blinkState = !blinkState;
  
  digitalWrite(LED_PIN, blinkState);
}



if (Serial1.available())
  {
    BluetoothData = Serial1.read();
   
 
 }

if(BluetoothData == 'S'){
    resetFlags();
  CiljnaVrijednost = 0;

}
else if(BluetoothData == 'F'){
    resetFlags();
  CiljnaVrijednost = -3;
   
}
else if(BluetoothData == 'B'){
   resetFlags();
  CiljnaVrijednost = 3;
}
else if(BluetoothData == 'R'){
  CiljnaVrijednost = 0;
 leftFlag = false;
  rightFlag = true;
}
else if(BluetoothData == 'L'){
  CiljnaVrijednost=0;
  rightFlag = false,
  leftFlag = true;
}
//Za PID
   
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  
  UlazSenzora = (euler[1] * 180/M_PI);
  myPID.Compute();

  //Serial.print(String(UlazSenzora) + ",");

  move(leftFlag, rightFlag); 
 
}
void move(bool left, bool right){
  
   
    int realSpeed;
    if (IzlaznaVrijednost < 0)
    {
       
        IzlaznaVrijednost = min(IzlaznaVrijednost, -1*MIN_ABS_SPEED);
        IzlaznaVrijednost = max(IzlaznaVrijednost, -255);
        realSpeed = max(MIN_ABS_SPEED, abs(IzlaznaVrijednost));
        
 //Serial.println((String)(realSpeed*0.5));
        digitalWrite(in1,LOW); 
        digitalWrite(in4,LOW);
        if(left){
          analogWrite(in2,realSpeed + 20); 
          analogWrite(in3, realSpeed - 20);
           //digitalWrite(in3,LOW);
        }
        else if(right){
            analogWrite(in3, realSpeed +20 );
            analogWrite(in2, realSpeed -20);
              //digitalWrite(in2,LOW);
        }
        else{
          
          analogWrite(in2, realSpeed);
           analogWrite(in3, realSpeed);
        }
    
       
    }
    else
    {
     
        IzlaznaVrijednost = max(IzlaznaVrijednost, MIN_ABS_SPEED);
        IzlaznaVrijednost = min(IzlaznaVrijednost, 255);
        realSpeed = max(MIN_ABS_SPEED, abs(IzlaznaVrijednost));
        
       // Serial.println((String)(realSpeed*0.5));
         analogWrite(in1,realSpeed);
      digitalWrite(in2,LOW);
      analogWrite(in4,realSpeed);
      digitalWrite(in3,LOW);
       
    }
    
   
   
   

}

void resetFlags(){
  rightFlag =false;
  leftFlag=false;
}
