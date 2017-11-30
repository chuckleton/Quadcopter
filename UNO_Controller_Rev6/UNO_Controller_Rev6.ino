#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

/*********************** USER PARAMETERS ***************************/
#define NUMCH 5                             //Number of radio channels
#define SUPPRESS_YAW_CONTROL 1              //Control Quad Yaw (1 = NO)

//DEBUG parameters
#define PRINT_DEBUG                         //Print info over Serial (115200 baud)
#define PRINT_ATTITUDE 0                    //Print Roll, Pitch, and Yaw angles
#define PRINT_RC_VALUES 0                   //Print adjusted RC inputs
#define PRINT_GAINS 1                       //Print PID Gains
#define PRINT_CONTROL 0                     //Print control values from PID
#define PRINT_OUTPUTS 0                     //Print final motor speeds
#define PRINT_DT 0                          //Print loop refresh rate
#define PRINT_OLDPROPORTIONAL 0             //Print last proportional error
#define PRINT_PROPORTIONAL 0                //Print proportional error
#define PRINT_INTEGRAL 0                    //Print integral error
#define PRINT_DERIVATIVE 0                  //Print derivative error

//Motors
#define FRONT 8                             //Front motor pin
#define BACK 9                              //Back motor pin
#define LEFT 10                              //Left motor pin
#define RIGHT 11                             //Right motor pin

#define GAIN 1                              //Gain to adjust, 0: roll, 1: pitch, 2: yaw
#define GAIN_DIVISOR 3825.0                  //What to divide raw gain input by (Gain = input (0-255) / GAIN_DIVISOR

#define KpRoll 0.0                          //Proportional Gains
#define KpPitch 0.0
#define KpYaw 0.0
#define KiRoll 0.0                          //Integral Gains (should usually be 0)
#define KiPitch 0.0
#define KiYaw 0.0
#define KdRoll 0.0                          //Derivative Gains
#define KdPitch 0.0
#define KdYaw 0.0

#define SERVO_FREQ 200                      //Frequency at which to update servo values
#define SERVOMIN 80*(1000000/SERVO_FREQ)/4096   //Minimum servo value (~1000 us)
#define SERVOMAX 2*SERVOMIN                 //Max servo value (~2000 us)

#define GYRO_MAF_NR 2                       //NOT IN USE
/********************** END USER PARAMETERS *************************/
#include "quaternionFilters.h"
#include "MPU9250.h"

MPU9250 myIMU;

double controls[3] = {0.0,0.0,0.0};      //control for each motor
double motVals[4] = {0.0,0.0,0.0,0.0};      //control for each motor

unsigned long lastIMUReading = 0;

//unsigned long currTime = 0;
//unsigned long dt;                                    //Time passed (milliseconds)
//float Dt;                                   //Time passed (seconds)
unsigned long oldtime;                               //Last loop run time

const int MOTORPINS[4] = {FRONT,BACK,LEFT,RIGHT}; //Motor Pins (0: FRONT, 1: BACK, 2: LEFT, 3: RIGHT)
Adafruit_PWMServoDriver mot = Adafruit_PWMServoDriver(0x40);

#define RX 0                                //Pin for serial input in arduino
#define IMU_SERIAL_RATE 115200              //Serial rate for ArduIMU

#define OFF SERVOMIN                              //Servo value when motor is off

int power1 = 125;
int testnum = 0;

byte raw[NUMCH] = {0,0,0,0,0};
volatile double inputs[] = {0.0,0.0,0.0,0.0,0.0};                // to the receiver's channels in the order listed here

byte loopCnt = 0;

void setup(){
  Serial.begin(IMU_SERIAL_RATE);            //Start serial
  init_imu();                               //Initialize MPU-9250
  initializePID();                          //Initial setup for PID objects
  mot.begin();                              //initialize PWM Board
  mot.setPWMFreq(SERVO_FREQ);               //Set PWM Frequency
  oldtime = millis();
}

void loop(){
  if(millis() - oldtime > 6){              //~167Hz
    oldtime = millis();
    updateIMU();
    //dt = currTime - oldtime;                //Calculate loop time (milliseconds)
    //Dt = (float)dt / 1000.0;                //Convert loop time to seconds
    if(oldtime - lastIMUReading > 150){
      stopMotors();                         //Stop all motors if no imu data in .15 seconds
      //Serial.println("NO IMU DATA");
    }else{
      control();
    }
    //Power the motors and run PID
    loopCnt++;
    if(loopCnt > 13){
      //outputRadioValues();
      //printIMURates();
     //Serial.println(currTime - lastIMUReading);
      #ifdef PRINT_DEBUG
        Debug();                              //Print debugging statements
      #endif
      loopCnt = 0;
    }
  }
}

/*void test1(){
  motors[0].write(power1);
  motors[1].write(power1);
  motors[2].write(power1);
  motors[3].write(power1);
  delay(10);
}*/

/*void test(){                                //Test method
  testnum++;                                //Turns on motor for 5.5 seconds and then turns it off
  Serial.println(testnum);
  if(testnum == 1){
    motors[0].write(OFF);
    motors[1].write(OFF);
    motors[2].write(OFF);
    motors[3].write(OFF);
    delay(2000);
  }
  Serial.println("STARTING");
  motors[0].write(power1);
  motors[1].write(power1);
  motors[2].write(power1);
  motors[3].write(power1);
  delay(5500);
  Serial.println("STOPPING");
  motors[0].write(OFF);
  motors[1].write(OFF);
  motors[2].write(OFF);
  motors[3].write(OFF);
  delay(1800);
}*/

