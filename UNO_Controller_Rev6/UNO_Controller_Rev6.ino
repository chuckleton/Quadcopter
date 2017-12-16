#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

#define SERIAL_RATE 115200                  //Serial rate

/*********************** USER PARAMETERS ***************************/
#define NUMCH 6                             //Number of radio channels
#define SUPPRESS_YAW_CONTROL 1              //Control Quad Yaw (1 = NO)

#define USE_ANGLE_PID 0                     //0 = only rate P loop (Acrobatic mode), 1 = rate and angle loops (stabilization mode)

#define TUNE_RATE_KP                        //which gain to tune
//#define TUNE_ANGLE_KP                     //only one should be uncommented at a time
//#define TUNE_ANGLE_KI                     //if no gains are being tuned comment all

//DEBUG parameters
//#define PRINT_DEBUG                       //Print info over Serial (115200 baud) (comment this line to not print any debug)
#define PRINT_ATTITUDE 0                    //Print Roll and Pitch angles
#define PRINT_ATTITUDE_RATES 0              //Print Roll, Pitch, and Yaw rates
#define PRINT_RC_VALUES 1                   //Print adjusted RC inputs
#define PRINT_GAINS 0                       //Print PID Gains
#define PRINT_ARMED 0                       //Print whether the quad is armed
#define PRINT_CONTROL 0                     //Print control values from PID
#define PRINT_OUTPUTS 0                     //Print final motor speeds
#define PRINT_DT 0                          //Print loop refresh rate

//Motors
#define FRONT_LEFT 0                             //Front motor pin
#define FRONT_RIGHT 1                              //Back motor pin
#define BACK_LEFT 2                             //Left motor pin
#define BACK_RIGHT 3                            //Right motor pin

#define GAIN 1                              //Gain to adjust, 0: roll, 1: pitch, 2: yaw
#define GAIN_DIVISOR 3825.0                 //What to divide raw gain input by (Gain = input (0-255) / GAIN_DIVISOR

#define KpRoll 0.0                          //Proportional Gains (for rate loop)
#define KpPitch 0.0
#define KpYaw 0.0                           //Yaw is only rate loop
#define KiYaw 0.0
#define KdYaw 0.0

#define KpRolla 0.0                         //Proportional Gains (for angle loop)
#define KpPitcha 0.0
#define KiRolla 0.0                         //Integral Gains (for angle loop)
#define KiPitcha 0.0

#define SERVO_FREQ 200                      //Frequency at which to update servo values
#define SERVOMIN 80*(1000000/SERVO_FREQ)/4096   //Minimum servo value (~1000 us)
#define SERVOMAX 2.35*SERVOMIN              //Max servo value (~2000 us)

#define GYRO_MAF_NR 2                       //NOT IN USE
/********************** END USER PARAMETERS *************************/

#define OFF SERVOMIN                              //Servo value when motor is off

#include "quaternionFilters.h"
#include "MPU9250.h"

MPU9250 myIMU;

double controls[3] = {0.0,0.0,0.0};         //control for each motor
double angControls[2] = {0.0,0.0};
double motVals[4] = {0.0,0.0,0.0,0.0};      //control for each motor

unsigned long lastIMUReading = 0;

unsigned long currTime = 0;
#if PRINT_DT == 1
  unsigned long dt;                         //Time passed (milliseconds)
#endif
unsigned long oldtime;                      //Last loop run time

Adafruit_PWMServoDriver mot = Adafruit_PWMServoDriver(0x40);

byte raw[NUMCH] = {0,0,0,0,0,0};                                 // [Power, Roll, Pitch, Yaw, Gain, Armed]
volatile double inputs[] = {0.0,0.0,0.0,0.0,0.0};                // [Power, Roll, Pitch, Yaw, Gain]
boolean ARMED = false;

byte loopCnt = 0;

void setup(){
  Serial.begin(SERIAL_RATE);            //Start serial
  init_imu();                               //Initialize MPU-9250
  initializePID();                          //Initial setup for PID objects
  mot.begin();                              //initialize PWM Board
  mot.setPWMFreq(SERVO_FREQ);               //Set PWM Frequency
  oldtime = millis();
}

void loop(){
  currTime = millis();
  if(currTime - oldtime > 6){              //~167Hz
    #if PRINT_DT == 1
      dt = currTime - oldtime;            //Calculate loop time (milliseconds)
    #endif
    oldtime = currTime;
    updateIMU();
    if(oldtime - lastIMUReading > 150 || !ARMED){
      stopMotors();                         //Stop all motors if no imu data in .15 seconds
      //Serial.println("STOPPING");
    }else{
      control();                            //Power the motors and run PID
    }
    loopCnt++;
    if(loopCnt > 13){
      //Serial.println(currTime - lastIMUReading);
      #ifdef PRINT_DEBUG
        Debug();                              //Print debugging statements
      #endif
      loopCnt = 0;
    }
  }
}

