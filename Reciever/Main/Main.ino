#include <EnableInterrupt.h>

/*
Copyright 2011 Lex Talionis (Lex.V.Talionis at gmail)
This program is free software: you can redistribute it 
and/or modify it under the terms of the GNU General Public 
License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any 
later version.

This code uses pin change interrupts and timer 1 to measure the 
time between the rise and fall of 3 channels of PPM 
(Though often called PWM, see http://forum.arduino.cc/index.php/topic,14146.html)
on a typical RC car receiver.  It could be extended to as
many channels as you like.  It uses the PinChangeInt library
to notice when the signal pin goes high and low, and the 
Timer1 library to record the time between.
*/

#define USE_RATE_PID

#define SERVO_FREQ 200                      //Frequency at which to update servo values
#define SERVOMIN 80*(1000000/SERVO_FREQ)/4096   //Minimum servo value (~1000 us)
#define SERVOMAX 2.35*SERVOMIN                 //Max servo value (~2000 us)

#define PIN_COUNT 6           //number of channels attached to the receiver

#define RC_PWR 2    //Arduino pins attached to the receiver
#define RC_RLL 4
#define RC_PTCH 3
#define RC_YAW 6
#define RC_KP 5
#define RC_ARM 7

#define LOOP_TIME 15
#define OFF SERVOMIN

byte pin[] = {RC_PWR, RC_RLL, RC_PTCH, RC_YAW, RC_KP, RC_ARM};    //for maximum efficiency these pins should be attached

#ifdef USE_RATE_PID
  const int PINS[4][PIN_COUNT] = {                //Pins for radio with adjustment parameters
    {1104, 1096, 1028, 1044, 1060, 1044},           //Lower bound (raw)
    {1776, 1688, 1850, 1872, 1876, 1842},           //Upper bound (raw)
    {SERVOMIN, -100, -100, -30, 0, 0},                //Lower Bound (adjusted)     //45
    {SERVOMAX - (SERVOMIN/10), 100, 100, 30, 254, 1},               //Upper Bound (adjusted)     //130
  };
#else
  const int PINS[4][PIN_COUNT] = {                //Pins for radio with adjustment parameters
    {1104, 1096, 1028, 1044, 1060, 1044},           //Lower bound (raw)
    {1776, 1688, 1850, 1872, 1876, 1842},           //Upper bound (raw)
    {SERVOMIN, -30, -30, -30, 0, 0},                //Lower Bound (adjusted)     //45
    {SERVOMAX - (SERVOMIN/10), 30, 30, 30, 254, 1},               //Upper Bound (adjusted)     //130
  };
#endif

const int armAve = (PINS[0][PIN_COUNT - 1] + PINS[1][PIN_COUNT - 1]) / 2;

String labels[] = {"PWR", "RLL", "PTCH", "YAW", "KP", "ARMED"};

byte state = 0;
byte burp = 0;  // a counter to see how many times the int has executed
byte cmd = 0;   // a place to put our serial data
byte i = 0;     // global counter for tracking what pin we are on

unsigned long lastSend = 0;

uint16_t rc_values[PIN_COUNT];
uint32_t rc_start[PIN_COUNT];
volatile uint16_t rc_shared[PIN_COUNT];                // to the receiver's channels in the order listed here
volatile int inputs[] = {0,0,0,0,0,0};                       // to the receiver's channels in the order listed here

void setup(){
  Serial.begin(115200);
  lastSend = millis();
  radioSetup();
}

void loop(){
  if(millis() - lastSend > LOOP_TIME){
    printData();
    //outputRadioValues();
    lastSend = millis();
  }
  rc_read_values();
  delay(1);
}

void printData(){
  mapRadioValues();
  for(int i = 0;i < PIN_COUNT;i++){
    Serial.write(byte(inputs[i]));
  }
  Serial.write(byte(255));
}

void radioSetup() {
  //Serial.print("Starting Radio Reciever");
  //Serial.println();            //warm up the serial port

  for (byte i = 0; i < PIN_COUNT; i++)
  {
    pinMode(pin[i], INPUT);     //set the pin to input
    digitalWrite(pin[i], HIGH); //use the internal pullup resistor
  }
  enableInterrupt(pin[0], calc_ch1, CHANGE);
  enableInterrupt(pin[1], calc_ch2, CHANGE);
  enableInterrupt(pin[2], calc_ch3, CHANGE);
  enableInterrupt(pin[3], calc_ch4, CHANGE);
  enableInterrupt(pin[4], calc_ch5, CHANGE);
  enableInterrupt(pin[5], calc_ch6, CHANGE);
}

void calc_ch1() { calc_input(0, pin[0]); }
void calc_ch2() { calc_input(1, pin[1]); }
void calc_ch3() { calc_input(2, pin[2]); }
void calc_ch4() { calc_input(3, pin[3]); }
void calc_ch5() { calc_input(4, pin[4]); }
void calc_ch6() { calc_input(5, pin[5]); }

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void mapRadioValues() {
  for (byte i = 0; i < PIN_COUNT; i++)
  {
    if (rc_values[i] > 800) {                                   //Eliminate bad values
      //Map values to servo outputs
      if(i == PIN_COUNT - 1){
        if(rc_values[i] < armAve){
          inputs[i] = PINS[2][PIN_COUNT - 1];
        }else{
          inputs[i] = PINS[3][PIN_COUNT - 1];
        }
      }else if(i == 1 || i == 3){
        inputs[i] = -(double)map(rc_values[i], PINS[0][i], PINS[1][i], PINS[2][i], PINS[3][i]);
      }else{
        inputs[i] = (double)map(rc_values[i], PINS[0][i], PINS[1][i], PINS[2][i], PINS[3][i]);
      }
      inputs[i] = constrain(inputs[i], PINS[2][i], PINS[3][i]);
    } else {
      inputs[0] = OFF;
    }
    if(inputs[PIN_COUNT - 1] == 0){
      inputs[0] = OFF;
      inputs[1] = 0;
      inputs[2] = 0;
      inputs[3] = 0;
      inputs[4] = 0;
    }
  }
}

void outputRadioValues() {
  mapRadioValues();

  for (byte i = 0; i < PIN_COUNT; i++)
  {
    Serial.print(labels[i]);
    Serial.print(F(":"));
    Serial.print(inputs[i], DEC);
    Serial.print(F("\t"));
  }
}

