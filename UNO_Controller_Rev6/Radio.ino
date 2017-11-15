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

#define NO_PORTB_PINCHANGES     //PinChangeInt setup
#define NO_PORTC_PINCHANGES    //only port D pinchanges (see: http://playground.arduino.cc/Learning/Pins)
#define PIN_COUNT 5           //number of channels attached to the receiver
#define MAX_PIN_CHANGE_PINS PIN_COUNT

#include "PinChangeInt.h"    // http://playground.arduino.cc/Main/PinChangeInt
#include "TimerOne.h"        // http://playground.arduino.cc/Code/Timer1

#define RC_PWR 2    //Arduino pins attached to the receiver
#define RC_RLL 4
#define RC_PTCH 3
#define RC_YAW 6
#define RC_KP 5

byte pin[] = {RC_PWR, RC_RLL, RC_PTCH, RC_YAW, RC_KP};    //for maximum efficiency these pins should be attached

const int PINS[4][PIN_COUNT] = {                //Pins for radio with adjustment parameters
  {1092, 1078, 1038, 1044, 1056},           //Lower bound (raw)
  {1870, 1814, 1854, 1869, 1869},           //Upper bound (raw)
  {SERVOMIN, -100, -100, -30, 0},                //Lower Bound (adjusted)     //45
  {SERVOMAX, 100, 100, 30, 1000},               //Upper Bound (adjusted)     //130
};

String labels[] = {"PWR", "RLL", "PTCH", "YAW", "KP"};

byte state = 0;
byte burp = 0;  // a counter to see how many times the int has executed
byte cmd = 0;   // a place to put our serial data
byte i = 0;     // global counter for tracking what pin we are on

void getRadioValues() {
  mapRadioValues();
}

void radioSetup() {
  //Serial.begin(115200);gest pulse in PPM is usually 2.1 milliseconds,
  //pick a period that gives you a little headroom.
  Timer1.stop();                //st
  Serial.print("Starting Radio Reciever");
  Serial.println();            //warm up the serial port

  Timer1.initialize(2200);    //lonop the counter
  Timer1.restart();            //set the clock to zero

  for (byte i = 0; i < PIN_COUNT; i++)
  {
    pinMode(pin[i], INPUT);     //set the pin to input
    digitalWrite(pin[i], HIGH); //use the internal pullup resistor
  }
  PCintPort::attachInterrupt(pin[i], rise, RISING); // attach a PinChange Interrupt to our first pin
}

void mapRadioValues() {
  for (byte i = 0; i < PIN_COUNT; i++)
  {
    /*if (time[i] > 800) {                                   //Eliminate bad values
      //Map values to servo outputs
      if(i == 1 || i == 3){
        inputs[i] = -(double)map(time[i], PINS[0][i], PINS[1][i], PINS[2][i], PINS[3][i]);
      }else{
        inputs[i] = (double)map(time[i], PINS[0][i], PINS[1][i], PINS[2][i], PINS[3][i]);
      }
      inputs[i] = constrain(inputs[i], PINS[2][i], PINS[3][i]);
    } else {
      inputs[0] = OFF;
    }*/
    //inputs[1] = -inputs[1];
    //inputs[3] = -inputs[3];
  }
  /*for(int i = 0;i < 4;i++){
    finalInputs[i] = (double)inputs[i];
    }*/
  //Kp[GAIN] = (float)inputs[4] / GAIN_DIVISOR;
  double gain = (double)inputs[4] / GAIN_DIVISOR;
  rollRatePID.SetTunings(gain, KiRoll, KdRoll);
  pitchRatePID.SetTunings(gain, KiPitch, KdPitch);
}

void outputRadioValues() {
  //cmd=Serial.read();        //while you got some time gimme a systems report
  //if (cmd=='p')
  //{
  mapRadioValues();

  for (byte i = 0; i < PIN_COUNT; i++)
  {
    Serial.print(labels[i]);
    Serial.print(F(":"));
    //Serial.print(time[i], DEC);
    Serial.print(F("\t"));
  }
  Serial.print(burp, DEC);
  Serial.println();
  /*Serial.print("\t");
    Serial.print(clockCyclesToMicroseconds(Timer1.pwmPeriod), DEC);
    Serial.print("\t");
    Serial.print(Timer1.clockSelectBits, BIN);
    Serial.print("\t");
    Serial.println(ICR1, DEC);*/
  //}
  //cmd=0;

}

void updateInterrupts() {
  switch (state)
  {
    case RISING: //we have just seen a rising edge
      PCintPort::detachInterrupt(pin[i]);
      PCintPort::attachInterrupt(pin[i], fall, FALLING); //attach the falling end
      state = 255;
      break;
    case FALLING: //we just saw a falling edge
      PCintPort::detachInterrupt(pin[i]);
      i++;                //move to the next pin
      i = i % PIN_COUNT;  //i ranges from 0 to PIN_COUNT
      PCintPort::attachInterrupt(pin[i], rise, RISING);
      state = 255;
      break;
  }
}

void resetInterrupts() {
  PCintPort::detachInterrupt(pin[i]);
  PCintPort::attachInterrupt(pin[i], rise, RISING);
  state = 255;
}

void rise()        //on the rising edge of the currently interesting pin
{
  Timer1.restart();        //set our stopwatch to 0
  Timer1.start();            //and start it up
  state = RISING;
  burp++;
}

void fall()        //on the falling edge of the signal
{
  state = FALLING;
  //time[i] = Timer1.read();  // The function below has been ported into the
  // the latest TimerOne class, if you have the
  // new Timer1 lib you can use this line instead
  /*if(i == 0){
    Serial.println(time[i]);
    }*/
  Timer1.stop();
}
