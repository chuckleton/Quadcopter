#define INPUT_MARKER byte(255)

void serialEvent(){
  if(Serial.available() > NUMCH){
    int i = 0;
    while(1){
      byte inCh = Serial.read();
      if(inCh == 255){
        break;
      }
      raw[i] = inCh;
      i++;
    }

    for(int i = 1;i < NUMCH-1;i++){
      inputs[i] = (int8_t)raw[i];
    }
    inputs[0] = (unsigned byte)raw[0];
    inputs[0] = constrain(inputs[0], SERVOMIN, SERVOMAX);
    inputs[NUMCH - 2] = (unsigned byte)raw[NUMCH - 2];
    inputs[NUMCH - 2] = constrain(inputs[NUMCH - 2], 0, 254);
    double gain = (double)inputs[NUMCH - 2] / GAIN_DIVISOR;
    #ifdef TUNE_RATE_KP
      rollRatePID.SetTunings(gain, 0.0, 0.0);
      pitchRatePID.SetTunings(gain, 0.0, 0.0);
    #endif
    #ifdef TUNE_ANGLE_KP
      rollPID.SetTunings(gain, KiRolla, 0.0);
      pitchPID.SetTunings(gain, KiPitcha, 0.0);
    #endif
    #ifdef TUNE_ANGLE_KI
      rollPID.SetTunings(KpRolla, gain, 0.0);
      pitchPID.SetTunings(KpRolla, gain, 0.0);
    #endif
    if((int8_t)raw[NUMCH - 1] == 1){
      ARMED = true;
    }else{
      ARMED = false;
    }
  }
}

