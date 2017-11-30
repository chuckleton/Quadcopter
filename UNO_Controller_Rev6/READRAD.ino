#define INPUT_MARKER byte(255)
#define NUM_INPUTS 5

void serialEvent(){
  if(Serial.available() > NUM_INPUTS){
    int i = 0;
    while(1){
      byte inCh = Serial.read();
      if(inCh == 255){
        break;
      }
      raw[i] = inCh;
      i++;
      //Serial.println(raw[i]);
    }
    /*Serial.println("Data:");
    for(int i = 0;i < 5;i++){
      Serial.println(raw[i]);
    }*/
    for(int i = 1;i < NUM_INPUTS-1;i++){
      inputs[i] = (int8_t)raw[i];
    }
    inputs[0] = (unsigned byte)raw[0];
    inputs[0] = constrain(inputs[0], SERVOMIN, SERVOMAX);
    inputs[NUM_INPUTS - 1] = (unsigned byte)raw[NUM_INPUTS - 1];
    inputs[NUM_INPUTS - 1] = constrain(inputs[NUM_INPUTS - 1], 0, 254);
    double gain = (double)inputs[NUM_INPUTS - 1] / GAIN_DIVISOR;
    rollRatePID.SetTunings(gain, KiRoll, KdRoll);
    pitchRatePID.SetTunings(gain, KiPitch, KdPitch);
  }
}

