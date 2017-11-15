#define INPUT_MARKER byte(255)
#define NUM_INPUTS 5

void serialEvent(){
  if(Serial.available() > NUM_INPUTS){
    if(Serial.readBytesUntil(INPUT_MARKER,raw,NUM_INPUTS+1) == NUM_INPUTS){
      for(int i = 1;i < NUM_INPUTS-1;i++){
        inputs[i] = (int)raw[i];
      }
      inputs[0] = (unsigned byte)raw[0];
      inputs[NUM_INPUTS - 1] = (unsigned byte)raw[NUM_INPUTS - 1];
    }
    double gain = (double)inputs[NUM_INPUTS - 1] / GAIN_DIVISOR;
    rollRatePID.SetTunings(gain, KiRoll, KdRoll);
    pitchRatePID.SetTunings(gain, KiPitch, KdPitch);
  }
}

