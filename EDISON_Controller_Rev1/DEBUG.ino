#ifdef PRINT_DEBUG
void Debug(){
  if(PRINT_ATTITUDE == 1){
    Serial.print(F("Attitude: "));
    printIMUAngles();
  }
  if(PRINT_ATTITUDE_RATES == 1){
    Serial.print(F("Attitude Rates: "));
    printIMURates();
  }
  if(PRINT_RC_VALUES == 1){
    Serial.print(F("R/C Values: "));
    for(int i = 0;i < NUMCH;i++){
      Serial.print(inputs[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }
  if(PRINT_GAINS == 1){
    Serial.print(F("Kp: "));
    Serial.println(getGain(),4);
  }
  if(PRINT_ARMED == 1){
    Serial.print(F("Armed: "));
    Serial.println(ARMED);
  }
  if(PRINT_CONTROL == 1){
    Serial.print(F("Control Values: "));
    for(int i = 0;i < 3;i++){
      Serial.print(controls[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }
  if(PRINT_OUTPUTS == 1){
    Serial.print(F("Output Values:\t"));
    Serial.print(F("FL: "));
    Serial.print(motVals[0]);
    Serial.print(", ");
    Serial.print(F("FR: "));
    Serial.print(motVals[1]);
    Serial.print(", ");
    Serial.print(F("BL: "));
    Serial.print(motVals[2]);
    Serial.print(", ");
    Serial.print(F("BR: "));
    Serial.println(motVals[3]);
  }
  if(PRINT_DT == 1){
    Serial.print(F("dt: "));
    Serial.print(dt);
  }
}
#endif
