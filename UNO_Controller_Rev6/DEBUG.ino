void Debug(){
  /*if(PRINT_ATTITUDE == 1){
    Serial.print(F("Attitude: "));
    for(int i = 0;i < 3;i++){
      Serial.print(angles[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }*/
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
  /*if(PRINT_DT == 1){
    Serial.print(F("Dt: "));
    Serial.print(dt);
  }*/
  /*if(PRINT_OLDPROPORTIONAL == 1){
    Serial.print(F("OldProportional: "));
    for(int i = 0;i < 3 - SUPPRESS_YAW_CONTROL;i++){
      Serial.print(oldproportional[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }
  if(PRINT_PROPORTIONAL == 1){
    Serial.print(F("Proportional: "));
    for(int i = 0;i < 3 - SUPPRESS_YAW_CONTROL;i++){
      Serial.print(proportional[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }
  if(PRINT_INTEGRAL == 1){
    Serial.print(F("Integral: "));
    for(int i = 0;i < 3 - SUPPRESS_YAW_CONTROL;i++){
      Serial.print(integral[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }
  if(PRINT_DERIVATIVE == 1){
    Serial.print(F("Derivative: "));
    for(int i = 0;i < 3 - SUPPRESS_YAW_CONTROL;i++){
      Serial.print(derivative[i]);
      Serial.print(F(","));
    }
    Serial.println();
  }*/
}
