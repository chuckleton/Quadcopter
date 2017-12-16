#define YAW_OFFSET 0                 //offset if front and back rotors appear to exert more torque than L and R

PID rollPID((double*)&myIMU.roll,&angControls[0],&inputs[1],KpRolla,KiRolla,0.0,DIRECT);       //create angle PID functions
PID pitchPID((double*)&myIMU.pitch,&angControls[1],&inputs[2],KpPitcha,KiPitcha,0.0,DIRECT);

PID rollRatePID((double*)&myIMU.gy,&controls[0],&angControls[0],KpRoll,0.0,0.0,DIRECT);       //create rate PID functions
PID pitchRatePID((double*)&myIMU.gx,&controls[1],&angControls[1],KpPitch,0.0,0.0,DIRECT);
PID yawRatePID((double*)&myIMU.gz,&controls[2],&inputs[3],KpYaw,KiYaw,KdYaw,DIRECT);

//CONTROL[0]: NEGATIVE WITH ROLL LEFT
//CONTROL[1]: NEGATIVE WITH PITCH FORWARD
//CONTROL[2]: NEGATIVE WITH YAW CCW

void initializePID(){
  rollRatePID.SetMode(AUTOMATIC);
  pitchRatePID.SetMode(AUTOMATIC);
  yawRatePID.SetMode(AUTOMATIC);
  rollRatePID.SetOutputLimits(-1000.0,1000.0);
  pitchRatePID.SetOutputLimits(-1000.0,1000.0);
  yawRatePID.SetOutputLimits(-500.0,500.0);
  rollRatePID.SetSampleTime(6);                          //167Hz PID refresh rate
  pitchRatePID.SetSampleTime(6);
  yawRatePID.SetSampleTime(6);
  if(USE_ANGLE_PID == 1){
    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-1000.0,1000.0);          
    pitchPID.SetOutputLimits(-1000.0,1000.0);
    rollPID.SetSampleTime(6);                            //167Hz PID refresh rate
    pitchPID.SetSampleTime(6);
  }
}

void control(){
  if(USE_ANGLE_PID == 1){
    rollPID.Compute();
    pitchPID.Compute();
  }else{
    angControls[0] = inputs[1];
    angControls[1] = inputs[2];
  }
  rollRatePID.Compute();                                //Calculate PID values
  pitchRatePID.Compute();
  yawRatePID.Compute();
  power();                                              //make rotors go fast
}

//Powers each motor based on PID outputs
void power(){
  motVals[0] = (int)(inputs[0] - controls[0] - controls[1]);        //FL
  motVals[1] = (int)(inputs[0] + controls[0] - controls[1]);        //FR
  motVals[2] = (int)(inputs[0] - controls[0] + controls[1]);        //BL
  motVals[3] = (int)(inputs[0] + controls[0] + controls[1]);        //BR
  for(int i = 0;i < 4;i++){
    motVals[i] = constrain(motVals[i],SERVOMIN,SERVOMAX);           //Make sure values are within limits
    mot.setPWM(i,0,motVals[i]);                                     //Power Motors
  }
}

void stopMotors(){
  for(int i = 0;i < 4;i++){
    mot.setPWM(i,0,OFF);                                            //Turn off all Motors
  }
}

double getGain(){                                                   //Returns the gain that's being tuned
  #ifdef TUNE_RATE_KP
    return rollRatePID.GetKp();
  #endif
  #ifdef TUNE_ANGLE_KP
    return rollPID.GetKp();
  #endif
  #ifdef TUNE_ANGLE_KI
    return rollPID.GetKi();
  #endif
}

