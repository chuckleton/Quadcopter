#define YAW_OFFSET 0                 //offset since front and back rotors appear to exert more torque than L and R

//PID rollPID(&angles[0],&controls[0],&finalInputs[1],KpRoll,KiRoll,KdRoll,DIRECT);       //create PID functions
//PID pitchPID(&angles[1],&controls[1],&finalInputs[2],KpPitch,KiPitch,KdPitch,DIRECT);

PID rollRatePID((double*)&myIMU.gy,&controls[0],&inputs[1],KpRoll,KiRoll,KdRoll,DIRECT);       //create PID functions
PID pitchRatePID((double*)&myIMU.gx,&controls[1],&inputs[2],KpPitch,KiPitch,KdPitch,DIRECT);
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
  rollRatePID.SetSampleTime(6);                         //167Hz PID refresh rate
  pitchRatePID.SetSampleTime(6);
  yawRatePID.SetSampleTime(6);
}

void control(){
  rollRatePID.Compute();                                //Calculate PID values
  pitchRatePID.Compute();
  yawRatePID.Compute();
  power();                                              //make rotors go fast
}

//Powers each motor based on PID outputs
void power(){
  motVals[0] = (int)(inputs[0] - controls[0] - controls[1]);
  motVals[1] = (int)(inputs[0] + controls[0] - controls[1]);
  motVals[2] = (int)(inputs[0] - controls[0] + controls[1]);
  motVals[3] = (int)(inputs[0] + controls[0] + controls[1]);
  for(int i = 0;i < 4;i++){
    motVals[i] = constrain(motVals[i],SERVOMIN,SERVOMAX);
    mot.setPWM(i,0,motVals[i]);
  }
}

//Calculates PID values
/*void calcPID(){
  for(int i = 0;i < 3 - SUPPRESS_YAW_CONTROL;i++){
    oldproportional[i] = proportional[i];
    proportional[i] = (float)inputs[i+1] - angles[i];               //P
    //integral[i] = integral[i] + (proportional[i]*Dt);             //I
    //integral[i] = constrain(integral[i],-1000.0,1000.0);          //Constrain I so it doesnt get too big
    derivative[i] = (proportional[i] - oldproportional[i]) / Dt;    //D
  }
}*/

void stopMotors(){
  for(int i = 0;i < 4;i++){
    mot.setPWM(i,0,OFF);                                        //Turn off all Motors
  }
}

double getGain(){
  return rollRatePID.GetKp();
}

