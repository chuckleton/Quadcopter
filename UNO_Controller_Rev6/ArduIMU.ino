#define IMU_TAG "DIYd"                      // message tag
#define IMU_VALUES 3                        // number of messages send

// union structure for serial received binary data
typedef union{
  int16_t word;
  byte valByte[2];
} IMUpacket;

// intialization of the structures for each passed value
IMUpacket serialPackets[IMU_VALUES];
char imuTag[] = IMU_TAG;                      // first tag in message
char IMUtagBuffer[sizeof(imuTag)];            // first tag in message buffer
boolean newData = false;                      // check for new data

// vars for checksum
byte msg_checksum_a;
byte msg_checksum_b;

// waits for n bytes to receive
void wait_for_bytes(byte number){
  while(Serial.available() < number);
}

// sums all the values and confront the result with the checksum passed
boolean checksum_is_true(){
  byte current_msg_checksum_a = 0;
  byte current_msg_checksum_b = 0;
  for(uint8_t i=0; i<IMU_VALUES; i++){
    current_msg_checksum_a += serialPackets[i].valByte[0];
    current_msg_checksum_b += current_msg_checksum_a;
    current_msg_checksum_a += serialPackets[i].valByte[1];
    current_msg_checksum_b += current_msg_checksum_a;
  }
  return (current_msg_checksum_a == msg_checksum_a && current_msg_checksum_b == msg_checksum_b);
}

/*void serialEvent() {
  if(Serial.available()>0){
    // check if the byte is the start of new messange
    wait_for_bytes(1);
    IMUtagBuffer[0] = Serial.read(); // D
    if(IMUtagBuffer[0]==imuTag[0]){
      wait_for_bytes(3);
      IMUtagBuffer[1] = Serial.read(); // I
      IMUtagBuffer[2] = Serial.read(); // Y
      IMUtagBuffer[3] = Serial.read(); // d
      if(IMUtagBuffer[1]==imuTag[1] && IMUtagBuffer[2]==imuTag[2] && IMUtagBuffer[3]==imuTag[3]){
        // new message
        newData = true;
        // receive all values and put it in the packet structure
        wait_for_bytes(8);
        serialPackets[0].valByte[0] = Serial.read();
        serialPackets[0].valByte[1] = Serial.read();    // roll
        serialPackets[1].valByte[0] = Serial.read();
        serialPackets[1].valByte[1] = Serial.read();    // pitch
        serialPackets[2].valByte[0] = Serial.read();
        serialPackets[2].valByte[1] = Serial.read();    // yaw
        msg_checksum_a = Serial.read();
        msg_checksum_b = Serial.read();                 // checksum
      } else newData = false;
    }
    // checksum for message
    if(newData && checksum_is_true()){
      for(int i = 0;i < 3;i++){
        angles[i] = serialPackets[i].word/100.0;
      }
      for(byte i=0;i<(GYRO_MAF_NR-1);i++){
        for(byte j = 0;j < 3;j++){
          anglesTemp[i][j]=anglesTemp[i+1][j];
        }
      }
      for(byte i = 0;i < 3;i++){
        anglesTemp[GYRO_MAF_NR-1][i] = angles[i];
      }
      gyroMAF();
      lastIMUReading = millis();
      // do stuff with received data values
    } else{
      msg_checksum_a = 0;
      msg_checksum_b = 0;
      newData = false;
    }
  }
  
}*/

/*void gyroMAF(){         //Gyro Moving Average Filter
  for(byte i = 0;i < 3;i++){
    angles[i] = 0;
  }
  for(byte i=0;i<GYRO_MAF_NR;i++){
    for(byte j = 0;j < 3;j++){
      angles[i] = angles[i]+anglesTemp[i][j];
    }
  }
  for(byte i = 0;i < 3;i++){
    angles[i] = (float)angles[i]/GYRO_MAF_NR;
  }
}*/
