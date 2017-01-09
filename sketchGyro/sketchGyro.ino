#include<Wire.h>
const int MPU=0x68;  
const int AK=0x0C;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,Max,May,Maz;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(MPU);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(AK);
  Wire.write(AK);  
  Wire.write(0); 
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read(); 
  AcZ=Wire.read()<<8|Wire.read(); 
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read(); 
  GyY=Wire.read()<<8|Wire.read();    
  GyZ=Wire.read()<<8|Wire.read();
  
  Serial.print(""); Serial.print(AcX);
  Serial.print(","); Serial.print(AcY);
  Serial.print(","); Serial.print(AcZ);
  Serial.print(","); Serial.print(GyX);
  Serial.print(","); Serial.print(GyY);
  Serial.print(","); Serial.println(GyZ);
  delay(1000);
}
