#include<Wire.h>
const int MPU=0x68;  
const int AK=0x0C;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,Max,May,Maz;
int AcX_OC=0, AcY_OC=0, AcZ_OC=0, GyX_OC=0 ,GyY_OC=0, GyZ_OC=0; // offset variables
float AcX_scalled,AcY_scalled,AcZ_scalled,GyX_scalled,GyY_scalled,GyZ_scalled; //Scalled Data varaibles
float Ac_scale_fact = 1, Gy_scale_fact = 1; // Scale factor variables

#define MPU9265_config 26            // R/W
#define MPU9265_gyro_config 27       // R/W
#define MPU9265_accel_config 28      // R/W
#define MPU9265_PWR1 107
#define MPU9265_PWR2 108
#define g 9.81



void MPU9265_ResetWake(){
  //Serial.println("Resetting MPU9265 and waking it up.....");
  Wire.beginTransmission(MPU);
  Wire.write(MPU9265_PWR1);
  Wire.write(0b10000000);
  Wire.endTransmission();
  
  delay(100); // Waiting for the reset to complete
   
  Wire.beginTransmission(MPU);
  Wire.write(MPU9265_PWR1);
 
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void MPU9265_SetGains(){
    int gyro = 0, accel = 1;
    byte Gy_byte, Ac_byte;
    
    // Setting up Gyro
    Wire.beginTransmission(MPU);
    Wire.write(MPU9265_gyro_config); // Address to the configuration register
    if (gyro==0)
    {
      Gy_scale_fact =(float)250*0.0305; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      Gy_byte = 0b00000000;
    }else if (gyro == 1)
    {
      Gy_scale_fact = 500*0.0305; // each data is of 16 bits that means, 500 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      Gy_byte = 0b00001000;
    }else if (gyro == 2)
    {
      Gy_scale_fact = 1000*0.0305;// each data is of 16 bits that means, 1000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      Gy_byte = 0b00010000;
    }else if (gyro == 3)
    {
      Gy_scale_fact = 2000*0.0305;  // each data is of 16 bits that means, 2000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      Gy_byte = 0b00011000;
    }else
    {
      Gy_scale_fact = 1;
    }  
      
    Wire.write(Gy_byte);
    Wire.endTransmission();
    //Serial.print("The gyro scale is set to ");
    //Serial.print(Gy_scale_fact);
    //Serial.println(" milli Degree/s");
    
    
    // Setting up Accel
    Wire.beginTransmission(MPU);
    Wire.write(MPU9265_accel_config); // Address to the configuration register
    if (accel==0)
    {
      Ac_scale_fact =(float)2*g*0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767       
      Ac_byte = 0b00000000;
    }else if (accel == 1)
    {
      Ac_scale_fact = 4*g*0.0305; // each data is of 16 bits that means, 4g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      Ac_byte = 0b00001000;
    }else if (accel == 2)
    {
      Ac_scale_fact = 8*g*0.0305;// each data is of 16 bits that means, 8g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      Ac_byte = 0b00010000;
    }else if (accel == 3)
    {
      Ac_scale_fact = 16*g*0.0305; // each data is of 16 bits that means, 16g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      Ac_byte = 0b00011000;
    }else
    {
      Ac_scale_fact = 1;
    }  
      
    Wire.write(Ac_byte);
    Wire.endTransmission();
    //Serial.print("The accel scale is set to ");
    //Serial.print(Ac_scale_fact);
    //Serial.println(" milli m/s^2");
    
 }

void MPU9265_OffsetCal(){
  //Serial.println("Calibrating gyroscope .... dont move the hardware ..........");
  //Serial.println("Calibrating accelrometer .... dont move the hardware ..........");  
  int GyX_OC_x=0,GyY_OC_y=0,GyZ_OC_z=0,AcX_OC_x=0,AcY_OC_y=0,AcZ_OC_z=0,i;
  
  MPU9265_ReadData();
  
  // Gyro & Accelerometer Offset Calculation
  GyX_OC_x=GyX;
  GyY_OC_y=GyY;
  GyZ_OC_z=GyZ;

  AcX_OC_x=AcX;
  AcY_OC_y=AcY;
  AcZ_OC_z=AcZ;
    
  for (i=1;i<=10;i++){
    MPU9265_ReadData();
    
    GyX_OC_x=(GyX_OC_x+GyX)/2;
    GyY_OC_y=(GyY_OC_y+GyY)/2;
    GyZ_OC_z=(GyZ_OC_z+GyZ)/2;

    AcX_OC_x=(AcX_OC_x+AcX)/2;
    AcY_OC_y=(AcY_OC_y+AcY)/2;
    AcZ_OC_z=(AcZ_OC_z+AcZ)/2;

    
    //Serial.print(".");
  }
  //Serial.println(".");
  GyX_OC=GyX_OC_x;
  GyY_OC=GyY_OC_y;
  GyZ_OC=GyZ_OC_z;

  AcX_OC=AcX_OC_x;
  AcY_OC=AcY_OC_y;
  AcZ_OC=AcZ_OC_z-(float)g*1000/Ac_scale_fact;
  
  
}


void setup(){
  Wire.begin();
  MPU9265_ResetWake(); 
  MPU9265_SetGains();
  MPU9265_OffsetCal();
  Serial.begin(9600);
}

void MPU9265_ReadData(){
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
  
  Wire.beginTransmission(AK);
  Wire.write(0x60);
  Wire.endTransmission(false);
  Wire.requestFrom(AK,6);
  Max=Wire.read()<<8|Wire.read();
  May=Wire.read()<<8|Wire.read();
  Maz=Wire.read()<<8|Wire.read();


  AcX_scalled = (float)(AcX - AcX_OC)*Ac_scale_fact/1000;
  AcY_scalled = (float)(AcY - AcY_OC)*Ac_scale_fact/1000;
  AcZ_scalled = (float)(AcZ - AcZ_OC)*Ac_scale_fact/1000;

  GyX_scalled = (float)(GyX - GyX_OC)*Gy_scale_fact/1000;
  GyY_scalled = (float)(GyY - GyY_OC)*Gy_scale_fact/1000;
  GyZ_scalled = (float)(GyZ - GyZ_OC)*Gy_scale_fact/1000;

  Serial.println("Data_scalled:");
  Serial.print(""); Serial.print(AcX_scalled);
  Serial.print(","); Serial.print(AcY_scalled);
  Serial.print(","); Serial.print(AcZ_scalled);
  Serial.print(","); Serial.print(GyX_scalled);
  Serial.print(","); Serial.print(GyY_scalled);
  Serial.print(","); Serial.println(GyZ_scalled);

  //Serial.print(",");Serial.print(Max);
  //Serial.print(",");Serial.print(May);
  //Serial.print(",");Serial.println(Maz);

  delay(100);
}



void loop(){

  
  MPU9265_ReadData();

  delay(100);
}
