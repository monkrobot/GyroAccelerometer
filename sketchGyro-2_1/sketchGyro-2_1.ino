#include<Wire.h>
const int MPU=0x68;  
const int AK=0x0C;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,Max,May,Maz;
int AcX_OC=0, AcY_OC=0, AcZ_OC=0, GyX_OC=0 ,GyY_OC=0, GyZ_OC=0; // offset variables
float AcX_scalled,AcY_scalled,AcZ_scalled,GyX_scalled,GyY_scalled,GyZ_scalled; //Scalled Data varaibles
float Ac_scale_fact = 1, Gy_scale_fact = 1; // Scale factor variables
float angle_x_gyro, angle_y_gyro, angle_z_gyro, angle_x_accel, angle_y_accel, angle_z_accel,angle_x=0,angle_y=0,angle_z=0, angle_x_k;
#define MPU9265_config 26            // R/W
#define MPU9265_gyro_config 27       // R/W
#define MPU9265_accel_config 28      // R/W
#define MPU9265_PWR1 107
#define MPU9265_PWR2 108
#define g 9.81
#define dt 20                       // time difference in milli seconds
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.05             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)

unsigned long t=0; // Time Variables
float y;
float S;
float K_0;
float K_1;


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
  
  //Serial.print("gyro_x register offset = ");
  //Serial.println(GyX_OC);
 
  
  //Serial.print("gyro_y register offect = ");
  //Serial.println(GyY_OC);
 
  
  //Serial.print("gyro_z register offset = ");
  //Serial.println(GyZ_OC);
 
  //Serial.print("Accel_x register offset = ");
  //Serial.println(AcX_OC);
  
  
  //Serial.print("Accel_y register offect = ");
  //Serial.println(AcY_OC);
  
  
  //Serial.print("Accel_z register offset = ");
  //Serial.println(AcZ_OC);

  
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

  //Serial.println("Data:");
  //Serial.print(""); Serial.print(AcX);
  //Serial.print(","); Serial.print(AcY);
  //Serial.print(","); Serial.print(AcZ);
  //Serial.print(","); Serial.print(GyX);
  //Serial.print(","); Serial.print(GyY);
  //Serial.print(","); Serial.println(GyZ);

  AcX_scalled = (float)(AcX - AcX_OC)*Ac_scale_fact/1000;
  AcY_scalled = (float)(AcY - AcY_OC)*Ac_scale_fact/1000;
  AcZ_scalled = (float)(AcZ - AcZ_OC)*Ac_scale_fact/1000;

  GyX_scalled = (float)(GyX - GyX_OC)*Gy_scale_fact/1000;
  GyY_scalled = (float)(GyY - GyY_OC)*Gy_scale_fact/1000;
  GyZ_scalled = (float)(GyZ - GyZ_OC)*Gy_scale_fact/1000;

    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  uint8_t c;
  HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1, 100);
  if(c >= 0x01) { // wait for magnetometer data ready bit to be set
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7, 100);  // Read the six raw data and ST2 registers sequentially into data array
    c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
      destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
   }
  }

  //Serial.println("Data_scalled:");
  //Serial.print(""); Serial.print(AcX_scalled);
  //Serial.print(","); Serial.print(AcY_scalled);
  //Serial.print(","); Serial.print(AcZ_scalled);
  //Serial.print(","); Serial.print(GyX_scalled);
  //Serial.print(","); Serial.print(GyY_scalled);
  //Serial.print(","); Serial.println(GyZ_scalled);
  delay(100);
}


float  Q_angle = 0.001f;
float  Q_gyro = 0.003f;
float  R_angle = 0.001f;
float  angle = 0.0f; // Reset the angle
float  bias = 0.0f; // Reset bias
float  P_00 = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
float  P_01 = 0.0f;
float  P_10 = 0.0f;
float  P_11 = 0.0f;


void loop(){
  t=millis(); 
  
  MPU9265_ReadData();
 
  angle_x_gyro = (GyX_scalled*((float)dt/1000)+angle_x);
  angle_y_gyro = (GyY_scalled*((float)dt/1000)+angle_y);
  angle_z_gyro = (GyZ_scalled*((float)dt/1000)+angle_z);

  angle_z_accel = atan(AcZ_scalled/(sqrt(AcY_scalled*AcY_scalled+AcX_scalled*AcX_scalled)))*(float)rad2degree;
  angle_y_accel = -atan(AcY_scalled/(sqrt(AcX_scalled*AcX_scalled+AcZ_scalled*AcZ_scalled)))*(float)rad2degree;
  angle_x_accel = atan(AcX_scalled/(sqrt(AcY_scalled*AcY_scalled+AcZ_scalled*AcZ_scalled)))*(float)rad2degree;


  
  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  angle_x = ((GyX_scalled - bias)*((float)dt / 1000) + angle_x);

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += +Q_gyro * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  /* Step 3 */
  y = angle_x_accel - angle_x; // Angle difference

  /* Step 4 */
  S = P_00 + R_angle; // Estimate error

  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 5 */
  K_0 = P_00 / S; // Kalman gain - This is a 2x1 vector
  K_1 = P_10 / S;

  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 6 */
  angle_x += K_0 * y;
  bias += K_1 * y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  angle_x_k = Filter_gain*angle_x_gyro+(1-Filter_gain)*angle_x_accel;
  angle_y = Filter_gain*angle_y_gyro+(1-Filter_gain)*angle_y_accel;
  angle_z = Filter_gain*angle_z_gyro+(1-Filter_gain)*angle_z_accel;

  Serial.print(""); Serial.print(angle_x);
  Serial.print(""); Serial.print(angle_x_k);
  Serial.print(","); Serial.print(angle_y);
  Serial.print(","); Serial.print(angle_z);
  Serial.print("\n");
  delay(100);
}
