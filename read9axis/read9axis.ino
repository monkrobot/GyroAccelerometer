
/**
 @file read9axis.ino
 @brief This is an Example for the FaBo 9Axis I2C Brick.

   http://fabo.io/202.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis fabo_9axis;

float mx,my,mz,mx_OC_x,my_OC_y,mz_OC_z,mx_X,my_Y,mz_Z;

float mxx,myy,mzz;

void setup() {
  Serial.begin(9600);
  //Serial.println("RESET");
  Serial.println();

  //Serial.println("configuring device.");

  if (fabo_9axis.begin()) {
    //Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    //Serial.println("device error");
    while(1);
  }
}

/*void average_mag(){
  int i;
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  mx_OC_x=mx;
  my_OC_y=my;
  mz_OC_z=mz;
  for (i=1;i<=100;i++){
    fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
    
    mx_X=(mx_OC_x+mx)/2;
    my_Y=(my_OC_y+my)/2;
    mz_Z=(mz_OC_z+mz)/2;

  }
  mxx=mx_X;
  myy=my_Y;
  mzz=mz_Z;
}*/

void loop() {
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  float temp;

  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  //average_mag();
  fabo_9axis.readTemperature(&temp);

  

  //Serial.println("Data_scalled:");
  Serial.print(""); Serial.print(ax);
  Serial.print(","); Serial.print(ay);
  Serial.print(","); Serial.print(az);
  Serial.print(","); Serial.print(gx);
  Serial.print(","); Serial.print(gy);
  Serial.print(","); Serial.print(gz);
  Serial.print(","); Serial.print(mx);
  Serial.print(","); Serial.print(my);
  Serial.print(","); Serial.println(mz);
  /*Serial.print(" "); Serial.print(mx);
  Serial.print(" "); Serial.print(my);
  Serial.print(" "); Serial.println(mz);*/

  /*Serial.print("ax: ");
  Serial.print(ax);
  Serial.print(" ay: ");
  Serial.print(ay);
  Serial.print(" az: ");
  Serial.println(az);

  Serial.print("gx: ");
  Serial.print(gx);
  Serial.print(" gy: ");
  Serial.print(gy);
  Serial.print(" gz: ");
  Serial.println(gz);

  Serial.print("mx: ");
  Serial.print(mx);
  Serial.print(" my: ");
  Serial.print(my);
  Serial.print(" mz: ");
  Serial.println(mz);

  Serial.print("temp: ");
  Serial.println(temp);*/

  delay(10);
}
