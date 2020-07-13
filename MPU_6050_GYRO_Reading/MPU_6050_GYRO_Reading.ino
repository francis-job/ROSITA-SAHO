#include<Wire.h>
int16_t Gyro_X,Gyro_Y,Gyro_Z;
int16_t AcX,AcY,AcZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

void setup()
{
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  }
  void loop()
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    Gyro_X = Wire.read()<<8 | Wire.read();
    Gyro_Y = Wire.read()<<8 | Wire.read();
    Gyro_Z = Wire.read()<<8 | Wire.read();
    Serial.print("X =");
    Serial.print(Gyro_X);
    Serial.print(" Y =");
    Serial.print(Gyro_Y);
    Serial.print(" Z =");
    Serial.println(Gyro_Z);
    AcX=Wire.read()<< 8 |Wire.read();
    AcY=Wire.read()<< 8|Wire.read();
    AcZ=Wire.read()<< 8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
 
     x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
     y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
     z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
 
     Serial.print("AngleX= ");
     Serial.println(x);
     Serial.print("AngleY= ");
     Serial.println(y);
 
     Serial.print("AngleZ= ");
     Serial.println(z);
     Serial.println("-----------------------------------------");
     delay(1000);
  }
