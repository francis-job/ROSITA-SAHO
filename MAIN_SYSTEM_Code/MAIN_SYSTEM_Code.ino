#include<Wire.h> //library for I2C communication
const int MPU_addr=0x68; // initialising the address of buffer register in MPU6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;//acclero and gyro values
 
int minVal=265; // minimum value set for the accelerometer reading
int maxVal=402; // maximum value set for the accelerometer readiing
// defining the x , y , and z axis
double x;     
double y;
double z;
// defining pins for rotary encoder
#define CLK PB6
#define DT PB7
// for the external PWM speed control  for ESC input
int pwm1 = 9;
int pwm2 = 10;
const int throttle_in = PA0;
int throttle_value= 0;
int throttle_out = 0;

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";

#define PROTOCOL_SWITCH PA6
int PROTOCOL_SWITCH_STATE = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("SAHO_MAIN_SYSTEM_LOG ");
  Serial.println("ACTIVE_PROTOCOL");
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  lastStateCLK = digitalRead(CLK);// Read the initial state of CLK

  pinMode(PROTOCOL_SWITCH,INPUT);
  PROTOCOL_SWITCH_STATE = digitalRead(PROTOCOL_SWITCH);
  
  if(PROTOCOL_SWITCH_STATE == HIGH)
  {
    Serial.print("MASTER_ACCESS_CONTROL_MODE");
  }
  else
  {
    Serial.print("LIMITED_ACCESS_CONTROL_MODE");
  }

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  }
  void loop()
  {
    // Read the current state of CLK
     currentStateCLK = digitalRead(CLK);

     //throttle monitoring and regulating code
     throttle_value =  analogRead(throttle_in);
     throttle_out = map(throttle_value, 0, 1023, 0, 255);
     analogWrite(pwm1,throttle_out);
     analogWrite(pwm2,throttle_out);
     int throttle_volt = analogRead(A0);
     float voltage =  throttle_volt * (5.0 / 1023.0);
     Serial.print("Thrott_Volt=") ;
     Serial.print(voltage);
     Serial.print("PWM_OUT_VALUE=") ;
     Serial.print(throttle_value);
     delay(5);
     // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
  {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CW so decrement
    if (digitalRead(DT) != currentStateCLK)
    {
      counter ++;
      currentDir ="CW";
    }
    else 
    {
      // Encoder is rotating CCW so increment
      counter --;
      currentDir ="CCW";
    }

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

 
  // Put in a slight delay to help debounce the reading
  delay(1);


  
  }
void tilt_angle()
{
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x3B);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_addr,14,true);
   AcX=Wire.read()<< 8 |Wire.read();
   AcY=Wire.read()<< 8 |Wire.read();  
   AcZ=Wire.read()<< 8 |Wire.read();
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
   if(90 > y > 45)
   {
     Serial.println("ELEVATION_DETECTED");
   }
   if(270 > y > 359)
   {
     Serial.println("DEMOTION_DETECTED");
   }
   delay(20);
}
