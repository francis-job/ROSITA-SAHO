#include<Wire.h> //library for I2C communication
const int MPU_addr=0x68; // initialising the a unique 7-bit address of I2C slave MPU6050(slave address-low address of pin)
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;//acclero and gyro values
//boundary condition for the  accelerometer Reading 
int minVal=265; // minimum value set for the accelerometer reading
int maxVal=402; // maximum value set for the accelerometer readiing
// defining the x , y , and z axis in the double floating type because it acquire continuous values with doube precision as compared to float.
double x;     
double y;//for converting to degree
double z;
// defining pins for rotary encoder
#define CLK PB6
#define DT PB7
// for the external PWM speed control  for ESC input
int pwm1 = PA6;
int pwm2 = PC7;
const int throttle_in = PA0; //Throttle input is connected to the Analog pin PA0
//defining variables required for the throttle voltage calculation
int throttle_value= 0;
int throttle_out = 0;
//variables for the Rotary encoder
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir =""; //defining a string to print the direction of Rotation

#define PROTOCOL_SWITCH PA8 // pin defined for toggle between protocols (pa8= protocol switch)
int PROTOCOL_SWITCH_STATE = 0; //current status of the protocol toggling switch
//variables defining for calculating the exceeding voltage value
int Lim_value = 0; 
int Lim_value_bit = 0;

void setup()
{
  Serial.begin(9600); //defining Serial UART communication Baud RATE
  Serial.println("SAHO_MAIN_SYSTEM_LOG \n");
  Serial.print("ACTIVE_PROTOCOL :");
  //defining pins for the Rotary encoder
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  lastStateCLK = digitalRead(CLK);// Read the initial state of CLK
  //defining the protocol switch in setup
  pinMode(PROTOCOL_SWITCH,INPUT); // defining mode of use of pin
  PROTOCOL_SWITCH_STATE = digitalRead(PROTOCOL_SWITCH); // reading the current state of the switch
  delay(1000);//1sec
  
  if(PROTOCOL_SWITCH_STATE == 1)
  {
    Serial.println("MASTER_ACCESS_CONTROL_MODE");
  }
  else
  {
    Serial.println("LIMITED_ACCESS_CONTROL_MODE");
  }

  Wire.begin(); // initialising the I2C communication
  Wire.beginTransmission(MPU_addr);//selecting address and establishing transmission
  Wire.write(0x6B);//master to slave
  Wire.write(0);
  Wire.endTransmission(true);
  
  }
  
  void loop()
  { 
     //throttle monitoring and regulating code
     throttle_value =  analogRead(throttle_in); // reading the analog input value from the throttle input
     throttle_out = map(throttle_value, 0, 1023, 0, 65535); //mapping the 10 bit ADC value to a 16 bit value for accurate pwm generation.(16 bit timer is available in ARM)
     float voltage =  throttle_value * (5.0 / 1023.0); //converting the 10 bit value to the corresponding voltage value between 0 - 5V
     
     if( PROTOCOL_SWITCH_STATE ==LOW && voltage >=3.5)//if the throttle input value exceeds max. speed then we have to limit the speed corresponding to 3.5V
       {
        Lim_value = voltage-3.5; // additional voltage is substracted from the threshold
        Lim_value_bit=map(Lim_value*1023/5,0,1023,0,65535);// the additiona value is mapped to its corresponding bit value
        
      //the corrected value is given as the pwm out
        analogWrite(pwm1,throttle_out - Lim_value);
        analogWrite(pwm2,throttle_out - Lim_value);
        Serial.println("LIMITED_VALUE_REACHED");
      }
      else if( PROTOCOL_SWITCH_STATE == LOW && voltage <= 3.5) // when the throttle input is less than the threshold the system will pass the exact values
      {
        analogWrite(pwm1,throttle_out );
        analogWrite(pwm2,throttle_out );
      }
       else if( PROTOCOL_SWITCH_STATE == HIGH) // when the throttle input is less than the threshold the system will pass the exact values
      {
        analogWrite(pwm1,throttle_out );
        analogWrite(pwm2,throttle_out );
      }
      //printing the necessary values
     Serial.println("Thrott_Volt=") ;
     Serial.print(voltage);
     Serial.println("PWM_OUT_VALUE=") ;
     Serial.print(throttle_value);

      // Read the current state of CLK
     currentStateCLK = digitalRead(CLK);
     // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
   if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
      {
       

       // If the DT state is different than the CLK state then
       // the encoder is rotating CW so decrement
      if (digitalRead(DT) != currentStateCLK)
        {
          counter ++;
          currentDir ="TURNING_RIGHT";
        }
      else 
       {
       // Encoder is rotating CCW so increment
         counter --;
         currentDir ="TURNING_LEFT";
       }
       

    Serial.println("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

 
  // Put in a slight delay to help debounce the reading
   delay(10);
   Wire.beginTransmission(MPU_addr);//Initiate the Wire library and join the I2C bus as a master or slave
   Wire.write(0x3B);//mpu6050 slave gives the accelrometer reading from the 3B register onwards 
   Wire.endTransmission(false); //for avoiding multiple master interpretation.connection establishment over.end transmission false state will send a restart, keeping the connection active
   Wire.requestFrom(MPU_addr,14,true);//true state will send a stop message after the request, releasing the I2C bus
   //after acquiring the byte values
   //16 bit values are read as two 8 bit higher and lower bits 
   AcX=Wire.read()<< 8 |Wire.read(); // higher order bits are read first and left shifted and then read the lower order bits
   AcY=Wire.read()<< 8 |Wire.read();  
   AcZ=Wire.read()<< 8 |Wire.read();
   
   //accelerometer reading are converted to angle in radian
   int xAng = map(AcX,minVal,maxVal,-90,90);
   int yAng = map(AcY,minVal,maxVal,-90,90);
   int zAng = map(AcZ,minVal,maxVal,-90,90);
   
   //angle in radians are converted to degrees
   x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
   y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
   z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  //printing all the X Y Z angle values in degrees
   Serial.print("AngleX= ");
   Serial.println(x);
 
   Serial.print("AngleY= ");
   Serial.println(y);
 
   Serial.print("AngleZ= ");
   Serial.println(z);

   // From these values the movement in Y direction is measured
   if((y>=45) && (y<=90))//if the y angle value is between 45deg and 90 deg it is considered as elevation.
   {
     Serial.println("ELEVATION_DETECTED");
   }
   if((y>=270) && (y<=359))//if the y angle value is between 270deg and 359 deg it is considered as demotion. 
   {
     Serial.println("DEMOTION_DETECTED");
   }
   Serial.println("-----------------------------------------");
  delay(100);
  }
