#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int pwm1 = 9;
int pwm2 = 10;
const int throttle_in = A0;
int throttle_value= 0;
int throttle_out = 0;
void setup()
{
  lcd.begin(16,2);
  }
  void loop()
  {
     lcd.setCursor(0, 0);
     throttle_value =  analogRead(throttle_in);
     throttle_out = map(throttle_value, 0, 1023, 0, 255);
     analogWrite(pwm1,throttle_out);
     analogWrite(pwm2,throttle_out);
     int throttle_volt = analogRead(A0);
     float voltage =  throttle_volt * (5.0 / 1023.0);
     lcd.print("Thrott_Volt=") ;
     lcd.setCursor(12, 0);
     lcd.print(voltage);
     lcd.setCursor(0, 1);
     lcd.print("IN=") ;
     lcd.setCursor(4,1);
     lcd.print(throttle_value);
     lcd.setCursor(0,2);
     delay(5);
     lcd.clear();
  }
     
     
