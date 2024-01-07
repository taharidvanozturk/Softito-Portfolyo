#include "Arduino.h"
#include "titan.h"
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
LiquidCrystal_I2C lcd(0x27,20,4);

//-----ledKontrol----
int ledS=0;
//-------------------
//-----fanBilgisi----
int fanS;
//-------------------
//-----suBaskın------
int suS;
//--------------------
//-----pid kodları----
int POT = A0;
double Setpoint, Input, Output; // PID parametreleri
double Kp = 10, Ki = 0.5, Kd = 7; // PID katsayıları
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, AUTOMATIC);

//-----pid kodları----
  void titan :: init(){
  int count=0;
  pinMode(buton_pin,INPUT_PULLUP);
  pinMode(rol,OUTPUT);
  pinMode(fan,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(sensor_pin,INPUT);
  

  //-----LedInterrupt---
  
  
  //-----pid kodları----
   // Hedeflenen sıcaklık değeri (örneğin 25 derece)
  pid.SetMode(AUTOMATIC); // PID'nin otomatik modda çalışmasını sağlar
  pid.SetOutputLimits(0, 255); // PWM sinyal aralığı (0-255)
  //-----pid kodları----
  lcd.init();
  lcd.backlight();
}

int titan :: fanControl(int t){
    
   if (t >= 30) {
    Setpoint = 30;
    //digitalWrite(fan, HIGH);
    digitalWrite(led, LOW);
    
    Input=t;
    pid.Compute();
    analogWrite(fan, Output); // Fan hızını ayarla

    lcd.setCursor(0,0);
    lcd.print("Sicaklik:");
    lcd.setCursor(9,0);
    lcd.print(t);  
    lcd.print("*C");
    
    lcd.setCursor(0,1);
    lcd.print("Sogutma Acik");
    fanS=1;
    
  } 
  else {
    Setpoint = 25;
    digitalWrite(fan, LOW);
    //digitalWrite(led, HIGH);
    
    Input=t;
    pid.Compute();
    analogWrite(led, Output); // Fan hızını ayarla
 
    lcd.setCursor(0,0);
    lcd.print("Sicaklik:");
    lcd.setCursor(9,0);
    lcd.print(t);   
    lcd.print("*C");
    
    lcd.setCursor(0,1);
    lcd.print("Isitma Acik");
    fanS=0;
  }
  return fanS;
}

int  titan::butonOkuma(){
  
if (digitalRead(buton_pin) == HIGH) {  //butona basinca röle açiliyor, tekrar basinca kapatiyor AYDINLATMA DEVRESİ
    if(digitalRead(led)==false){  
    digitalWrite(rol, HIGH);
    Serial.println("a");
    Serial.println(digitalRead(buton_pin));
    if(ledS==0){
    lcd.clear();
    lcd.print("Lamba acik");
    delay(500);
    
  }
    }
    ledS=1;
    
}
else {
  if(digitalRead(led)==true){
   digitalWrite(rol, LOW);
    Serial.println("b");
   Serial.println(digitalRead(buton_pin));
   if(ledS==1){
    lcd.clear();
    lcd.print("Lamba kapali");
    delay(500);
    }
  }
   ledS=0;
}
return ledS;
  
}
int titan :: suKontrol(){
  int su_baskin_degeri = analogRead(sensor_pin);

  su_baskin_degeri = map(su_baskin_degeri, 0, 1023, 100, 0);

  Serial.print("Nem : ");

  Serial.print(su_baskin_degeri);

  Serial.println("%");
  
  if(su_baskin_degeri>=45){
    Serial.println("su basti");
    suS=1;
  }
  else{

    suS=0;
  }
return suS;
}
