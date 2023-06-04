#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int LeftLed1 = 0;
int LeftLed2 = 1;
int LeftButton = 2;
int RightButton = 3;
int RightSwitch = 4;
int RightSwitchValue;
int LeftSwitch = 5;
int RightLed1 = 7;
int RightLed2 = 6;
int RightButtonValueNew;
int RightButtonValueOld;
int FlightMode;
int ButtonFlightMode;
const int ManualMode = 0;
const int HoverMode = 1;
const int AutonomousMode = 2;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LeftSwitch,INPUT);
  pinMode(LeftButton,INPUT);
  pinMode(RightSwitch,INPUT);
  pinMode(RightButton,INPUT);
  pinMode(LeftLed1,OUTPUT);
  pinMode(LeftLed2,OUTPUT);
  pinMode(RightLed1,OUTPUT);
  pinMode(RightLed2,OUTPUT);
  
  digitalWrite(LeftLed1, LOW);
  delay(10);
  digitalWrite(LeftLed2, LOW);
  delay(10);
  digitalWrite(RightLed1, LOW);
  delay(10);
  digitalWrite(RightLed2, LOW);
  delay(10);

  RightButtonValueNew=0;
  RightButtonValueOld=0;
  FlightMode=ManualMode;
  ButtonFlightMode=HoverMode;
}

void loop() {
  Serial.print("Left X= ");
  Serial.print(analogRead(A0));
  Serial.print(" Y= ");
  Serial.print(analogRead(A1));
  Serial.print(" Right X= ");
  Serial.print(analogRead(A2));
  Serial.print(" Y= ");
  Serial.print(analogRead(A3));

  Serial.print(" Right Switch = ");
  RightSwitchValue = digitalRead(RightSwitch);
  Serial.print(RightSwitchValue);

  Serial.print(" Button = ");
  RightButtonValueOld = RightButtonValueNew;
  RightButtonValueNew = digitalRead(RightButton);
  Serial.print(RightButtonValueNew);

  droneState();
  digitalWrite(LeftLed1, LOW);
  delay(1);
  digitalWrite(LeftLed2, LOW);
}

int droneState(){
  if(RightSwitchValue == 0)
  {
    FlightMode=ManualMode;
    ButtonFlightMode=HoverMode;
  }else{
    FlightMode=ButtonFlightMode;
    if(RightButtonValueOld == 1 && RightButtonValueNew == 0){
      if(ButtonFlightMode==HoverMode){
        ButtonFlightMode=AutonomousMode;
      }else{
        ButtonFlightMode=HoverMode; 
      }
    }
  }

  if(FlightMode==ManualMode){
    digitalWrite(RightLed1, LOW);
    digitalWrite(RightLed2, LOW);
    Serial.println("  MANUAL MODE");
  }
    if(FlightMode==HoverMode){
    digitalWrite(RightLed1, HIGH);
    digitalWrite(RightLed2, LOW);
    Serial.println(" HOVER MODE");
  }
    if(FlightMode==AutonomousMode){
    digitalWrite(RightLed1, HIGH);
    digitalWrite(RightLed2, HIGH);
    Serial.println(" AUTONOMOUS MODE");
  }
}

