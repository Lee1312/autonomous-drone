#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int AutonomousLED = 2;
int Switch = 3;
int SwitchValue;
int Button = 4;
int ButtonValueNew;
int ButtonValueOld;
int FlightMode;
int ButtonFlightMode;
const int ManualMode = 0;
const int HoverMode = 1;
const int AutonomousMode = 2;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Switch,INPUT);
  pinMode(Button,INPUT);
  ButtonValueNew=0;
  ButtonValueOld=0;
  FlightMode=ManualMode;
  ButtonFlightMode=HoverMode;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Right X= ");
  Serial.print(analogRead(A0));
  Serial.print(" Y= ");
  Serial.print(analogRead(A1));
  Serial.print(" Left X= ");
  Serial.print(analogRead(A2));
  Serial.print(" Y= ");
  Serial.print(analogRead(A3));
  Serial.print(" Switch = ");
  SwitchValue = digitalRead(Switch);
  Serial.print(SwitchValue);
  Serial.print(" Button = ");
  ButtonValueOld = ButtonValueNew;
  ButtonValueNew = digitalRead(Button);
  Serial.print(ButtonValueNew);
  droneState();
}

int droneState(){
  if(SwitchValue == 0)
  {
    FlightMode=ManualMode;
    ButtonFlightMode=HoverMode;
  }else{
    FlightMode=ButtonFlightMode;
    if(ButtonValueOld == 1 && ButtonValueNew == 0){
      if(ButtonFlightMode==HoverMode){
        ButtonFlightMode=AutonomousMode;
      }else{
        ButtonFlightMode=HoverMode; 
      }
    }
  }

  if(FlightMode==ManualMode){
    Serial.println("  MANUAL MODE");
  }
    if(FlightMode==HoverMode){
    Serial.println(" HOVER MODE");
  }
    if(FlightMode==AutonomousMode){
    Serial.println(" AUTONOMOUS MODE");
  }
}
