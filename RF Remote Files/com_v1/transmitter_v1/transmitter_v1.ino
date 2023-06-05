#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Pins on Controller CE/CNS
RF24 radio(10, 9);

const byte address[6] = "00001";

// int LeftLed1 = 0;
// int LeftLed2 = 1;
// int LeftButton = 2;
int RightButton = 3;
int RightSwitch = 4;
int RightSwitchValue;
int LeftSwitch = 5;
int LeftSwitchValue;
int RightLed1 = 7;
int RightLed2 = 6;
int RightButtonValueNew;
int RightButtonValueOld;
int FlightMode;
int ButtonFlightMode;
const int ManualMode = 0;
const int HoverMode = 1;
const int AutonomousMode = 2;


struct Signal {
byte throttle;
byte pitch;
byte roll;
byte yaw;
byte state;
};

Signal data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  pinMode(LeftSwitch,INPUT);
  pinMode(RightSwitch,INPUT);
  pinMode(RightButton,INPUT);
  pinMode(RightLed1,OUTPUT);
  pinMode(RightLed2,OUTPUT);
  RightButtonValueNew=0;
  RightButtonValueOld=0;
  LeftSwitchValue=0;
  FlightMode=ManualMode;
  ButtonFlightMode=HoverMode;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(" Left Switch = ");
  LeftSwitchValue = digitalRead(LeftSwitch);
  Serial.println(LeftSwitchValue);
  if(LeftSwitchValue){
    Serial.print("Left Y= ");
    data.throttle = mapJoystickValues(analogRead(A0),524,770,1020,false);
    Serial.print(data.throttle);
    Serial.print(" X= ");
    data.yaw = mapJoystickValues(analogRead(A1),12,515,1020,false);
    Serial.print(analogRead(A1));
    Serial.print(" Right Y= ");
    data.pitch = mapJoystickValues(analogRead(A2),12,520,1020,false);
    Serial.print(data.pitch);
    Serial.print(" X= ");
    data.roll = mapJoystickValues(analogRead(A3),12,518,1020,false);
    Serial.print(data.roll);
    
    //DRONE STATE   
    Serial.print(" Right Switch = ");
    RightSwitchValue = digitalRead(RightSwitch);
    Serial.print(RightSwitchValue);

    Serial.print(" Button = ");
    RightButtonValueOld = RightButtonValueNew;
    RightButtonValueNew = digitalRead(RightButton);
    Serial.println(RightButtonValueNew);

    data.state = droneState();

    radio.write(&data, sizeof(Signal));
  }
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
  val = map(val, lower, middle, 0, 128);
  else
  val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
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
    return ManualMode;
  }
    if(FlightMode==HoverMode){
    digitalWrite(RightLed1, HIGH);
    digitalWrite(RightLed2, LOW);
    Serial.println(" HOVER MODE");
    return HoverMode;
  }
    if(FlightMode==AutonomousMode){
    digitalWrite(RightLed1, HIGH);
    digitalWrite(RightLed2, HIGH);
    Serial.println(" AUTONOMOUS MODE");
    return AutonomousMode;
  }
}
