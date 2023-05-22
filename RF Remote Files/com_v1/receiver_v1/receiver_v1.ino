#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(2,15);
const byte address[6] = "00001";
unsigned long lastRecvTime = 0;

int Motor1Pin = 22;
int Motor2Pin = 0;
int Motor3Pin = 0;
int Motor4Pin = 0;


int throttle=0;
int pitch=0;
int roll=0;
int yaw=0;
int state=0;

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
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(Motor1Pin,OUTPUT);
  ledcSetup(0, 250, 12);
  ledcAttachPin(Motor1Pin, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    ResetData(); // Signal lost.. Reset data
  }
  Serial.print("Left Y= ");
  throttle = map(data.throttle, 0, 255, 1000, 2000);
  if(throttle>1200){
    throttle=1200;
  }
  Serial.print(throttle);
  Serial.print(" X = ");
  Serial.print(data.yaw);
  Serial.print(" Right Y= ");
  Serial.print(data.pitch);
  Serial.print(" X = ");
  Serial.print(data.roll);
  Serial.print("STATE");
  Serial.println(data.state);
  if(throttle>1200){
    throttle=1200;
  }
  ledcWrite(0,1.024*throttle);
}


void recvData(){
  while ( radio.available() ) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();   // receive the data | data alınıyor
  }
}

void ResetData()
{
// Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
// The middle position for Potenciometers. (254/2=127) | Potansiyometreler için orta konum
data.throttle = 0;  
data.yaw = 128;
data.roll = 128;
data.pitch = 128;
data.state = 0;
}