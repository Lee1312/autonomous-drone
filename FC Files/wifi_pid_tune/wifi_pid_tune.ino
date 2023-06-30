#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
// SSID and password of Wifi connection:
const char* ssid = "Baka Coka";
const char* password = "M@rence1337";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

//Google 'HTML COMPRESS' and use it to compress the whole HTML site to a single string!
String webpage = "<!DOCTYPE html><html><head><title>ESP32 PID-Tune</title></head><body style='background-color: #EEEEEE;'><span style='color: #003366;'><h1>ESP32 PID-Tune</h1><h2>SET PID Values</h2><p>Write DECIMAL VALUES with DOT (example '3.14')</p><table bgcolor='#000000'> <tr bgcolor='#ffffff'> <th></th> <th>Pitch</th> <th>Roll</th> <th>Yaw</th> </tr bgcolor='#ffffff'><tr bgcolor='#ffffff'> <td>P VALUES</td> <td><input type='number' id='p-pitch' step='0.01'></td> <td><input type='number' id='p-roll' step='0.01'></td> <td><input type='number' id='p-yaw' step='0.01'></td></tr><tr bgcolor='#ffffff'> <td>I VALUES</td> <td><input type='number' id='i-pitch' step='0.01'></td> <td><input type='number' id='i-roll' step='0.01'></td> <td><input type='number' id='i-yaw' step='0.01'></td></tr><tr bgcolor='#ffffff'> <td>D VALUES</td> <td><input type='number' id='d-pitch' step='0.01'></td> <td><input type='number' id='d-roll' step='0.01'></td> <td><input type='number' id='d-yaw' step='0.01'></td></tr></table> <p><button type='button' id='btn'> Send PID values to ESP32 </button></p><h3>ACKNOWLEDGE that ESP32 recived the values</h3><p>(Values from both tables should match)</p><table cellspacing='3' bgcolor='#000000'> <caption>Current ESP32 PID Values</caption> <tr bgcolor='#ffffff'> <th></th> <th>Pitch</th> <th>Roll</th> <th>Yaw</th> </tr><tr bgcolor='#ffffff'> <td>P VALUES</td> <td><span id='p-pitch-ack'></span></td> <td><span id='p-roll-ack'></span></td> <td><span id='p-yaw-ack'></span></td></tr><tr bgcolor='#ffffff'> <td>I VALUES</td> <td><span id='i-pitch-ack'></span></td> <td><span id='i-roll-ack'></span></td> <td><span id='i-yaw-ack'></span></td></tr><tr bgcolor='#ffffff'> <td>D VALUES</td> <td><span id='d-pitch-ack'></span></td> <td><span id='d-roll-ack'></span></td> <td><span id='d-yaw-ack'></span></td></tr></table> <h2>GRAPHS</h2></span></body><script>var Socket;document.getElementById('btn').addEventListener('click',send_pid_to_Esp32);function send_pid_to_Esp32(){ var p_pitch_val = document.getElementById('p-pitch').value; var p_roll_val = document.getElementById('p-roll').value; var p_yaw_val = document.getElementById('p-yaw').value; var i_pitch_val = document.getElementById('i-pitch').value; var i_roll_val = document.getElementById('i-roll').value; var i_yaw_val = document.getElementById('i-yaw').value; var d_pitch_val = document.getElementById('d-pitch').value; var d_roll_val = document.getElementById('d-roll').value; var d_yaw_val = document.getElementById('d-yaw').value;var pid = {p_pitch: p_pitch_val, p_roll:p_roll_val, p_yaw: p_yaw_val, i_pitch: i_pitch_val, i_roll: i_roll_val, i_yaw: i_yaw_val, d_pitch: d_pitch_val, d_roll: d_roll_val, d_yaw: d_yaw_val}; console.log('Send PID to ESP32'); console.log(JSON.stringify(pid));Socket.send(JSON.stringify(pid));}function init(){Socket = new WebSocket('ws://'+window.location.hostname+':81/');Socket.onmessage = function(event){processCommand(event);};}function processCommand(event){var obj = JSON.parse(event.data); console.log('Recived PID from ESP32');console.log(obj);document.getElementById('p-pitch-ack').innerHTML = obj.p_pitch_ack;document.getElementById('p-roll-ack').innerHTML = obj.p_roll_ack; document.getElementById('p-yaw-ack').innerHTML = obj.p_yaw_ack; document.getElementById('i-pitch-ack').innerHTML = obj.i_pitch_ack;document.getElementById('i-roll-ack').innerHTML = obj.i_roll_ack; document.getElementById('i-yaw-ack').innerHTML = obj.i_yaw_ack; document.getElementById('d-pitch-ack').innerHTML = obj.d_pitch_ack;document.getElementById('d-roll-ack').innerHTML = obj.d_roll_ack; document.getElementById('d-yaw-ack').innerHTML = obj.d_yaw_ack;}window.onload = function(event){init();}</script></html>";

StaticJsonDocument<200> doc_tx;
StaticJsonDocument<200> doc_rx;

//Defines
#define gyro250 0   //Gyro Fullscale Range +/- 250deg/sec
#define gyro500 1   //Gyro Fullscale Range +/- 500deg/sec
#define gyro1000 2  //Gyro Fullscale Range +/- 1000deg/sec
#define gyro2000 3  //Gyro Fullscale Range +/- 2000deg/sec
#define accel2 0    //Accel Fullscale Range +/- 2g
#define accel4 1    //Accel Fullscale Range +/- 4g
#define accel8 2    //Accel Fullscale Range +/- 8g
#define accel16 3   //Accel Fullscale Range +/- 16g

//LED Define
int LEDRedPin = 12;
int LEDGreenPin = 14;

//Gyroscope variables
short gyro_x, gyro_y, gyro_z;
float RateRoll, RatePitch, RateYaw;
float RateRoll_Calib,RatePitch_Calib,RateYaw_Calib;
int Rate_Calib_Number;
//Gyroscope settings
int gyroScaleRange;
float gyroNormalizer;

//Define Radio and CommuncationChannel
RF24 radio(2,15);
const byte address[6] = "00001";

//Define structure of the RadioSignal
struct Signal {
byte throttle;
byte pitch;
byte roll;
byte yaw;
byte state;
};
//Init
Signal data;

//Timers
unsigned long lastRecvTime = 0;
unsigned long LoopTimer;

//Define JoyStickValues (1000-2000) from the (0-255) Signal Data, byte values
float InputThrottle=0;
float InputPitch=0;
float InputRoll=0;
float InputYaw=0;
int DroneState=0;

//PID Variables
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0,0,0};

float PRateRoll=0; float PRatePitch=0; float PRateYaw = 0;
float IRateRoll=0; float IRatePitch=0; float IRateYaw = 0;
float DRateRoll=0; float DRatePitch=0; float DRateYaw = 0;
//--------

//Motor Variables
int Motor1Pin = 32;
int Motor2Pin = 33;
int Motor3Pin = 25;
int Motor4Pin = 26;
float Motor1Input,Motor2Input,Motor3Input,Motor4Input;  

void recvData(){
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();   // receive the data | data alınıyor
  }
}

void ResetData()
{
  data.throttle = 0;  
  data.yaw = 128;
  data.roll = 128;
  data.pitch = 128;
  data.state = 0;
}

//MPU6050 Precision
void setupMPU(int gyroSetting, int accelSetting){
  if (gyroSetting==gyro250) {
  gyroScaleRange=0b00000000;  //Set register to 0
  gyroNormalizer=131.0;   //Set LSB Sensitivity Normalizer
  }
  else if (gyroSetting==gyro500) {
  gyroScaleRange=0b00001000;  //Set register to 1
  gyroNormalizer=65.5;    //Set LSB Sensitivity Normalizer
  }
  else if (gyroSetting==gyro1000) {
  gyroScaleRange=0b00010000;  //Set register to 2
  gyroNormalizer=32.8;    //Set LSB Sensitivity Normalizer
  }
  else if (gyroSetting==gyro2000){
  gyroScaleRange=0b00011000;  //Set register to 3
  gyroNormalizer=16.4;    //Set LSB Sensitivity Normalizer
  }

  // if(accelSetting==accel2){
  // accelScaleRange=0b00000000;  //Set register to 0  
  // accelNormalizer=16384.0;    //Set LSB Sensitivity Normalizer
  // }
  // else if(accelSetting==accel4){
  // accelScaleRange=0b00001000;  //Set register to 1   
  // accelNormalizer=8192.0;     //Set LSB Sensitivity Normalizer
  // }
  // else if(accelSetting==accel8){
  // accelScaleRange=0b00010000;  //Set register to 2    
  // accelNormalizer=4096.0;     //Set LSB Sensitivity Normalizer 
  // }
  // else if(accelSetting==accel16){
  // accelScaleRange=0b00011000;  //Set register to 3 
  // accelNormalizer=2048.0;     //Set LSB Sensitivity Normalizer
  // }

  initMPU();
}

//MPU6050 Initialization
void initMPU(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x6B);  //Write to register 0x6B to get out of sleep mode! (Note: Sleep is automatically turned on once the module recives power)
  Wire.write(0b00000000);  //Set SLEEP register to 0. (And all other modes to 0)
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1A); //Low Pass Filter Config!
  Wire.write(0x05); //Set Lowpass to 10Hz config!
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1B); //Register for Gyroscope Configuration!
  Wire.write(gyroScaleRange); //Setting the gyro to +/- 250deg/sec! 0->250deg, 1->500deg, 2->1000deg, 3->2000deg
  Wire.endTransmission();


  // Wire.beginTransmission(0b1101000);  //Device adress 0x68
  // Wire.write(0x1C); //Register for Accelerometer Configuration!
  // Wire.write(accelScaleRange); // Setting the accel to +/- 2g! 0->2g, 1->4g, 2->8g, 3->16g
  // Wire.endTransmission();
}

//Access and read data from Gyroscope
void readGyroscope(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x43);  //Starting Register for Gyrometer Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Requests the registers from (3B->40)
  while(Wire.available()<6);
  gyro_x=Wire.read()<<8|Wire.read();
  gyro_y=Wire.read()<<8|Wire.read();
  gyro_z=Wire.read()<<8|Wire.read();

  processGyroData();
}

//Get Degree Rate from Gyroscope
void processGyroData(){
  //Normalized data
  RateRoll=gyro_x/gyroNormalizer;
  RatePitch=gyro_y/gyroNormalizer; 
  RateYaw=gyro_z/gyroNormalizer;   
}

void PID_Equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;       
  float Dterm = D * (Error-PrevError)/0.004;            
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  PIDReturn[0]=PIDOutput;  
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

//Resets PID Variables
void ResetPID(){
  PrevErrorRateRoll=0;
  PrevErrorRatePitch=0;
  PrevErrorRateYaw=0;
  PrevItermRateRoll=0;
  PrevItermRatePitch=0;
  PrevItermRateYaw=0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);                 
  Serial.println("START SETUP");
  //LED Setup
  pinMode(LEDRedPin,OUTPUT);  
  pinMode(LEDGreenPin,OUTPUT); 

  WiFi.begin(ssid, password);
  Serial.println("Establishing connection to WiFi with SSID: " + String(ssid));
 
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LEDRedPin, HIGH);
    digitalWrite(LEDGreenPin, LOW); 
    delay(250);
    digitalWrite(LEDRedPin, LOW);
    digitalWrite(LEDGreenPin, HIGH); 
    delay(250);
    Serial.print(".");
  }
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", [](){
    server.send(200, "text\html",webpage);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  digitalWrite(LEDRedPin, LOW);
  digitalWrite(LEDGreenPin, LOW); 
  delay(500);
  digitalWrite(LEDRedPin, HIGH);
  digitalWrite(LEDGreenPin, HIGH); 
  delay(500);
  digitalWrite(LEDRedPin, LOW);
  digitalWrite(LEDGreenPin, LOW); 
  delay(500);  
  digitalWrite(LEDRedPin, HIGH);

  Serial.println("SERVER and LED SETUP FINISH");
  
   //Start communicating with MPU
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  setupMPU(gyro250,accel4);

  Serial.println("GYRO AND RADIO FINISH");

  //DONT TOUCH DRONE WHILE CALIBRATION
  for(int i=0; i<2000; i++) //Calibrate the Gyroscope
  {
    readGyroscope();
    RateRoll_Calib+=RateRoll;
    RatePitch_Calib+=RatePitch;
    RateYaw_Calib+=RateYaw;
    delay(1);
  }
  
  //Calibartion variables for our GYRO  
  RateRoll_Calib/=2000;
  RatePitch_Calib/=2000;
  RateYaw_Calib/=2000;

  Serial.println("GYRO CALIB FINISH");

  //PWM MOTOR SETUP
  pinMode(Motor1Pin,OUTPUT);
  pinMode(Motor2Pin,OUTPUT);
  pinMode(Motor3Pin,OUTPUT);
  pinMode(Motor4Pin,OUTPUT);

  ledcSetup(0, 250, 12);
  ledcSetup(1, 250, 12);
  ledcSetup(2, 250, 12);
  ledcSetup(3, 250, 12);

  ledcAttachPin(Motor1Pin, 0);
  ledcAttachPin(Motor2Pin, 1);
  ledcAttachPin(Motor3Pin, 2);
  ledcAttachPin(Motor4Pin, 3);
  //--------

  //Radio Reciver Code
  radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  LoopTimer = micros();  
  digitalWrite(LEDRedPin, LOW);
  digitalWrite(LEDGreenPin, HIGH);
  Serial.print("FINISHED SETUP");

}

void loop() {
  // put your main code here, to run repeatedly:

  server.handleClient();
  webSocket.loop();

  readGyroscope();
  RateRoll -= RateRoll_Calib;
  RatePitch -= RatePitch_Calib;
  RateYaw -= RateYaw_Calib;

  //Read and transform Reciver Inputs
  recvData();
   // Signal lost if more than 1 seconde elapsed
  if ( millis() - lastRecvTime > 1000 ) {
    ResetData();
  }
  InputThrottle = map(data.throttle, 0, 255, 1000, 2000); 
  DesiredRateRoll = map(data.roll, 0, 255 ,-75 , 75);
  DesiredRatePitch = map(data.pitch, 0, 255 ,-75 , 75);
  DesiredRateYaw = map(data.yaw, 0, 255 ,-75 , 75);

  //Calculate the errors
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  //PID Calculations
  //ROLL
  PID_Equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];  
  PrevItermRateRoll = PIDReturn[2];
  //PITCH
  PID_Equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];  
  PrevItermRatePitch = PIDReturn[2];
  //YAW
  PID_Equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];  
  PrevItermRateYaw = PIDReturn[2];
  
  //Limit Throttle <=1800 to leave room for Manuvers
  if(InputThrottle > 1800) InputThrottle = 1800;  

  //Motor Calculations !!! HIGHLY DEPENDS ON MOTOR ORIENTATION AND ESC ORIENTATION !!!
  Motor1Input = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  Motor2Input = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  Motor3Input = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  Motor4Input = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

  //Avoid overloading motors after equation!
  if(Motor1Input > 2000) Motor1Input=1999;
  if(Motor2Input > 2000) Motor2Input=1999;
  if(Motor3Input > 2000) Motor3Input=1999;
  if(Motor4Input > 2000) Motor4Input=1999;
    
  //Keep motor spining if no input but throttle is used!
  int ThrottleIdle = 1150;  
  if(Motor1Input < ThrottleIdle) Motor1Input=ThrottleIdle;
  if(Motor2Input < ThrottleIdle) Motor2Input=ThrottleIdle;
  if(Motor3Input < ThrottleIdle) Motor3Input=ThrottleIdle;
  if(Motor4Input < ThrottleIdle) Motor4Input=ThrottleIdle;
  
  //If Throttle belove 1050 shut motors down
  int ThrottleCutOff = 1000;
  if(InputThrottle < 1050){
    Motor1Input = ThrottleCutOff;
    Motor2Input = ThrottleCutOff;
    Motor3Input = ThrottleCutOff;
    Motor4Input = ThrottleCutOff;
    ResetPID();
  } 

  //Send PWM to Motors
  ledcWrite(0,Motor1Input);
  ledcWrite(1,Motor2Input);
  ledcWrite(2,Motor3Input);
  ledcWrite(3,Motor4Input);

  print_Data();
  
  //Finish the 250Hz Controll Loop!
  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();  

}


void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length ){
  switch(type){
    case WStype_DISCONNECTED:
      Serial.println("Client Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Client Connected");
      //Add code when connected
      break;
    case WStype_TEXT: //check response from client
      Serial.println("RECIVED MESSAGE");
      DeserializationError error = deserializeJson(doc_rx,payload);
      if(error){
        Serial.println("Deserialization failed");
        return;        
      }else{
        PRatePitch = doc_rx["p_pitch"];
        PRateRoll = doc_rx["p_roll"];
        PRateYaw = doc_rx["p_yaw"];
        IRatePitch = doc_rx["i_pitch"];
        IRateRoll = doc_rx["i_roll"];
        IRateYaw = doc_rx["i_yaw"];
        DRatePitch = doc_rx["d_pitch"];
        DRateRoll = doc_rx["d_roll"];
        DRateYaw = doc_rx["d_yaw"];                
        Serial.println("Received PID info!");
        Serial.println("P-PITCH: "+ String(PRatePitch));
        Serial.println("P-ROLL: "+ String(PRateRoll));
        Serial.println("P-YAW: "+ String(PRateYaw));
        Serial.println("I-PITCH: "+ String(IRatePitch));
        Serial.println("I-ROLL: "+ String(IRateRoll));
        Serial.println("I-YAW: "+ String(IRateYaw));
        Serial.println("D-PITCH: "+ String(DRatePitch));
        Serial.println("D-ROLL: "+ String(DRateRoll));
        Serial.println("D-YAW: "+ String(DRateYaw));

        sendAcknowledge();
      }  
      Serial.println();
      break;
  }
}


void sendAcknowledge(){
  String jsonString = "";
  JsonObject objectjson = doc_tx.to<JsonObject>();
  objectjson["p_pitch_ack"]=PRatePitch;
  objectjson["p_roll_ack"]=PRateRoll;
  objectjson["p_yaw_ack"]=PRateYaw;
  objectjson["i_pitch_ack"]=IRatePitch;
  objectjson["i_roll_ack"]=IRateRoll;
  objectjson["i_yaw_ack"]=IRateYaw;
  objectjson["d_pitch_ack"]=DRatePitch;
  objectjson["d_roll_ack"]=DRateRoll;
  objectjson["d_yaw_ack"]=DRateYaw;
  serializeJson(doc_tx,jsonString);
  Serial.println(jsonString);
  webSocket.broadcastTXT(jsonString);
}

void print_Data(){
  
  Serial.print("Gyro (deg) X= ");
  Serial.print(RateRoll);
  Serial.print(" Y= ");
  Serial.print(RatePitch);
  Serial.print(" Z= ");  
  Serial.print(RateYaw);
  
  Serial.print(" Throttle= ");
  Serial.print(InputThrottle);
  Serial.print(" Roll= ");
  Serial.print(InputRoll);
  Serial.print(" Pitch= ");
  Serial.print(InputPitch);
  Serial.print(" Yaw= ");
  Serial.print(InputYaw);
  
  // Serial.print(" DesiredRoll= ");
  // Serial.print((int)data.roll);
  // Serial.print(" <=> ");
  // Serial.print( DesiredRateRoll);
  // Serial.print(" DesiredPitch= ");
  // Serial.print((int)data.pitch);
  // Serial.print(" <=> ");
  // Serial.print(DesiredRatePitch);
  // Serial.print(" DesiredYaw= ");
  // Serial.print((int)data.yaw );
  // Serial.print(" <=> ");
  // Serial.println( DesiredRateYaw);
  
  Serial.print("M1= ");
  Serial.print(Motor1Input);
  Serial.print(" M2= ");
  Serial.print(Motor2Input);
  Serial.print(" M3= ");
  Serial.print(Motor3Input);
  Serial.print(" M4= ");
  Serial.println(Motor4Input);
}