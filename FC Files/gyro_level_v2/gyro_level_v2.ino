/*
  V2 - GyroLevel with Variable adjustment - ESP32 + MPU6050
  How to access MPU-6050 and registers and get the readings!
*/

//Include Wire Library
#include <Wire.h>

//Defines
#define gyro250 0   //Gyro Fullscale Range +/- 250deg/sec
#define gyro500 1   //Gyro Fullscale Range +/- 500deg/sec
#define gyro1000 2  //Gyro Fullscale Range +/- 1000deg/sec
#define gyro2000 3  //Gyro Fullscale Range +/- 2000deg/sec
#define accel2 0    //Accel Fullscale Range +/- 2g
#define accel4 1    //Accel Fullscale Range +/- 4g
#define accel8 2    //Accel Fullscale Range +/- 8g
#define accel16 3   //Accel Fullscale Range +/- 16g

//Gyroscope variables
short gyro_x, gyro_y, gyro_z;
float rotX, rotY, rotZ;
//Gyroscope settings
int gyroScaleRange;
float gyroNormalizer;

//Accelerometer Values
short acc_x, acc_y, acc_z;
float gForceX, gForceY, gForceZ;
//Accelerometer settings;
int accelScaleRange;
float accelNormalizer;


void setup() {
  // Start Serial Monitor                                                 
  Serial.begin(115200);
  // Init I2C
  Wire.begin();
  //  1. Establish communication with MPU
  //  2. Setup all registers to read the data
  setupMPU(gyro250,accel4);
}

void loop() {
  // put your main code here, to run repeatedly:
  readAccelerometer();
  readGyroscope();
  printData();
  delay(100);
}

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

  if(accelSetting==accel2){
  accelScaleRange=0b00000000;  //Set register to 0  
  accelNormalizer=16384.0;    //Set LSB Sensitivity Normalizer
  }
  else if(accelSetting==accel4){
  accelScaleRange=0b00001000;  //Set register to 1   
  accelNormalizer=8192.0;     //Set LSB Sensitivity Normalizer
  }
  else if(accelSetting==accel8){
  accelScaleRange=0b00010000;  //Set register to 2    
  accelNormalizer=4096.0;     //Set LSB Sensitivity Normalizer 
  }
  else if(accelSetting==accel16){
  accelScaleRange=0b00011000;  //Set register to 3 
  accelNormalizer=2048.0;     //Set LSB Sensitivity Normalizer
  }

  initMPU();
}

void initMPU(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x6B);  //Write to register 0x6B to get out of sleep mode! (Note: Sleep is automatically turned on once the module recives power)
  Wire.write(0b00000000);  //Set SLEEP register to 0. (And all other modes to 0)
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1B); //Register for Gyroscope Configuration!
  Wire.write(gyroScaleRange); //Setting the gyro to +/- 250deg/sec! 0->250deg, 1->500deg, 2->1000deg, 3->2000deg
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1C); //Register for Accelerometer Configuration!
  Wire.write(accelScaleRange); // Setting the accel to +/- 2g! 0->2g, 1->4g, 2->8g, 3->16g
  Wire.endTransmission();
}

void readAccelerometer(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x3B);  //Starting Register for Accelerometer Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Requests the registers from (3B->40)
  while(Wire.available()<6);
  acc_x=Wire.read()<<8|Wire.read();
  acc_y=Wire.read()<<8|Wire.read();
  acc_z=Wire.read()<<8|Wire.read();

  processAccelerometerData();
}

void processAccelerometerData(){
  gForceX=acc_x/accelNormalizer; 
  gForceY=acc_y/accelNormalizer; 
  gForceZ=acc_z/accelNormalizer; 
}

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

void processGyroData(){
  rotX=gyro_x/gyroNormalizer; 
  rotY=gyro_y/gyroNormalizer; 
  rotZ=gyro_z/gyroNormalizer;   
}

void printData(){
  Serial.print("Gyro (deg) ");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print("Accel (g) ");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.println();
}