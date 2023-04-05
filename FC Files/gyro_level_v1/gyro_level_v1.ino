/*
  V1 - Primitive GyroLevel - ESP32 + MPU6050
  How to access MPU-6050 and registers and get the readings!
*/

//Include Wire Library
#include <Wire.h>

//Gyroscope variables
short gyro_x, gyro_y, gyro_z;
float rotX, rotY, rotZ;

short acc_x, acc_y, acc_z;
float gForceX, gForceY, gForceZ;


void setup() {
  // Start Serial Monitor                                                 
  Serial.begin(115200);
  // Init I2C
  Wire.begin();
  //  1. Establish communication with MPU
  //  2. Setup all registers to read the data
  setupMPU();
}

void loop() {
  // put your main code here, to run repeatedly:
  readAccelerometer();
  readGyroscope();
  printData();
  delay(100);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x6B);  //Write to register 0x6B to get out of sleep mode! (Note: Sleep is automatically turned on once the module recives power)
  Wire.write(0b00000000);  //Set SLEEP register to 0. (And all other modes to 0)
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1B); //Register for Gyroscope Configuration!
  Wire.write(0b00000000); //Setting the gyro to +/- 250deg/sec! 0->250deg, 1->500deg, 2->1000deg, 3->2000deg
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x1C); //Register for Accelerometer Configuration!
  Wire.write(0b00000000); // Setting the accel to +/- 2g! 0->2g, 1->4g, 2->8g, 3->16g
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
  gForceX=acc_x/16384.0; 
  gForceY=acc_y/16384.0; 
  gForceZ=acc_z/16384.0; 
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
  rotX=gyro_x/131.0; 
  rotY=gyro_y/131.0; 
  rotZ=gyro_z/131.0;   
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