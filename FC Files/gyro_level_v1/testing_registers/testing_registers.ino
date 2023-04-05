#include <Wire.h>

short accelX_H, accelY_H, accelZ_H, accelX_L,accelY_L,accelZ_L;
float gForceX, gForceY, gForceZ;

short temp_H,temp_L;

short gyroX_H, gyroY_H, gyroZ_H,gyroX_L,gyroY_L,gyroZ_L;
float rotX, rotY, rotZ;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}


void loop() {
  recordAccelRegisters();
  recordTempRegisters();
  recordGyroRegisters();
  printData();
  delay(10);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX_H = Wire.read(); //Store first two bytes into accelX
  accelX_L = Wire.read();
  accelY_H = Wire.read();
  accelY_L = Wire.read(); //Store middle two bytes into accelY
  accelZ_H = Wire.read();
  accelZ_L = Wire.read(); //Store last two bytes into accelZ
  // processAccelData();
}

// void processAccelData(){
//   gForceX = accelX / 16384.0;
//   gForceY = accelY / 16384.0; 
//   gForceZ = accelZ / 16384.0;
// }

void recordTempRegisters(){
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x41); //Starting register for Accel Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,2); //Request Accel Registers (41 - 42)
  while(Wire.available() < 2);
  temp_H = Wire.read(); //Store first two bytes into accelX
  temp_L = Wire.read();
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX_H = Wire.read();
  gyroX_L = Wire.read(); //Store first two bytes into accelX
  gyroY_H = Wire.read();
  gyroY_L = Wire.read(); //Store middle two bytes into accelY
  gyroZ_H = Wire.read();
  gyroZ_L = Wire.read(); //Store last two bytes into accelZ
  // processGyroData();
}

// void processGyroData() {
//   rotX = gyroX / 131.0;
//   rotY = gyroY / 131.0; 
//   rotZ = gyroZ / 131.0;
// }

void printData() {
  // Serial.print("Gyro (deg)");
  // Serial.print(" X=");
  // Serial.print(rotX);
  // Serial.print(" Y=");
  // Serial.print(rotY);
  // Serial.print(" Z=");
  // Serial.print(rotZ);
  // Serial.print(" Accel (g)");
  // Serial.print(" X=");
  // Serial.print(gForceX);
  // Serial.print(" Y=");
  // Serial.print(gForceY);
  // Serial.print(" Z=");
  // Serial.println(gForceZ);

  Serial.print("Gyro (raw)");
  Serial.print(" X_H=");
  Serial.print(gyroX_H);
  Serial.print(" X_L=");
  Serial.print(gyroX_L);  
  Serial.print(" Y_H=");
  Serial.print(gyroY_H);
  Serial.print(" Y_L=");
  Serial.print(gyroY_L);  
  Serial.print(" Z_H=");
  Serial.print(gyroZ_H);
  Serial.print(" Z_L=");
  Serial.print(gyroZ_L); 
  Serial.print(" TEMP_H=");
  Serial.print(temp_H);
  Serial.print(" TEMP_L=");
  Serial.print(temp_L);     
  Serial.print(" Accel (raw)");
  Serial.print(" X_H=");
  Serial.print(accelX_H);
  Serial.print(" X_L=");
  Serial.print(accelX_L);
  Serial.print(" Y_H=");
  Serial.print(accelY_H);
  Serial.print(" Y_L=");
  Serial.print(accelY_L);
  Serial.print(" Z_H=");
  Serial.print(accelZ_H);
  Serial.print(" Z_L=");
  Serial.println(accelZ_L);
}
