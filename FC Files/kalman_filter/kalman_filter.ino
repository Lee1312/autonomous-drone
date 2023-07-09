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
float RateRoll, RatePitch, RateYaw;
float RateRoll_Calib,RatePitch_Calib,RateYaw_Calib;
//Gyroscope settings
int gyroScaleRange;
float gyroNormalizer;

//Accelerometer Values
short acc_x, acc_y, acc_z;
float gForceX, gForceY, gForceZ;
float AngleRoll,AnglePitch;
//Accelerometer settings;
int accelScaleRange;
float accelNormalizer;

//Kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertanityRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertanityPitch = 2*2;
float Kalman1DOutput[]={0,0};

//Timers
unsigned long LoopTimer;

//Kalman Filter
void kalman_1d(float KalmanState,float KalmanUncertanity, float KalmanInput, float KalmanMeasurement){
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertanity = KalmanUncertanity + 0.004*0.004 + 4 * 4;
  float KalmanGain = KalmanUncertanity / (KalmanUncertanity + 3*3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertanity = (1-KalmanGain) * KalmanUncertanity;

  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertanity;
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
  //Normalized and Calibrated Data
  gForceX=acc_x/accelNormalizer-0.05; 
  gForceY=acc_y/accelNormalizer+0.03; 
  gForceZ=acc_z/accelNormalizer+0.07; 

  AngleRoll = atan(gForceY/sqrt(gForceX*gForceX+gForceZ*gForceZ));  //ROLL Angle in Radians -> Oko X ose
  AnglePitch = -atan(gForceX/sqrt(gForceY*gForceY+gForceZ*gForceZ));  //PITCH Angle in Radians - Oko Y ose

  AngleRoll *= 180.0/3.14159;
  AnglePitch *= 180.0/3.14159;
}

void readGyroscope(){
  Wire.beginTransmission(0b1101000);  //Device adress 0x68
  Wire.write(0x43);  //Starting Register for Gyrometer Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Requests the registers from (43->48)
  while(Wire.available()<6);
  gyro_x=Wire.read()<<8|Wire.read();
  gyro_y=Wire.read()<<8|Wire.read();
  gyro_z=Wire.read()<<8|Wire.read();

  processGyroData();
}

//Get Degree Rate from Gyroscope
void processGyroData(){
  //Normalized data
  RateRoll=(float)gyro_x/gyroNormalizer;
  RatePitch=(float)gyro_y/gyroNormalizer; 
  RateYaw=(float)gyro_z/gyroNormalizer;   
}

void setup() {
  // Start Serial Monitor                                                 
  Serial.begin(115200);
  // Init I2C
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  setupMPU(gyro500, accel8);

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
  LoopTimer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  readAccelerometer();
  readGyroscope();
  RateRoll -= RateRoll_Calib;
  RatePitch -= RatePitch_Calib;
  RateYaw -= RateYaw_Calib;

  kalman_1d(KalmanAngleRoll, KalmanUncertanityRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertanityRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertanityPitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertanityPitch = Kalman1DOutput[1];

  Serial.print("Roll_Angle_[°]:");
  Serial.print(KalmanAngleRoll);
  Serial.print(",");
  Serial.print("Pitch_Angle_[°]:");
  Serial.println(KalmanAnglePitch);

  while (micros()- LoopTimer<4000);
  LoopTimer = micros();
  
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
  Wire.write(0x1A); //Low Pass Filter Config!
  Wire.write(0x05); //Set Lowpass to 10Hz config!
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
