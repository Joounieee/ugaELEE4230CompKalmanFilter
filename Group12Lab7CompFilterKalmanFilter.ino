
#include <Wire.h>
#include <MLX90393.h> 
#include <SparkFunLSM6DSO.h>

MLX90393 MLX;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)
LSM6DSO LSM;

float magX, magY, magZ, magTemp, accelX, accelY, accelZ, gyrX, gyrY, gyrZ, IMUTemp;
uint16_t accelXRaw, accelYRaw, accelZRaw, gyrXRaw, gyrYRaw, gyrZRaw, accelRange, gyroRange;
float gyroDataRange, accelDataRange;

float prevAccelX;
float prevGyrX;
float lowPass, highPass;
unsigned long currTime,prevTime,dTime;
float x_m, y_m;

float alpha = 0.98;
float comp_fil_angle_x = 0;
float comp_fil_angle_y = 0;
float comp_fil_angle_z = 0;
float comp_fil_accel_x,comp_fil_accel_y,comp_fil_mag_z;
float mag_norm;

// Kalman filter stuff
static const float R = 40; // noise covariance
static const float H = 1.00; // measurement map scalar
float kalman_fil_angle_z = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  //failsafe error checking to prevent irri\tation with no output. basically checks to make sure both sensors are connected and running
  if (!LSM.begin()) {
    //Serial.println("IMU not found. Check wiring!");
    delay(500);
  }
  if (!MLX.begin()) {
    //Serial.println("Magnetometer not found. Check wiring!");
    delay(500);
  }
  
  //gain is set as 1 for this example, basically meaning no gain is applied to signal.
  MLX.setGainSel(1);

  //resolution is set to 0 for each axis, representing the finest resolution the sensor is capable of.
  MLX.setResolution(0, 0, 0); //x, y, z

  //oversampling and digital filtering are untouched
  MLX.setOverSampling(0);
  MLX.setDigitalFiltering(0);
  
  if (LSM.initialize(BASIC_SETTINGS)) {
    //Serial.println("Loaded Basic Settings");
  }
  delay(500);


  
}//end of setup

//Below are functions for pulling data from sensors. Each 3-axis "piece" is passed into individual functions to make it easier to manipulate. 
//Simply call the function in the loop and then decide which data you want to print/use

//function to read magnetometer data
void read_Mag_Data() {
  MLX.readData(data);
  magX = data.x;
  magY = data.y;
  magZ = data.z;
}

//function to read magnetometer temp data
void read_Mag_Temp() {
  MLX.readData(data);
  magTemp = data.t;
}

//function to read IMU accelerometer data
void read_LSM_Accel_Data() {
  accelX = LSM.readFloatAccelX();
  accelY = LSM.readFloatAccelY();
  accelZ = LSM.readFloatAccelZ();
  accelXRaw = LSM.readRawAccelX();
  accelYRaw = LSM.readRawAccelY();
  accelZRaw = LSM.readRawAccelZ();
  accelRange = LSM.getAccelRange();
}

//function to read IMU gyro data
void read_LSM_Gyro_Data() {
  gyrX = LSM.readFloatGyroX();
  gyrY = LSM.readFloatGyroY();
  gyrZ = LSM.readFloatGyroZ();
  gyrXRaw = LSM.readRawGyroX();
  gyrYRaw = LSM.readRawGyroY();
  gyrZRaw = LSM.readRawGyroZ();
  gyroRange = LSM.getGyroRange();
}

//function to read IMU temp data
void read_LSM_Temp() {
  IMUTemp = LSM.readTempF();
}

float kalmanFilter(float U) {
  static float Q = 10; // initial estimated covariance
  static float P = 0; // initial error covariance (has to be 0)
  static float U_hat = 0; // intial estimated state (assume we dont know)
  static float kalmanGain = 0; // initial kalman gain

  kalmanGain = P*H/(H*P*H+R);
  U_hat = U_hat + kalmanGain*(U-H*U_hat);

  P = (1-kalmanGain*H)*P+Q;

  return U_hat;
}

//loop below polls functions above to gather and then print data to serial monitor
void loop()
{
  //function calling section. Will "pull" all data (accelX, gyrX etc.) from functions above to be called individually later in the loop
  //ultimately gives the user easy power over what data is used vs what isn't
  read_LSM_Accel_Data();

  
  prevTime = micros();
  read_LSM_Gyro_Data();
  currTime = micros();
  dTime = (currTime - prevTime)*0.000001; // calculating delta time
  
  read_LSM_Temp();
  read_Mag_Data();
  read_Mag_Temp();

  // normalize mag data
  mag_norm = sqrt(pow(magX,2)+pow(magY,2)+pow(magZ,2));
  magX = magX / mag_norm;
  magY = magY / mag_norm;
  magZ = magZ / mag_norm;

  comp_fil_accel_y = atan2(accelX , sqrt( pow(accelY,2) + pow(accelZ,2) )); // pitch
  comp_fil_accel_x = atan2(accelY , sqrt( pow(accelX,2) + pow(accelZ,2) )); // roll
  //comp_fil_accel_z = atan2(accelZ , sqrt( pow(accelX,2) + pow(accelZ,2) ));


  x_m = magX * cos(comp_fil_accel_y) + magY * sin(comp_fil_accel_x) * sin(comp_fil_accel_y)
              + magZ * cos(comp_fil_accel_x) * sin(comp_fil_accel_y); 
  y_m = -magY * cos(comp_fil_accel_x) + magZ * sin(comp_fil_accel_x);
  comp_fil_mag_z = atan2(y_m,x_m);      // yaw

  comp_fil_angle_x = (1 - alpha)*(comp_fil_angle_x + gyrX*dTime) + alpha*comp_fil_accel_x; // rotate around x axis
  comp_fil_angle_y = (1 - alpha)*(comp_fil_angle_y + gyrY*dTime) + alpha*comp_fil_accel_y; // rotate around y axis
  comp_fil_angle_z = (1 - alpha)*(comp_fil_angle_z + gyrZ*dTime) + 5*alpha*comp_fil_mag_z; // rotate around z axis

  kalman_fil_angle_z = kalmanFilter(comp_fil_angle_z);
  

  //Serial.print("GyroX:");
  //Serial.print(gyrX);
  //Serial.print(",");
  //Serial.print("accelY:");
  //Serial.print(accelY);
  //Serial.print(",");

  /*
  Serial.print("magx:");
  Serial.print(magX);
  Serial.print(",");
  Serial.print("magy:");
  Serial.print(magY);
  Serial.print(",");
  Serial.print("magz:");
  Serial.println(magZ);
  */

  /*
  Serial.print("comp_fil_angle_x:");
  Serial.println(comp_fil_angle_x);
  */

  /*
  Serial.print("comp_fil_angle_y:");
  Serial.println(comp_fil_angle_y);
  */
  
  //Serial.print("gyroZ:");
  //Serial.print(gyrZ);
  //Serial.print(",");
  Serial.print("comp_fil_angle_z:");
  Serial.print(comp_fil_angle_z);
  Serial.print(",");
  Serial.print("kalman_fil_angle_z:");
  Serial.println(kalman_fil_angle_z);
  
  /*
  Serial.print("comp_fil_angle_x:");
  Serial.print(comp_fil_angle_x);
  Serial.print(",");
  Serial.print("comp_fil_angle_y:");
  Serial.print(comp_fil_angle_y);
  Serial.print(",");
  Serial.print("comp_fil_angle_z:");
  Serial.println(comp_fil_angle_z);
  */

}
