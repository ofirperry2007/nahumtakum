#include "Arduino.h"
#include "nahumtakum.h"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

unsigned long timer = 0;


nahumtakum::nahumtakum(int motin1, int motin2, int motEna){
  _motin1 = motin1;
  _motin2 = motin2;
  _motEna = motEna;
  pinMode(_motin1, OUTPUT);
  pinMode(_motin2, OUTPUT);
  pinMode(_motEna, OUTPUT);

}

void nahumtakum::begin() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!n");
}

void nahumtakum::run() {
  mpu.update();
  if ((millis() - timer) > 10) {  // print data every 10ms
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("tZ : ");
    Serial.println(mpu.getAngleZ());
    timer = millis();
  }
}
int nahumtakum::getpitch(){
  return int(mpu.getAngleY());
}
double nahumtakum::PIDcalc(double sp, int pv){
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
  // Serial.print("current time = ");Serial.println(currentTime); //for serial plotter
  //Serial.println("\t"); //for serial plotter
  error = sp - pv;              // determine error
  cumError += (error * elapsedTime);            // compute integral
  rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
  //Serial.print("rateError = "); Serial.println(rateError);
  //Serial.print("elapsed time = "); Serial.println(elapsedTime);
  //delay(1000);

  if(rateError < 0.3 || rateError > -0.3){cumError = 0;}// reset the Integral commulator
  //Serial.print("I = "); Serial.println(cumError);

  double out = kp*error + ki*cumError + kd*rateError; //PID output               

  lastError = error;                                 //remember current error
  previousTime = currentTime;                        //remember current time
  if(out > 254){out = 254;}    //limit the function for smoother operation
  if(out < -254){out = -254;}
  Serial.print("out value = "); Serial.println(out);
  return out;    //the function returns the PID output value 
}

void nahumtakum::tumble(){
  int inp = PIDcalc(0, getpitch()); //0 is the setpoint
  if(inp > 0){
    analogWrite(_motin1, HIGH);
    analogWrite(_motin2, LOW);
  } else {
    analogWrite(_motin1, LOW);
    analogWrite(_motin2, HIGH);
  }
  analogWrite(_motEna, abs(inp));
}