#include "HardwareSerial.h"
#include "Arduino.h"
#include "nahumtakum.h"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

unsigned long timer = 0;
bool debug = false;
bool integralflag = false;


nahumtakum::nahumtakum(int in1, int in2, int Ena){
  _in1 = in1;
  _in2 = in2;
  _Ena = Ena;
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_Ena, OUTPUT);

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
  previousTime = millis();
  Serial.println("Done!n");
}

void nahumtakum::run() {
  mpu.update();
  if ((millis() - timer) > 10) {  // print data every 10ms
    if(debug){
      Serial.print("X: ");
      Serial.print(mpu.getAngleX());
      Serial.print("\tY : ");
      Serial.print(mpu.getAngleY());
      Serial.print("\t Z : ");
      Serial.println(mpu.getAngleZ());
    }
    timer = millis();
  }
}
int nahumtakum::getpitch(){
  return int(mpu.getAngleY());
}

double nahumtakum::PIDcalc(double sp, int pv) {
    currentTime = millis(); // Get current time
    elapsedTime = (currentTime - previousTime) / 1000.0; // Compute time elapsed in seconds

    if (elapsedTime == 0) {
        elapsedTime = 0.001; // Prevent division by zero by setting a small value
    }

    error = sp - pv; // Determine error
    //Serial.print("Current Error: "); Serial.println(error);
    //Serial.print("Last Error: "); Serial.println(lastError);
    if (!integralflag) { // Error is still accumulating, did not change direction yet
        cumError += (error * elapsedTime); // Compute integral
    } else { // Error has changed direction
        cumError = 0; // Reset cumulative error
        Serial.println("Resetting Integral value");
    }

    rateError = (error - lastError) / elapsedTime; // Compute derivative deltaError/deltaTime
    //Serial.print("rateError = "); Serial.println(rateError);

    // Check if the error has changed direction
    if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0 || error ==0)) {
        integralflag = true;
    } else {
        integralflag = false;
    }

    Serial.print("I = "); Serial.println(cumError);

    out = error * _kp + cumError * _ki + rateError * _kd; // PID output

    lastError = error; // Remember current error
    previousTime = currentTime; // Remember current time

    // Limit the output for smoother operation
    if (out > 254) { out = 254; }
    if (out < -254) { out = -254; }

    Serial.print("out value = "); Serial.println(out);
    return out; // The function returns the PID output value
}

void nahumtakum::tumble(int kp, int ki, int kd){
  _kp = kp;
  _ki = ki;
  _kd = kd;
  int pitch = getpitch();
  Serial.print("pitch = "); Serial.println(pitch);
  int inp = PIDcalc(0, pitch); //0 is the setpoint
  if(inp < 0){
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
  } else {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
  }
  inp = map(abs(inp), 0, 254, 120, 254);
  analogWrite(_Ena, inp);
}