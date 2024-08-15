/*Example using servo library and inverse kinematics library. Tested with NEMA 23 stepper motor and MKS57D driver using CAN as communication protocol*/


#include <IKKJ.h>
#include <MKS57_CAN.h>//Download this library from GSW repo



IKKJ  arm(220.0,  220.0,  50.0);//Set arm lengths adn base heigth in mm

MKS57_CAN driver1(4,  5,  1000, 10, 10);//Create stepper driver

void setup() {
Serial.begin(115200);

driver1.begin();
driver1.enableMotor(0x1, 0x1);
driver1.setZero(0x1);
delay(2000);

}

void loop() {

for(int i = 150;  i < 200;  i++){

  arm.solveIk(i, 150.0, 100.0);

  // Print the calculated angles and coordinates
  Serial.print("Alpha: "); Serial.println(arm.getAlpha());
  Serial.print("Beta: "); Serial.println(arm.getBeta());
  Serial.print("Gamma: "); Serial.println(arm.getGamma());
  
  Serial.print("Degree  Alpha: "); Serial.println(arm.getAngleAlpha());
  Serial.print("Degree  Beta: "); Serial.println(arm.getAngleBeta());
  Serial.print("Degree  Gamma: "); Serial.println(arm.getAngleGamma());
  float angleHex  = arm.getAngleAlpha() * 45.51;
  driver1.sendPositionMode4Message(0x1, 300, 2, angleHex);

  Serial.print("X: "); Serial.println(arm.getCX());
  Serial.print("Y: "); Serial.println(arm.getCY());
  Serial.print("Z: "); Serial.println(arm.getCZ());
  
  delay(500);
}

for(int i = 200;  i > 150;  i--){

  arm.solveIk(i, 150.0, 100.0);

  // Print the calculated angles and coordinates
  Serial.print("Alpha: "); Serial.println(arm.getAlpha());
  Serial.print("Beta: "); Serial.println(arm.getBeta());
  Serial.print("Gamma: "); Serial.println(arm.getGamma());
  
  Serial.print("Degree  Alpha: "); Serial.println(arm.getAngleAlpha());
  Serial.print("Degree  Beta: "); Serial.println(arm.getAngleBeta());
  Serial.print("Degree  Gamma: "); Serial.println(arm.getAngleGamma());
  float angleHex  = arm.getAngleAlpha() * 45.51;//Relationship between max value and step number/degrees
  driver1.sendPositionMode4Message(0x1, 300, 2, angleHex);

  Serial.print("X: "); Serial.println(arm.getCX());
  Serial.print("Y: "); Serial.println(arm.getCY());
  Serial.print("Z: "); Serial.println(arm.getCZ());
  
  delay(500);
}

}
