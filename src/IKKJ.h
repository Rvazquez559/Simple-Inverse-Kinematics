#ifndef IKKJ_H
#define IKKJ_H

#include  <Arduino.h>
#include  <mat.h>

class IKKJ{

  public:

  IKKJ(float  link1,  float link2,  float baseHeight);

  void solveIk(float  X,  float Y,  float Z);

  //Inverse kinematic get joint angles
  float getAlpha();
  float getBeta();
  float getGamma();

  float getAngleAlpha();
  float getAngleBeta();
  float getAngleGamma();

  //Forward kinematic get angles
  float getCX();
  float getCY();
  float getCZ();

  private:

  //Link lengths  and base height
  float L1;
  float L2;
  float H;

  //Coordinates  and angles
  float x,  y,  z;
  float alpha, beta, gama;
  float LS, cX, cY, cZ;

  //Calculations functions
  float calculateL0(float z,  float H,  float x,  float y);
  float calculateCosA(float L2, float L0, float L1);
  float calculateCosB(float L1, float L2, float L0);
  float calculateTanD(float z,  float H,  float x,  float y);
  float calculateAlpha(float cosB,  float tanD);
  float calculateBeta(float cosA, float tanD);
  float calculateGamma(float  x,  float y);
  float calculateLs(float L1, float L2, float alpha,  float beta);
  float calculateCx(float LS, float gama);
  float calculateCy(float LS, float gama);
  float calculateCz(float H,  float L1, float L2, float alpha,  float beta);


};

#endif