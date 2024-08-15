/* 
 * This file is part of the distribution (https://github.com/Rvazquez559/Simple-Inverse-Kinematics).
 * Copyright (c) 2024 Roberto Vazquez.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */







#ifndef IKKJ_H
#define IKKJ_H

#include  <Arduino.h>
#include  <math.h>

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