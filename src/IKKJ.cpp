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



#include "IKKJ.h"

IKKJ::IKKJ(float  link1,  float link2,  float baseHeight){
  L1  = link1;
  L2  = link2;
  H   = baseHeight;
}

void  IKKJ::solveIk(float X, float Y, float Z){
  x = X;
  y = Y;
  z = Z;


  //Inverse K
  float L0    = calculateL0(z,  H,  x,  y);
  float cosA  = calculateCosA(L2, L0, L1);
  float cosB  = calculateCosB(L1, L2, L0);
  float tanD  = calculateTanD(z,  H,  x,  y);
  
  alpha = calculateAlpha(cosB,tanD);
  beta  = calculateBeta(cosA, tanD);
  gama  = calculateGamma(x, y);

  //Forward K
  LS  = calculateLs(L1, L2, alpha,  beta);
  cX  = calculateCx(LS, gama);
  cY  = calculateCy(LS, gama);
  cZ  = calculateCz(H,  L1, L2, alpha,  beta);

}

//Retreive angles radians

  float IKKJ::getAlpha(){return alpha;}
  float IKKJ::getBeta(){return  beta;}
  float IKKJ::getGamma(){return gama;}

  float IKKJ::getAngleAlpha(){return  alpha * 180/PI;}
  float IKKJ::getAngleBeta(){return beta  * 180/PI;}
  float IKKJ::getAngleGamma(){return  gama  * 180/PI;}

//Retreive coordinates

  float IKKJ::getCX(){return  cX;}
  float IKKJ::getCY(){return  cY;}
  float IKKJ::getCZ(){return  cZ;}

//Calculations functions

  float IKKJ::calculateL0(float z, float H, float x,  float y){
    return  sqrt(sq(z - H) + sq(x) + sq(y));
  }

  float IKKJ::calculateCosA(float L2, float L0, float L1){
    return (sq(L2) + sq(L0) - sq(L1)) / (2 * L2 * L0);
  }

  float IKKJ::calculateCosB(float L1, float L2, float L0){
    return (sq(L1) + sq(L0) - sq(L2)) / (2 * L1 * L0);
  }

  float IKKJ::calculateTanD(float z,  float H,  float x,  float y){
    return  (z-H) / sqrt(sq(x)  + sq(y));
  }

  float IKKJ::calculateAlpha(float  cosB, float tanD){
    return  acos(cosB)  + atan(tanD);
  }

  float IKKJ::calculateBeta(float cosA, float tanD){
    return  -(acos(cosA)  - atan(tanD));
  }

  float IKKJ::calculateGamma(float x,  float y){
    return  atan2(x,  y);
  }

  float IKKJ::calculateLs(float L1, float L2, float alpha,  float beta){
    return  (L1 * cos(alpha)) + (L2 * cos(beta));
  }

  float IKKJ::calculateCx(float LS, float gama){
    return  LS  * cos(gama);
  }

  float IKKJ::calculateCy(float LS, float gama){
    return  LS  * sin(gama);
  }

  float IKKJ::calculateCz(float H,  float L1, float L2, float alpha,  float beta){
    return  H + L1  * sin(alpha)  + L2  * sin(beta);
  }




























