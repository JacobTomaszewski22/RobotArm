//The CPP code for Joint.h
//Created by Jacob Tomaszewski - 28/10/2023

#include "Arduino.h"
#include "Joint.h"
using namespace BLA;

Joint::Joint(float passed_theta, float passed_alpha, float passed_r, float passed_d, byte passed_pin)
    {
      this->alpha = passed_alpha;
      this->r = passed_r;
      this->d = passed_d;
      this->theta = passed_theta;
      this->pin = passed_pin;

      servo.attach(this->pin);
      servo.write(this->theta);

      populateT();
    }

void Joint::setTheta(float new_theta)
  {
    theta = new_theta;
    servo.write(theta);
  }

float Joint::getTheta()
  {
    return theta;
  }

  //=============Populating the T matrix with the DH parameters=============
void Joint::populateT()
  {
    float radtheta = theta * pi/180;
    float radalpha = alpha * pi/180;
    //T = {cos(radtheta), -sin(radtheta) * cos(this->alpha), sin(radtheta)*sin(this->alpha), this->r * cos(radtheta), sin(radtheta), cos(radtheta)*cos(this->alpha), -cos(radtheta)*sin(this->alpha), r * sin(radtheta), 0, sin(this->alpha), cos(this->alpha), this->d, 0, 0, 0, 1};
    //T = {cos(theta), -sin(theta) * cos(alpha), sin(theta)*sin(alpha), r * cos(theta), sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r * sin(theta), 0, sin(alpha), cos(alpha), d, 0, 0, 0, 1};
    
    T = {cos(radtheta), -sin(radtheta) * cos(radalpha), sin(radtheta)*sin(radalpha), r * cos(radtheta), sin(radtheta), cos(radtheta)*cos(radalpha), -cos(radtheta)*sin(radalpha), r * sin(radtheta), 0, sin(radalpha), cos(radalpha), d, 0, 0, 0, 1};

  }

float Joint::getT(byte row, byte col)
{
  return this->T(row,col);
}

//Returns the address of the Transformation matrix in Joint. Need to store in a pointer, as such:
// BLA::Matrix<4,4> *TPtr = j1.getFullT();
//This stores the address of J1's T, in the pointer TPtr
BLA::Matrix<4,4>* Joint::getFullT()
{
	return &T;
}


