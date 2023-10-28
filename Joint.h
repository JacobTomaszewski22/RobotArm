//The joint class header file
//Created by Jacob Tomaszewski - 28/10/23
#ifndef Joint_h
#define Joint_h
#define pi 3.1415926
#include "Arduino.h"
#include <Servo.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

class Joint
{
  private:
  //the private dh quantities describing the robot's structure. These wont change (no prismatic joints) so they can be private. Same with the pin number
    float alpha;
    float r;
    float d;
    int pin;
  
  public:
    float theta;
    Servo servo;
    //float T[4][4];
    BLA::Matrix<4,4> T;

    //=============Initialisation methods=============
    // PASS IN THETA AND ALPHA AS DEGREES!
    Joint::Joint(float passed_theta, float passed_alpha, float passed_r, float passed_d, byte passed_pin);
    //need to put in a destructor to clear the servo classes

  	//=============GET AND SET METHODS=============
  	void Joint::setTheta(float new_theta);
  	float Joint::getTheta();

  	//=============Populating the Transformation matrix with the DH parameters=============
  	void Joint::populateT();
  	float Joint::getT(byte row, byte col);
  	
  	BLA::Matrix<4,4>* Joint::getFullT();


};

#endif
