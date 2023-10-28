// The robotic arm class cpp file
// Created by Jacob Tomaszewski - 28/10/23
#include "Arduino.h"
#include "RobotArm.h"
using namespace BLA;

RobotArm::RobotArm(Joint* PassedJ1, Joint* PassedJ2, Joint* PassedJ3)
{
	//First we need to store the passed in Joint addresses
	J1ptr = PassedJ1;
	J2ptr = PassedJ2;
	J3ptr = PassedJ3;
	
	//Then we need to use these pointers to construct the overall transform matrix
	//via calling the populateT method
	populateT();
}

void RobotArm::populateT()
{
	//get the pointer to the adress of the J1 transform
	BLA::Matrix<4,4> *TPtr = J1ptr->getFullT();
	//Store it in a matrix via the pointers submatrix function. There may be a nicer way to do this...
	BLA::Matrix<4,4> J1T = TPtr->Submatrix<4, 4>(0, 0);
	
	//Now do this for each other joint
	//Joint 2
	TPtr = J2ptr->getFullT();
	BLA::Matrix<4,4> J2T = TPtr->Submatrix<4, 4>(0, 0);
	
	//Joint 3
	TPtr = J3ptr->getFullT();
	BLA::Matrix<4,4> J3T = TPtr->Submatrix<4, 4>(0, 0);
	
	//Now perform the transform multiplication
	//Link 3 * Link 2 * Link 1
	//The correct results of this has been verified via an external calculator
	this->overallTransform = (J3T * J2T) * J1T;
}

//Returns the address of the 4x4 overall transform matrix
BLA::Matrix<4,4>* RobotArm::getFullT()
{
	return &overallTransform;
}

//Returns the position subsection of the transform matrix for forward kinematic equation resolutions
BLA::Matrix<1,3>* RobotArm::getendposn()
{
	//get a submatrix of the transform matrix of length 1,3 starting at 0th row and 3rd collumn
	BLA::Matrix<1,3> posn = overallTransform.Submatrix<1,3>(0,3);
	return &posn;
}