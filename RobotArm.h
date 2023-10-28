//A robotic arm class header file. 
//Created by Jacob Tomaszewski - 28/10/23
#ifndef RobotArm_h
#define RobotArm_h
#define pi 3.1415926
#include "Arduino.h"
#include <Joint.h>

using namespace BLA;


//A robot arm class. For forward kinematic equations, update the associated joint angles, then
// call the populateT method to construct the full transform matrix, then call the getendposn method
// to resolve the forward kinematic equation for the end position
// 
class RobotArm
{
	private:

	public:
/*=================ATTRIBUTES=================*/
//Joint pointers. Doesn't need to use the  space for whole pointers
		Joint* J1ptr;							
		Joint* J2ptr;
		Joint* J3ptr;
		
		BLA::Matrix<4,4> overallTransform;		//Transform matrix

/*=================INITIALISATION METHOD=================*/
//The creator function
/*

Example: ***PROBABLY A BETTER WAY TO DO THIS*** 
in main:
	//MAKE THE JOINT OBJECTS
  Joint j1(20, 90, 0, 1.5, 7);
  Joint j2(0, 180, 10, 2.4, 6);
  Joint j3(0, 0, 11.4, 0, 5);
  
  //MAKE THE POINTERS TO THE JOINTS
  Joint *J1PTR = &j1;
  Joint *J2PTR = &j2;
  Joint *J3PTR = &j3;
  
  //PASS THE POINTERS TO THE ROBOTIC ARM UPON INITIALISATION
  RobotArm r1(J1PTR, J2PTR, J3PTR);
===========================================================*/
		RobotArm::RobotArm(Joint* PassedJ1, Joint* PassedJ2, Joint* PassedJ3);

/*=================OTHER METHODS=================*/

/*============getFullT============
The get function of the full stored transform matrix. 
***ALMOST DEFINITELY A BETTER WAY OF DOING THIS...***
Example:
in main:
	BLA::Matrix<4,4> *FullTptr = RobotArm.getFullT(); <- creates a pointer to the full T Matrix
	BLA::Matrix<4,4> FullT = FullTptr->Submatrix<4,4>(0,0); <- calls the submatrix function to store the full matrix
===================================*/
		BLA::Matrix<4,4>* RobotArm::getFullT();
		
/*============populateT============
FORWARD KINEMATICS. A method to create the full Transform from the transform matrices from the stored Joint classes
just needs to be called each time you need to forward kinematics calculation. Then to access the end point
call the getendposn method
===================================*/
		void RobotArm::populateT();
		
/* ============getendposn============
A method to get the 3D cartesian coordinates of the end effector position
in terms of the base coordinates
***ALMOST DEFINITELY A BETTER WAY OF DOING THIS...***
Example:
in main:
	BLA::Matrix<1,3> *Endposnptr = RobotArm.getendposn(); <- creates a pointer to the Matrix
	BLA::Matrix<1,3> endpoint = Endposnptr->Submatrix<1,3>(0,0); <- references the submatrix function to store the full matrix
===================================*/
		BLA::Matrix<1,3>* RobotArm::getendposn();
		
		

};

#endif
