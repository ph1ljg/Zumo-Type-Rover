/*
* CQuatToDcm.cpp
*
* Created: 25/02/2021 17:43:00
* Author: philg
*/

#include "Includes.h"
#include "CQuatToDcm.h"

// default constructor
CQuatToDcm::CQuatToDcm()
{
} //CQuatToDcm

// default destructor
CQuatToDcm::~CQuatToDcm()
{
} //~CQuatToDcm


//Covert a quaternion into a full three - dimensional rotation matrix.
//	 param Quat : A 4 element array representing the quaternion(q0, q1, q2, q3)
//	 return : A 3x3 element matrix representing the full 3D rotation matrix. This rotation matrix converts a point in the local reference
//		frame to a point in the global reference frame.
void CQuatToDcm::Convert(float Quat[], Matrix3f &DCM)
{
	float q0, q1, q2, q3;
	// Extract the values from Q
	q0 = Quat[0];
	q1 = Quat[1];
	q2 = Quat[2];
	q3 = Quat[3];

		 
	//First row of the rotation matrix
	DCM.a.x = 2 * (q0 * q0 + q1 * q1) - 1;
	DCM.a.y = 2 * (q1 * q2 - q0 * q3);
	DCM.a.z = 2 * (q1 * q3 + q0 * q2);

	// Second row of the rotation matrix
	DCM.b.x = 2 * (q1 * q2 + q0 * q3);
	DCM.b.y = 2 * (q0 * q0 + q2 * q2) - 1;
	DCM.b.z = 2 * (q2 * q3 - q0 * q1);

	// Third row of the rotation matrix
	DCM.c.x = 2 * (q1 * q3 - q0 * q2);
	DCM.c.y = 2 * (q2 * q3 + q0 * q1);
	DCM.c.z = 2 * (q0 * q0 + q3 * q3) - 1;
}


/*

//Create Rotation Matrix rm from Quaternion 
double rm[3][3];

rm[1][1] = quat.w() * quat.w() + quat.x() * quat.x() - quat.y() * quat.y() - quat.z() * quat.z();
rm[1][2] = 2 * quat.x() * quat.y() - 2 * quat.w() * quat.z();
rm[1][3] = 2 * quat.x() * quat.z() + 2 * quat.w() * quat.y();
rm[2][1] = 2 * quat.x() * quat.y() + 2 * quat.w() * quat.z();
rm[2][2] = quat.w() * quat.w() - quat.x() * quat.x() + quat.y() * quat.y() - quat.z() * quat.z();
rm[2][3] = 2 * quat.y() * quat.z() - 2 * quat.w() * quat.x();
rm[3][1] = 2 * quat.x() * quat.z() - 2 * quat.w() * quat.y();
rm[3][2] = 2 * quat.y() * quat.z() + 2 * quat.w() * quat.x();
rm[3][3] = quat.w() * quat.w() - quat.x() * quat.x() - quat.y() * quat.y() + quat.z() * quat.z();

// Display Rotation Matrix 
Serial.print(rm[1][1], 5); Serial.print("  \t");
Serial.print(rm[1][2], 5); Serial.print("  \t");
Serial.println(rm[1][3], 5);
Serial.print(rm[2][1], 5); Serial.print("  \t");
Serial.print(rm[2][2], 5); Serial.print("  \t");
Serial.println(rm[2][3], 5);
Serial.print(rm[3][1], 5); Serial.print("  \t");
Serial.print(rm[3][2], 5); Serial.print("  \t");
Serial.println(rm[3][3], 5);

// Create Roll Pitch Yaw Angles from Quaternions 
double yy = quat.y() * quat.y(); // 2 Uses below

double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + yy));
double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));


*/