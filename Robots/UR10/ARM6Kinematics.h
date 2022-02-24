#ifndef ARM_KINEMATICS_H_
#define ARM_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);
const double RAD_TO_DEG = 180.0/PI;


//UR5E VALUES!!!
// https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/parameters-for-calculations-of-kinematics-and-dynamics-45257/

///////////////////////////////////////////////////////
//UR5E VALUES!!!
///////////////////////////////////////////////////////
// const double shoulderOffsetZ = 0.1625; //ground to shoulder pitch height
// const double shoulderOffsetY= 0.1333; //shoulderpitch .13585 - elbow .1197
// const double lowerArmLength=0.425;
// const double upperArmLength=0.3922;
// const double wristLength = 0.0997;
// const double handLength = 0.0996;



///////////////////////////////////////////////////////
//UR10E VALUES!!!
///////////////////////////////////////////////////////
const double shoulderOffsetZ = 0.1807; //ground to shoulder pitch height
const double shoulderOffsetY= 0.17415; //shoulderpitch .13585 - elbow .1197
const double lowerArmLength=0.6127;
const double upperArmLength=0.57155;
const double wristLength = 0.11985;
const double handLength = 0.11655;


extern double toolOffsetX;
extern double toolOffsetZ;

double mod_angle(double q);
Transform ARM6_kinematics_forward_arm(const double *q);
Transform ARM6_kinematics_forward_wrist(const double *q);
std::vector<double> ARM6_kinematics_inverse_arm(Transform trArm, const double *qOrg, bool wristRoll1);

std::vector<double> ARM6_kinematics_inverse_arm2(Transform trArm, const double *qOrg, bool wristRoll1);

#endif
