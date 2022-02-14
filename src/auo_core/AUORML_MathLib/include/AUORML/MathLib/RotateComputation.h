#ifndef __RotateComputation_h__
#define __RotateComputation_h__

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <Eigen/Dense>
using namespace Eigen;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get quaternion w.r.t specific euler angle.
//!
//! \param yaw
//! Rotation degree around yaw axis.
//!
//! \param pitch
//! Rotation degree around pitch axis.
//!
//! \param roll
//! Rotation degree around roll axis.
//!
//! \return 
//! Vector of quaternion. q0~q2 is sub-element of rotation axis. q3 is rotation degree component.
//! 
//! ----------------------------------------------------------

Vector4d Euler2Quaternion(double yaw, double pitch, double roll);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get euler angle w.r.t specific quaternion. 
//!
//! \param q0
//! Sub-element of rotation axis.
//!
//! \param q1
//! Sub-element of rotation axis.
//!
//! \param q2
//! Sub-element of rotation axis.
//!
//! \param q3
//! Rotation degree component.
//!
//! \return 
//! Vector of euler angle. The format is in Yaw-Pitch-Roll
//! 
//! ----------------------------------------------------------

Vector3d QuantenionToEuler(double q0, double q1, double q2, double q3);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Mapping rotation degree component to degree.
//!
//! \param q3
//! Rotation degree component.
//!
//! \return 
//! Degree of rotation.
//! 
//! ----------------------------------------------------------

double QuantenionToDegree(double q3);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Decomposite rotaiton matrix to yaw-pitch-roll euler angle.
//!
//! \param Rotation
//! Rotation matrix.
//!
//! \return 
//! Vector of yaw-pitch-roll euler angle.
//! 
//! ----------------------------------------------------------

Vector3d RotationMatrixToYPR(Matrix3d Rotation);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Composite yaw-pitch-roll euler angle to rotaiton matrix.
//!
//! \param yaw
//! Rotation degree around yaw axis.
//!
//! \param pitch
//! Rotation degree around pitch axis.
//!
//! \param roll
//! Rotation degree around roll axis.
//!
//! \return 
//! Rotation matrix w.r.t yaw-pitch-roll euler angle.
//! 
//! ----------------------------------------------------------

Matrix3d YPRToRotationMatrix(double yaw, double pitch, double roll);

#endif

