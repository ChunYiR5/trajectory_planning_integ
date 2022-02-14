#ifndef __Kinematics_Class_H__
#define __Kinematics_Class_H__

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef AXIS
#define AXIS	6 
#endif

#ifndef PRECISION
#define PRECISION   10
#endif
	
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <math.h>
#include <assert.h>
#include <random>
#include <vector>

#include "AUORML/MathLib/RotateComputation.h"

using namespace Eigen;


//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! BaseClass for various kinds of kinematics class
//!
//! ----------------------------------------------------------
class Kinematics{

public:

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Kinematics class constructor.
//!
//! ----------------------------------------------------------

    Kinematics(){};

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Kinematics class destructor.
//!
//! ----------------------------------------------------------

    virtual ~Kinematics() = 0; // Virtual destructor is for polymorphy usuage and need definition in order to fix link error problem. 

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Fix the float point from calculation error.
//!
//! \brief 
//! Fix the float point from calculation error due to computation like arcsin, arctan, SVD .... etc. The
//! precision to fixed is defined in the begining of this file. Currently is at 10th behind decimal.
//! 
//! \param value
//! Value need to be fixed.
//!
//! \return
//! Value after fixed.
//!
//! ----------------------------------------------------------

    double fixDecimal(double value);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Fix the rotation deg.
//!
//! \details 
//! Fix the rotation deg from 0 ~ 2pi to -pi ~ pi for easier implment.
//!
//! \param rad
//! Rotation quantity need to be fixed.
//!
//! \return
//! Value after fixed.
//!
//! ----------------------------------------------------------

    double InPNpi(double rad);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compute rotation matrix around X axis.
//!
//! \param rad
//! Rotation degree.
//!
//! \return
//! Homogenous Matrix w.r.t rotation around X axis.
//!
//! ----------------------------------------------------------

    Eigen::Matrix4d GetRX(double rad);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compute rotation matrix around Y axis.
//!
//! \param rad
//! Rotation degree.
//!
//! \return
//! Homogenous Matrix w.r.t rotation around Y axis.
//!
//! ----------------------------------------------------------

    Eigen::Matrix4d GetRY(double rad);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compute rotation matrix around Z axis.
//!
//! \param rad
//! Rotation degree.
//!
//! \return
//! Homogenous Matrix w.r.t rotation around Z axis.
//!
//! ----------------------------------------------------------

    Eigen::Matrix4d GetRZ(double rad);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compute translation vector between two coordinate.
//!
//! \param X 
//! Translation quantity in X direction w.r.t reference frame.
//!
//! \param Y
//! Translation quantity in Y direction w.r.t reference frame.
//!
//! \param Z
//! Translation quantity in Z direction w.r.t reference frame.
//!
//! \return
//! Homogenous Matrix w.r.t translation along XYZ direction.
//! 
//! ----------------------------------------------------------

    Eigen::Matrix4d GetTrans(double X, double Y, double Z);
    
//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Base class for computing forward kinematics w.r.t specific robot configuration.
//!
//! \details 
//! Compute the forward kinematics of whole body w.r.t input robot configuration.
//! This function will output homogenous matrix of all joints coordinate w.r.t robot base frame.
//! This function is pure virtual function adn needs to be implement by child class.
//!
//! \param jointAngles
//! Specific robot configuration.
//!
//! \return 
//! Homogenous matrix with 4X24 dimension which contains coordinate transform of all joints.
//!
//! ----------------------------------------------------------

    virtual Eigen::Matrix<double, 4, AXIS*4> GetForwardKinematics(Eigen::Matrix<double, 1, AXIS> jointAngles) = 0;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Base class for computing EndEffector Pose w.r.t specific robot configuration.
//!
//! \details 
//! Compute the forward kinematics of end effector w.r.t input robot configuration.
//! This function will output pose array of end effector w.r.t robot base frame.
//!
//! \param jointAngles
//! Specific robot configuration.
//!
//! \return 
//! Pose Array of robot end effector. The output format is XYZYPR.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, 6> GetEndEffectorPose(Eigen::Matrix<double, 1, AXIS> jointAngles);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Virtual function to get the robot geometry jacobian based on specific joint position.
//!
//! \param jointAngles
//! The motor position of specific robot configuration.
//!
//! \return 
//! A 6X6 AXIS matrix.
//!
//! ----------------------------------------------------------

    virtual Matrix<double, 6, AXIS> GetRobotJacobian(Matrix<double, 1, AXIS> jointAngles) const = 0;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Base class for computing EndEffector Velocity w.r.t specific robot configuration.
//!
//! \details 
//! Compute the forward diffential kinematics of end effector w.r.t input robot configuration.
//! This function will output velocity pose array of end effector w.r.t robot base frame.
//!
//! \param jointAngles
//! Specific robot configuration.
//!
//! \return 
//! Velocity Pose Array of robot end effector.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, 6> GetEndEffectorVelocity(Eigen::Matrix<double, 1, AXIS> jointAngles, Eigen::Matrix<double, 1, AXIS> jointVelocity);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Virtual function to get the robot geometry jacobian based on specific joint position and joint velocity.
//!
//! \param jointAngles
//! The motor position of specific robot configuration.
//!
//! \param jointVelocity
//! The motor velocity of specific robot configuration.
//!
//! \return 
//! A 6 X AXIS matrix.
//!
//! ----------------------------------------------------------

    virtual Matrix<double, 6, AXIS> GetRobotJacobianDot(Matrix<double, 1, AXIS> jointAngles, Eigen::Matrix<double, 1, AXIS> jointVelocity) const = 0;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Base class for computing EndEffector Acceleration w.r.t specific robot configuration.
//!
//! \details 
//! Compute the forward diffential kinematics of end effector w.r.t input robot configuration.
//! This function will output Acceleration pose array of end effector w.r.t robot base frame.
//!
//! \param jointAngles
//! Specific robot configuration.
//!
//! \return 
//! Acceleration Pose Array of robot end effector.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, 6> GetEndEffectorAcceleration(Eigen::Matrix<double, 1, AXIS> jointAngles, Eigen::Matrix<double, 1, AXIS> jointVelocity, Eigen::Matrix<double, 1, AXIS> jointAcceleration);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Base class for solving inverse kinematics problem w.r.t current robot configuration and specific target 6DOF pose.
//!
//! \details 
//! Solve the inverse kinematics problem w.r.t input target 6DOF pose.
//! In gerneral, 6 AXIS robot with 3 adajent joint intersect at one points or 3 adajent rotation axis pararrel
//! to each other will exist analytical solution.
//! This is a pure virtual function and needs to be implemented in child class.
//!
//! \param targetPose
//! Specific robot targetPose, where the format is XYZYPR.
//!
//! \param referenceJoint
//! Joint position for solving minimal distance solution.
//!
//! \param solution
//! Reference of solution matrix.
//!
//! \return 
//! If robot has non-singular solution between 8 different set, return true and modify the reference solution
//! by minimal distance joints solution.
//! 
//! Otherwise, return false and modify the reference solution by NAN vector.
//!
//! ----------------------------------------------------------

    virtual bool GetInverseKinematics(Eigen::Matrix<double, 1, 6> targetPose, Eigen::Matrix<double, 1, AXIS> referenceJoint, Eigen::Ref<Matrix<double, 1, AXIS>> solution) = 0;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Decomposition the XYZ position & RPY Euler angle from Homogenous Matrix.
//!
//! \param homogenousMatrix 
//! A 4X4 Matrix that represent rotation and translation relationship between 2 data set.
//! The transformation might be same points between 2 different coordinate or 2 different group data under
//! same coordinate.
//! 
//! \return 
//! A 1X6 vector that contain XYZRPY info of input homogenousMatrix. 
//! 
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, 6> DecompositionXYZRPY(Eigen::Matrix4d homogenousMatrix);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Decomposition the XYZ position & YPR Euler angle from Homogenous Matrix.
//!
//! \param homogenousMatrix 
//! A 4X4 Matrix that represent rotation and translation relationship between 2 data set.
//! The transformation might be same points between 2 different coordinate or 2 different group data under
//! same coordinate.
//! 
//! \return 
//! A 1X6 vector that contain XYZYPR info of input homogenousMatrix. 
//! 
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, 6> DecompositionXYZYPR(Eigen::Matrix4d homogenousMatrix);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Decomposition the XYZ position & Quaternion from Homogenous Matrix.
//!
//! \param homogenousMatrix 
//! A 4X4 Matrix that represent rotation and translation relationship between 2 data set.
//! The transformation might be same points between 2 different coordinate or 2 different group data under
//! same coordinate.
//! 
//! \return 
//! A 1X7 vector that contain XYZQuaternion info of input homogenousMatrix.
//!  
//! ----------------------------------------------------------    

    Eigen::Matrix<double, 1, 7> DecompositionXYZQuaternion(Eigen::Matrix4d homogenousMatrix);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compute the homegenous matrix between robot base and end-effector.
//!
//! \param targetPose 
//! Specific robot end-effector pose. The format is XYZYPR.
//!
//! \return 
//! A 4X4 homegenous matrix.
//!
//! ----------------------------------------------------------

    Eigen::Matrix4d GetEndEffectorTransformationMatrix(Eigen::Matrix<double, 1, 6> targetPose);

}; // class Kinematics

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief IDH model based kinematics class
//!
//! ----------------------------------------------------------
class IDHKinematics : public Kinematics{

public:

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Improved DH Kinematics Class constructor, wherw the constructor needs IDH Table as initialize parameter.
//!
//! \param IDHTable
//! IDHParam Table to construct the robot kinematics.
//! The IDH Table need to be following format: theta, d, a, alpha, beta. (UNIT: mm, rad)
//!
//!  - theta: rotation degree based on Zi-1 axis that move Xi-1 to Xi.
//!  - d: translation distance based on Zi-1 axis that move Xi-1 to Xi.
//!  - a: translation distance based on Xi axis that move Zi-1 to Zi.
//!  - alpha: rotation degree based on Xi axis that move Zi-1 to Zi.
//!  - beta: rotation degree based on Yi that fixed the mechanical error.
//!
//! \param robotType
//! Robot type for kinematics solver. Currently choosable option is "Industrial" and "UR". Default is "Industrial".
//!
//! ----------------------------------------------------------

    IDHKinematics(Eigen::Matrix<double, AXIS, 5> IDHTable, std::string robotType = "Industrial") : Kinematics(){
        MatrixXd deepCopy = IDHTable;
        this -> IDHTable = deepCopy;

        if( robotType == "Industrial" || robotType == "INDUSTRIAL" || robotType == "industrial" ){
            this -> robotType = "Industrial";
            printf("Kinematics solver initialize of specific robotType successfully.");
        }
        else if( robotType == "UR" || robotType == "ur" ){
            this -> robotType = "UR";
            printf("Kinematics solver initialize of specific robotType successfully.");
        }
        else{
            this -> robotType = "None";
            printf("Kinematics solver initialize of specific robotType failed. Please try again");
        }
    };

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! IDHKinematics class destructor
//!
//! ----------------------------------------------------------

    ~IDHKinematics(){};

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solving Forward kinematics based on IDH Model.
//!
//! \details
//! Compute the forward kinematics of whole body w.r.t input robot configuration.
//! This function will output homogenous matrix of all joints coordinate w.r.t robot base frame.
//!
//! \param jointAngles
//! Specific robot configuration.
//!
//! \return 
//! Homogenous matrix with 4X24 dimension which contains coordinate transform of all joints.

    Eigen::Matrix<double, 4, AXIS*4> GetForwardKinematics(Eigen::Matrix<double, 1, AXIS> jointAngles) override ;

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Compute the Homogenous matrix between adajent joint.
//! The homegenous matrix represents the transformation matrix between i-1 joint and i joint, 
//! where the function input is the i-1 joint's IDH paramters.
//!
//! \param theta 
//! Rotation degree based on Zi-1 axis that move Xi-1 to Xi.
//!
//! \param d
//! Translation distance based on Zi-1 axis that move Xi-1 to Xi.
//!
//! \param a
//! Translation distance based on Xi axis that move Zi-1 to Zi.
//!
//! \param alpha
//! Rotation degree based on Xi axis that move Zi-1 to Zi.
//! 
//! \param beta
//! Rotation degree based on Yi that fixed the mechanical error.
//!
//! \return
//! Homogenous matrix between adajent i-1 joint and i joint.
//!
//! ----------------------------------------------------------

    Eigen::Matrix4d GetIndividualTransMatrix(double theta, double d, double a, double alpha, double beta);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint1 w.r.t industrial type.
//!
//! \param wristX
//! X position of Wrist Center w.r.t base frame
//!
//! \param wristY
//! Y position of Wrist Center w.r.t base frame
//!
//! \return
//! The general industrial type of 6 AXIS robot contains two different solution at shoulder. As a result, if the robot configuration 
//! is not in shoulder singular, return the right shoulder and left shoulder solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Vector2d SolveIKJ1(double wristX, double wristY);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint2 and joint3 of industrial type.
//!
//! \param j1
//! Joint1 motor degree solved by
//!
//! \param wristX
//! X position of Wrist Center w.r.t base frame
//!
//! \param wristY
//! Y position of Wrist Center w.r.t base frame
//!
//! \param wristZ
//! Z position of Wrist Center w.r.t base frame
//!
//! \return
//! The general industrial type of 6 AXIS robot contains two different solution at elbow. As a result, if the robot configuration 
//! is not in elbow singular, return the up elbow and down elbow solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Vector4d SolveIKJ23(double j1, double wristX, double wristY, double wristZ);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint4, joint5 and joint6 of industrial type.
//!
//! \param j1
//! Joint1 motor degree of specific robot configuration.
//!
//! \param j2
//! Joint2 motor degree of specific robot configuration.
//!
//! \param j3
//! Joint3 motor degree of specific robot configuration.
//!
//! \param Ex
//! X position of End effector w.r.t base frame.
//!
//! \param Ey
//! Y position of End effector w.r.t base frame.
//!
//! \param Ez
//! Z position of End effector w.r.t base frame.
//!
//! \param nx
//! X sub-component of normal vector w.r.t targetPose.
//!
//! \param ny
//! Y sub-component of normal vector w.r.t targetPose.
//!
//! \param nz
//! Z sub-component of normal vector w.r.t targetPose.
//!
//! \param ox
//! X sub-component of orientation vector w.r.t targetPose.
//!
//! \param oy
//! Y sub-component of orientation vector w.r.t targetPose.
//!
//! \param oz
//! Z sub-component of orientation vector w.r.t targetPose.
//!
//! \param ax
//! X sub-component of approach vector w.r.t targetPose.
//!
//! \param ay
//! Y sub-component of approach vector w.r.t targetPose.
//!
//! \param az
//! Z sub-component of approach vector w.r.t targetPose.
//!
//! \return
//! The general industrial type of 6 AXIS robot contains two different solution at wrist. As a result, if the robot configuration 
//! is not in wrist singular, return the up wrist and down wrist solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Matrix<double, 6, 1> SolveIKJ456(double j1, double j2, double j3, double Ex, double Ey, double Ez, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Inverse kinematics solver implmentation w.r.t Industrial-type robot.
//!
//! \details 
//! Solve the inverse kinematics problem according to target 6DOF pose.
//! In gerneral, 6 AXIS robot with 3 adajent joint intersect at one points or 3 adajent rotation axis parallel
//! to each other will exist analytical solution.
//!
//! Industrial-type robot is belong to 3 adajent joint intersect at one points. In other words, if target pose doesn't located in some
//! singularities, the solver will return eight different set of joint positions as solution. Otherwise, ik solver
//! will return correct solution depends on singularties num. 
//!
//! \param targetPose
//! Specific robot targetPose, where the format is 1X6 vector of XYZYPR.
//!
//! \return 
//! Return whole solution set of 8 different kinds joint position if possible.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 8, 6> SolveIndustrialIK(Eigen::Matrix<double, 1, 6> targetPose);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint1 w.r.t UR type.
//!
//! \param px
//! X position of end-effector w.r.t targetPose.
//!
//! \param py
//! Y position of end-effector w.r.t targetPose.
//!
//! \param ax
//! X sub-component of approach vector w.r.t targetPose.
//!
//! \param ay
//! Y sub-component of approach vector w.r.t targetPose.
//!
//! \return
//! The general type of 6 AXIS robot contains two different solution at shoulder. As a result, if the robot configuration 
//! is not in shoulder singular, return the right shoulder and left shoulder solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Vector2d SolveURIKJ1(double px, double py, double ax, double ay);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint1 w.r.t UR type.
//!
//! \param j1
//! Joint1 position solved by SolveURIKJ1().
//!
//! \param ax
//! X sub-component of approach vector w.r.t targetPose.
//!
//! \param ay
//! Y sub-component of approach vector w.r.t targetPose.
//!
//! \return
//! The general type of 6 AXIS robot contains two different solution at wrist. As a result, if the robot configuration 
//! is not in wrist singular, return the up wrist and low wrist solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Vector2d SolveURIKJ5(double j1, double ax, double ay);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint1 w.r.t UR type.
//!
//! \param j1
//! Joint1 position solved by SolveURIKJ1().
//!
//! \param j5
//! Joint5 position solved by SolveURIKJ5().
//!
//! \param nx
//! X sub-component of normal vector w.r.t targetPose.
//!
//! \param ny
//! Y sub-component of normal vector w.r.t targetPose.
//!
//! \param ox
//! X sub-component of orientation vector w.r.t targetPose.
//!
//! \param oy
//! Y sub-component of orientation vector w.r.t targetPose.
//!
//! \return
//! Joint6 solution w.r.t to joint1 and joint5 which are solved by SolveURIKJ1() and SolveURIKJ5().
//! 
//! ----------------------------------------------------------

    double SolveURIKJ6(double j1, double j5, double nx, double ny, double ox, double oy);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint234 w.r.t UR type.
//!
//! \param j1
//! Joint1 position solved by SolveURIKJ1().
//!
//! \param j5
//! Joint5 position solved by SolveURIKJ5().
//!
//! \param j6
//! Joint6 position solved by SolveURIKJ6().
//!
//! \param px
//! X position of end-effector w.r.t targetPose.
//!
//! \param py
//! Y position of end-effector w.r.t targetPose.
//!
//! \param pz
//! Z position of end-effector w.r.t targetPose.
//!
//! \param nx
//! X sub-component of normal vector w.r.t targetPose.
//!
//! \param ny
//! Y sub-component of normal vector w.r.t targetPose.
//!
//! \param nz
//! Z sub-component of normal vector w.r.t targetPose.
//!
//! \param ox
//! X sub-component of orientation vector w.r.t targetPose.
//!
//! \param oy
//! Y sub-component of orientation vector w.r.t targetPose.
//!
//! \param oz
//! Z sub-component of orientation vector w.r.t targetPose.
//!
//! \param ax
//! X sub-component of approach vector w.r.t targetPose.
//!
//! \param ay
//! Y sub-component of approach vector w.r.t targetPose.
//!
//! \param az
//! Z sub-component of approach vector w.r.t targetPose.
//!
//! \return
//! The general type of 6 AXIS robot contains two different solution at elbow. As a result, if the robot configuration 
//! is not in elbow singular, return the up elbow and low elbow solution. 
//!
//! Otherwise, return nan value for singular checking.
//! 
//! ----------------------------------------------------------

    Eigen::Matrix<double, 6, 1> SolveURIKJ234(double j1, double j5, double j6, double px, double py, double pz, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint2 w.r.t UR type, which is called by SolveURIKJ234().
//!
//! \param j3
//! Joint3 position solved by SolveURIKJ234().
//!
//! \param a2
//! a2 value w.r.t IDHTable.
//!
//! \param a3
//! a3 value w.r.t IDHTable.
//!
//! \param x
//! X value solved by SolveURIKJ234().
//!
//! \param y
//! Y value solved by SolveURIKJ234().
//!
//! \return
//! Joint2 solution w.r.t to joint3 which is solved by SolveURIKJ234().
//! 
//! ----------------------------------------------------------

    double SolveURIKJ2(double j3, double a2, double a3, double x, double y);

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! Solve the IK problem of joint4 w.r.t UR type, which is called by SolveURIKJ234().
//!
//! \param j1
//! Joint1 position solved by SolveURIKJ1().
//!
//! \param j2
//! Joint2 position solved by SolveURIKJ234().
//!
//! \param j3
//! Joint3 position solved by SolveURIKJ234().
//!
//! \param j5
//! Joint5 position solved by SolveURIKJ5().
//!
//! \param j6
//! Joint6 position solved by SolveURIKJ6().
//!
//! \param nx
//! X sub-component of normal vector w.r.t targetPose.
//!
//! \param ny
//! Y sub-component of normal vector w.r.t targetPose.
//!
//! \param nz
//! Z sub-component of normal vector w.r.t targetPose.
//!
//! \param ox
//! X sub-component of orientation vector w.r.t targetPose.
//!
//! \param oy
//! Y sub-component of orientation vector w.r.t targetPose.
//!
//! \param oz
//! Z sub-component of orientation vector w.r.t targetPose.
//!
//! \param ax
//! X sub-component of approach vector w.r.t targetPose.
//!
//! \param ay
//! Y sub-component of approach vector w.r.t targetPose.
//!
//! \param az
//! Z sub-component of approach vector w.r.t targetPose.
//!
//! \return
//! Joint2 solution w.r.t to joint3 which is solved by SolveURIKJ234().
//! 
//! ----------------------------------------------------------

    double SolveURIKJ4(double j1, double j2, double j3, double j5, double j6, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Inverse kinematics solver implmentation w.r.t UR-type robot.
//!
//! \details 
//! Solve the inverse kinematics problem according to target 6DOF pose.
//! In gerneral, 6 AXIS robot with 3 adajent joint intersect at one points or 3 adajent rotation axis parallel
//! to each other will exist analytical solution.
//!
//! UR-type robot is belong to 3 adajent parallel joint. In other words, if target pose doesn't located in some
//! singularities, the solver will return eight different set of joint positions as solution. Otherwise, ik solver
//! will return correct solution depends on singularties num. 
//!
//! \param targetPose
//! Specific robot targetPose, where the format is 6X1 vector of XYZYPR.
//!
//! \return 
//! Return whole solution set of 8 different kinds joint position if possible.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 8, 6> SolveURIK(Eigen::Matrix<double, 1, 6> targetPose);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get Inverse kinematics computation result.
//!
//! \details 
//! Solve the inverse kinematics problem w.r.t target 6DOF pose.
//! User need to choose if currently solved industrial-type or UR-type robot.
//!
//! \param targetPose
//! Specific robot targetPose, where the format is XYZYPR.
//!
//! \param referenceJoint
//! Joint position for solving minimal distance solution.
//!
//! \param solution
//! Reference of solution matrix.
//!
//! \return 
//! If robot has non-singular solution between 8 different set, return true and modify the reference solution
//! by minimal distance joints solution.
//! 
//! Otherwise, return false and modify the reference solution by NAN vector.
//!
//! ----------------------------------------------------------

    bool GetInverseKinematics(Eigen::Matrix<double, 1, 6> targetPose, Eigen::Matrix<double, 1, AXIS> referenceJoint, Eigen::Ref<Matrix<double, 1, AXIS>> solution) override;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get Inverse kinematics computation result with position constraint.
//!
//! \details 
//! Solve the inverse kinematics problem w.r.t target 6DOF pose.
//! User need to choose if currently solved industrial-type or UR-type robot.
//!
//! \param targetPose
//! Specific robot targetPose, where the format is XYZYPR.
//!
//! \param referenceJoint
//! Joint position for solving minimal distance solution.
//!
//! \param posMaxBound
//! Joint position max bound.
//!
//! \param posMinBound
//! Joint position min bound.
//!
//! \param solution
//! Reference of solution matrix.
//!
//! \return 
//! If robot has non-singular solution between 8 different set, return true and modify the reference solution
//! by minimal distance joints solution.
//! 
//! Otherwise, return false and modify the reference solution by NAN vector.
//!
//! ----------------------------------------------------------

    bool GetInverseKinematicsWithConstraint(Eigen::Matrix<double, 1, 6> targetPose, Eigen::Matrix<double, 1, AXIS> referenceJoint, Eigen::Matrix<double, 1, AXIS> posMaxBound, Eigen::Matrix<double, 1, AXIS> posMinBound, Eigen::Ref<Matrix<double, 1, AXIS>> solution);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Find the minimal distance joint position between 8 different set.
//!
//! \param JointSet
//! 8 different combination of joint position w.r.t same targetPose.
//!
//! \param BenchmarkJoint
//! The compare joint position set. Usually it is current robot configuration.
//!
//! \return 
//! If robot has non-singular solution between 8 different set, return minimal distance joints solution.
//! 
//! Otherwise, return NAN vector.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 1, AXIS> FindMinDistanceJoint(Matrix<double, 8, AXIS> JointSet, Matrix<double, AXIS, 1> BenchmarkJoint);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Unit test function for IDH Kinematics solver
//!
//! \details
//! Each turn return generate one set of joint positions and compare the F.K. and I.K. result.
//! 
//! \param episode
//! Examine episode. 
//!
//! \param ifShowDebugInfo
//! True to show detailed info during testing.
//!
//! \return
//! Return true if solve inverse Kinematics successfully.
//!
//! ----------------------------------------------------------
    
    void TestKinematics(int episode, bool ifShowDebugInfo);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get the robot geometry jacobian based on specific joint position.
//!
//! \param jointAngles
//! The motor position of specific robot configuration.
//!
//! \return 
//! A 6X6 AXIS matrix.
//!
//! ----------------------------------------------------------

    Matrix<double, 6, AXIS> GetRobotJacobian(Matrix<double, 1, AXIS> jointAngles) const override ;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get the robot geometry jacobian differential based on specific joint position and velocity.
//!
//! \param jointAngles
//! The motor position of specific robot configuration.
//!
//! \param jointVelocity
//! The motor velocity of specific robot configuration.
//!
//! \return 
//! A 6 X AXIS matrix.
//!
//! ----------------------------------------------------------

    Matrix<double, 6, AXIS> GetRobotJacobianDot(Matrix<double, 1, AXIS> jointAngles, Matrix<double, 1, AXIS> jointVelocity) const override ;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Get the current IDH Table.
//!
//! \return 
//! IDH Table.
//!
//! ----------------------------------------------------------

    Matrix<double, AXIS, 5> GetIDHTable() const;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Change the current IDH Table.
//!
//! \param  IDHTable
//! New IDH Table.
//!
//! ----------------------------------------------------------

    void ChangeIDHTable(Matrix<double, AXIS, 5> IDHTable);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Change the current RobotType.
//!
//! \param  robotType 
//! New robotType.
//!
//! \return
//! Return true if change successfully.
//!
//! ----------------------------------------------------------

    bool ChangeRobotType(std::string robotType);

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Compesnsate the inverse the kinematics
//!
//! \param  PoseError
//! The residual beteween the oringal dh pose and the update dh pose.
//!
//! \param  jacobian
//! The  state of jacobian.
//!
//! \return
//! Return 6*1 compensated motor degree.
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, AXIS, 1> InverseKinematicsCompensation(Eigen::Matrix<double, 3, 1> PoseError, Eigen::Matrix<double, 3, AXIS> jacobian);

private:

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! IDH Table for keeping robot kinematics parameters.
//!
//! ----------------------------------------------------------

    Matrix<double, AXIS, 5> IDHTable;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Robot type for kinematics solver. Currently choosable option is "Industrial" and "UR".
//!
//! ----------------------------------------------------------

    std::string robotType;

}; // class IDHKinematics

//  ---------------------- Doxygen info ----------------------
//! 
//!
//! \brief MDH model based kinematics class
//!
//! ----------------------------------------------------------
class MDHKinematics : public Kinematics{

public:

//  ---------------------- Doxygen info ---------------------- 
//!
//! \brief 
//! MDHKinematics Class constructor.
//!
//! \param MDHTable
//! MDHParam Table to construct the robot kinematics.
//! The MDH Table need to be following format: theta, d, a, alpha, beta.
//!
//!  - theta: rotation degree based on Zi axis that move Xi-1 to Xi.
//!  - d: translation distance based on Zi axis that move Xi-1 to Xi.
//!  - a: translation distance based on Xi-1 axis that move Zi-1 to Zi.
//!  - alpha: rotation degree based on Xi-1 axis that move Zi-1 to Zi.
//!  - beta: rotation degree based on Yi that fixed the mechanical error.
//!
//! \param robotType
//! Robot type for kinematics solver. Currently choosable option is "Industrial" and "UR". Default is "Industrial".
//!
//! ----------------------------------------------------------

    MDHKinematics(Eigen::Matrix<double, AXIS, 5> MDHTable, std::string robotType = "Industrial"){
        MatrixXd deepCopy = MDHTable;
        this -> MDHTable = deepCopy;

        if( robotType == "Industrial" || robotType == "INDUSTRIAL" || robotType == "industrial" ){
            this -> robotType = "Industrial";
            printf("Kinematics solver initialize of specific robotType successfully.");
        }
        else if( robotType == "UR" || robotType == "ur" ){
            this -> robotType = "UR";
            printf("Kinematics solver initialize of specific robotType successfully.");
        }
        else{
            this -> robotType = "None";
            printf("Kinematics solver initialize of specific robotType failed. Please try again");
        }
    };

//  ---------------------- Doxygen info ----------------------
//! 
//! \brief 
//! MDHKinematics class destructor
//!
//! ----------------------------------------------------------

    ~MDHKinematics(){};

//  ---------------------- Doxygen info ----------------------
//! 
//!
//! \brief 
//! Function used to Compute Forward kinematics based on IDH Model.
//!
//! \param jointAngles
//! The motor degree of specific robot configuration. (UNIT: rad)
//!
//! \return
//! A 4 by AXIS*4 Eigen:: Homogenous Matrix that represent 6D.O.F pose infomation of each joint coordinate
//! based on robot base coordinate. 
//!
//! ----------------------------------------------------------

    Eigen::Matrix<double, 4, AXIS*4> GetForwardKinematics(Eigen::Matrix<double, 1, AXIS> jointAngles);

//  ---------------------- Doxygen info ----------------------
//! 
//!
//! \brief 
//! Function use to Compute the Homogenous matrix between adajent joint.
//!
//! \details
//! The homegenous matrix represent the transformation matrix between i-1 joint and i joint, 
//! where the function input is the i-1 joint's IDH paramters.
//!
//! \param theta 
//! 
//! \param d
//!
//! \param a
//!
//! \param alpha
//!
//! \param beta
//!
//! \return
//! 
//! ----------------------------------------------------------

    Eigen::Matrix4d GetIndividualTransMatrix(double Theta, double D, double A, double Alpha, double Beta);


private:

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! IDH Table for keeping robot kinematics parameters.
//!
//! ----------------------------------------------------------

    Matrix<double, AXIS, 5> MDHTable;

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Robot type for kinematics solver. Currently choosable option is "Industrial" and "UR".
//!
//! ----------------------------------------------------------

    std::string robotType;

}; // class MDHKinematics


#endif
