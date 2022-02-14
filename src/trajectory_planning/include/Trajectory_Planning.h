#ifndef __TRAJECTORY_PLANNING__
#define __TRAJECTORY_PLANNING__


#include "AUORML/Dynamics/Dynamics.h"
#include "AUORML/Kinematics/Kinematics.h"
#include "AUORML/Constraint/Constraint.h"
#include "AUORML/MathLib/RotateComputation.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>

using namespace std;
using namespace Eigen;

class TrajectoryPlanning
{

public:
 
    TrajectoryPlanning(std::shared_ptr<IDHKinematics> _kinematicsPtr, std::shared_ptr<Constraint> _constraintPtr);

    const std::shared_ptr<Constraint> GetConstraintPtr(void) const;
    const std::shared_ptr<IDHKinematics> GetKinematicsPtr(void) const;
    
    void Print_instruction(void);
    void Initialization(void);
    void Scurve(VectorXd joints_i , VectorXd joints_f, int mode);
    void Scurve_sp(VectorXd joints_i, VectorXd joints_f, int mode);
    void Circle_Plan3D(Vector3d Center, double Radius, Vector3d normal_vector, double TargetRad, int posture_mode);
    void PTPLine_Plan(Matrix<double, 6, 1> InitialPosition, Matrix<double, 6, 1> TargetPosition);
    void PTPPlan(Matrix<double, 6, 1> InitialJoints, Matrix<double, 6, 1> TargetJoints);
    void save_trajectory_data(void);

    double time_current = 0.0;
    bool Function_EndFlag = false;

private:
    
    const std::shared_ptr<Constraint>       constraintPtr;
    const std::shared_ptr<IDHKinematics>    kinematicsPtr;
    
    /*--- Vel Acc control ---*/
    double CartVelBound[TASKSPACE], CartAccBound[TASKSPACE], CartJerkBound[TASKSPACE];
    const double ParameterRatio = 0.5;
    const double alpha = 0.75;

    /*--- general ---*/
    Matrix<double, AXIS, 1> currentJoint = MatrixXd::Zero(AXIS, 1);
    Matrix<double, AXIS, 1> CarPosCmd, CarVelCmd, CarAccCmd;
    Matrix<double, AXIS, 1> JointPosCmd, JointVelCmd, JointAccCmd;

    /*--- Scurve variable */
    VectorXd t_result;
    Matrix<double, Dynamic, Dynamic> Scurve_pos, Scurve_vel, Scurve_acc;

    /*--- Scurve_sp variable ---*/
    double jerk, t_top, t0, t1, t2, t3, t4, t5, t6, t7;
    VectorXd p;
    VectorXd iPosCmd, iVelCmd, iAccCmd;
    list<double> TBase_list;

    /*--- Circle mode variable ---*/
    Matrix<double, 1, AXIS> current_pos;
    Vector3d v_normal;
    Matrix<double, 3, 2> ab_matrix;

    /*--- Circle mode posture variable ---*/
    // case 1 //
    double roll, pitch, yaw;
    Matrix3d R_initial;
    // case 2 //
    Matrix3d R;
    double depression_angle = 45;
    const double depression_angle_limit = 45;
    // case 3 //
    Vector3d YPR;
    
    /*--- for modify posture variable ---*/
    Vector3d iPosture;
    Vector3d pass_pos_diff , pass_vel_diff , pass_jointcmd_diff;
    Vector3d past_posture_pos , past_posture_vel , past_joint456_cmd;
    Vector3d sum_joint456_diff;

    /*--- Data record ---*/
    list<Matrix<double, AXIS, 1>> CarPosCmd_list, CarVelCmd_list, CarAccCmd_list;
    list<Matrix<double, AXIS, 1>> JointPosCmd_list, JointVelCmd_list, JointAccCmd_list;

    /*--- function ---*/
    Matrix3d RotationVector(Matrix3d RotationM, Vector3d Axis);
    Matrix3d GetTfiMatrix(double pit, double yaw, double roll);
    Matrix3d GetTfiMatrixDot(double pit, double yaw, double roll, double pitDot, double yawDot, double rollDot);

};

#endif