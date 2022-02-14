#ifndef __CONSTRAINT__
#define __CONSTRAINT__

#include "AUORML/Utility/Header.h"
#include "AUORML/RobotModel/RobotModel.h"
#include "AUORML/MathLib/MathLib.h"

class Constraint{

public :

    Constraint(const std::shared_ptr<RobotModel> _robotModelPtr);

    // Setting soft limit of joint position bound. These function needs to check if input exceed hard limit.
    bool    SetSoftPosMaxBound(double newPosMaxBound[AXIS]);
    bool    SetSoftPosMinBound(double newPosMinBound[AXIS]);

    // Getting soft limit of joint position bound.
    void    GetSoftPosMaxBound(double (&softPosMaxBound)[AXIS]);
    void    GetSoftPosMinBound(double (&softPosMinBound)[AXIS]);

    // Getting hard limit of joint position bound.
    void    GetHardPosMaxBound(double (&hardPosMaxBound)[AXIS]);
    void    GetHardPosMinBound(double (&hardPosMinBound)[AXIS]);

    // Setting soft limit of joint velocity, acceleration, jerk bound. These function needs to check if input exceed hard limit.
    bool    SetSoftVelBound(double newVelBound[AXIS]);
    bool    SetSoftAccBound(double newAccBound[AXIS]);
    bool    SetSoftJerkBound(double newJerkBound[AXIS]);

    // Getting soft limit of joint velocity, acceleration, jerk bound.
    void    GetSoftVelBound(double  (&softVelBound)[AXIS]);
    void    GetSoftAccBound(double  (&softAccBound)[AXIS]);
    void    GetSoftJerkBound(double (&softJerkBound)[AXIS]);

    // Getting hard limit of joint velocity, acceleration, jerk bound.
    void    GetHardVelBound(double  (&hardVelBound)[AXIS]);
    void    GetHardAccBound(double  (&hardAccBound)[AXIS]);
    void    GetHardJerkBound(double (&hardJerkBound)[AXIS]);

    // Getting emergency limit of joint  acceleration, jerk bound.
    void    GetEmergencyAccBound(double  (&emergencyAccBound)[AXIS]);
    void    GetEmergencyJerkBound(double (&emergencyJerkBound)[AXIS]);

    // Getting hard limit of cartesian velocity, acceleration, jerk bound.
    void    GetHardCartVelBound(double  (&hardCartVelBound)[TASKSPACE]);
    void    GetHardCartAccBound(double  (&hardCartAccBound)[TASKSPACE]);
    void    GetHardCartJerkBound(double (&hardCartJerkBound)[TASKSPACE]);

    // Check if pos exceed current bound.
    bool    CheckJointPosLegality(double cuurentPos[AXIS]);
    bool    CheckJointVelLegality(double cuurentVel[AXIS]);

private :

    // Getting result for comparing data and benchmark limit. 
    bool    CheckIfExceedHardLimit(double check[AXIS], const double benchMark[AXIS]);

    // Soft limit of joint position bound. These limit can be adjusted.
    double       softPosMaxBound[AXIS];
    double       softPosMinBound[AXIS];

    // Absolute limit of joint position bound. These limit can't be adjusted.
    const double hardPosMaxBound[AXIS];
    const double hardPosMinBound[AXIS];

    // Soft limit of joint velocity, acceleration, jerk bound. These limit can be adjusted.
    double		 softVelBound[AXIS];
    double		 softAccBound[AXIS];
    double		 softJerkBound[AXIS];

    // Absolute limit of joint velocity, acceleration, jerk bound. These limit can't be adjusted.
    const double hardVelBound[AXIS];
    const double hardAccBound[AXIS];
    const double hardJerkBound[AXIS];

    // Absolute limit of joint acceleration, jerk bound during emergency stop. These limit can't be adjusted.
    const double emergencyAccBound[AXIS];
    const double emergencyJerkBound[AXIS];

    // Absolute limit of cartesian velocity, acceleration, jerk bound. These limit can't be adjusted.
    const double hardCartVelBound[TASKSPACE];
    const double hardCartAccBound[TASKSPACE];
    const double hardCartJerkBound[TASKSPACE];

    // smart pointer of robot model instance
    const std::shared_ptr<RobotModel> robotModelPtr;

};

#endif