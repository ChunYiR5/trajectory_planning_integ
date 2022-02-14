#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include "AUORML/Utility/Header.h"
#include <Eigen/Dense>

using namespace Eigen;

class RobotModel{

public:

    RobotModel();

    Matrix<double, AXIS, 5> DH_Parameter;
    
    const double maxRotateRad[AXIS];
    const double minRotateRad[AXIS];
    const double ratedVelocity[AXIS];
    const double ratedTorque[AXIS];

};

#endif