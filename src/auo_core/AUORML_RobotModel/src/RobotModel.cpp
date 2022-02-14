#include "AUORML/RobotModel/RobotModel.h"

RobotModel::RobotModel() :  DH_Parameter( (Eigen::Matrix<double, AXIS, 5>() << 
                            0,          179.5,   0,           M_PI / 2,   0,
                            M_PI / 2,   0,       635.5,              0,   0, 
                            0,          0,       0,           M_PI / 2,   0,
                            0,          432.0,   0,          -M_PI / 2,   0,
                            0,          0,       0,           M_PI / 2,   0,
                            0,          161.3,   0,                  0,   0).finished() ),

                            maxRotateRad{ 2* PI, 2* PI, 2* PI, 2* PI, 2* PI, 2* PI },

                            minRotateRad{ -2* PI, -2* PI, -2* PI, -2* PI, -2* PI, -2* PI },

                            ratedVelocity{ PI/2, PI/2, PI/2, PI/2, PI/2, PI/2 },

                            ratedTorque{ 1000, 1000, 1000, 1000, 1000, 1000 } // temp value
{

    
}