#include "AUORML/MathLib/RotateComputation.h"

/*
*  All the Euler angle element is decomposition in the sequence of Yaw-Pitch-Roll
*/

Vector4d Euler2Quaternion(double yaw , double pitch , double roll ){

    double q0, q1, q2, q3;
    q0 = -sin(yaw / 2) * sin(pitch / 2) * cos(roll / 2) + sin(roll / 2) * cos(yaw / 2) * cos(pitch / 2);
    q1 = sin(yaw / 2) * sin(roll / 2) * cos(pitch / 2) + sin(pitch / 2) * cos(yaw / 2) * cos(roll / 2);
    q2 = sin(yaw / 2) * cos(roll / 2) * cos(pitch / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q3 = sin(yaw / 2) * sin(roll / 2) * sin(pitch / 2) + cos(yaw / 2) * cos(roll / 2) * cos(pitch / 2);

    Vector4d quanternion(q0, q1, q2, q3);
    
    return quanternion;
}

Vector3d QuantenionToEuler(double q0 , double q1 , double q2 , double q3){

    Matrix3d RotationMatrix ;
    
    RotationMatrix <<   1 - 2 * (pow(q1, 2)) - 2 * (pow(q2, 2)),    2 * q0 * q1 - 2 * q2 * q3,                 2 * q0 * q2 + 2 * q1 * q3,
                        2 * q0 * q1 + 2 * q2 * q3,                  1 - 2 * (pow(q0, 2)) - 2 * (pow(q2, 2)),   2 * q1 * q2 - 2 * q0 * q3,
                        2 * q0 * q2 - 2 * q1 * q3,                  2 * q1 * q2 + 2 * q0 * q3,                 1 - 2 * (pow(q0, 2)) - 2 * (pow(q1, 2)) ;

    Vector3d YPR = RotationMatrixToYPR(RotationMatrix);

    return YPR;
}

double QuantenionToDegree(double q3){

    double theta = acos(q3) * 2; // (rad)

    return theta;
}

Vector3d RotationMatrixToYPR(Matrix3d Rotation){

    double yaw, pitch, roll;
    
    
    if( abs(Rotation(2,0)) != 1 ){
        yaw = atan2(Rotation(1, 0), Rotation(0, 0));
        pitch = atan2(-Rotation(2, 0), sqrt( pow(Rotation(2, 1),2) + pow(Rotation(2, 2),2) ) );
        roll = atan2(Rotation(2, 1), Rotation(2, 2));
    }
        
    else if( Rotation(2,0) == 1 ){
        yaw = atan2(-Rotation(1, 2), -Rotation(0, 2));
        pitch = -(M_PI / 2);
        roll = 0;
    }

    else if( Rotation(2,0) == -1 ){
        yaw = atan2(Rotation(1, 2), Rotation(0, 2));
        pitch = (M_PI / 2);
        roll = 0;
    }

    Vector3d YPR(yaw, pitch, roll);

    return YPR;
}

Matrix3d YPRToRotationMatrix(double yaw, double pitch, double roll){
    
    Matrix3d Rotation;
    
    Rotation << cos(yaw) * cos(pitch),     -sin(yaw) * cos(roll) + sin(roll) * sin(pitch) * cos(yaw),    sin(yaw) * sin(roll) + sin(pitch) * cos(yaw) * cos(roll),
                sin(yaw) * cos(pitch),      sin(yaw) * sin(roll) * sin(pitch) + cos(yaw) * cos(roll),    sin(yaw) * sin(pitch) * cos(roll) - sin(roll) * cos(yaw),
                -sin(pitch),                sin(roll) * cos(pitch),                                      cos(roll) * cos(pitch) ;

    return Rotation;
}