#ifndef TRAPEZOID
#define TRAPEZOID

	#include "AUORML/Utility/Header.h"
	#include "AUORML/MathLib/MathLib.h"
	#include "AUORML/Kinematics/Kinematics.h"

	bool VelocityTrapezoid( double InitialVel[AXIS] , double FianlVel[AXIS] , double (&iVelCmd)[AXIS] ) ;

#endif