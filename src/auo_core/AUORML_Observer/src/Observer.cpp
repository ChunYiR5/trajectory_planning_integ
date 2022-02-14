#include "AUORML/Observer/Observer.h"

double ResidualSignal[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} ;
double IntergrationItem[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} ;
double Ki[AXIS] = { 30.0, 30.0, 30.0, 30.0, 30.0, 30.0 } ;

void ExternalForceObserver( double Pos[AXIS], double Vel[AXIS],  double CtrlTorq[AXIS], double (&ExternalTorque)[AXIS] )
{
	// Caculate momentum
	double momentum[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Matrix<double, AXIS, AXIS> inertiaMatrix;
	GetInertiaMatrix(Pos, inertiaMatrix);

	for(int ijoint = 0; ijoint < AXIS ; ijoint++ ){
		for(int jjoint = 0; jjoint < AXIS ; jjoint++ ){
			momentum[ijoint] += inertiaMatrix(ijoint, jjoint) * Vel[jjoint];
		}
	}

	// Caculate Coriolis transpose * vel
	double coriolisTransTorq[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Matrix<double, AXIS, AXIS> coriolisMatrix;
	GetCoriolisMatrix(Pos, Vel, coriolisMatrix);
	coriolisMatrix = coriolisMatrix.transpose();

	for(int ijoint = 0; ijoint < AXIS ; ijoint++ ){
		for(int jjoint = 0; jjoint < AXIS ; jjoint++ ){
			coriolisTransTorq[ijoint] += coriolisMatrix(ijoint, jjoint) * Vel[jjoint];
		}
	}

	// GetGravity Torque
	double gravityTorque[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	GetGravityTorque(Pos, gravityTorque);

	// Caculate Friction Torque
	double frictionTorque[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	GetFrictionTorque(Vel, frictionTorque);

	// Compute Integration term
	for(int ijoint = 0; ijoint < AXIS ; ijoint++ ){
		// Note: Can't detect without friction
		//IntergrationItem[ijoint] += ( CtrlTorq[ijoint] + coriolisTransTorq[ijoint] + ResidualSignal[ijoint] ) * SAMPLINGTIME;
		IntergrationItem[ijoint] += ( CtrlTorq[ijoint] + coriolisTransTorq[ijoint] - frictionTorque[ijoint] + ResidualSignal[ijoint] ) * SAMPLINGTIME;
		//IntergrationItem[ijoint] += ( CtrlTorq[ijoint] + coriolisTransTorq[ijoint] - frictionTorque[ijoint] - gravityTorque[ijoint] + ResidualSignal[ijoint] ) * SAMPLINGTIME;
	}

	// Compute Residual signal
	for(int ijoint = 0; ijoint < AXIS ; ijoint++ ){
		ResidualSignal[ijoint] = Ki[ijoint] * ( momentum[ijoint] - IntergrationItem[ijoint] );
		ExternalTorque[ijoint] = ResidualSignal[ijoint];	
	}

}


bool DetectCollision( double ExternalTorque[AXIS] ){

	// j1 offset: 7
	// j2 offset: 10
	// j3 offset: 10
	// j4 offset: 7
	// j5 offset: 3
	// j6 offset: 3

	double threshold[AXIS] = { 7.0, 10.0, 10.0, 7.0, 3.0, 2.0 };

	for(int ijoint = 0; ijoint < AXIS ; ijoint++ ){
		if( abs(ExternalTorque[ijoint]) > threshold[ijoint] )
			return true;
	}
	
	return false;
}


double PastInput[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double PastInput2[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double PastOutput[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double PastOutput2[AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void IIRFilter( double input[AXIS], double (&output)[AXIS], double Count ){

	// Denominator
	//double a1 = 1;					// 由 Matlab fdatool調, ( Fs = 1000 Hz, Fc = 50 Hz )
	//double a2 = -1.5610180758007182 ;
	//double a3 =  0.64135153805756318 ;

	// Numerator
	//double b1 = 1 * 0.020083365564211236 ;		// b * gain
	//double b2 = 2 * 0.020083365564211236 ;
	//double b3 = 1 * 0.020083365564211236 ;

	//// Denominator
	double a1 = 1;					// 由 Matlab fdatool調, ( Fs = 1000 Hz, Fc = 20 Hz )
	double a2 = -1.8226949251963083        ;
	double a3 =  0.83718165125602262       ;

	//// Numerator
	double b1 = 1 * 0.0036216815149286421      ;		// b * gain
	double b2 = 2 * 0.0036216815149286421      ;
	double b3 = 1 * 0.0036216815149286421      ;

	if ( Count == 0 )
	{
		for(int ijoint = 0 ; ijoint < AXIS ; ijoint++)
		{
			output[ijoint] = b1 * input[ijoint];
			PastInput[ijoint] = input[ijoint];
			PastOutput[ijoint] = output[ijoint];
		}
	}
	else if ( Count == 1 )
	{
		for(int ijoint = 0 ; ijoint < AXIS ; ijoint++)
		{
			output[ijoint] = b1 * input[ijoint] + b2 * PastInput[ijoint] - a2 * PastOutput[ijoint];
			PastInput2[ijoint] = PastInput[ijoint];
			PastInput[ijoint] = input[ijoint];
			PastOutput2[ijoint] = PastOutput[ijoint];
			PastOutput[ijoint] = output[ijoint];
		}
	}
	else
	{
		for(int ijoint = 0 ; ijoint < AXIS ; ijoint++)
		{
			output[ijoint] = b1 * input[ijoint] + b2 * PastInput[ijoint] + b3 * PastInput2[ijoint] - a2 * PastOutput[ijoint] - a3 * PastOutput2[ijoint];
			PastInput2[ijoint] = PastInput[ijoint];
			PastInput[ijoint] = input[ijoint];
			PastOutput2[ijoint] = PastOutput[ijoint];
			PastOutput[ijoint] = output[ijoint];
		}
	}

}

//-------------------------------- LSF1/4 --------------------------------

double LSFProfile[AXIS][4] = { 0.0 } ;

void LSF_Initialize( double initialPos[AXIS] )
{
	for(int ijoint = 0 ; ijoint < AXIS ; ijoint++ )
	{
		//LSF initialize
		for(int j = 0 ; j < 4 ; j++ )
		{
			LSFProfile[ijoint][j] = initialPos[ijoint] ;
		}
	}
}

void EstimateVelocity_LSF( double Pos[AXIS] , double (&LSF_Vel)[AXIS] )
{
	for(int ijoint = 0 ; ijoint < AXIS ; ijoint++ )
	{
		LSF_Vel[ijoint] = 0 ;

		//LSF estimator
		for(int j = 0 ; j < 3 ; j++ )
		{
			LSFProfile[ijoint][j] = LSFProfile[ijoint][j+1] ;
		}

		LSFProfile[ijoint][3] = Pos[ijoint] ;


		//OutPut
		LSF_Vel[ijoint] = ( LSFProfile[ijoint][3] * 0.3 + LSFProfile[ijoint][2] * 0.1 - LSFProfile[ijoint][1] * 0.1 - LSFProfile[ijoint][0] * 0.3 ) / (SAMPLINGTIME) ;
	}
}