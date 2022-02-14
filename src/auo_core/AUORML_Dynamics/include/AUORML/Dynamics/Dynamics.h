#ifndef ROBOTMODEL
#define ROBOTMODEL 

	#include "AUORML/Utility/Header.h" 
	#include <Eigen/Dense>

	using namespace Eigen;

	void RobotModel_Beta( char *path );
	bool ReadFullDynamicParameter( const std::string& name );
	void GetInertiaMatrix( double Pos[AXIS], Matrix<double, AXIS, AXIS>& inertiaMatarix );
	void GetInertiaTorque( double Pos[AXIS], double Acc[AXIS], double (&InertiaTorque)[AXIS] );
	void GetCoriolisMatrix( double Pos[AXIS], double Vel[AXIS], Matrix<double, AXIS, AXIS>& coriolisMatarix  );
	void GetCoriolisTorque( double Pos[AXIS], double Vel[AXIS], double (&CoriolisTorque)[AXIS] );
	void GetFrictionTorque( double Vel[AXIS] , double (&FrictionTorque)[AXIS] );
	void GetGravityTorque( double Pos[AXIS] , double (&GravityTorque)[AXIS]);
	
#endif