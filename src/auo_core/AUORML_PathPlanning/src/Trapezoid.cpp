#include "AUORML/PathPlanning/Trapezoid.h"
#include "stdio.h"

double maxValue = 0;
double Time = 0;
double Ta = 0, Tb = 0, t1 = 0, t2 = 0, t3 = 0;
double jerk[AXIS] = { 0.0 };


bool VelocityTrapezoid( double InitialVel[AXIS] , double FianlVelCmd[AXIS] , double (&iVelCmd)[AXIS] )
{
	if ( Time == 0 )
	{
		double _Amax = 100 ;			
		double _Jerk = 300 ;
			
		for	( int ijoint = 0 ; ijoint < AXIS ; ijoint++ )
		{	
			if ( abs( FianlVelCmd[ijoint] - InitialVel[ijoint] ) > maxValue )
			{
				maxValue = abs( FianlVelCmd[ijoint] - InitialVel[ijoint] ) ;
			}
		}
		
		Ta = _Amax / _Jerk ;
		Tb = ( maxValue - Ta * _Amax ) / _Amax ;
		
		for ( int ijoint = 0 ; ijoint < AXIS ; ijoint++ )
		{
			jerk[ijoint] =  ( FianlVelCmd[ijoint] - InitialVel[ijoint] ) / ( Ta * ( Ta + Tb )  ) ;
		}
		
		t1 = Ta ;
		t2 = Ta + Tb ;
		t3 = 2 *Ta + Tb ;
	}
	if(  Time <= t3 )
	{
		for ( int ijoint = 0 ; ijoint < AXIS ; ijoint++ )
		{
			if ( Time <= t1 )
			{				
				iVelCmd[ijoint] = jerk[ijoint] * pow(Time,2) / 2 ;				
			}
			else if ( Time > t1 && Time <= t2 )
			{
				iVelCmd[ijoint] = jerk[ijoint] * t1 * ( Time - t1 ) + jerk[ijoint] * pow(t1,2) / 2 ;
			}
			else if ( Time > t2 && Time <= t3 )
			{				
				iVelCmd[ijoint] = jerk[ijoint] * t1 * ( t2 - t1 ) + jerk[ijoint] * pow(t1,2) / 2 - ( jerk[ijoint] * pow(Time-t2,2) / 2 );
			}
		}
		
		Time = Time + SAMPLINGTIME ;
	}
	else
	{	
		for ( int ijoint = 0 ; ijoint < AXIS ; ijoint++ ) // 
		{
			iVelCmd[ijoint] = FianlVelCmd[ijoint] ;
		}
		
		Time = 0 ; // 
		maxValue = 0 ;
		return true ;
	}

	return false;
}

//void TrapezoidInitialize(void)