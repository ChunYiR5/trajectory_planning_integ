#ifndef _OBSERVER_
#define _OBSERVER_

#include "AUORML/Utility/Header.h" 
#include "AUORML/Dynamics/Dynamics.h"

    // ----- External Torque Observer ----- //
    void ExternalForceObserver( double Pos[AXIS], double Vel[AXIS],  double CtrlTorq[AXIS], double (&ExternalTorque)[AXIS] );
    bool DetectCollision( double ExternalTorque[AXIS] );
    void IIRFilter( double input[AXIS], double (&output)[AXIS], double Count );

    // ----- Velocity Observer ----- //
    void LSF_Initialize( double initialPos[AXIS] );
    void EstimateVelocity_LSF( double Pos[AXIS] , double (&LSF_Vel)[AXIS] );

#endif