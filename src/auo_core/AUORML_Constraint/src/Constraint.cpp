#include "AUORML/Constraint/Constraint.h"

Constraint::Constraint(std::shared_ptr<RobotModel> _robotModelPtr) : hardPosMaxBound{ _robotModelPtr -> maxRotateRad[0],
                                                                                      _robotModelPtr -> maxRotateRad[1],
                                                                                      _robotModelPtr -> maxRotateRad[2],
                                                                                      _robotModelPtr -> maxRotateRad[3],
                                                                                      _robotModelPtr -> maxRotateRad[4],
                                                                                      _robotModelPtr -> maxRotateRad[5]  },

                                                                     hardPosMinBound{ _robotModelPtr -> minRotateRad[0],
                                                                                      _robotModelPtr -> minRotateRad[1],
                                                                                      _robotModelPtr -> minRotateRad[2],
                                                                                      _robotModelPtr -> minRotateRad[3],
                                                                                      _robotModelPtr -> minRotateRad[4],
                                                                                      _robotModelPtr -> minRotateRad[5]  },                

                                                                     hardVelBound{    _robotModelPtr -> ratedVelocity[0],
                                                                                      _robotModelPtr -> ratedVelocity[1],
                                                                                      _robotModelPtr -> ratedVelocity[2],
                                                                                      _robotModelPtr -> ratedVelocity[3],
                                                                                      _robotModelPtr -> ratedVelocity[4],
                                                                                      _robotModelPtr -> ratedVelocity[5]  },
                                                                    
                                                                     hardAccBound{    hardVelBound[0] * 2,
                                                                                      hardVelBound[1] * 2,
                                                                                      hardVelBound[2] * 2,
                                                                                      hardVelBound[3] * 2,
                                                                                      hardVelBound[4] * 2,
                                                                                      hardVelBound[5] * 2, }, // temp value

                                                                     hardJerkBound{   hardAccBound[0] * 3,
                                                                                      hardAccBound[1] * 3,
                                                                                      hardAccBound[2] * 3,
                                                                                      hardAccBound[3] * 3,
                                                                                      hardAccBound[4] * 3,
                                                                                      hardAccBound[5] * 3  }, // temp value

                                                                     emergencyAccBound{  DEG2RAD(500),
                                                                                         DEG2RAD(500),
                                                                                         DEG2RAD(500),
                                                                                         DEG2RAD(500),
                                                                                         DEG2RAD(500),
                                                                                         DEG2RAD(500)  }, // temp value

                                                                     emergencyJerkBound{ DEG2RAD(10000),
                                                                                         DEG2RAD(10000),
                                                                                         DEG2RAD(10000),
                                                                                         DEG2RAD(10000),
                                                                                         DEG2RAD(10000),
                                                                                         DEG2RAD(10000)  }, // temp value

                                                                     hardCartVelBound{   100, 
                                                                                         100, 
                                                                                         100, 
                                                                                         DEG2RAD(10), 
                                                                                         DEG2RAD(10), 
                                                                                         DEG2RAD(10)  }, // temp value (mm/s)

                                                                     hardCartAccBound{   300,
                                                                                         300, 
                                                                                         300, 
                                                                                         DEG2RAD(30), 
                                                                                         DEG2RAD(30), 
                                                                                         DEG2RAD(30)  }, // temp value (mm/s^2)

                                                                     hardCartJerkBound{  1000,
                                                                                         1000, 
                                                                                         1000, 
                                                                                         DEG2RAD(100), 
                                                                                         DEG2RAD(100), 
                                                                                         DEG2RAD(100)  }, // temp value (mm/s^3)

                                                                     robotModelPtr(_robotModelPtr)

{
    softPosMaxBound[0] = DEG2RAD(360);
    softPosMaxBound[1] = DEG2RAD(40);
    softPosMaxBound[2] = DEG2RAD(245);
    softPosMaxBound[3] = DEG2RAD(360);
    softPosMaxBound[4] = DEG2RAD(145);
    softPosMaxBound[5] = DEG2RAD(360);
    
    softPosMinBound[0] = DEG2RAD(-360);
    softPosMinBound[1] = DEG2RAD(-40);
    softPosMinBound[2] = DEG2RAD(-60);
    softPosMinBound[3] = DEG2RAD(-360);
    softPosMinBound[4] = DEG2RAD(-145);
    softPosMinBound[5] = DEG2RAD(-360);

    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        softVelBound[ijoint]  = PI/6; // 90 deg/s
        softAccBound[ijoint]  = softVelBound[ijoint] * 3;
        softJerkBound[ijoint] = softAccBound[ijoint] * 3;
    }
}

bool Constraint::SetSoftPosMaxBound(double newPosMaxBound[AXIS]){

    if(CheckIfExceedHardLimit(newPosMaxBound, hardPosMaxBound))
        return false;

    else{
        for(int ijoint = 0; ijoint < AXIS; ijoint ++)
            this -> softPosMaxBound[ijoint] = newPosMaxBound[ijoint];
    }

    return true;
}

bool Constraint::SetSoftPosMinBound(double newPosMinBound[AXIS]){

    double inversePosBound[AXIS] = { 0.0 };
    double inversePosMinBound[AXIS] = { 0.0 };

    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        inversePosBound[ijoint] = -1 * newPosMinBound[ijoint];
        inversePosMinBound[ijoint] = -1 * hardPosMinBound[ijoint];
    }

    if(CheckIfExceedHardLimit(inversePosBound, inversePosMinBound))
        return false;

    else{
        for(int ijoint = 0; ijoint < AXIS; ijoint ++)
            this -> softPosMinBound[ijoint] = newPosMinBound[ijoint];
    }

    return true;
}

void Constraint::GetSoftPosMaxBound(double (&softPosMaxBound)[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        softPosMaxBound[ijoint] = this -> softPosMaxBound[ijoint];
}

void Constraint::GetSoftPosMinBound(double (&softPosMinBound)[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        softPosMinBound[ijoint] = this -> softPosMinBound[ijoint];    
}

void Constraint::GetHardPosMaxBound(double (&hardPosMaxBound)[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        hardPosMaxBound[ijoint] = this -> hardPosMaxBound[ijoint];       
}

void Constraint::GetHardPosMinBound(double (&hardPosMinBound)[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        hardPosMinBound[ijoint] = this -> hardPosMinBound[ijoint];  
}

bool Constraint::SetSoftVelBound(double newVelBound[AXIS]){
    
    // Vel bound always positive
    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        if(newVelBound[ijoint] < 0)
            return false;
    }

    if(CheckIfExceedHardLimit(newVelBound, hardVelBound))
        return false;

    else{
        for(int ijoint = 0; ijoint < AXIS; ijoint ++)
            this -> softVelBound[ijoint] = newVelBound[ijoint];
    }

    return true;
}

bool Constraint::SetSoftAccBound(double newAccBound[AXIS]){
 
    // Acc bound always positive
    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        if(newAccBound[ijoint] < 0)
            return false;
    }

    if(CheckIfExceedHardLimit(newAccBound, hardAccBound))
        return false;

    else{
        for(int ijoint = 0; ijoint < AXIS; ijoint ++)
            this -> softAccBound[ijoint] = newAccBound[ijoint];
    }

    return true;
}

bool Constraint::SetSoftJerkBound(double newJerkBound[AXIS]){
 
    // Jerk bound always positive
    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        if(newJerkBound[ijoint] < 0)
            return false;
    }

    if(CheckIfExceedHardLimit(newJerkBound, hardJerkBound))
        return false;

    else{
        for(int ijoint = 0; ijoint < AXIS; ijoint ++)
            this -> softJerkBound[ijoint] = newJerkBound[ijoint];
    }

    return true;
}

void Constraint::GetSoftVelBound(double (&softVelBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        softVelBound[ijoint] = this -> softVelBound[ijoint];
}

void Constraint::GetSoftAccBound(double (&softAccBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        softAccBound[ijoint] = this -> softAccBound[ijoint];
}

void Constraint::GetSoftJerkBound(double (&softJerkBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        softJerkBound[ijoint] = this -> softJerkBound[ijoint];
}

void Constraint::GetHardVelBound(double (&hardVelBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        hardVelBound[ijoint] = this -> hardVelBound[ijoint];
}

void Constraint::GetHardAccBound(double (&hardAccBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        hardAccBound[ijoint] = this -> hardAccBound[ijoint];
}

void Constraint::GetHardJerkBound(double (&hardJerkBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        hardJerkBound[ijoint] = this -> hardJerkBound[ijoint];
}

void Constraint::GetEmergencyAccBound(double (&emergencyAccBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        emergencyAccBound[ijoint] = this -> emergencyAccBound[ijoint];    
}

void Constraint::GetEmergencyJerkBound(double (&emergencyJerkBound)[AXIS]){
    for(int ijoint = 0; ijoint < AXIS; ijoint ++)
        emergencyJerkBound[ijoint] = this -> emergencyJerkBound[ijoint];        
}

void Constraint::GetHardCartVelBound(double (&hardCartVelBound)[TASKSPACE]){
    for(int itask = 0; itask < TASKSPACE; itask ++)
        hardCartVelBound[itask] = this -> hardCartVelBound[itask];      
}

void Constraint::GetHardCartAccBound(double (&hardCartAccBound)[TASKSPACE]){
    for(int itask = 0; itask < TASKSPACE; itask ++)
        hardCartAccBound[itask] = this -> hardCartAccBound[itask];      
}

void Constraint::GetHardCartJerkBound(double (&hardCartJerkBound)[TASKSPACE]){
    for(int itask = 0; itask < TASKSPACE; itask ++)
        hardCartJerkBound[itask] = this -> hardCartJerkBound[itask];      
}

bool Constraint::CheckJointPosLegality(double currentPos[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        if(currentPos[ijoint] > this -> softPosMaxBound[ijoint] || currentPos[ijoint] < this -> softPosMinBound[ijoint] )
            return false;
    }
        
    return true;
}

bool Constraint::CheckJointVelLegality(double currentVel[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        if( abs(currentVel[ijoint]) > this -> hardVelBound[ijoint] ) // here need to use soft?
            return false;
    }
        
    return true;
}

bool Constraint::CheckIfExceedHardLimit(double check[AXIS], const double benchMark[AXIS]){

    for(int ijoint = 0; ijoint < AXIS; ijoint++){
        if(check[ijoint] > benchMark[ijoint])
            return true;
    }

    return false;    
}