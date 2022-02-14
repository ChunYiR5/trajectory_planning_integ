#include "AUORML/Kinematics/Kinematics.h"
#include <iostream>
#include <fstream>
#include <string>
#include <random>

// ----- Define and Namespace //
#ifndef AXIS
#define AXIS        6
#endif

#define IDHParamsNum        5 // theta, d, a, alpha, beta

#define _DLLExport __declspec (dllexport)

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Detect if input string is double string.
//!
//! \details
//! Check if input string is double, if not, return false, else return true.
//! Since the method of atof() always return 0.0 if input isn't number, the atof() can't detect between non-numerical and 0.
//! Hence here use like this way.
//!
//! \param in
//! Double string. Ex: "1", "456".
//!
//! \param res
//! Reference of numerical value of input double string.
//! 
//! \return
//! Return true if input is double string. Otherwise return false.
//!
//! ----------------------------------------------------------

bool parse_double(std::string in, double& res) {
    try{
        size_t read= 0;
        res = std::stod(in, &read);
        if (in.size() != read)
            return false;
    } 
    catch (std::invalid_argument) {
        return false;
    }    
    return true;
}

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Show debug string if input mode equal "DEBUG". This func is for developer use.
//!
//! \param mode
//! User specific mode. Only workable when input is "DEBUG".
//!
//! \param debugInfo
//! DebugInfo that needs to be showed on terminal.
//! 
//! ----------------------------------------------------------

void ShowDebugInfo(std::string mode, std::string debugInfo){
    if(mode == "DEBUG")
        std::cout << debugInfo << std::endl;
}

extern "C"  _DLLExport void AUORMLCSHARP_Euler2Quaternion(double* quaternion){

    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    auto result = Euler2Quaternion(yaw, pitch, roll);

    quaternion[0] = result[0] ;
    quaternion[1] = result[1] ;
    quaternion[2] = result[2] ;
    quaternion[3] = result[3] ;

} 

extern "C" _DLLExport IDHKinematics* createKinematics(){

    Eigen::Matrix<double, AXIS, IDHParamsNum> DHTable;
    std::string filePath = "config.csv";
    std::string mode = "RELEASE";

    // ----- File I/O ----- //
    std::fstream file(filePath);
    if (!file.is_open()){
        ShowDebugInfo(mode, "File can't be open or not exist. Please Check again.");
        return NULL;
    }
        
    // ----- Start Read Line ------ //
    int axisCount = 0;
    std::string line;
    std::vector<double> matrixData;
    std::vector<double> axisDataNumVec;

    while (getline( file, line,'\n')){  // read file when catch new line unit
        axisCount++; // each time get one line, increase axis num by 1

        std::istringstream templine(line); // change string to stream
        std::string data;

        int eachAxisDataNum = 0;
        while (std::getline( templine, data,',')){ // read file until reach comma
            
            // check if data contain non-number
            double dataValue;
            if( !parse_double(data, dataValue) ){
                ShowDebugInfo(mode, "File content non numerical (may be string?) .... return false.");
                return NULL;
            }

            matrixData.push_back( dataValue );  // change string to num
            eachAxisDataNum++;
        }

        axisDataNumVec.push_back(eachAxisDataNum);
    }

    // ----- Check if each line is with enough data and if missing data ----- //
    
    if( axisCount != AXIS ){ // a. check if file is empty
        ShowDebugInfo(mode, "The File Missing some axis.");
        return NULL;
    }

    if( matrixData.size() != AXIS*IDHParamsNum ){ // b. check if total dh parameters not equal to 5*AXIS
        ShowDebugInfo(mode, "DH Parameters are not in correct format, please check again");
        return NULL;
    }

    for(int ijoint=0; ijoint < AXIS; ijoint++){ // c. check if each axis with correct dh parameter
        if( axisDataNumVec[ijoint] != IDHParamsNum ){
            ShowDebugInfo(mode, "AXIS " + std::to_string(ijoint+1) + " has incorrect parameter num.");
            return NULL;  
        }
    }

    // ----- pass all test, return the DH Matrix and axisNum ----- //
    for(int ijoint=0; ijoint < AXIS; ijoint++){
        for(int iDHParams=0; iDHParams < IDHParamsNum; iDHParams++){
                
            if( iDHParams == 0 || iDHParams == 3 || iDHParams == 4) // Change alpha, beta, theta from deg to rad
                DHTable(ijoint, iDHParams) = matrixData[ijoint*IDHParamsNum + iDHParams] * M_PI / 180;
            else
                DHTable(ijoint, iDHParams) = matrixData[ijoint*IDHParamsNum + iDHParams];
        }
    }

    // ----- everything fine, return true ------ //
    return new IDHKinematics(DHTable, "Industrial");
}

extern "C" _DLLExport  void freeKinematics(IDHKinematics* instance) {
    delete instance;
}

extern "C"  _DLLExport void AUORMLCSHARP_GetEndEffectorTransformationMatrix(IDHKinematics* instance, double* TargetPos, double** HomogenousMatrix){

    Eigen::Matrix<double, 1, 6> _targetPose = ( Eigen::Matrix<double, 1, 6>() << TargetPos[0], TargetPos[1], TargetPos[2], TargetPos[3], TargetPos[4], TargetPos[5] ).finished();

    auto result = instance -> GetEndEffectorTransformationMatrix(_targetPose);

    for(int irow = 0; irow < 4; irow ++){
        for(int icol = 0; icol < 4; icol ++){
            HomogenousMatrix[irow][icol] = result(irow,icol);
        }
    }
} 

extern "C"  _DLLExport double AUORMLCSHARP_InPNpi(IDHKinematics* instance, double inpValue){

    return instance -> InPNpi(inpValue);
} 

extern "C"  _DLLExport void AUORMLCSHARP_GetRX(IDHKinematics* instance, double rad, double** HomogenousMatrix){

    auto result = instance -> GetRX(rad);

    for(int irow = 0; irow < 4; irow ++){
        for(int icol = 0; icol < 4; icol ++){
            HomogenousMatrix[irow][icol] = result(irow,icol);
        }
    }
} 

extern "C"  _DLLExport void AUORMLCSHARP_GetRY(IDHKinematics* instance, double rad, double** HomogenousMatrix){

    auto result = instance -> GetRY(rad);

    for(int irow = 0; irow < 4; irow ++){
        for(int icol = 0; icol < 4; icol ++){
            HomogenousMatrix[irow][icol] = result(irow,icol);
        }
    }
} 

extern "C"  _DLLExport void AUORMLCSHARP_GetRZ(IDHKinematics* instance, double rad, double** HomogenousMatrix){

    auto result = instance -> GetRY(rad);

    for(int irow = 0; irow < 4; irow ++){
        for(int icol = 0; icol < 4; icol ++){
            HomogenousMatrix[irow][icol] = result(irow,icol);
        }
    }
} 

extern "C"  _DLLExport void AUORMLCSHARP_GetTrans(IDHKinematics* instance, double x, double y, double z, double** HomogenousMatrix){

    auto result = instance -> GetTrans(x, y, z);

    for(int irow = 0; irow < 4; irow ++){
        for(int icol = 0; icol < 4; icol ++){
            HomogenousMatrix[irow][icol] = result(irow,icol);
        }
    }
} 

extern "C"  _DLLExport bool AUORMLCSHARP_GetInverseKinematics(IDHKinematics* instance, double* targetPose, double* referenceJoint, double* solution){

    Eigen::Matrix<double, 1, 6> _targetPose = ( Eigen::Matrix<double, 1, 6>() << targetPose[0], targetPose[1], targetPose[2], targetPose[3], targetPose[4], targetPose[5] ).finished();
    Eigen::Matrix<double, 1, 6> _referenceJoint = ( Eigen::Matrix<double, 1, 6>() << referenceJoint[0], referenceJoint[1], referenceJoint[2], referenceJoint[3], referenceJoint[4], referenceJoint[5] ).finished();
    Eigen::Matrix<double, 1, 6> _solution;

    auto result = instance -> GetInverseKinematics(_targetPose, _referenceJoint, _solution);

    for(int ijoint = 0; ijoint < AXIS; ijoint ++){
        solution[ijoint] = _solution(0, ijoint);
    }

    return result;
} 

