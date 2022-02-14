#include "AUORML/Kinematics/Kinematics.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <iostream>
#include <fstream>
#include <string>
#include <random>

// ----- Define and Namespace //
#ifndef AXIS
#define AXIS        6
#endif

#define IDHParamsNum        5 // theta, d, a, alpha, beta
namespace py = pybind11;

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

//  ---------------------- Doxygen info ----------------------
//!
//! \brief 
//! Read the kinematics data from filePath.
//!
//! \details
//! This function will check whether dh files is in correct format:
//! a). examine in input np array is 6*5.  b). examine if the config file is in correct format.
//!
//! \param filePath
//! Path where dh param data located. Default is in same dir and name as "config.csv".
//! The config file is in format ->  theta1, d1, a1, alpha1, beta1 // axis1, unit: mm, deg
//!                                  theta2, d2, a2, alpha2, beta2 // axis2, .... etc
//!
//! \param mode
//! User specific mode. Choosable option is "RELEASE" and "DEBUG", Default is "RELEASE".
//!
//! \param debugInfo
//! DebugInfo that needs to be showed on terminal.
//! 
//! ----------------------------------------------------------

bool ReadDenavitHartenbergConfig(Eigen::Ref<Matrix<double, AXIS, IDHParamsNum, Eigen::RowMajor>> DHTable, std::string filePath,  std::string mode = "RELEASE"){

    // ----- File I/O ----- //
    std::fstream file(filePath);
    if (!file.is_open()){
        ShowDebugInfo(mode, "File can't be open or not exist. Please Check again.");
        return false;
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
                return false;
            }

            matrixData.push_back( dataValue );  // change string to num
            eachAxisDataNum++;
        }

        axisDataNumVec.push_back(eachAxisDataNum);
    }

    // ----- Check if each line is with enough data and if missing data ----- //
    
    if( axisCount != AXIS ){ // a. check if file is empty
        ShowDebugInfo(mode, "The File Missing some axis.");
        return false;
    }

    if( matrixData.size() != AXIS*IDHParamsNum ){ // b. check if total dh parameters not equal to 5*AXIS
        ShowDebugInfo(mode, "DH Parameters are not in correct format, please check again");
        return false;
    }

    for(int ijoint=0; ijoint < AXIS; ijoint++){ // c. check if each axis with correct dh parameter
        if( axisDataNumVec[ijoint] != IDHParamsNum ){
            ShowDebugInfo(mode, "AXIS " + std::to_string(ijoint+1) + " has incorrect parameter num.");
            return false;  
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
    return true;
}

PYBIND11_MODULE(pyKinematics, m)
{
    m.doc() = "pybind11 of C++ kinematics library";      // module doc string
    
    /* ==================== Sample Module of function binding ============================ 
    
    m.def("print",                                     // function name
    &print,                                            // function pointer
    "A function which test echo hello world"           //function doc string
    );

    */

    /* ==================== Sample Module of class binding ============================ 
    
    py::class_<IDHKinematics>(m, "IDHKinematics")                                                                  // class name
    .def(py::init<Eigen::Matrix<double, AXIS, 5>>())                                                               // class constructor
    .def("GetForwardKinematics", &IDHKinematics::GetForwardKinematics, "Forward Kinematics binding of IDH Model")  // class function1
    .def("GetEndEffectorPose", &IDHKinematics::GetEndEffectorPose, "End effector pose array of IDH Model")         // class function2
    .def("GetInverseKinematics", &IDHKinematics::GetInverseKinematics, "Inverse Kinematics binding of IDH Model"); // class function3
    
    */
    
    // Func: ReadKinematicsConfig
    m.def("ReadDenavitHartenbergConfig", &ReadDenavitHartenbergConfig, "Read DH Table from file" );

    // Func: Function from rotation computation class
    m.def("Euler2Quaternion", &Euler2Quaternion, "" );
    m.def("QuantenionToEuler", &QuantenionToEuler, "" );
    m.def("QuantenionToDegree", &QuantenionToDegree, "" );
    m.def("RotationMatrixToYPR", &RotationMatrixToYPR, "" );
    m.def("YPRToRotationMatrix", &YPRToRotationMatrix, "" );

    // Class: IDHKinematics
    py::class_<IDHKinematics>(m, "IDHKinematics")
        .def(py::init<Eigen::Matrix<double, AXIS, 5>, std::string>())
        .def("GetForwardKinematics", &IDHKinematics::GetForwardKinematics, "Get Forward Kinematics binding of IDH Model")
        .def("GetEndEffectorPose", &IDHKinematics::GetEndEffectorPose, "Get End effector pose array of IDH Model")
        .def("GetEndEffectorTransformationMatrix", &IDHKinematics::GetEndEffectorTransformationMatrix, "GetEndEffectorTransformationMatrix binding of IDH Model")
        .def("GetInverseKinematics", &IDHKinematics::GetInverseKinematics, "Inverse Kinematics binding of IDH Model")
        .def("DecompositionXYZRPY", &IDHKinematics::DecompositionXYZRPY, "DecompositionXYZRPY binding of IDH Model")
        .def("DecompositionXYZYPR", &IDHKinematics::DecompositionXYZYPR, "DecompositionXYZYPR binding of IDH Model")
        .def("DecompositionXYZQuaternion", &IDHKinematics::DecompositionXYZQuaternion, "DecompositionXYZQuaternion binding of IDH Model")
        .def("GetRobotJacobian", &IDHKinematics::GetRobotJacobian, "GetRobotJacobian binding of IDH Model")
        .def("GetRobotJacobianDot", &IDHKinematics::GetRobotJacobianDot, "GetRobotJacobianDot binding of IDH Model")
        .def("GetIDHTable", &IDHKinematics::GetIDHTable, "GetIDHTable binding of IDH Model")
        .def("ChangeIDHTable", &IDHKinematics::ChangeIDHTable, "ChangeIDHTable binding of IDH Model")
        .def("ChangeRobotType", &IDHKinematics::ChangeRobotType, "ChangeRobotType binding of IDH Model")
        .def("TestKinematics", &IDHKinematics::TestKinematics, "Test Kinematics correctness")

        // 2022.01.04 update
        .def("SolveIndustrialIK", &IDHKinematics::SolveIndustrialIK, "SolveIndustrialIK binding of IDH Model.")
        .def("SolveURIK", &IDHKinematics::SolveURIK, "SolveURIK binding of IDH Model.");
}