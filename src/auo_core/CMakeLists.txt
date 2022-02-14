cmake_minimum_required(VERSION 3.10.2)
project(auo_core)

# Common cmake code applied to all moveit packages
find_package(auo_common REQUIRED)
auo_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Testing
enable_testing()

# set auorml core include directory
set(THIS_PACKAGE_INCLUDE_DIRS
    AUORML_Constraint/include
    AUORML_Kinematics/include
    AUORML_MathLib/include
    AUORML_PathPlanning/include
    AUORML_Dynamics/include
    AUORML_Utility/include
    AUORML_Observer/include
    AUORML_RobotModel/include
    googletest/include
	)

# set install library list	
set(THIS_PACKAGE_LIBRARIES
    AUORML_Constraint
    AUORML_MathLib
    AUORML_Kinematics	
    AUORML_PathPlanning
    AUORML_Dynamics
    AUORML_Utility
    AUORML_Observer
    AUORML_RobotModel
    pyKinematics
	  csharpKinematics
	)

# set project dependencies		
set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
	  eigen3_cmake_module
	)

	
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(AUORML_Constraint)
add_subdirectory(AUORML_MathLib)
add_subdirectory(AUORML_Kinematics)
add_subdirectory(AUORML_PathPlanning)
add_subdirectory(AUORML_Dynamics)
add_subdirectory(AUORML_Utility)
add_subdirectory(AUORML_Observer)
add_subdirectory(AUORML_RobotModel)
add_subdirectory(python_binding)
add_subdirectory(kinematics_csharp_binding)
add_subdirectory(googletest)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cppcheck_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #set(ament_cmake_flake8_FOUND TRUE)
  #set(ament_cmake_uncrustify_FOUND TRUE)
  #set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()