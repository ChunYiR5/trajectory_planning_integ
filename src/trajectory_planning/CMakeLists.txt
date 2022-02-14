cmake_minimum_required(VERSION 3.5)
project(trajectory_planning)

find_package(auo_common REQUIRED)
auo_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auo_core REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ------- Package Include dependencies ------ #
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  auo_core
  Eigen3
  eigen3_cmake_module
)

include_directories(include)

# ------- Production Setting ------- #

# TRAJECTORY_PLANNING.lib
set(TRAJECTORY_PLANNING Trajectory_Planning)
add_library(${TRAJECTORY_PLANNING} SHARED src/Trajectory_Planning.cpp)
target_compile_definitions(${TRAJECTORY_PLANNING}
   PRIVATE "TRAJECTORY_PLANNING_COMPOSITION_DLL")
set_target_properties(${TRAJECTORY_PLANNING}
  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${TRAJECTORY_PLANNING} ${THIS_PACKAGE_INCLUDE_DEPENDS} )
target_link_libraries(${TRAJECTORY_PLANNING})


# main.exe
set(MAIN main)
add_executable(${MAIN} src/main.cpp )
set_target_properties(${MAIN}
  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MAIN} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${MAIN} ${TRAJECTORY_PLANNING} )

# -------- Install Setting ---------- #

install(
  TARGETS 
    ${TRAJECTORY_PLANNING}  
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS
    ${MAIN} 
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

  
ament_package()
