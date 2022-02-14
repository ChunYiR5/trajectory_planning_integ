set(KINEMATICS_CSHARP_BINDING csharpKinematics)

# python module of kinematics setting
add_library(${KINEMATICS_CSHARP_BINDING} SHARED src/csharpKinematics.cpp)
set_target_properties(${KINEMATICS_CSHARP_BINDING} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${KINEMATICS_CSHARP_BINDING}
Eigen3
)
target_link_libraries(${KINEMATICS_CSHARP_BINDING} PRIVATE AUORML_Kinematics
														   AUORML_MathLib)                    