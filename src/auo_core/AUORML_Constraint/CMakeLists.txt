set(AUORML_LIB_NAME AUORML_Constraint)

add_library(${AUORML_LIB_NAME} SHARED src/Constraint.cpp)
set_target_properties(${AUORML_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${AUORML_LIB_NAME}
Eigen3
)
target_link_libraries(${AUORML_LIB_NAME} AUORML_RobotModel AUORML_MathLib)

install(DIRECTORY include/ DESTINATION include)