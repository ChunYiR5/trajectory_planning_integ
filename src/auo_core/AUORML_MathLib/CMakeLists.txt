set(AUORML_LIB_NAME AUORML_MathLib)

add_library(${AUORML_LIB_NAME} SHARED src/MathLib.cpp
									  src/RotateComputation.cpp)

set_target_properties(${AUORML_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${AUORML_LIB_NAME}
Eigen3
)

add_executable(rotateComputation_unittest1 test/rotateComputation_unittest1.cpp )
target_link_libraries(rotateComputation_unittest1 gtest_main gtest ${AUORML_LIB_NAME})
add_test(NAME rotateComputation_unittest1 COMMAND rotateComputation_unittest1)


install(DIRECTORY include/ DESTINATION include)
