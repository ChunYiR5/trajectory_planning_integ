set(AUORML_LIB_NAME AUORML_Observer)

add_library(${AUORML_LIB_NAME} SHARED src/Observer.cpp)
set_target_properties(${AUORML_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${AUORML_LIB_NAME}
Eigen3
)
target_link_libraries(${AUORML_LIB_NAME} AUORML_Dynamics)

install(DIRECTORY include/ DESTINATION include)