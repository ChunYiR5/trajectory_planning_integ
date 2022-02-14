set(AUORML_LIB_NAME AUORML_Dynamics)

add_library(${AUORML_LIB_NAME} SHARED src/Dynamics.cpp)
set_target_properties(${AUORML_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${AUORML_LIB_NAME}
Eigen3
)
target_link_libraries(${AUORML_LIB_NAME} )

install(DIRECTORY include/ DESTINATION include)