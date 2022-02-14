set(AUORML_LIB_NAME AUORML_PathPlanning)

add_library(${AUORML_LIB_NAME} SHARED src/Trapezoid.cpp)
set_target_properties(${AUORML_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${AUORML_LIB_NAME}
)
target_link_libraries(${AUORML_LIB_NAME} AUORML_Kinematics )
	
install(DIRECTORY include/ DESTINATION include)