set(PY_LIB_NAME pyKinematics)

# python module of kinematics setting
pybind11_add_module(${PY_LIB_NAME} SHARED src/pyKinematics.cpp)
set_target_properties(${PY_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PY_LIB_NAME}
Eigen3
)
target_link_libraries(${PY_LIB_NAME} PRIVATE AUORML_Kinematics
                                             AUORML_MathLib)                    