set(PY_LIB_NAME pyKinematics)

# python binding
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
add_subdirectory(pybind11)

# install pybind module
add_subdirectory(pyKinematics)