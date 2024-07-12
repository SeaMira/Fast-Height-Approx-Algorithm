set(FST_H_APROX_HPP_LIST "${PROJECT_SOURCE_DIR}/include/MeshFunctions/mesh_functions.hpp")
set(FST_H_APROX_CPP_LIST "mesh_functions.cpp")
add_library(fastHAprox  ${FST_H_APROX_CPP_LIST} ${FST_H_APROX_HPP_LIST})
target_include_directories(fastHAprox PUBLIC ../include)

