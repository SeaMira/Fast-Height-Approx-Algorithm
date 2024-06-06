cmake_minimum_required(VERSION 3.16.3)
project(playCircle)
set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(extern)
add_subdirectory(src)
add_subdirectory(test)

#quitar para evitar test
enable_testing()

target_link_libraries(${PROJECT_NAME} circle)