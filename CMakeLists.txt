cmake_minimum_required(VERSION 3.16.3)
project(playCircle)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(extern)


find_package(CGAL REQUIRED)
message("CGAL Version: ${CGAL_VERSION}")
find_package(ImageMagick COMPONENTS Magick++)
include_directories(${ImageMagick_INCLUDE_DIRS})


# demo boundfield
add_executable(bf_simple_demo
    src/BoundField_demo.cpp)
target_link_libraries(bf_simple_demo PUBLIC
    CGAL
    ${ImageMagick_LIBRARIES})
target_include_directories(bf_simple_demo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(bf_simple_demo PUBLIC #required by imagemagick for some reason
    MAGICKCORE_HDRI_ENABLE
    MAGICKCORE_QUANTUM_DEPTH=16)


#quitar para evitar test
add_subdirectory(test)
enable_testing()