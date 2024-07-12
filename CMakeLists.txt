cmake_minimum_required(VERSION 3.16.3)
project(fastHeightAprox)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)
add_subdirectory(extern)


find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
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


if(CGAL_Qt5_FOUND)
    #required to use basic_viewer
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
  endif()
  if ( CGAL_FOUND )
    #create the executable of the application
    create_single_source_cgal_program("main.cpp")
    if(CGAL_Qt5_FOUND )
      #link it with the required CGAL libraries
      target_link_libraries(fastHAprox PUBLIC CGAL::CGAL_Qt5 CGAL::CGAL_Basic_viewer)
      target_link_libraries(main PUBLIC fastHAprox)
    endif()
  else()
      message(STATUS "This program requires the CGAL library, and will not be compiled.")
  endif()


#quitar para evitar test
add_subdirectory(test)
enable_testing()