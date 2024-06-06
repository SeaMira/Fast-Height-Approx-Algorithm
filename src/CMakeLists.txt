set(HPP_LIST "${PROJECT_SOURCE_DIR}/include/Circle/circle.hpp")
set(CPP_LIST "circle.cpp")

add_library(circle  ${CPP_LIST} ${HEADER_LIST})
target_include_directories(circle PUBLIC ../include)