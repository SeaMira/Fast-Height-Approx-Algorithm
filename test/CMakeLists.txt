add_executable(BMHeapTests
    testBMHeap.cpp
)

target_compile_options(BMHeapTests PRIVATE
    -Wall
    -Wextra
    -pedantic
)

#https://google.github.io/googletest/quickstart-cmake.html
target_link_libraries(BMHeapTests
    gtest
)

target_include_directories(BMHeapTests PUBLIC ../include)

include(GoogleTest)
gtest_discover_tests(BMHeapTests)