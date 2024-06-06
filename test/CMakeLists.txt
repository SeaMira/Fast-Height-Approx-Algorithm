add_executable(
  CircleTests
  testCircleClass.cpp
  testCircleClassFixtures.cpp
)

#https://google.github.io/googletest/quickstart-cmake.html
target_link_libraries(
    CircleTests
    gtest_main
    circle
)

include(GoogleTest)
gtest_discover_tests(CircleTests)