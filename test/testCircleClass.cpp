#include <gtest/gtest.h>
#include <Circle/circle.hpp>
#include <cmath>
//ctest -R uwu
//https://google.github.io/googletest/reference/assertions.html



TEST(CircleClass, TestCalculatearea) {
    circle c1(1);
    circle c2(0);

    EXPECT_EQ(c1.calculateArea(), M_PI);
    EXPECT_EQ(c2.calculateArea(), 0);
}


// Demonstrate some basic assertions.
TEST(CircleClass, TestPointInCircle) {
    circle c1(1);
    circle c2(0);

    EXPECT_TRUE(c1.pointInCircle(0.1, 0.1));
    EXPECT_FALSE(c2.pointInCircle(0.1, 0.1));
}