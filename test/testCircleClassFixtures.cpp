#include <gtest/gtest.h>
#include <Circle/circle.hpp>
#include <cmath>
//Ver clase en:
//https://google.github.io/googletest/primer.html

class CircleClass2 : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.
   circle *c1;
   circle *c2;

   void SetUp() override {
      // Code here will be called immediately after the constructor (right
      // before each test).
       c1 = new circle(1);
       c2 = new circle(0);
   }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
     delete c1;
     delete c2;
  }
/*
    CircleClass2() {
        // You can do set-up work for each test here    
    }
    
  ~CircleClass2() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
     // Code here will be called immediately after the constructor (right
     // before each test).
     std::cout<<"Inicia Test"<<std::endl;
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }

  // Class members declared here can be used by all tests in the test suite
  // for Foo.
  */
};

TEST_F(CircleClass2, TestCalculatearea2) {

    EXPECT_EQ(c1->calculateArea(), M_PI);
    EXPECT_EQ(c2->calculateArea(), 0);
}


// Demonstrate some basic assertions.
TEST_F(CircleClass2, TestPointInCircle2) {

    EXPECT_TRUE(c1->pointInCircle(0.1, 0.1));
    EXPECT_FALSE(c2->pointInCircle(0.1, 0.1));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}