#include "auorml/MathLib/RotateComputation.h"
#include "gtest/gtest.h"

namespace {

    // This test is named "quanternionDegree", and belongs to the "RotationComputationTest" test case.    
    TEST(RotationComputationTest, quanternionDegree) {
    
    // acos(0) * 2 = pi
    EXPECT_DOUBLE_EQ(3.1415927410125732, float(QuantenionToDegree(0)) );

    // acos(0.7) * 2 = 1.5907976627349854 rad
    EXPECT_DOUBLE_EQ(1.5907976627349854, float(QuantenionToDegree(0.7)) );

    // Acos can't receieved value > 1, it will return nan value
    // and According to the IEEE standard, NaN values have the odd property that comparisons involving them are always false. 
    // That is, for a float f, f != f will be true only if f is NaN.
    EXPECT_NE((QuantenionToDegree(1.5)), (QuantenionToDegree(1.5)) ); 

    }

}