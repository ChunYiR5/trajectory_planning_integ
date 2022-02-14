#include <limits.h>
#include "AUORML/Kinematics/Kinematics.h"
#include "gtest/gtest.h"

namespace {

    TEST(InPNpiTest, UnderPI) {
    
    // This test is named "UnderPI", and belongs to the "InPNpiTest" test case.
    
    EXPECT_EQ(1, Factorial(-5));
    EXPECT_EQ(1, Factorial(-1));
    EXPECT_GT(Factorial(-10), 0);

    // <TechnicalDetails>
    //
    // EXPECT_EQ(expected, actual) is the same as
    //
    //   EXPECT_TRUE((expected) == (actual))
    //
    // except that it will print both the expected value and the actual
    // value when the assertion fails.  This is very helpful for
    // debugging.  Therefore in this case EXPECT_EQ is preferred.
    //
    // On the other hand, EXPECT_TRUE accepts any Boolean expression,
    // and is thus more general.
    //
    // </TechnicalDetails>
    }

}