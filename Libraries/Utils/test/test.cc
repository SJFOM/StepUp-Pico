#include "Libraries/Utils/src/lib/utils.h"
#include "gtest/gtest.h"

using ::testing::Test;

class UtilsTest : public ::testing::Test
{
protected:
    Utils::ExponentialMovingAverage moving_average;
    virtual void SetUp()
    {
        // function = new MockTestFunction();
        ;
    }
    virtual void TearDown()
    {
        // delete function;
        ;
    }
};

TEST_F(UtilsTest, SampleTest1)
{
    ;
}
