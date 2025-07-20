#include "Libraries/Utils/src/lib/utils.h"
#include "gtest/gtest.h"

using ::testing::Test;
using ::Utils::ExponentialMovingAverage;

class UtilsTest : public ::testing::Test
{
protected:
    ExponentialMovingAverage *moving_average;
    virtual void SetUp()
    {
        moving_average = new ExponentialMovingAverage(0.1f);
    }
    virtual void TearDown()
    {
        delete moving_average;
    }
};

TEST_F(UtilsTest, CreateExpMovingAverageInstanceWithValidAndInvalidInputs)
{
    // If alpha value provided, expect that an assert will catch an out of
    // bounds value (valid range is between 0 to 1.0 inclusive)

    // Valid inputs
    ASSERT_TRUE(new ExponentialMovingAverage());
    ASSERT_TRUE(new ExponentialMovingAverage(0.00001f));
    ASSERT_TRUE(new ExponentialMovingAverage(0.9f));
    ASSERT_TRUE(new ExponentialMovingAverage(1.0f));

    // Invalid inputs
    ASSERT_DEATH(new ExponentialMovingAverage(-1.f), "");
    ASSERT_DEATH(new ExponentialMovingAverage(-00001.f), "");
    ASSERT_DEATH(new ExponentialMovingAverage(1.1f), "");
}
