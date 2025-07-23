#include "Libraries/Utils/src/lib/utils.h"
#include "gtest/gtest.h"

#include <cmath>

using ::testing::Test;
using ::Utils::ExponentialMovingAverage;

class UtilsTest : public ::testing::Test
{
protected:
    const float c_alpha = 0.1f;
    ExponentialMovingAverage *moving_average;
    virtual void SetUp()
    {
        moving_average = new ExponentialMovingAverage(c_alpha);
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

TEST_F(UtilsTest, CheckFunctionalityMatchesExpectedOutputs)
{
    // Simple test with 0's to check moving average has no bias to the average
    // output value
    moving_average->push(0);
    moving_average->push(0);
    moving_average->push(0);
    moving_average->push(0);
    EXPECT_EQ(moving_average->getAverage(), 0.f);

    // Impulse test to check decaying behaviour of the exponential moving
    // average filter
    float start_value = 1.f;
    float exp_decay_output = 0.f;
    float prev_exp_decay_output = c_alpha * start_value;
    moving_average->push(start_value);
    exp_decay_output = moving_average->getAverage();
    EXPECT_LT(exp_decay_output, start_value);

    for (int i = 0; i < 100; i++)
    {
        moving_average->push(0);
        exp_decay_output = moving_average->getAverage();
        EXPECT_NEAR(prev_exp_decay_output * (1 - c_alpha),
                    exp_decay_output,
                    Utils::csx_epsilon);
        prev_exp_decay_output = exp_decay_output;
    }
}

TEST_F(UtilsTest, CheckAlphaValueChangesFilterResponseAsExpected)
{
    // Use different values of alpha to change the mangitude of the filter
    // effect - a.k.a it's "speed" - how fast the filter effect takes hold
    float alpha_slow = 0.9f;
    float alpha_medium = 0.5f;
    float alpha_fast = 0.1f;
    ExponentialMovingAverage *moving_average_slow =
        new ExponentialMovingAverage(alpha_slow);
    ExponentialMovingAverage *moving_average_medium =
        new ExponentialMovingAverage(alpha_medium);
    ExponentialMovingAverage *moving_average_fast =
        new ExponentialMovingAverage(alpha_fast);

    // Placeholder variables
    float starting_value = 100.f;
    float exp_decay_output_slow, exp_decay_output_medium,
        exp_decay_output_fast = 0.f;
    float diff_slow, diff_medium, diff_fast = 0.f;

    // Test the "slow" moving filter (high value of alpha)
    moving_average_slow->push(starting_value);
    exp_decay_output_slow = moving_average_slow->getAverage();
    diff_slow = starting_value - exp_decay_output_slow;

    // Test the "medium" moving filter (mid value of alpha)
    moving_average_medium->push(starting_value);
    exp_decay_output_medium = moving_average_medium->getAverage();
    diff_medium = starting_value - exp_decay_output_medium;

    // Test the "fast" moving filter (low value of alpha)
    moving_average_fast->push(starting_value);
    exp_decay_output_fast = moving_average_fast->getAverage();
    diff_fast = starting_value - exp_decay_output_fast;

    // If alpha is working as expected, we would expect a larger diff between
    // first and second value for the "fast" moving average vs the medium. This
    // logic then follows for "medium" vs "slow" moving averages.
    EXPECT_GT(diff_medium, diff_slow);
    EXPECT_GT(diff_fast, diff_medium);

    delete moving_average_slow;
    delete moving_average_medium;
    delete moving_average_fast;
}