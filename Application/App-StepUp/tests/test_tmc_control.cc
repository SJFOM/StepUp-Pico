#include <gtest/gtest.h>

#include "../include/tmc_control.hpp"

class TMCControlTest : public ::testing::Test
{
protected:

    TMCControl tmc_control;
    virtual void SetUp()
    {
        tmc_control = new TMCControl();
    }
    virtual void TearDown()
    {
        delete tmc_control;
    }
};

// TEST_F(TMCControlTest, ShouldSetCorrectVelocity)
// {
//     EXPECT_CALL(*m_tmcx_controller, move()).Times(1);
//     tmc_control.updateMovementDynamics(1000,1);
    // if (_velocity < 0 && direction == -1)
    // {
    //     // Do nothing
    //     ;
    // }
    // if (_velocity > 0 && direction == 1)
    // {
    //     // Do nothing
    //     ;
    // }
    // if (_velocity < 0 && direction == 1)
    // {
    //     // Invert direction
    //     _velocity = -_velocity;
    // }
    // if (_velocity > 0 && direction == -1)
    // {
    //     // Invert direction
    //     _velocity = -_velocity;
    // }
    // if (direction == 0)
    // {
    //     // Stop motor
    //     _velocity = 0;
    // }

// }