#include "gtest/gtest.h"
#include "Libraries/TestLibrary/test/mocks/mock_function.hpp"

using ::testing::_;
using ::testing::Return;

class FunctionTest : public ::testing::Test
{
protected:
    MockTestFunction *function;
    virtual void SetUp()
    {
        function = new MockTestFunction();
    }
    virtual void TearDown()
    {
        delete function;
    }
};

TEST_F(FunctionTest, SampleTest1){
    EXPECT_CALL(*function, begin()).Times(1);
    function->begin();
}

TEST_F(FunctionTest, SampleTest2){
    EXPECT_CALL(*function, isTrueOrFalse(true)).Times(1);
    function->isTrueOrFalse(true);
}
