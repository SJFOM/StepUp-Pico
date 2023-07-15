#ifndef MOCK_FUNCTION_HPP
#define MOCK_FUNCTION_HPP

#include "gmock/gmock.h"
#include "Libraries/TestLibrary/src/lib/test_function.hpp"

class MockTestFunction : public TestFunction
{
public:
    MOCK_METHOD0(begin, void());
    MOCK_METHOD1(isTrueOrFalse, bool(bool input));
};

#endif  // MOCK_FUNCTION_HPP
