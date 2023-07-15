#ifndef TEST_FUNCTION_HPP
#define TEST_FUNCTION_HPP

class TestFunction
{
public:
    TestFunction() {}
    virtual ~TestFunction() {}

    virtual void begin()
    {
        ;
    }

    virtual bool isTrueOrFalse(bool input)
    {
        return input;
    }
};

#endif  // TEST_FUNCTION_HPP