/**
 * @file ControlInterface.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONTROL_INTERFACE_H_
#define CONTROL_INTERFACE_H_

class ControlInterface {
public:
    virtual bool init() = 0;
    virtual void processJob() = 0;
protected:
private:
    // virtual void* callBack
    unsigned interface_count;
};

#endif // CONTROL_INTERFACE_H_