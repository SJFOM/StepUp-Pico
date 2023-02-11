#ifndef TMC_CONTROL_H_
#define TMC_CONTROL_H_

#include "../../../Interfaces/ControlInterface.h"
#include "../../../Libraries/TMC_API/ic/TMC2300.h"
#include "../../../Libraries/TMC_API/helpers/Config.h"

class TMCControl: ControlInterface
{
public:
    TMCControl();
    ~TMCControl();
    bool init();
    void processJob();
protected:
private:
    unsigned interface_count;
};

#endif // TMC_CONTROL_H_