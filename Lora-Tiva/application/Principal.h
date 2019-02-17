#ifndef __PRINCIPAL_H___
#define __PRINCIPAL_H___

#include "Board.h"
#include "LoraDevice.h"

class Principal
{
public:
    Principal(void);
    ~Principal(void);
    void execute(void);

protected:

private:
	Board board;
    LoraDevice* lora_device;
};

#endif // __PRINCIPAL_H___
