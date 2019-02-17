#include "Principal.h"
#include "LoraDevice.h"
#include "LoraTracker.h"
#include "LoraBase.h"
#include "../config.h"

Principal::Principal(void)
{
    return;
}

Principal::~Principal(void)
{
    return;
}

void Principal::execute(void)
{
#if defined(DEVICE_MODE_BASE)
    LoraBase lora_device { &(this->board) };
#elif defined (DEVICE_MODE_TRACKER)
    LoraTracker lora_device { &(this->board) };
#else
    #error "Device mode not defined"
#endif

    this->lora_device = (LoraDevice *)&lora_device;

    this->board.init();
    this->lora_device->start();

    return;
}
