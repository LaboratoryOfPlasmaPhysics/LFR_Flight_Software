#include "grspw.h"

int grspw_set_ie(unsigned char value, unsigned int* ctrlReg)
{
    // IE = bit 3
    // Interrupt Enable

    int ret = 0;

    if (value == 0)
    {
        *ctrlReg = *ctrlReg & 0xfffffff7;
    }
    else if (value == 1)
    {
        *ctrlReg = *ctrlReg | 0x00000008;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

int grspw_set_tq(unsigned char value, unsigned int* ctrlReg)
{
    // TQ = bit 8
    // Tick-out IRQ

    int ret = 0;

    if (value == 0)
    {
        *ctrlReg = *ctrlReg & 0xfffffeff;
    }
    else if (value == 1)
    {
        *ctrlReg = *ctrlReg | 0x00000100;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

int grspw_set_tr(unsigned char value, unsigned int* ctrlReg)
{
    // TR = bit 11
    // Enable timecode reception

    int ret = 0;

    if (value == 0)
    {
        *ctrlReg = *ctrlReg & 0xfffff7ff;
    }
    else if (value == 1)
    {
        *ctrlReg = *ctrlReg | 0x00000800;
    }
    else
    {
        ret = -1;
    }

    return ret;
}
