#ifndef GSCMEMORY_HPP_
#define GSCMEMORY_HPP_

#include "lfr_common_headers/fsw_params.h"
#include "fsw_debug.h"

#ifndef LEON3
    #define LEON3
#endif

#define REGS_ADDR_PLUGANDPLAY 0xFFFFF000
#define ASR16_REG_ADDRESS                                                                          \
    0x90400040 // Ancillary State Register 16 = Register protection control register (FT only)

#define DEVICEID_LEON3   0x003
#define DEVICEID_LEON3FT 0x053
#define VENDORID_GAISLER 0x01

// CCR
#define POS_FT 19
//
#define POS_ITE           12
#define COUNTER_FIELD_ITE 0x00003000 // 0000 0000 0000 0000 0011 0000 0000 0000
#define COUNTER_MASK_ITE  0xffffcfff // 1111 1111 1111 1111 1100 1111 1111 1111
#define POS_IDE           10
#define COUNTER_FIELD_IDE 0x00000c00 // 0000 0000 0000 0000 0000 1100 0000 0000
#define COUNTER_MASK_IDE  0xfffff3ff // 1111 1111 1111 1111 1111 0011 1111 1111
//
#define POS_DTE           8
#define COUNTER_FIELD_DTE 0x00000300 // 0000 0000 0000 0000 0000 0011 0000 0000
#define COUNTER_MASK_DTE  0xfffffcff // 1111 1111 1111 1111 1111 1100 1111 1111
#define POS_DDE           6
#define COUNTER_FIELD_DDE 0x000000c0 // 0000 0000 0000 0000 0000 0000 1100 0000
#define COUNTER_MASK_DDE  0xffffff3f // 1111 1111 1111 1111 1111 1111 0011 1111

// ASR16
#define POS_FPFTID 30
#define POS_FPRF   27
#define POS_FDI    16 // FP RF protection enable/disable
#define POS_IUFTID 14
#define POS_IURF   11
#define POS_IDI    0 // IU RF protection enable/disable

#define COUNTER_FIELD_FPRF 0x38000000 // 0011 1000 0000 0000 0000 0000 0000 0000
#define COUNTER_MASK_FPRF  0xc7ffffff // 1100 0111 1111 1111 1111 1111 1111 1111

#define COUNTER_FIELD_IURF 0x00003800 // 0000 0000 0000 0000 0011 1000 0000 0000
#define COUNTER_MASK_IURF  0xffffc7ff // 1111 1111 1111 1111 1100 0111 1111 1111

volatile unsigned int* asr16Ptr = (volatile unsigned int*)ASR16_REG_ADDRESS;


//***************************
// CCR Cache control register

static unsigned int CCR_getValue()
{
    unsigned int cacheControlRegister = 0;
    __asm__ __volatile__("lda [%%g0] 2, %0" : "=r"(cacheControlRegister) :);
    return cacheControlRegister;
}

static void CCR_setValue(unsigned int cacheControlRegister)
{
    __asm__ __volatile__("sta %0, [%%g0] 2" : : "r"(cacheControlRegister));
}

static void CCR_resetCacheControlRegister()
{
    unsigned int cacheControlRegister;
    cacheControlRegister = 0x00;
    CCR_setValue(cacheControlRegister);
}

static void CCR_enableInstructionCache()
{
    // [1:0] Instruction Cache state (ICS)
    // Indicates the current data cache state according to the following: X0 = disabled, 01 =
    // frozen, 11 = enabled.
    unsigned int cacheControlRegister;
    cacheControlRegister = CCR_getValue();
    cacheControlRegister = (cacheControlRegister | 0x3);
    CCR_setValue(cacheControlRegister);
}

static void CCR_enableDataCache()
{
    // [3:2] Data Cache state (DCS)
    // Indicates the current data cache state according to the following: X0 = disabled, 01 =
    // frozen, 11 = enabled.
    unsigned int cacheControlRegister;
    cacheControlRegister = CCR_getValue();
    cacheControlRegister = (cacheControlRegister | 0xc);
    CCR_setValue(cacheControlRegister);
}

static void CCR_enableInstructionBurstFetch()
{
    // [16] Instruction burst fetch (IB). This bit enables burst fill during instruction fetch.
    unsigned int cacheControlRegister;
    cacheControlRegister = CCR_getValue();
    // set the bit IB to 1
    cacheControlRegister = (cacheControlRegister | 0x10000);
    CCR_setValue(cacheControlRegister);
}

void CCR_getInstructionAndDataErrorCounters(
    unsigned int* instructionErrorCounter, unsigned int* dataErrorCounter)
{
    // [13:12] Instruction Tag Errors (ITE) - Number of detected parity errors in the instruction
    // tag cache. Only available if fault-tolerance is enabled (FT field in this register is
    // non-zero). [11:10] Instruction Data Errors (IDE) - Number of detected parity errors in the
    // instruction data cache. Only available if fault-tolerance is enabled (FT field in this
    // register is non-zero).

    unsigned int cacheControlRegister;
    unsigned int iTE;
    unsigned int iDE;
    unsigned int dTE;
    unsigned int dDE;

    cacheControlRegister = CCR_getValue();
    iTE = (cacheControlRegister & COUNTER_FIELD_ITE) >> POS_ITE;
    iDE = (cacheControlRegister & COUNTER_FIELD_IDE) >> POS_IDE;
    dTE = (cacheControlRegister & COUNTER_FIELD_DTE) >> POS_DTE;
    dDE = (cacheControlRegister & COUNTER_FIELD_DDE) >> POS_DDE;

    *instructionErrorCounter = iTE + iDE;
    *dataErrorCounter = dTE + dDE;

    // reset counters
    cacheControlRegister = cacheControlRegister & COUNTER_FIELD_ITE & COUNTER_FIELD_IDE
        & COUNTER_FIELD_DTE & COUNTER_FIELD_DDE;

    CCR_setValue(cacheControlRegister);
}

//*******************************************
// ASR16 Register protection control register

static void ASR16_resetRegisterProtectionControlRegister()
{
    *asr16Ptr = 0x00;
}

void ASR16_get_FPRF_IURF_ErrorCounters(
    unsigned int* fprfErrorCounter, unsigned int* iurfErrorCounter)
{
    /** This function is used to retrieve the integer unit register file error counter and the
     * floating point unit register file error counter
     *
     * @return void
     *
     * [29:27] FP RF error counter - Number of detected parity errors in the FP register file.
     * [13:11] IU RF error counter - Number of detected parity errors in the IU register file.
     *
     */

    unsigned int asr16;

    asr16 = *asr16Ptr;
    *fprfErrorCounter = (asr16 & COUNTER_FIELD_FPRF) >> POS_FPRF;
    *iurfErrorCounter = (asr16 & COUNTER_FIELD_IURF) >> POS_IURF;

    // reset the counter to 0
    asr16 = asr16 & COUNTER_MASK_FPRF & COUNTER_FIELD_IURF;

    *asr16Ptr = asr16;
}

static void faultTolerantScheme()
{
    // [20:19] FT scheme (FT) - “00” = no FT, “01” = 4-bit checking implemented
    unsigned int cacheControlRegister;
    const unsigned int* const plugAndPlayRegister=(unsigned int*)REGS_ADDR_PLUGANDPLAY;
    const unsigned int vendorId = ((*plugAndPlayRegister) & 0xff000000) >> 24;
    const unsigned int deviceId = ((*plugAndPlayRegister) & 0x00fff000) >> 12;

    cacheControlRegister = CCR_getValue();

    if ((vendorId == VENDORID_GAISLER) & (deviceId == DEVICEID_LEON3FT))
    {
        LFR_PRINTF("in faultTolerantScheme *** Leon3FT detected\n");
        LFR_PRINTF(
            "                       *** vendorID = 0x%x, deviceId = 0x%x\n", vendorId, deviceId);
        LFR_PRINTF("ASR16 IU RF protection, bit 0  (IDI) is: 0x%x (0 => protection enabled)\n",
            (*asr16Ptr >> POS_IDI) & 1);
        LFR_PRINTF("ASR16 FP RF protection, bit 16 (FDI) is: 0x%x (0 => protection enabled)\n",
            (*asr16Ptr >> POS_FDI) & 1);
        LFR_PRINTF("ASR16 IU FT ID bits [15:14] is: 0x%x (2 => 8-bit parity without restart)\n",
            (*asr16Ptr >> POS_IUFTID) & 0x3);
        LFR_PRINTF("ASR16 FP FT ID bits [31:30] is: 0x%x (1 => 4-bit parity with restart)\n",
            (*asr16Ptr >> POS_FPFTID) & 0x03);
        LFR_PRINTF("CCR   FT bits [20:19] are: 0x%x (1 => 4-bit parity with restart)\n",
            (cacheControlRegister >> POS_FT) & 0x3);

        // CCR The FFT bits are just read, the FT scheme is set to “01” = 4-bit checking implemented
        // by default

        // ASR16 Ancillary State Register configuration (Register protection control register)
        // IU RF protection is set by default, bit 0 IDI = 0
        // FP RF protection is set by default, bit 16 FDI = 0
    }
    else
    {
        LFR_PRINTF("in faultTolerantScheme *** Leon3FT not detected\n");
        LFR_PRINTF(
            "                       *** vendorID = 0x%x, deviceId = 0x%x\n", vendorId, deviceId);
    }
}

#endif /* GSCMEMORY_HPP_ */
