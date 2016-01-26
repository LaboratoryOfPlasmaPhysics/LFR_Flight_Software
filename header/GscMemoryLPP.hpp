#ifndef GSCMEMORY_HPP_
#define GSCMEMORY_HPP_

#ifndef LEON3
#define LEON3
#endif

#define REGS_ADDR_PLUGANDPLAY   0xFFFFF000
#define ASR16_REG_ADDRESS       0x90400040  // Ancillary State Register 16 = Register protection control register (FT only)

#define DEVICEID_LEON3      0x003
#define DEVICEID_LEON3FT    0x053
#define VENDORID_GAISLER    0x01

// CCR
#define POS_ITE             12
#define COUNTER_FIELD_ITE   0x00003000      // 0000 0000 0000 0000 0011 0000 0000 0000
#define COUNTER_MASK_ITE    0xffffcfff      // 1111 1111 1111 1111 1100 1111 1111 1111
#define POS_IDE             10
#define COUNTER_FIELD_IDE   0x00000c00      // 0000 0000 0000 0000 0000 1100 0000 0000
#define COUNTER_MASK_IDE    0xfffff3ff      // 1111 1111 1111 1111 1111 0011 1111 1111
//
#define POS_DTE             8
#define COUNTER_FIELD_DTE   0x00000300      // 0000 0000 0000 0000 0000 0011 0000 0000
#define COUNTER_MASK_DTE    0xfffffcff      // 1111 1111 1111 1111 1111 1100 1111 1111
#define POS_DDE             6
#define COUNTER_FIELD_DDE   0x000000c0      // 0000 0000 0000 0000 0000 0000 1100 0000
#define COUNTER_MASK_DDE    0xffffff3f      // 1111 1111 1111 1111 1111 1111 0011 1111

// ASR16
#define POS_FPRF            27
#define COUNTER_FIELD_FPRF  0x38000000      // 0011 1000 0000 0000 0000 0000 0000 0000
#define COUNTER_MASK_FPRF   0xc7ffffff      // 1100 0111 1111 1111 1111 1111 1111 1111
#define POS_IURF            11
#define COUNTER_FIELD_IURF  0x00003800      // 0000 0000 0000 0000 0011 1000 0000 0000
#define COUNTER_MASK_IURF   0xffffc7ff      // 1111 1111 1111 1111 1100 0111 1111 1111

volatile unsigned int *asr16Ptr = (volatile unsigned int *) ASR16_REG_ADDRESS;

static inline void flushCache()
{
    /**
    * Flush the data cache and the instruction cache.
    *
    * @param void
    *
    * @return void
    */

    asm("flush");
}

//***************************
// CCR Cache control register

static unsigned int CCR_getValue()
{
    unsigned int cacheControlRegister = 0;
    __asm__ __volatile__("lda [%%g0] 2, %0" : "=r"(cacheControlRegister) : );
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
    // Indicates the current data cache state according to the following: X0 = disabled, 01 = frozen, 11 = enabled.
    unsigned int cacheControlRegister;
    cacheControlRegister = CCR_getValue();
    cacheControlRegister = (cacheControlRegister | 0x3);
    CCR_setValue(cacheControlRegister);
}

static void CCR_enableDataCache()
{
    // [3:2] Data Cache state (DCS)
    // Indicates the current data cache state according to the following: X0 = disabled, 01 = frozen, 11 = enabled.
    unsigned int cacheControlRegister;
    cacheControlRegister = CCR_getValue();
    cacheControlRegister = (cacheControlRegister | 0xc);
    CCR_setValue(cacheControlRegister);
}

static void CCR_faultTolerantScheme()
{
    // [20:19] FT scheme (FT) - “00” = no FT, “01” = 4-bit checking implemented
    unsigned int cacheControlRegister;
    unsigned int *plugAndPlayRegister;
    unsigned int vendorId;
    unsigned int deviceId;

    plugAndPlayRegister = (unsigned int*) REGS_ADDR_PLUGANDPLAY;
    vendorId = ( (*plugAndPlayRegister) & 0xff000000 ) >> 24;
    deviceId = ( (*plugAndPlayRegister) & 0x00fff000 ) >> 12;

    if( (vendorId == VENDORID_GAISLER) & (deviceId ==DEVICEID_LEON3FT) )
    {
        PRINTF("in faultTolerantScheme *** Leon3FT detected, configure the CCR FT bits\n");
        cacheControlRegister = CCR_getValue();
        cacheControlRegister = (cacheControlRegister | 0xc);
        CCR_setValue(cacheControlRegister);
    }
    else
    {
        PRINTF("in faultTolerantScheme *** not a Leon3FT, no need to configure the CCR FT bits\n");
        PRINTF2("                       *** vendorID = 0x%x, deviceId = 0x%x\n", vendorId, deviceId);
    }
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

static void CCR_getInstructionAndDataErrorCounters( unsigned int* instructionErrorCounter, unsigned int* dataErrorCounter )
{
    // [13:12] Instruction Tag Errors (ITE) - Number of detected parity errors in the instruction tag cache.
    // Only available if fault-tolerance is enabled (FT field in this register is non-zero).
    // [11:10] Instruction Data Errors (IDE) - Number of detected parity errors in the instruction data cache.
    // Only available if fault-tolerance is enabled (FT field in this register is non-zero).

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
    *dataErrorCounter        = dTE + dDE;

    // reset counters
    cacheControlRegister = cacheControlRegister
            & COUNTER_FIELD_ITE
            & COUNTER_FIELD_IDE
            & COUNTER_FIELD_DTE
            & COUNTER_FIELD_DDE;

    CCR_setValue(cacheControlRegister);
}

//*******************************************
// ASR16 Register protection control register

static void ASR16_get_FPRF_IURF_ErrorCounters( unsigned int* fprfErrorCounter, unsigned int* iurfErrorCounter)
{
    /** This function is used to retrieve the integer unit register file error counter and the floating point unit
     *  register file error counter
     *
     * @return void
     *
     * [29:27] FP RF error counter - Number of detected parity errors in the FP register file.
     * [13:11] IU RF error counter - Number of detected parity errors in the IU register file.
     *
     */

    unsigned int asr16;

    asr16 = *asr16Ptr;
    *fprfErrorCounter = ( asr16 & COUNTER_FIELD_FPRF ) >> POS_FPRF;
    *iurfErrorCounter = ( asr16 & COUNTER_FIELD_IURF ) >> POS_IURF;

    // reset the counter to 0
    asr16 = asr16
            & COUNTER_MASK_FPRF
            & COUNTER_FIELD_IURF;

    *asr16Ptr = asr16;
}

#endif /* GSCMEMORY_HPP_ */
