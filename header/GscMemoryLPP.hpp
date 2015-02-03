#ifndef GSCMEMORY_HPP_
#define GSCMEMORY_HPP_

static unsigned int getCacheControlRegister(){

#ifndef LEON3
#define LEON3
#endif

#ifdef LEON3
    unsigned int cacheControlRegister = 0;
    __asm__ __volatile__("lda [%%g0] 2, %0" : "=r"(cacheControlRegister) : );
    return cacheControlRegister;
#endif
}

static void setCacheControlRegister(unsigned int cacheControlRegister){

#ifdef LEON3
    __asm__ __volatile__("sta %0, [%%g0] 2" : : "r"(cacheControlRegister));
#endif
}


/**
                 * Flush the data cache and the instruction cache.
                 *
                 * @return
                 */
static inline void flushCache() {
    asm("flush");
}


static void enableInstructionCache() {

#ifdef LEON3
    unsigned int cacheControlRegister;
    cacheControlRegister = getCacheControlRegister();
    cacheControlRegister = (cacheControlRegister | 0x3);
    setCacheControlRegister(cacheControlRegister);
#endif
}

static void enableDataCache() {

#ifdef LEON3
    unsigned int cacheControlRegister;
    cacheControlRegister = getCacheControlRegister();
    cacheControlRegister = (cacheControlRegister | 0xc);
    setCacheControlRegister(cacheControlRegister);
#endif
}

static void enableInstructionBurstFetch() {

#ifdef LEON3
    unsigned int cacheControlRegister;
    cacheControlRegister = getCacheControlRegister();
    // set the bit IB to 1
    cacheControlRegister = (cacheControlRegister | 0x10000);
    setCacheControlRegister(cacheControlRegister);
#endif
}

#endif /* GSCMEMORY_HPP_ */
