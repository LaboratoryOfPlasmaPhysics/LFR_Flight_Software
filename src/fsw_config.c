#include <drvmgr/ambapp_bus.h>

// GRSPW0 resources
struct drvmgr_key grlib_grspw_0n1_res[] = {
    {"txBdCnt", KEY_TYPE_INT, {(unsigned int)50}}, // 7 SWF_F0, 7 SWF_F1, 7 SWF_F2, 7 CWF_F3, 7 CWF_F1 ou 7 CWF_F2
    	{"rxBdCnt", KEY_TYPE_INT, {(unsigned int)10}},
    	{"txDataSize", KEY_TYPE_INT, {(unsigned int)4096}},
    	{"txHdrSize", KEY_TYPE_INT, {(unsigned int)20+12}}, // 12 is for the auxiliary header, when needed
    	{"rxPktSize", KEY_TYPE_INT, {(unsigned int)248+4}},
    	KEY_EMPTY
};

// If RTEMS_DRVMGR_STARTUP is defined we override the "weak defaults" that is defined by the LEON3 BSP.

struct drvmgr_bus_res grlib_drv_resources = {
    .next = NULL,
    .resource = {
        {DRIVER_AMBAPP_GAISLER_GRSPW_ID, 0, &grlib_grspw_0n1_res[0]},
//        {DRIVER_AMBAPP_GAISLER_APBUART_ID, 0, &grlib_drv_res_apbuart0[0]},
//        {DRIVER_AMBAPP_GAISLER_APBUART_ID, 1, &grlib_drv_res_apbuart1[0]},
        RES_EMPTY /* Mark end of device resource array */
    }
};


