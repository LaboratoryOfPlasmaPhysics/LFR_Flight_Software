#include <ccsds_types.h>
#include <drvmgr/ambapp_bus.h>
#include <drvmgr/drvmgr.h>

// GRSPW0 resources
struct drvmgr_key grlib_grspw_0n1_res[] = {
    { "txBdCnt", KEY_TYPE_INT,
        { (unsigned int)TXBDCNT } }, // 7 SWF_F0, 7 SWF_F1, 7 SWF_F2, 7 CWF_F3, 7 CWF_F1 ou 7 CWF_F2
    { "rxBdCnt", KEY_TYPE_INT, { (unsigned int)RXBDCNT } },
    { "txDataSize", KEY_TYPE_INT, { (unsigned int)TXDATASIZE } },
    { "txHdrSize", KEY_TYPE_INT, { (unsigned int)TXHDRSIZE } },
    { "rxPktSize", KEY_TYPE_INT, { (unsigned int)RXPKTSIZE } }, KEY_EMPTY
};

struct drvmgr_bus_res grlib_drv_resources = { NULL,
    {
        { DRIVER_AMBAPP_GAISLER_GRSPW_ID, 0, &grlib_grspw_0n1_res[0] },
        { DRIVER_AMBAPP_GAISLER_GRSPW_ID, 1, &grlib_grspw_0n1_res[0] },
        RES_EMPTY /*  Mark end of resource array */
    } };
