#include <stddef.h>

#include "lfr_common_headers/fsw_params.h"
#include "fsw_debug.h"
#include "mock_globals.h"

filterPar_t filterPar = { 0 };
Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet = { 0 };
fbins_masks_t fbins_masks = { 0 };

volatile int sm_f0[NB_RING_NODES_SM_F0 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };
volatile int sm_f1[NB_RING_NODES_SM_F1 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };
volatile int sm_f2[NB_RING_NODES_SM_F2 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };

ring_node sm_ring_f0[NB_RING_NODES_SM_F0] = { { 0 } };
ring_node sm_ring_f1[NB_RING_NODES_SM_F1] = { { 0 } };
ring_node sm_ring_f2[NB_RING_NODES_SM_F2] = { { 0 } };

ring_node* getRingNodeForAveraging(unsigned char frequencyChannel)
{
    ring_node* node;

    node = NULL;
    switch (frequencyChannel)
    {
        case CHANNELF0:
            node = sm_ring_f0;
            break;
        case CHANNELF1:
            node = sm_ring_f1;
            break;
        case CHANNELF2:
            node = sm_ring_f2;
            break;
        default:
            break;
    }

    return node;
}


void init_ring(
    ring_node ring[], unsigned char nbNodes, volatile int buffer[], unsigned int bufferSize)
{
    DEBUG_CHECK_PTR(ring);
    DEBUG_CHECK_PTR(buffer);
    unsigned char i;

    //***************
    // BUFFER ADDRESS
    for (i = 0; i < nbNodes; i++)
    {
        ring[i].coarseTime = INT32_ALL_F;
        ring[i].fineTime = INT32_ALL_F;
        ring[i].sid = INIT_CHAR;
        ring[i].status = INIT_CHAR;
        ring[i].buffer_address = (void*)&(buffer[i * bufferSize]);
    }

    //*****
    // NEXT
    ring[nbNodes - 1].next = (ring_node*)&ring[0];
    for (i = 0; i < nbNodes - 1; i++)
    {
        ring[i].next = (ring_node*)&ring[i + 1];
    }

    //*********
    // PREVIOUS
    ring[0].previous = (ring_node*)&ring[nbNodes - 1];
    for (i = 1; i < nbNodes; i++)
    {
        ring[i].previous = (ring_node*)&ring[i - 1];
    }
}

void lfr_test_init()
{
    init_ring(sm_ring_f0, NB_RING_NODES_SM_F0, sm_f0, TOTAL_SIZE_SM);
    init_ring(sm_ring_f1, NB_RING_NODES_SM_F1, sm_f1, TOTAL_SIZE_SM);
    init_ring(sm_ring_f2, NB_RING_NODES_SM_F2, sm_f2, TOTAL_SIZE_SM);
}
