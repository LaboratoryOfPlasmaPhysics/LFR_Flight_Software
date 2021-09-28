#include "LFR_TB_utils.h"
#include <stdlib.h>

volatile int Nope_duration = 0;
volatile int Matrix_change_of_basis_128_freq_duration = 0;
volatile int Matrix_change_of_basis_duration = 0;
volatile int ASM_divide_0_duration = 0;
volatile int ASM_divide_1_duration = 0;
volatile int ASM_divide_8_duration = 0;
volatile int ASM_compress_divide_and_mask_duration = 0;
volatile int SM_average_with_clear_duration = 0;
volatile int SM_average_duration = 0;

volatile int done = 0;

volatile float input_matrix[25 * 128];
volatile float output_matrix[25 * 128];
volatile float b_trans[3 * 3 * 2 * 128];
volatile float e_trans[2 * 2 * 2 * 128];
static _Complex float intermediary[25];

volatile float ams_buffer[25 * 128 * 8];
ring_node ring_node_tab_buff[NB_SM_BEFORE_AVF0_F1];
ring_node* ring_node_tab[NB_SM_BEFORE_AVF0_F1];
asm_msg asm_msg_;

rtems_task Init(rtems_task_argument ignored)
{
    (void)ignored;
    /** This is the RTEMS INIT taks, it is the first task launched by the system.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The INIT task create and run all other RTEMS tasks.
     *
     */
#ifndef TSIM
    initCache();
#endif

    fill_matrix(input_matrix, 25 * 128);
    fill_matrix(output_matrix, 25 * 128);
    fill_matrix(b_trans, 3 * 3 * 2 * 128);
    fill_matrix(e_trans, 2 * 2 * 2 * 128);
    fill_matrix(ams_buffer, 25 * 128 * 8);

    for (int i = 0; i < 8; i++)
    {
        ring_node_tab[i] = &ring_node_tab_buff[i];
        ring_node_tab[i]->next = ring_node_tab[(i + 1) % 8];
        ring_node_tab[i]->previous = ring_node_tab[(i + 7) % 8];
        ring_node_tab[i]->buffer_address = (void*)&ams_buffer[i * 25 * 128];
    }

    BENCH(ARG({ ; }), Nope_duration, 10000);

    BENCH(ARG(
              {
                  for (int freq = 0; freq < 128; freq++)
                      Matrix_change_of_basis(intermediary, input_matrix + (freq * 25),
                          b_trans + (freq * 3 * 3 * 2), e_trans + (freq * 2 * 2 * 2),
                          output_matrix + (freq * 25));
              }),
        Matrix_change_of_basis_128_freq_duration, 10);

    BENCH(ARG({
        Matrix_change_of_basis(intermediary, input_matrix, b_trans, e_trans, output_matrix);
    }),
        Matrix_change_of_basis_duration, 1000);

    BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 0., 0, 128); }), ASM_divide_0_duration, 10);

    BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 1., 0, 128); }), ASM_divide_1_duration, 10);

    BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 8., 0, 128); }), ASM_divide_8_duration, 10);

    BENCH(ARG({
        ASM_compress_divide_and_mask(input_matrix, output_matrix, 8, NB_BINS_COMPRESSED_SM_SBM_F0,
            NB_BINS_TO_AVERAGE_ASM_SBM_F0, ASM_F0_INDICE_START, CHANNELF0);
    }),
        ASM_compress_divide_and_mask_duration, 10);

    BENCH(ARG({
        SM_average(input_matrix, output_matrix, ring_node_tab, 0, 0, &asm_msg_, CHANNELF0,
            ASM_F0_INDICE_START, ASM_F0_INDICE_START + ASM_F0_KEEP_BINS);
    }),
        SM_average_with_clear_duration, 10);

    BENCH(ARG({
        SM_average(input_matrix, output_matrix, ring_node_tab, 1, 1, &asm_msg_, CHANNELF0,
            ASM_F0_INDICE_START, ASM_F0_INDICE_START + ASM_F0_KEEP_BINS);
    }),
        SM_average_duration, 10);

#ifndef TSIM
    while (1)
        done = 1;
#else
    printf("done!\n");
    exit(0);
#endif
}
