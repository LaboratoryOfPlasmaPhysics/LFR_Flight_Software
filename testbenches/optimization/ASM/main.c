#include "LFR_TB_utils.h"

volatile int Matrix_change_of_basis_duration = 0;
volatile int ASM_divide_0_duration = 0;
volatile int ASM_divide_1_duration = 0;
volatile int ASM_divide_8_duration = 0;
volatile int ASM_compress_divide_and_mask_duration = 0;

volatile int done = 0;

volatile float input_matrix[25 * 128];
volatile float output_matrix[25 * 128];
volatile float b_trans[3 * 3 * 2 * 128];
volatile float e_trans[2 * 2 * 2 * 128];

rtems_task Init(rtems_task_argument ignored) {
  (void)ignored;
  /** This is the RTEMS INIT taks, it is the first task launched by the system.
   *
   * @param unused is the starting argument of the RTEMS task
   *
   * The INIT task create and run all other RTEMS tasks.
   *
   */

  initCache();

  fill_matrix(input_matrix, 25 * 128);
  fill_matrix(output_matrix, 25 * 128);
  fill_matrix(b_trans, 3 * 3 * 2 * 128);
  fill_matrix(e_trans, 2 * 2 * 2 * 128);

  BENCH(ARG({
          for (int freq = 0; freq < 128; freq++)
            Matrix_change_of_basis(
                input_matrix + freq * 25, b_trans + freq * 3 * 3 * 2,
                e_trans + freq * 2 * 2 * 2, output_matrix + freq * 25);
        }),
        Matrix_change_of_basis_duration);
  BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 0.); }),
        ASM_divide_0_duration);
  BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 1.); }),
        ASM_divide_1_duration);
  BENCH(ARG({ ASM_divide(input_matrix, output_matrix, 8.); }),
        ASM_divide_8_duration);

  BENCH(ARG({
          ASM_compress_divide_and_mask(
              input_matrix, output_matrix, 8, NB_BINS_COMPRESSED_SM_SBM_F0,
              NB_BINS_TO_AVERAGE_ASM_SBM_F0, ASM_F0_INDICE_START, CHANNELF0);
        }),
        ASM_compress_divide_and_mask_duration);

  while (1)
      done=1;
    ;
}
