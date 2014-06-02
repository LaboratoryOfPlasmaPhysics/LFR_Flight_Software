#ifndef FSW_PROCESSING_H_INCLUDED
#define FSW_PROCESSING_H_INCLUDED

#include <rtems.h>
#include <grspw.h>
#include <math.h>
#include <stdlib.h> // abs() is in the stdlib
#include <stdio.h>  // printf()
#include <math.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"

typedef struct ring_node_sm
{
    struct ring_node_sm *previous;
    struct ring_node_sm *next;
    int buffer_address;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
} ring_node_sm;

typedef struct ring_node_asm
{
    struct ring_node_asm *next;
    float  matrix[ TOTAL_SIZE_SM ];
    unsigned int status;
} ring_node_asm;

typedef struct
{
    Header_TM_LFR_SCIENCE_BP_t header;
    unsigned char data[ 30 * 22 ];   // MAX size is 22 * 30 [TM_LFR_SCIENCE_BURST_BP2_F1]
} bp_packet;

typedef struct
{
    Header_TM_LFR_SCIENCE_BP_with_spare_t header;
    unsigned char data[ 9 * 13 ];   // only for TM_LFR_SCIENCE_NORMAL_BP1_F0 and F1
} bp_packet_with_spare;

typedef struct
{
    ring_node_asm *norm;
    ring_node_asm *burst_sbm;
    rtems_event_set event;
    unsigned int coarseTime;
    unsigned int fineTime;
} asm_msg;

extern volatile int sm_f0[ ];
extern volatile int sm_f1[ ];
extern volatile int sm_f2[ ];

// parameters
extern struct param_local_str param_local;

// registers
extern time_management_regs_t *time_management_regs;
extern spectral_matrix_regs_t *spectral_matrix_regs;

extern rtems_name  misc_name[5];
extern rtems_id    Task_id[20];         /* array of task ids */

// ISR
rtems_isr spectral_matrices_isr( rtems_vector_number vector );
rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector );

//******************
// Spectral Matrices
void reset_nb_sm( void );
// SM
void SM_init_rings( void );
void SM_reset_current_ring_nodes( void );
void SM_generic_init_ring(ring_node_sm *ring, unsigned char nbNodes, volatile int sm_f[] );
// ASM
void ASM_generic_init_ring(ring_node_asm *ring, unsigned char nbNodes );
void ASM_init_header( Header_TM_LFR_SCIENCE_ASM_t *header);
void ASM_send(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id);

//*****************
// Basic Parameters

void BP_reset_current_ring_nodes( void );
void BP_init_header( Header_TM_LFR_SCIENCE_BP_t *header,
                     unsigned int apid, unsigned char sid,
                     unsigned int packetLength , unsigned char blkNr);
void BP_init_header_with_spare( Header_TM_LFR_SCIENCE_BP_with_spare_t *header,
                                unsigned int apid, unsigned char sid,
                                unsigned int packetLength, unsigned char blkNr );
void BP_send( char *data,
              rtems_id queue_id ,
              unsigned int nbBytesToSend , unsigned int sid );

//******************
// general functions
void reset_spectral_matrix_regs( void );
void set_time(unsigned char *time, unsigned char *timeInBuffer );
unsigned long long int get_acquisition_time( unsigned char *timePtr );
void close_matrix_actions(unsigned int *nb_sm, unsigned int nb_sm_before_avf, rtems_id task_id,
                           ring_node_sm *node_for_averaging, ring_node_sm *ringNode);

extern rtems_status_code get_message_queue_id_prc1( rtems_id *queue_id );
extern rtems_status_code get_message_queue_id_prc2( rtems_id *queue_id );

//***************************************
// DEFINITIONS OF STATIC INLINE FUNCTIONS
static inline void SM_average( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int nbAverageNormF0, unsigned int nbAverageSBM1F0 );
static inline void ASM_reorganize_and_divide(float *averaged_spec_mat, float *averaged_spec_mat_reorganized,
                               float divider );
static inline void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat,
                                  float divider,
                                  unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage , unsigned char ASMIndexStart);
static inline void ASM_convert(volatile float *input_matrix, char *output_matrix);

void SM_average( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int nbAverageNormF0, unsigned int nbAverageSBM1F0 )
{
    float sum;
    unsigned int i;

    for(i=0; i<TOTAL_SIZE_SM; i++)
    {
        sum = ( (int *) (ring_node_tab[0]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[1]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[2]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[3]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[4]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[5]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[6]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[7]->buffer_address) ) [ i ];

        if ( (nbAverageNormF0 == 0) && (nbAverageSBM1F0 == 0) )
        {
            averaged_spec_mat_f0[ i ] = sum;
            averaged_spec_mat_f1[ i ] = sum;
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 != 0) )
        {
            averaged_spec_mat_f0[ i ] = ( averaged_spec_mat_f0[  i ] + sum );
            averaged_spec_mat_f1[ i ] = ( averaged_spec_mat_f1[  i ] + sum );
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 == 0) )
        {
            averaged_spec_mat_f0[ i ] = ( averaged_spec_mat_f0[ i ] + sum );
            averaged_spec_mat_f1[ i ] = sum;
        }
        else
        {
            PRINTF2("ERR *** in SM_average *** unexpected parameters %d %d\n", nbAverageNormF0, nbAverageSBM1F0)
        }
    }
}

void ASM_reorganize_and_divide( float *averaged_spec_mat, float *averaged_spec_mat_reorganized, float divider )
{
    int frequencyBin;
    int asmComponent;
    unsigned int offsetAveragedSpecMatReorganized;
    unsigned int offsetAveragedSpecMat;

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++ )
        {
            offsetAveragedSpecMatReorganized =
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetAveragedSpecMat            =
                    asmComponent * NB_BINS_PER_SM
                    + frequencyBin;
            averaged_spec_mat_reorganized[offsetAveragedSpecMatReorganized  ] =
                    averaged_spec_mat[ offsetAveragedSpecMat ] / divider;
        }
    }
}

void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat , float divider,
                                 unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage, unsigned char ASMIndexStart )
{
    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int k;

    // build data
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix; frequencyBin++ )
        {
            offsetCompressed =  // NO TIME OFFSET
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetASM =         // NO TIME OFFSET
                    asmComponent * NB_BINS_PER_SM
                    + ASMIndexStart
                    + frequencyBin * nbBinsToAverage;
            compressed_spec_mat[ offsetCompressed ] = 0;
            for ( k = 0; k < nbBinsToAverage; k++ )
            {
                compressed_spec_mat[offsetCompressed ] =
                        ( compressed_spec_mat[ offsetCompressed ]
                        + averaged_spec_mat[ offsetASM + k ] ) / (divider * nbBinsToAverage);
            }
        }
    }
}

void ASM_convert( volatile float *input_matrix, char *output_matrix)
{
    unsigned int frequencyBin;
    unsigned int asmComponent;
    char * pt_char_input;
    char * pt_char_output;
    unsigned int offsetInput;
    unsigned int offsetOutput;

    pt_char_input = (char*) &input_matrix;
    pt_char_output = (char*) &output_matrix;

    // convert all other data
    for( frequencyBin=0; frequencyBin<NB_BINS_PER_SM; frequencyBin++)
    {
        for ( asmComponent=0; asmComponent<NB_VALUES_PER_SM; asmComponent++)
        {
            offsetInput  =       (frequencyBin*NB_VALUES_PER_SM) + asmComponent   ;
            offsetOutput = 2 * ( (frequencyBin*NB_VALUES_PER_SM) + asmComponent ) ;
            pt_char_input =  (char*) &input_matrix [ offsetInput  ];
            pt_char_output = (char*) &output_matrix[ offsetOutput ];
            pt_char_output[0] = pt_char_input[0];   // bits 31 downto 24 of the float
            pt_char_output[1] = pt_char_input[1];   // bits 23 downto 16 of the float
        }
    }
}

#endif // FSW_PROCESSING_H_INCLUDED
