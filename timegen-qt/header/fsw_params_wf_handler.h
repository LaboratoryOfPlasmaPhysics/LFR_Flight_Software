#ifndef FSW_PARAMS_WF_HANDLER_H
#define FSW_PARAMS_WF_HANDLER_H

#define WFRM_BUFFER 8128    // (NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK) + TIME_OFFSET + 62
                            // (2688 * 3 ) + 2 + 62 = 8128 = 0X1FC0
                            //  8128 * 4 = 32512 = 0x7F00

#endif // FSW_PARAMS_WF_HANDLER_H
