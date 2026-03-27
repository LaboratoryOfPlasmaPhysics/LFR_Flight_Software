#include <emscripten.h>
#include <stdint.h>
#include "basic_parameters.h"

EMSCRIPTEN_KEEPALIVE
void bp1_compute(const float* spectral_matrices, uint8_t count, uint8_t* bp1_out) {
    compute_BP1(spectral_matrices, count, bp1_out);
}

EMSCRIPTEN_KEEPALIVE
uint32_t bp1_bytes_per_bin(void) {
    return NB_BYTES_BP1;
}
