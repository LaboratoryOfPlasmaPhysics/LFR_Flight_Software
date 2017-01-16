/** Global variables used by the processing functions.
 *
 * @file
 * @author P. LEROY
 *
 */

// TOTAL = 32 coefficients * 4 = 128 octets * 3 * 12 = 4608 octets
// SX 12 coefficients
float K14_sx_re = 1;
float K14_sx_im = 1;
float K15_sx_re = 1;
float K15_sx_im = 1;
float K24_sx_re = 1;
float K24_sx_im = 1;
float K25_sx_re = 1;
float K25_sx_im = 1;
float K34_sx_re = 1;
float K34_sx_im = 1;
float K35_sx_re = 1;
float K35_sx_im = 1;
// NY 8 coefficients
float K24_ny_re = 1;
float K24_ny_im = 1;
float K25_ny_re = 1;
float K25_ny_im = 1;
float K34_ny_re = 1;
float K34_ny_im = 1;
float K35_ny_re = 1;
float K35_ny_im = 1;
// NZ 8 coefficients
float K24_nz_re = 1;
float K24_nz_im = 1;
float K25_nz_re = 1;
float K25_nz_im = 1;
float K34_nz_re = 1;
float K34_nz_im = 1;
float K35_nz_re = 1;
float K35_nz_im = 1;
// PE 4 coefficients
float K44_pe = 1;
float K55_pe = 1;
float K45_pe_re = 1;
float K45_pe_im = 1;

#define ALPHA_M (M_PI / 4)

float Alpha_M = ALPHA_M;
