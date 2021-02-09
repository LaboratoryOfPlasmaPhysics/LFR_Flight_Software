fsw_inc = include_directories(
    '../header',
    '../header/lfr_common_headers',
    '../header/processing',
    '../LFR_basic-parameters',
    '../src'
    )

fsw_srcs = files(
    'wf_handler.c',
    'tc_handler.c',
    'fsw_misc.c',
    'fsw_init.c',
    'fsw_globals.c',
    'fsw_spacewire.c',
    'tc_load_dump_parameters.c',
    'tm_lfr_tc_exe.c',
    'tc_acceptance.c',
    'processing/fsw_processing.c',
    'processing/avf0_prc0.c',
    'processing/avf1_prc1.c',
    'processing/avf2_prc2.c',
    'processing/ASM/spectralmatrices.c',
    'lfr_cpu_usage_report.c'
)

fsw_extra_files = files(
    '../header/wf_handler.h',
    '../header/tc_handler.h',
    '../header/grlib_regs.h',
    '../header/fsw_misc.h',
    '../header/fsw_init.h',
    '../header/fsw_spacewire.h',
    '../header/tc_load_dump_parameters.h',
    '../header/tm_lfr_tc_exe.h',
    '../header/tc_acceptance.h',
    '../header/processing/fsw_processing.h',
    '../header/processing/avf0_prc0.h',
    '../header/processing/avf1_prc1.h',
    '../header/processing/avf2_prc2.h',
    '../header/processing/ASM/spectralmatrices.h',
    '../header/fsw_params_wf_handler.h',
    '../header/lfr_cpu_usage_report.h',
    '../header/lfr_common_headers/ccsds_types.h',
    '../header/lfr_common_headers/fsw_params.h',
    '../header/lfr_common_headers/fsw_params_nb_bytes.h',
    '../header/lfr_common_headers/fsw_params_processing.h',
    '../header/lfr_common_headers/tm_byte_positions.h',
    '../LFR_basic-parameters/basic_parameters.h',
    '../LFR_basic-parameters/basic_parameters_params.h',
    '../header/GscMemoryLPP.hpp'
)

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : false)

c_args = []
link_args = ['-Xlinker', '-Map=fsw-@0@.map'.format(fsw_version)]
if get_option('fix-b2bst')
    c_args += '-mfix-b2bst'
    link_args += '-mfix-b2bst'
endif

fsw = executable('fsw', fsw_srcs,
                include_directories: fsw_inc,
                c_args: c_args,
                link_args: link_args,
                dependencies:[lfr_bp_dep, m_dep],
                extra_files: fsw_extra_files)


objcopy = find_program('objcopy')
objdump = find_program('objdump')
size = find_program('size')
b2bst_scan = find_program('../sparc/leon3ft-b2bst-scan_wrapper')

data_srec = custom_target('data_srec',
  input : fsw,
  output : 'RpwLfrApp_XXXX_data_rev-@0@.srec'.format(fsw_version),
  command : [objcopy, '-j', '.data', '-F', 'srec', '@INPUT@', '@OUTPUT@'
  ],
  build_by_default: true
)

text_srec = custom_target('text_srec',
  input : fsw,
  output : 'RpwLfrApp_XXXX_text_rev-@0@.srec'.format(fsw_version),
  command : [objcopy, '-j', '.text', '-F', 'srec', '@INPUT@', '@OUTPUT@'
  ],
  build_by_default: true
)

if get_option('fix-b2bst')
    check_b2bst = custom_target('check_b2bst',
      input : fsw,
      output : 'check_b2bst.log',
      command : [b2bst_scan, objdump, '@INPUT@', '@OUTPUT@'],
      build_by_default: true
    )
endif
