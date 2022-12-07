fsw_inc = include_directories(
    '../header',
    '../header/lfr_common_headers',
    '../header/processing',
    '../LFR_basic-parameters',
    '.'
    )

fsw_srcs = files(
    'fsw_misc.c',
    'fsw_init.c',
    'fsw_globals.c',
    'fsw_spacewire.c',
    'fsw_watchdog.c',
    'fsw_housekeeping.c',
    'hw/timer.c',
    'hw/uart.c',
    'hw/wf_handler.c',
    'tc_tm/tc_load_dump_parameters.c',
    'tc_tm/tm_lfr_tc_exe.c',
    'tc_tm/tc_acceptance.c',
    'tc_tm/tc_handler.c',
    'mitigations/PAS_filtering.c',
    'mitigations/reaction_wheel_filtering.c',
    'processing/fsw_processing.c',
    'processing/calibration_matrices.c',
    'processing/avf0_prc0.c',
    'processing/avf1_prc1.c',
    'processing/avf2_prc2.c',
    'processing/ASM/spectralmatrices.c',
    'lfr_cpu_usage_report.c'
)

fsw_extra_files = files(
    '../header/fsw_compile_warnings.h',
    '../header/fsw_debug.h',
    '../header/fsw_globals.h',
    '../header/fsw_misc.h',
    '../header/fsw_init.h',
    '../header/fsw_spacewire.h',
    '../header/fsw_watchdog.h',
    '../header/fsw_housekeeping.h',
    '../header/hw/lfr_regs.h',
    '../header/hw/timer.h',
    '../header/hw/uart.h',
    '../header/hw/wf_handler.h',
    '../header/tc_tm/tc_load_dump_parameters.h',
    '../header/tc_tm/tm_lfr_tc_exe.h',
    '../header/tc_tm/tc_acceptance.h',
    '../header/tc_tm/tc_handler.h',
    '../header/mitigations/PAS_filtering.h',
    '../header/mitigations/reaction_wheel_filtering.h',
    '../header/processing/iir_filter.h',
    '../header/processing/fsw_processing.h',
    '../header/processing/calibration_matrices.h',
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

if meson.is_cross_build()
    cc = meson.get_compiler('c')
    m_dep = cc.find_library('m', required : false)

    c_args = ['-std=gnu99','-fno-math-errno', '-fdump-rtl-expand', target_endian_def]
    link_args = ['-Xlinker', '-Map=fsw-@0@.map'.format(fsw_version)]
    if get_option('fix-b2bst')
        c_args += '-mfix-b2bst'
        link_args += '-mfix-b2bst'
    endif

    if get_option('lpp-destid')
        c_args += '-DLPP_DPU_DESTID'
    endif

    fsw_dependencies = [lfr_bp_dep, m_dep]
    if get_option('with-gcov')
        fsw_dependencies += [libgcov_dep]
    endif


    fsw = executable('fsw', fsw_srcs,
                    include_directories: fsw_inc,
                    c_args: c_args,
                    pie: false,
                    link_args: link_args,
                    dependencies:fsw_dependencies,
                    extra_files: fsw_extra_files)


    objcopy = find_program('objcopy')
    objdump = find_program('objdump')
    size = find_program('size')
    b2bst_scan = find_program('../sparc/leon3ft-b2bst-scan_wrapper')
    cally = find_program('../python_scripts/cally.sh')

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

    assembly_dump = custom_target('fsw_asm_dump',
      input : fsw,
      output : 'fsw.S',
      capture: true,
      command : [objdump, '-drwCSl', '@INPUT@'],
      build_by_default: true
    )

    cally_callgraphs = custom_target('cally_callgraphs',
        input : fsw,
        output : 'all_calgraph.png',
        command : [cally, meson.project_source_root(), meson.project_build_root()],
        build_by_default: false
    )

    if get_option('fix-b2bst')
        check_b2bst = custom_target('check_b2bst',
          input : fsw,
          output : 'check_b2bst.log',
          command : [b2bst_scan, objdump, '@INPUT@', '@OUTPUT@'],
          build_by_default: true
        )
    endif
else
    warning('You are not using a cross build, no LFR firmware will be built.')
endif
