if get_option('with-benchmarks')
    if meson.is_cross_build()
        cc = meson.get_compiler('c')
        m_dep = cc.find_library('m', required : false)

        c_args = ['-std=gnu99','-fno-math-errno', target_endian_def, '-DENABLE_BENCHMARKS', '-DPRINTF(...)']
        link_args = []
        if get_option('fix-b2bst')
            c_args += '-mfix-b2bst'
            link_args += '-mfix-b2bst'
        endif

        bench_lfr_src = ['../../src/processing/ASM/spectralmatrices.c',
                    '../../src/processing/calibration_matrices.c',
                    '../../src/mitigations/PAS_filtering.c',
                    '../../src/mitigations/reaction_wheel_filtering.c',
                    '../../tests/common/mock_globals.c']

        bench_lfr_dep = declare_dependency(
                            sources:bench_lfr_src,
                            compile_args: c_args,
                            link_args: link_args,
                            dependencies: m_dep,
                            include_directories: ['../../header','common'] )

        subdir('ASM')
        subdir('BasicParameters1')
        subdir('Memory_write')
    endif
endif
