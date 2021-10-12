
catch_dep = dependency('catch2', main : true)
pybind11_dep = dependency('pybind11')
pymod = import('python')
python3 = pymod.find_installation('python3')

rtems_mock_inc = include_directories('common/rtems_mock')

lfr_sources = ['../src/processing/ASM/spectralmatrices.c',
'../src/processing/calibration_matrices.c',
'../src/mitigations/PAS_filtering.c',
'../src/mitigations/reaction_wheel_filtering.c',
'common/mock_globals.c']


lfr_dep = declare_dependency(include_directories: [rtems_mock_inc, '../header'], sources: lfr_sources)

if get_option('with-pywrapper')
    pywrapper = python3.extension_module('lfr',
                    'common/py_wrappers.cpp',
                    native: true,
                    cpp_args: ['-std=c++17', build_machine_endian_def],
                    c_args: [build_machine_endian_def],
                    install: not meson.is_cross_build(),
                    install_dir: python3.get_path('platlib'),
                    extra_files: ['common/FSW_helpers.hpp'],
                    dependencies: [lfr_dep, lfr_bp_dep, pybind11_dep, python3.dependency(embed:true, native:true)])


    test_scripts = {
        'SM_calibration':'SM_calibration/test.py'
    }

    if get_option('with-tests')
        foreach test_name,test_script:test_scripts
            test(test_name, python3,
                args:[meson.current_source_dir()+'/'+test_script, '--verbose'],
                env:['PYTHONPATH='+meson.current_build_dir()]
            )
        endforeach
    endif
endif
library('tests_fake',[],
        native: true,
        extra_files:['SM_calibration/test.py'])

if get_option('with-tests')
    tests ={
        'iir_filter':
        {
            'sources': ['iir_filter/main.cpp'],
            'native': true,
            'dependencies': [catch_dep, lfr_dep],
            'cpp_args': ['-std=c++17']
        },
        'basic_parameters':
        {
            'sources': ['basic_parameters/main.cpp'],
            'native': true,
            'dependencies': [catch_dep, lfr_dep, lfr_bp_dep],
            'cpp_args': ['-std=c++17', build_machine_endian_def],
            'c_args': [build_machine_endian_def]
        },
        'spectral_matrices':
        {
            'sources': ['spectral_matrices/main.cpp'],
            'native': true,
            'dependencies': [catch_dep, lfr_dep],
            'cpp_args': ['-std=c++17', build_machine_endian_def],
            'c_args': [build_machine_endian_def]
        },
        'helpers':
        {
            'sources': ['helpers/main.cpp'],
            'native': true,
            'dependencies': [catch_dep, lfr_dep],
            'cpp_args': ['-std=c++17']
        }
    }

    foreach test_name,test_params:tests
        test_exe = executable(test_name, kwargs: test_params, extra_files: 'SM_calibration/test.py')
        test('test-' + test_name, test_exe)
    endforeach
endif
