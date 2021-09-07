
catch_dep = dependency('catch2', main : true)
pybind11_dep = dependency('pybind11')
pymod = import('python')
python3 = pymod.find_installation('python3')

rtems_mock_inc = include_directories('common/rtems_mock')

lfr = library('lfr',
            ['../src/processing/ASM/spectralmatrices.c',
            '../src/mitigations/PAS_filtering.c',
            '../src/mitigations/reaction_wheel_filtering.c',
            'common/mock_globals.c'],
            include_directories: [rtems_mock_inc, '../header'],
            extra_files: ['common/FSW_helpers.hpp'],
            native: true)

lfr_dep = declare_dependency(link_with: lfr,include_directories: [rtems_mock_inc, '../header'] )

pywrappers = python3.extension_module('lfr',
                    'common/py_wrappers.cpp',
                    native: true,
                    cpp_args: ['-std=c++17', build_machine_endian_def],
                    c_args: [build_machine_endian_def],
                    install: true,
                    install_dir: python3.get_path('platlib'),
                    dependencies: [lfr_dep, lfr_bp_dep, pybind11_dep, python3.dependency(embed:true, native:true)])


tests ={
    'helpers':
    {
        'sources': ['helpers/main.cpp'],
        'native': true,
        'dependencies': [catch_dep, lfr_dep],
        'cpp_args': ['-std=c++17']
    }
}


test_scripts = {
    'SM_calibration':'SM_calibration/test.py'
}

if get_option('with-tests')
    foreach test_name,test_params:tests
        test_exe = executable(test_name, kwargs: test_params, extra_files: 'SM_calibration/test.py')
        test('test-' + test_name, test_exe)
    endforeach

    foreach test_name,test_script:test_scripts
        test(test_name, python3,
            args:[meson.current_source_dir()+'/'+test_script, '--verbose'],
            env:['PYTHONPATH='+meson.current_build_dir()]
        )
    endforeach
endif

library('tests_fake',[],
        native: true,
        extra_files:['SM_calibration/test.py'])
