
catch_dep = dependency('catch2', main : true)

rtems_mock_inc = include_directories('common/rtems_mock')

tests ={
    'SM_calibration':
    {
        'sources': ['SM_calibration/main.cpp', '../src/processing/ASM/spectralmatrices.c'],
        'native': true,
        'dependencies': [catch_dep],
        'include_directories': [rtems_mock_inc, '../header']
    }
}

if get_option('with-tests')
    foreach test_name,test_params:tests
        test_exe = executable(test_name, kwargs: test_params)
        test('test-' + test_name, test_exe)
    endforeach
endif
