project('LFR_FSW', 'c', 'cpp',
        license : 'GPL-2.0-or-later',
        default_options : ['warning_level=3'])

add_global_arguments(
            '-DSW_VERSION_N1=@0@'.format(get_option('SW_VERSION_N1')),
            '-DSW_VERSION_N2=@0@'.format(get_option('SW_VERSION_N2')),
            '-DSW_VERSION_N3=@0@'.format(get_option('SW_VERSION_N3')),
            '-DSW_VERSION_N4=@0@'.format(get_option('SW_VERSION_N4')),
            language: 'c'
    )

fsw_version = '@0@-@1@-@2@-@3@'.format(get_option('SW_VERSION_N1'),
                                       get_option('SW_VERSION_N2'),
                                       get_option('SW_VERSION_N3'),
                                       get_option('SW_VERSION_N4'))

if(target_machine.endian() == 'big')
    target_endian_def = '-DLFR_BIG_ENDIAN'
else
    target_endian_def = '-DLFR_LITTLE_ENDIAN'
endif

if(build_machine.endian() == 'big')
    build_machine_endian_def = '-DLFR_BIG_ENDIAN'
else
    build_machine_endian_def = '-DLFR_LITTLE_ENDIAN'
endif


subdir('LFR_basic-parameters')
subdir('src')
subdir('tests')
subdir('testbenches/optimization')

cppcheck = find_program('cppcheck', required : false)

if cppcheck.found()
  run_target('cppcheck',
              command : [cppcheck, '--enable=all',
              '--project=' + join_paths(meson.build_root(), 'compile_commands.json')]
              )
endif
