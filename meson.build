project('LFR_FSW', 'c', 'cpp')
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

subdir('LFR_basic-parameters')
subdir('src')
subdir('tests')

cppcheck = find_program('cppcheck', required : false)

if cppcheck.found()
  run_target('cppcheck',
              command : [cppcheck, '--enable=all',
              '--project=' + join_paths(meson.build_root(), 'compile_commands.json')]
              )
endif
