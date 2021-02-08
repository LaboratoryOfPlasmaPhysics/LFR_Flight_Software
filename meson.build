project('LFR_FSW', 'c')
add_global_arguments(
            '-DSW_VERSION_N1=@0@'.format(get_option('SW_VERSION_N1')),
            '-DSW_VERSION_N2=@0@'.format(get_option('SW_VERSION_N2')),
            '-DSW_VERSION_N3=@0@'.format(get_option('SW_VERSION_N3')),
            '-DSW_VERSION_N4=@0@'.format(get_option('SW_VERSION_N4')),
            language: 'c'
    )

subdir('LFR_basic-parameters')
subdir('src')

cppcheck = find_program('cppcheck', required : false)

if cppcheck.found()
  run_target('cppcheck',
              command : [cppcheck, '--enable=all',
              '--project=' + join_paths(meson.build_root(), 'compile_commands.json')]
              )
endif
