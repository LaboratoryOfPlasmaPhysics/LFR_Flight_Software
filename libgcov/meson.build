
libgcov_dep = declare_dependency(compile_args:['-DGCOV_USE_EXIT', '-DGCOV_ENABLED', '-fprofile-arcs', '-ftest-coverage'],
                                 include_directories:  '.',
                                 sources: ['gcov-io.c','libgcov.c'])


gcovr = find_program('gcovr')
rtems_gcov = find_program('gcov')

gcovr_html = custom_target('gcovr',
  output : 'gcov.html',
  command : [gcovr,
            '--exclude=\'.*gcov.*\'',
            '--gcov-executable=@0@'.format(rtems_gcov),
            '--object-directory=@0@'.format(meson.project_build_root()),
            '-r=@0@'.format(meson.project_source_root()),
            '--html',
            '--html-details',
            '-o=@0@/gcov.html'.format(meson.current_build_dir())
  ],
  build_by_default: false
)

message(rtems_gcov.full_path())
