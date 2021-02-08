project('Name', 'cpp',default_options : ['cpp_std=c++14'])

#qt5 = import('qt5')
#qt5core = dependency('qt5', modules : 'Core')
#qt5widgets = dependency('qt5', modules : 'Widgets')
#qt5gui = dependency('qt5', modules : 'Gui')
#qt5svg = dependency('qt5', modules : 'Svg')
#qt5xml = dependency('qt5', modules : 'Xml')
#qt5network = dependency('qt5', modules : 'Network')
#qt5printsupport = dependency('qt5', modules : 'PrintSupport')
#qt5test = dependency('qt5', modules : 'Test')

subdir('src')

#if build_machine.system()=='darwin'
#  install_data('build_cfg/mac/sciqlopLOGO.icns', install_dir : 'Contents/Resources')
#  install_data('build_cfg/mac/Info.plist', install_dir : 'Contents')
#  meson.add_install_script('build_cfg/mac/install_script.sh')
#elif host_machine.system()=='windows'
#  meson.add_install_script('build_cfg/windows/install_script.sh')
#endif

my_inc = include_directories(['include'])

my_lib = library('lib_name',
                       gui_sources,
                       gui_moc_files,
                       link_with : sciqlop_core,
                       include_directories : [my_inc],
                       dependencies : [qt5core, qt5printsupport, qt5gui, qt5widgets, qt5network],
                       install : true
                       )

my_app = executable('exe_name',
                       [],
                       link_with : [my_lib],
                       include_directories : [my_inc],
                       dependencies : [qt5core, qt5printsupport, qt5gui, qt5widgets, qt5network],
                       install : true
                       )

cppcheck = find_program('cppcheck', required : false)

if cppcheck.found()
  run_target('cppcheck',
              command : [cppcheck, '--enable=all',
              '--project=' + join_paths(meson.build_root(), 'compile_commands.json')]
              )
endif
