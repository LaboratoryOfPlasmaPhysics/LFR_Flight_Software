



asm_bench = executable('asm_bench', ['main.c'],
                dependencies:[bench_lfr_dep],
                extra_files:['asm_bench.py', 'README.md']
                )


objcopy = find_program('objcopy')
objdump = find_program('objdump')
size = find_program('size')

assembly_dump = custom_target('asm_bench',
  input : asm_bench,
  output : 'asm_bench.S',
  capture: true,
  command : [objdump, '-drwCSl', '@INPUT@'],
  build_by_default: true
)

