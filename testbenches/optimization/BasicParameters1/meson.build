
matrix_bench = executable('bp1_bench', ['main.c'],
                c_args: ['-DENABLE_DEAD_CODE'],
                dependencies:[bench_lfr_dep,lfr_bp_dep],
                extra_files:'basic_parameters1_benchmark.py'
                )


objcopy = find_program('objcopy')
objdump = find_program('objdump')
size = find_program('size')

assembly_dump = custom_target('bp1_bench_asm_dump',
  input : matrix_bench,
  output : 'bp1_bench.S',
  capture: true,
  command : [objdump, '-drwCSl', '@INPUT@'],
  build_by_default: true
)

