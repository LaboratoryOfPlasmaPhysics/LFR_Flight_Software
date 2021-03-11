



matrix_bench = executable('matrix_bench', ['main.c'],
                dependencies:[bench_lfr_dep],
                extra_files:'matrix_change_of_basis_benchmark.py'
                )


objcopy = find_program('objcopy')
objdump = find_program('objdump')
size = find_program('size')

assembly_dump = custom_target('matrix_bench_asm_dump',
  input : matrix_bench,
  output : 'matrix_bench.S',
  capture: true,
  command : [objdump, '-drwCSl', '@INPUT@'],
  build_by_default: true
)

