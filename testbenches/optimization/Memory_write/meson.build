



memory_write_bench = executable('memory_write_benchmark', ['main.c'],
                dependencies:[bench_lfr_dep],
                extra_files:['memory_write_benchmark.py','README.md']
                )


objcopy = find_program('objcopy')
objdump = find_program('objdump')
size = find_program('size')

assembly_dump = custom_target('memory_write_benchmark_asm_dump',
  input : memory_write_bench,
  output : 'memory_write_benchmark_bench.S',
  capture: true,
  command : [objdump, '-drwCSl', '@INPUT@'],
  build_by_default: true
)

