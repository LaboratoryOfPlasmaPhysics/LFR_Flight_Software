# Memory Read/Write benchmark

This benchmark helps to get some numbers about memory access costs.


Read  -> 512k words = 0.419s ->  ~0.8us/Word or 1,251MWord/s
Write -> 512k words = 0.337s ->  ~0.64us/Word or 1,556MWord/s

This is not only the time to read or write a single word, this also integrate pointer increment.
