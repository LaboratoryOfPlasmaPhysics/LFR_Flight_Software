BP_SRC_DIR  := ../../LFR_basic-parameters
HEADER_DIR  := ../../header
SHIM_DIR    := shim_headers

SOURCES     := $(BP_SRC_DIR)/basic_parameters.c bp1_wrapper.c
OUTPUT      := bp1.js

EXPORTED_FUNCTIONS := _bp1_compute,_bp1_bytes_per_bin,_malloc,_free

EMCC_FLAGS := \
	-O2 \
	-DLFR_LITTLE_ENDIAN \
	-I. \
	-I$(SHIM_DIR) \
	-I$(BP_SRC_DIR) \
	-I$(HEADER_DIR) \
	-s MODULARIZE=1 \
	-s EXPORT_NAME='createBP1Module' \
	-s EXPORTED_FUNCTIONS='$(EXPORTED_FUNCTIONS)' \
	-s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]' \
	-s ALLOW_MEMORY_GROWTH=1 \
	--no-entry

.PHONY: all clean shims

all: shims $(OUTPUT)

shims:
	mkdir -p $(SHIM_DIR)/processing/ASM
	mkdir -p $(SHIM_DIR)/lfr_common_headers
	cp spectralmatrices_shim.h $(SHIM_DIR)/processing/ASM/spectralmatrices.h
	cp $(HEADER_DIR)/lfr_common_headers/fsw_params_processing.h \
		$(SHIM_DIR)/lfr_common_headers/fsw_params_processing.h
	printf '#pragma once\n#include "fsw_params_processing.h"\n' \
		> $(SHIM_DIR)/lfr_common_headers/fsw_params.h

$(OUTPUT): shims $(SOURCES)
	emcc $(EMCC_FLAGS) $(SOURCES) -o $@

clean:
	rm -rf $(SHIM_DIR) $(OUTPUT) bp1.wasm
