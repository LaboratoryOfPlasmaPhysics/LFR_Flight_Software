// Load WASM BP1 module and compute BP1 from averaged spectral matrix

import { decodeBP1 } from './bp1-decode.js';

let modulePromise = null;

function loadModule() {
  if (modulePromise) return modulePromise;
  if (typeof createBP1Module !== 'function') {
    return Promise.reject(new Error(
      'WASM module not loaded. Ensure <script src="wasm/bp1.js"></script> is in index.html.'
    ));
  }
  modulePromise = createBP1Module();
  return modulePromise;
}

const N_BINS = 128;
const FLOATS_PER_BIN = 25;

export async function computeBP1(averaged) {
  const mod = await loadModule();

  const bytesPerBin = mod._bp1_bytes_per_bin();
  const nFloats = N_BINS * FLOATS_PER_BIN;
  const smByteLen = nFloats * 4;        // float32 = 4 bytes
  const outByteLen = N_BINS * bytesPerBin;

  const smPtr = mod._malloc(smByteLen);
  const outPtr = mod._malloc(outByteLen);
  try {
    mod.HEAPF32.set(averaged.subarray(0, nFloats), smPtr >> 2);
    mod._bp1_compute(smPtr, N_BINS, outPtr);
    const rawOut = new Uint8Array(mod.HEAPU8.buffer, outPtr, outByteLen);
    const result = new Uint8Array(outByteLen);
    result.set(rawOut);
    return decodeBP1(result, N_BINS);
  } finally {
    mod._free(smPtr);
    mod._free(outPtr);
  }
}
