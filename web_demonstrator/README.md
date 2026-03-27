# LFR Processing Pipeline Demonstrator

Interactive browser-based visualization of the Solar Orbiter LFR signal processing chain.

## Quick Start

```bash
# Serve the static files
cd web_demonstrator
python3 -m http.server 8080
# Open http://localhost:8080
```

## Building the WASM Module (optional)

The BP1 stage uses the actual flight software (`basic_parameters.c`) compiled to WebAssembly.
Pre-built binaries are included, but to rebuild:

```bash
# Requires Emscripten (https://emscripten.org)
cd wasm && make
```

## Pipeline Stages

1. **INPUT** -- Synthetic 5-channel signal (E1, E2, B1, B2, B3)
2. **IIR** -- Anti-aliasing filter (5-cell biquad cascade, flight coefficients)
3. **f0** -- Decimation to 24,576 Hz
4. **f1/f2/f3** -- Frequency band branching (IIR + CIC decimation)
5. **WINDOW** -- 256-point Hanning window
6. **FFT** -- 256-point real-to-complex FFT -> 128 bins
7. **SM** -- 5x5 cross-spectral matrix computation
8. **AVG** -- Spectral matrix averaging
9. **BP1** -- Basic Parameters via WASM (actual flight code)

## Pre-baked Scenarios

- **Pure Tone** -- 1 kHz sine on B1
- **Whistler Wave** -- Circularly polarized frequency sweep
- **Broadband Turbulence** -- Pink noise on all channels
- **Multi-Component** -- Mixed frequencies on E and B channels

## Known Limitations

- f3 band (16 Hz) produces 0 samples with the default 50 ms duration -- this is expected since f3 requires >100 ms of data
- BP1 Poynting vector and phase velocity estimates are not yet decoded/displayed (v2 backlog)
