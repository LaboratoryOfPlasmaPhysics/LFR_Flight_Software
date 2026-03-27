// Stage definitions — metadata + render functions

import * as plots from './plots.js';
import { HANNING_256 } from './dsp.js';

// Extract a 5x5 magnitude matrix from the flat 25-element SM layout at a given bin.
// Flat layout per bin: for each row i, then j from i to 4:
//   i==j -> 1 float (auto-spectrum), else 2 floats (re, im of cross-spectrum)
function extractSM5x5(sm, bin) {
  const base = bin * 25;
  const mat = Array.from({ length: 5 }, () => new Array(5).fill(0));
  let idx = base;
  for (let i = 0; i < 5; i++) {
    const autoVal = sm[idx++];
    mat[i][i] = autoVal;
    for (let j = i + 1; j < 5; j++) {
      const re = sm[idx++];
      const im = sm[idx++];
      const mag = Math.sqrt(re * re + im * im);
      mat[i][j] = mag;
      mat[j][i] = mag;
    }
  }
  return mat;
}

function findPeakBin(sm) {
  // Auto-spectrum offsets within each 25-element block: B1B1=0, B2B2=9, B3B3=16
  let maxPower = -Infinity;
  let peakBin = 0;
  for (let bin = 0; bin < 128; bin++) {
    const base = bin * 25;
    const power = sm[base] + sm[base + 9] + sm[base + 16];
    if (power > maxPower) {
      maxPower = power;
      peakBin = bin;
    }
  }
  return peakBin;
}

export const STAGES = [
  {
    id: 'input',
    title: 'INPUT',
    description: `<h3>Input Signal</h3>
      <p>LFR measures <strong>5 physical channels</strong>: 2 electric field components
      (E1, E2) and 3 magnetic field components (B1, B2, B3) from the search coil magnetometer.</p>
      <p>The RHF1401 ADC samples all channels at <code>~98.304 kHz</code> (14-bit resolution).</p>
      <p>Select a pre-baked scenario or configure custom signals above.</p>`,
    render: async (container, data) => {
      plots.timeDomain(container, data.input, 98304, {
        title: 'Input Waveform (5 channels)',
        names: data.channelNames,
      });
    },
  },
  {
    id: 'iir',
    title: 'IIR',
    description: `<h3>IIR Anti-Aliasing Filter</h3>
      <p>A 5-cell cascade of second-order sections (biquads) with <strong>9-bit fixed-point
      coefficients</strong> removes content above the Nyquist frequency before decimation.</p>
      <p>This prevents aliasing artifacts when the sample rate is reduced to produce f0.</p>`,
    render: async (container, data) => {
      const b1idx = 2; // B1 is index 2 in [E1,E2,B1,B2,B3]
      plots.multiPlot(container, [
        {
          fn: plots.timeDomain,
          args: [
            [data.input[b1idx], data.filtered[b1idx]],
            98304,
            { title: 'B1: Raw vs Filtered', names: ['Raw B1', 'Filtered B1'] },
          ],
        },
      ]);
    },
  },
  {
    id: 'decimate-f0',
    title: '÷4 → f0',
    description: `<h3>Decimation to f0</h3>
      <p>The filtered signal is decimated by a factor of <strong>4</strong>, reducing the
      sample rate from ~98.304 kHz to <strong>24,576 Hz</strong>.</p>
      <p>This is the widest bandwidth channel (DC to ~10 kHz), used for all downstream processing.</p>`,
    render: async (container, data) => {
      plots.timeDomain(container, data.f0, 24576, {
        title: 'f0 Waveform (24,576 Hz)',
        names: data.channelNames,
      });
    },
  },
  {
    id: 'branches',
    title: 'f1/f2/f3',
    description: `<h3>Frequency Band Branching</h3>
      <p>From f0, three narrower bands are derived:</p>
      <ul>
        <li><strong>f1</strong> (4096 Hz): IIR filter + ÷6 decimation</li>
        <li><strong>f2</strong> (256 Hz): CIC filter ÷16 + IIR + ÷6</li>
        <li><strong>f3</strong> (16 Hz): CIC filter ÷256 + IIR + ÷6</li>
      </ul>
      <p>The CIC (Cascaded Integrator-Comb) filters provide efficient high-ratio decimation.</p>`,
    render: async (container, data) => {
      const b1idx = 2;
      plots.multiPlot(container, [
        { fn: plots.timeDomain, args: [[data.f0[b1idx]], 24576, { title: 'B1 @ f0 (24,576 Hz)', names: ['B1'] }] },
        { fn: plots.timeDomain, args: [[data.f1[b1idx]], 4096, { title: 'B1 @ f1 (4,096 Hz)', names: ['B1'] }] },
        { fn: plots.timeDomain, args: [[data.f2[b1idx]], 256, { title: 'B1 @ f2 (256 Hz)', names: ['B1'] }] },
        { fn: plots.timeDomain, args: [[data.f3[b1idx]], 16, { title: 'B1 @ f3 (16 Hz)', names: ['B1'] }] },
      ]);
    },
  },
  {
    id: 'window',
    title: 'WINDOW',
    description: `<h3>Hanning Window</h3>
      <p>Before the FFT, each 256-sample block is multiplied by a <strong>Hanning window</strong>
      to reduce spectral leakage.</p>
      <p>Without windowing, a finite-length signal block creates artificial discontinuities at
      its edges, spreading energy across all frequency bins.</p>`,
    render: async (container, data) => {
      plots.multiPlot(container, [
        {
          fn: plots.timeDomain,
          args: [[HANNING_256], 256, { title: 'Hanning Window (256 points)', names: ['Window'] }],
        },
        {
          fn: plots.timeDomain,
          args: [[data.windowed[0]], 256, { title: 'B1 After Windowing', names: ['B1 windowed'] }],
        },
      ]);
    },
  },
  {
    id: 'fft',
    title: 'FFT',
    description: `<h3>256-Point FFT</h3>
      <p>A <strong>256-point real-to-complex FFT</strong> (Actel CoreFFT IP in the FPGA) transforms
      each windowed block from time domain to frequency domain.</p>
      <p>This yields <strong>128 useful frequency bins</strong>. The frequency resolution depends
      on the channel: f0 → 96 Hz/bin, f1 → 16 Hz/bin, f2 → 1 Hz/bin.</p>`,
    render: async (container, data) => {
      const fftB1 = data.fftResults[0]; // B1 is index 0 in SM order
      const nBins = fftB1.re.length;
      const binWidth = 24576 / 256;
      const magnitudes = new Float32Array(nBins);
      const binFreqs = new Float32Array(nBins);
      for (let i = 0; i < nBins; i++) {
        magnitudes[i] = Math.sqrt(fftB1.re[i] ** 2 + fftB1.im[i] ** 2);
        binFreqs[i] = i * binWidth;
      }
      plots.spectrum(container, magnitudes, binFreqs, {
        title: 'B1 FFT Magnitude',
        name: 'B1',
      });
    },
  },
  {
    id: 'sm',
    title: 'SM',
    description: `<h3>Cross-Spectral Matrix</h3>
      <p>For each frequency bin, the 5 complex FFT outputs are combined into a
      <strong>5×5 Hermitian matrix</strong>:</p>
      <p><code>SM[i][j] = conj(FFT_i) × FFT_j</code></p>
      <p>This gives <strong>25 real values per bin</strong>: 5 auto-spectra (diagonal) and
      10 complex cross-spectra (upper triangle, 20 reals). Total: 128 × 25 = 3200 floats.</p>`,
    render: async (container, data) => {
      const peakBin = findPeakBin(data.sm);
      const mat = extractSM5x5(data.sm, peakBin);
      const binWidth = 24576 / 256;
      const freq = (peakBin * binWidth).toFixed(0);
      plots.heatmap(container, mat, {
        title: `Spectral Matrix at peak bin ${peakBin} (~${freq} Hz)`,
        labels: data.smChannelNames,
      });
    },
  },
  {
    id: 'avg',
    title: 'AVG',
    description: `<h3>Spectral Matrix Averaging</h3>
      <p>Multiple spectral matrices are averaged to reduce noise. The FPGA produces
      <strong>96 SMs/s at f0</strong>, which are accumulated in groups of 8.</p>
      <p>Longer averaging periods (4s for Normal mode BP1) further reduce variance,
      trading time resolution for signal-to-noise ratio.</p>`,
    render: async (container, data) => {
      if (data.smList.length < 2) {
        container.innerHTML = '<p style="color:#e6edf3;padding:1em;">Insufficient duration: need at least 2 FFT blocks (512 f0 samples) for meaningful averaging.</p>';
        return;
      }
      const peakBin = findPeakBin(data.averaged);
      const mat = extractSM5x5(data.averaged, peakBin);
      const binWidth = 24576 / 256;
      const freq = (peakBin * binWidth).toFixed(0);
      plots.heatmap(container, mat, {
        title: `Averaged SM at peak bin ${peakBin} (~${freq} Hz) — ${data.smList.length} blocks`,
        labels: data.smChannelNames,
      });
    },
  },
  {
    id: 'bp1',
    title: 'BP1',
    description: `<h3>Basic Parameters (BP1)</h3>
      <p>The <strong>actual flight software</strong> (<code>basic_parameters.c</code>) runs
      in your browser via WebAssembly to compute BP1 from the averaged spectral matrix:</p>
      <ul>
        <li><strong>PSDB</strong>: Magnetic power spectral density (B1² + B2² + B3²)</li>
        <li><strong>PSDE</strong>: Electric power spectral density (E1² + E2²)</li>
        <li><strong>Ellipticity</strong>: Wave polarization shape (0 = linear, 1 = circular)</li>
        <li><strong>Degree of polarization</strong>: Coherence of the wave field</li>
        <li><strong>Normal vector</strong>: Wave propagation direction (θ, φ)</li>
        <li><strong>Poynting flux</strong>: Energy flow direction</li>
      </ul>`,
    render: null, // Task 9
  },
];
