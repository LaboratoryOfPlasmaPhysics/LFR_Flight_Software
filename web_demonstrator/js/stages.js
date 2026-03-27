// Stage definitions — metadata + render functions
// Render functions will be filled in as each stage is implemented.

export const STAGES = [
  {
    id: 'input',
    title: 'INPUT',
    description: `<h3>Input Signal</h3>
      <p>LFR measures <strong>5 physical channels</strong>: 2 electric field components
      (E1, E2) and 3 magnetic field components (B1, B2, B3) from the search coil magnetometer.</p>
      <p>The RHF1401 ADC samples all channels at <code>~98.304 kHz</code> (14-bit resolution).</p>
      <p>Select a pre-baked scenario or configure custom signals above.</p>`,
    render: null, // Task 3
  },
  {
    id: 'iir',
    title: 'IIR',
    description: `<h3>IIR Anti-Aliasing Filter</h3>
      <p>A 5-cell cascade of second-order sections (biquads) with <strong>9-bit fixed-point
      coefficients</strong> removes content above the Nyquist frequency before decimation.</p>
      <p>This prevents aliasing artifacts when the sample rate is reduced to produce f0.</p>`,
    render: null, // Task 4
  },
  {
    id: 'decimate-f0',
    title: '÷4 → f0',
    description: `<h3>Decimation to f0</h3>
      <p>The filtered signal is decimated by a factor of <strong>4</strong>, reducing the
      sample rate from ~98.304 kHz to <strong>24,576 Hz</strong>.</p>
      <p>This is the widest bandwidth channel (DC to ~10 kHz), used for all downstream processing.</p>`,
    render: null, // Task 4
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
    render: null, // Task 5
  },
  {
    id: 'window',
    title: 'WINDOW',
    description: `<h3>Hanning Window</h3>
      <p>Before the FFT, each 256-sample block is multiplied by a <strong>Hanning window</strong>
      to reduce spectral leakage.</p>
      <p>Without windowing, a finite-length signal block creates artificial discontinuities at
      its edges, spreading energy across all frequency bins.</p>`,
    render: null, // Task 6
  },
  {
    id: 'fft',
    title: 'FFT',
    description: `<h3>256-Point FFT</h3>
      <p>A <strong>256-point real-to-complex FFT</strong> (Actel CoreFFT IP in the FPGA) transforms
      each windowed block from time domain to frequency domain.</p>
      <p>This yields <strong>128 useful frequency bins</strong>. The frequency resolution depends
      on the channel: f0 → 96 Hz/bin, f1 → 16 Hz/bin, f2 → 1 Hz/bin.</p>`,
    render: null, // Task 6
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
    render: null, // Task 7
  },
  {
    id: 'avg',
    title: 'AVG',
    description: `<h3>Spectral Matrix Averaging</h3>
      <p>Multiple spectral matrices are averaged to reduce noise. The FPGA produces
      <strong>96 SMs/s at f0</strong>, which are accumulated in groups of 8.</p>
      <p>Longer averaging periods (4s for Normal mode BP1) further reduce variance,
      trading time resolution for signal-to-noise ratio.</p>`,
    render: null, // Task 7
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
