// DSP Core — IIR, CIC, FFT, Spectral Matrix (flight-equivalent coefficients)

// --- Anti-aliasing IIR (9-bit fixed-point / 128) ---
export const AA_SOS = [
  { b: [58/128, -66/128, 58/128], a: [1.0, -189/128, 111/128] },
  { b: [58/128, -57/128, 58/128], a: [1.0, -162/128,  81/128] },
  { b: [29/128, -17/128, 29/128], a: [1.0, -136/128,  55/128] },
  { b: [15/128,   4/128, 15/128], a: [1.0, -114/128,  33/128] },
  { b: [15/128,  24/128, 15/128], a: [1.0, -100/128,  20/128] },
];

// --- F0 -> F1 IIR (floating-point SOS with per-cell gain) ---
const F0_F1_GAINS = [
  0.566196896119831, 0.474937156750133, 0.347712822970540,
  0.200868393871900, 0.0910613125308450,
];

const F0_F1_SOS_RAW = [
  [1.0, -1.61171504942096, 1.0, 1.0, -1.68876443778669, 0.908610171614583],
  [1.0, -1.53324505744412, 1.0, 1.0, -1.51088513595779, 0.732564401274351],
  [1.0, -1.30646173160060, 1.0, 1.0, -1.30571711968384, 0.546869268827102],
  [1.0, -0.651038739239370, 1.0, 1.0, -1.08747326287406, 0.358436944718464],
  [1.0,  1.24322747034001, 1.0, 1.0, -0.929530176676438, 0.224862726961691],
];

export const F0_F1_SOS = F0_F1_SOS_RAW.map((row, i) => ({
  b: [row[0] * F0_F1_GAINS[i], row[1] * F0_F1_GAINS[i], row[2] * F0_F1_GAINS[i]],
  a: [row[3], row[4], row[5]],
}));

// --- IIR biquad cascade (Direct Form II Transposed) ---
export function iirFilter(input, sosSections) {
  let x = Float32Array.from(input);
  for (const { b, a } of sosSections) {
    const y = new Float32Array(x.length);
    let d1 = 0, d2 = 0;
    for (let n = 0; n < x.length; n++) {
      const w = x[n] - a[1] * d1 - a[2] * d2;
      y[n] = b[0] * w + b[1] * d1 + b[2] * d2;
      d2 = d1;
      d1 = w;
    }
    x = y;
  }
  return x;
}

// --- Downsample (take every R-th sample) ---
export function downsample(input, R) {
  const len = Math.floor(input.length / R);
  const out = new Float32Array(len);
  for (let i = 0; i < len; i++) out[i] = input[i * R];
  return out;
}

// --- CIC decimation filter (3 integrators, decimate, 3 combs with D=2) ---
export function cicDecimate(input, R) {
  const D = 2;
  // Integrator stage (3 cascaded)
  const integ = new Float64Array(input.length);
  let s0 = 0, s1 = 0, s2 = 0;
  for (let i = 0; i < input.length; i++) {
    s0 += input[i];
    s1 += s0;
    s2 += s1;
    integ[i] = s2;
  }

  // Decimate
  const decLen = Math.floor(input.length / R);
  const dec = new Float64Array(decLen);
  for (let i = 0; i < decLen; i++) dec[i] = integ[i * R];

  // Comb stage (3 cascaded, differential delay D=2)
  let comb = dec;
  for (let stage = 0; stage < 3; stage++) {
    const next = new Float64Array(comb.length);
    for (let i = 0; i < comb.length; i++) {
      next[i] = comb[i] - (i >= D ? comb[i - D] : 0);
    }
    comb = next;
  }

  // Normalize by (R*D)^3
  const norm = (R * D) ** 3;
  const out = new Float32Array(comb.length);
  for (let i = 0; i < comb.length; i++) out[i] = comb[i] / norm;
  return out;
}

// --- Hanning window (256 points) ---
export const HANNING_256 = new Float32Array(256);
for (let i = 0; i < 256; i++) {
  HANNING_256[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / 255));
}

export function applyWindow(block) {
  const out = new Float32Array(256);
  for (let i = 0; i < 256; i++) out[i] = block[i] * HANNING_256[i];
  return out;
}

// --- 256-point radix-2 DIT FFT ---
function bitReverse(n, bits) {
  let r = 0;
  for (let i = 0; i < bits; i++) {
    r = (r << 1) | (n & 1);
    n >>= 1;
  }
  return r;
}

export function fft256(block) {
  const N = 256;
  const LOG2N = 8;
  const re = new Float64Array(N);
  const im = new Float64Array(N);

  // Bit-reversal permutation
  for (let i = 0; i < N; i++) {
    const j = bitReverse(i, LOG2N);
    re[j] = block[i];
  }

  // Cooley-Tukey butterfly
  for (let s = 1; s <= LOG2N; s++) {
    const m = 1 << s;
    const halfM = m >> 1;
    const wRe = Math.cos(-2 * Math.PI / m);
    const wIm = Math.sin(-2 * Math.PI / m);
    for (let k = 0; k < N; k += m) {
      let tRe = 1, tIm = 0;
      for (let j = 0; j < halfM; j++) {
        const u = k + j;
        const v = u + halfM;
        const xRe = tRe * re[v] - tIm * im[v];
        const xIm = tRe * im[v] + tIm * re[v];
        re[v] = re[u] - xRe;
        im[v] = im[u] - xIm;
        re[u] += xRe;
        im[u] += xIm;
        const newTRe = tRe * wRe - tIm * wIm;
        tIm = tRe * wIm + tIm * wRe;
        tRe = newTRe;
      }
    }
  }

  // Return first 128 bins (positive frequencies)
  return {
    re: new Float32Array(re.subarray(0, 128)),
    im: new Float32Array(im.subarray(0, 128)),
  };
}

// --- Cross-spectral matrix for one bin ---
// Flight order: B1B1, B1B2_re, B1B2_im, B1B3_re, B1B3_im,
//   B1E1_re, B1E1_im, B1E2_re, B1E2_im,
//   B2B2, B2B3_re, B2B3_im, B2E1_re, B2E1_im, B2E2_re, B2E2_im,
//   B3B3, B3E1_re, B3E1_im, B3E2_re, B3E2_im,
//   E1E1, E1E2_re, E1E2_im, E2E2
// Channel order in fftChannels: [B1, B2, B3, E1, E2]
export function crossSpectralMatrix(fftChannels, binIndex) {
  const sm = new Float32Array(25);
  const ch = fftChannels.map(c => ({
    re: c.re[binIndex],
    im: c.im[binIndex],
  }));

  let idx = 0;
  for (let i = 0; i < 5; i++) {
    // Auto-spectrum: conj(Xi)*Xi = |Xi|^2
    sm[idx++] = ch[i].re * ch[i].re + ch[i].im * ch[i].im;
    for (let j = i + 1; j < 5; j++) {
      // Cross-spectrum: conj(Xi)*Xj
      sm[idx++] = ch[i].re * ch[j].re + ch[i].im * ch[j].im;   // real part
      sm[idx++] = ch[i].re * ch[j].im - ch[i].im * ch[j].re;   // imag part (conj(i)*j)
    }
  }
  return sm;
}

// --- Full spectral matrix (128 bins x 25) ---
export function fullSpectralMatrix(fftChannels) {
  const result = new Float32Array(128 * 25);
  for (let bin = 0; bin < 128; bin++) {
    const sm = crossSpectralMatrix(fftChannels, bin);
    result.set(sm, bin * 25);
  }
  return result;
}

// --- Average multiple spectral matrices ---
export function averageSM(matrices) {
  const len = matrices[0].length;
  const avg = new Float32Array(len);
  for (const m of matrices) {
    for (let i = 0; i < len; i++) avg[i] += m[i];
  }
  const n = matrices.length;
  for (let i = 0; i < len; i++) avg[i] /= n;
  return avg;
}

// --- Full DSP pipeline ---
const SM_CHANNEL_NAMES = ['B1', 'B2', 'B3', 'E1', 'E2'];
const INPUT_CHANNEL_NAMES = ['E1', 'E2', 'B1', 'B2', 'B3'];

export function runDSPPipeline(raw) {
  const ADC_RATE = 98304;
  const channels = [raw.E1, raw.E2, raw.B1, raw.B2, raw.B3];

  // 1. IIR anti-aliasing
  const filtered = channels.map(ch => iirFilter(ch, AA_SOS));

  // 2. Downsample /4 -> f0 (24576 Hz)
  const f0 = filtered.map(ch => downsample(ch, 4));

  // 3. f0 -> f1: IIR + /6 (4096 Hz)
  const f1 = f0.map(ch => downsample(iirFilter(ch, F0_F1_SOS), 6));

  // 4. f0 -> f2: CIC /16 + IIR + /6 (256 Hz)
  const f2 = f0.map(ch => downsample(iirFilter(cicDecimate(ch, 16), F0_F1_SOS), 6));

  // 5. f0 -> f3: CIC /256 + IIR + /6 (16 Hz)
  const f3 = f0.map(ch => downsample(iirFilter(cicDecimate(ch, 256), F0_F1_SOS), 6));

  // 6. Reorder to SM order [B1, B2, B3, E1, E2] = indices [2,3,4,0,1] from E1,E2,B1,B2,B3
  const smOrder = [2, 3, 4, 0, 1];

  // 7-9. Window + FFT + SM + averaging for each frequency band with enough samples
  function processBand(bandChannels) {
    const smCh = smOrder.map(i => bandChannels[i]);
    const blockSize = 256;
    const numBlocks = Math.floor(smCh[0].length / blockSize);
    if (numBlocks === 0) return null;
    const windowed = smCh.map(ch => applyWindow(ch.subarray(0, blockSize)));
    const fftResults = windowed.map(w => fft256(w));
    const sm = fullSpectralMatrix(fftResults);
    const smList = [];
    for (let b = 0; b < numBlocks; b++) {
      const offset = b * blockSize;
      const blockFFTs = smCh.map(ch => fft256(applyWindow(ch.subarray(offset, offset + blockSize))));
      smList.push(fullSpectralMatrix(blockFFTs));
    }
    const averaged = smList.length > 0 ? averageSM(smList) : sm;
    return { windowed, fftResults, sm, smList, averaged };
  }

  const bands = {
    f0: { rate: 24576, channels: f0, label: 'f0 (24,576 Hz)', binWidth: 24576 / 256 },
    f1: { rate: 4096,  channels: f1, label: 'f1 (4,096 Hz)',  binWidth: 4096 / 256 },
    f2: { rate: 256,   channels: f2, label: 'f2 (256 Hz)',     binWidth: 256 / 256 },
  };
  for (const [key, band] of Object.entries(bands)) {
    const result = processBand(band.channels);
    if (result) Object.assign(band, result);
  }

  return {
    input: channels,
    filtered,
    f0, f1, f2, f3,
    bands,
    channelNames: INPUT_CHANNEL_NAMES,
    smChannelNames: SM_CHANNEL_NAMES,
  };
}
