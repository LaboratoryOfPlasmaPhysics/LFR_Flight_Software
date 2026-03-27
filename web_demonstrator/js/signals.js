// Signal generation scenarios for LFR WASM demonstrator

import { runDSPPipeline } from './dsp.js';

const ADC_RATE = 98304;
const DURATION = 0.05;
const N = Math.round(ADC_RATE * DURATION);

function zeros() { return new Float32Array(N); }

function sineWave(freq, phase = 0) {
  const out = new Float32Array(N);
  const w = 2 * Math.PI * freq / ADC_RATE;
  for (let i = 0; i < N; i++) out[i] = Math.sin(w * i + phase);
  return out;
}

function frequencySweep(f0, f1, phase = 0) {
  const out = new Float32Array(N);
  const rate = (f1 - f0) / (N - 1);
  for (let i = 0; i < N; i++) {
    const freq = f0 + rate * i;
    const t = i / ADC_RATE;
    out[i] = Math.sin(2 * Math.PI * (f0 * t + 0.5 * rate * ADC_RATE * t * t) + phase);
  }
  return out;
}

// Voss-McCartney pink noise
function pinkNoise() {
  const out = new Float32Array(N);
  const numRows = 12;
  const rows = new Float64Array(numRows);
  let runningTotal = 0;
  for (let i = 0; i < numRows; i++) {
    rows[i] = (Math.random() - 0.5) * 2;
    runningTotal += rows[i];
  }
  const maxKey = (1 << numRows) - 1;
  for (let i = 0; i < N; i++) {
    const key = i & maxKey;
    if (key !== 0) {
      // Find lowest set bit to determine which row to update
      const lsb = key & (-key);
      const row = Math.round(Math.log2(lsb));
      if (row < numRows) {
        runningTotal -= rows[row];
        rows[row] = (Math.random() - 0.5) * 2;
        runningTotal += rows[row];
      }
    }
    out[i] = (runningTotal + (Math.random() - 0.5) * 2) / (numRows + 1);
  }
  return out;
}

const SCENARIOS = {
  'pure-tone': () => ({
    E1: zeros(), E2: zeros(),
    B1: sineWave(1000), B2: zeros(), B3: zeros(),
  }),

  'whistler': () => ({
    E1: zeros(), E2: zeros(),
    B1: frequencySweep(500, 2000, 0),
    B2: frequencySweep(500, 2000, Math.PI / 2),
    B3: zeros(),
  }),

  'turbulence': () => ({
    E1: pinkNoise(), E2: pinkNoise(),
    B1: pinkNoise(), B2: pinkNoise(), B3: pinkNoise(),
  }),

  'multi-component': () => {
    const e1 = new Float32Array(N);
    const e2 = new Float32Array(N);
    const s200 = sineWave(200);
    const s3k = sineWave(3000);
    for (let i = 0; i < N; i++) {
      e1[i] = s200[i] + s3k[i];
      e2[i] = s200[i] + s3k[i];
    }
    return { E1: e1, E2: e2, B1: sineWave(200), B2: zeros(), B3: zeros() };
  },
};

export function generateSignal(scenarioId) {
  const gen = SCENARIOS[scenarioId];
  if (!gen) throw new Error(`Unknown scenario: ${scenarioId}`);
  return gen();
}

export function runPipeline(scenarioId) {
  return runDSPPipeline(generateSignal(scenarioId));
}
