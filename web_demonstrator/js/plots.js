// Plotly.js wrapper helpers for LFR demonstrator

const DARK_LAYOUT = {
  paper_bgcolor: 'rgba(0,0,0,0)',
  plot_bgcolor: '#0d1117',
  font: { color: '#e6edf3', size: 11 },
  margin: { t: 30, r: 20, b: 40, l: 50 },
  xaxis: { gridcolor: '#30363d', zerolinecolor: '#30363d' },
  yaxis: { gridcolor: '#30363d', zerolinecolor: '#30363d' },
};

const COLORS = ['#58a6ff', '#3fb950', '#d29922', '#f778ba', '#bc8cff'];
const CHANNEL_NAMES = ['E1', 'E2', 'B1', 'B2', 'B3'];

export function timeDomain(container, signals, sampleRate, options = {}) {
  const n = signals[0].length;
  const t = Array.from({ length: n }, (_, i) => i / sampleRate);
  const traces = signals.map((sig, i) => ({
    x: t, y: Array.from(sig),
    name: options.names?.[i] ?? CHANNEL_NAMES[i],
    line: { color: COLORS[i % COLORS.length], width: 1 },
    type: 'scatter', mode: 'lines',
  }));
  const layout = {
    ...DARK_LAYOUT,
    title: options.title ?? 'Time Domain',
    xaxis: { ...DARK_LAYOUT.xaxis, title: options.xLabel ?? 'Time (s)' },
    yaxis: { ...DARK_LAYOUT.yaxis, title: options.yLabel ?? 'Amplitude' },
    showlegend: true, legend: { x: 1, xanchor: 'right', y: 1 },
  };
  Plotly.newPlot(container, traces, layout, { responsive: true });
}

export function spectrum(container, magnitudes, binFreqs, options = {}) {
  const traces = [{
    x: Array.from(binFreqs),
    y: Array.from(magnitudes),
    type: 'scatter', mode: 'lines',
    line: { color: options.color ?? COLORS[0], width: 1.5 },
    name: options.name ?? 'Magnitude',
  }];
  const layout = {
    ...DARK_LAYOUT,
    title: options.title ?? 'Frequency Spectrum',
    xaxis: { ...DARK_LAYOUT.xaxis, title: 'Frequency (Hz)' },
    yaxis: { ...DARK_LAYOUT.yaxis, title: 'Magnitude (dB)', type: 'log' },
  };
  Plotly.newPlot(container, traces, layout, { responsive: true });
}

export function heatmap(container, matrix, options = {}) {
  const labels = options.labels ?? CHANNEL_NAMES;
  const traces = [{
    z: matrix, type: 'heatmap',
    x: labels, y: labels,
    colorscale: 'Viridis', showscale: true,
  }];
  const layout = {
    ...DARK_LAYOUT,
    title: options.title ?? 'Spectral Matrix',
    yaxis: { ...DARK_LAYOUT.yaxis, autorange: 'reversed' },
  };
  Plotly.newPlot(container, traces, layout, { responsive: true });
}

export function barChart(container, values, labels, options = {}) {
  const traces = [{
    x: labels, y: Array.from(values),
    type: 'bar',
    marker: { color: options.color ?? COLORS[0] },
  }];
  const layout = {
    ...DARK_LAYOUT,
    title: options.title ?? '',
    xaxis: { ...DARK_LAYOUT.xaxis, title: options.xLabel ?? '' },
    yaxis: { ...DARK_LAYOUT.yaxis, title: options.yLabel ?? '' },
  };
  Plotly.newPlot(container, traces, layout, { responsive: true });
}

export function polar(container, theta, r, options = {}) {
  const traces = [{
    type: 'scatterpolar', mode: 'markers',
    theta: Array.from(theta), r: Array.from(r),
    marker: { color: COLORS[0], size: 6 },
  }];
  const layout = {
    ...DARK_LAYOUT,
    title: options.title ?? '',
    polar: {
      bgcolor: '#0d1117',
      angularaxis: { gridcolor: '#30363d', linecolor: '#30363d' },
      radialaxis: { gridcolor: '#30363d', linecolor: '#30363d' },
    },
  };
  Plotly.newPlot(container, traces, layout, { responsive: true });
}

export function multiPlot(container, plotConfigs) {
  container.innerHTML = '';
  for (const cfg of plotConfigs) {
    const div = document.createElement('div');
    div.style.marginBottom = '12px';
    container.appendChild(div);
    cfg.fn(div, ...cfg.args);
  }
}
