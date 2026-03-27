import { STAGES } from './stages.js';

let currentStageIndex = 0;
let pipelineData = null; // filled after pipeline run

// --- Pipeline bar ---
function buildPipelineBar() {
  const bar = document.getElementById('pipeline-bar');
  bar.innerHTML = '';
  STAGES.forEach((stage, i) => {
    if (i > 0) {
      const arrow = document.createElement('span');
      arrow.className = 'stage-arrow';
      arrow.textContent = '→';
      bar.appendChild(arrow);
    }
    const badge = document.createElement('span');
    badge.className = 'stage-badge' + (i === currentStageIndex ? ' active' : '');
    badge.textContent = stage.title;
    badge.dataset.index = i;
    badge.addEventListener('click', () => goToStage(i));
    bar.appendChild(badge);
  });
}

function goToStage(index) {
  currentStageIndex = Math.max(0, Math.min(index, STAGES.length - 1));
  updateUI();
}

function updateUI() {
  // Update badges
  document.querySelectorAll('.stage-badge').forEach((el, i) => {
    el.classList.toggle('active', i === currentStageIndex);
    el.classList.toggle('done', pipelineData && i < currentStageIndex);
  });

  // Update nav buttons
  document.getElementById('prev-btn').disabled = currentStageIndex === 0;
  document.getElementById('next-btn').disabled = currentStageIndex === STAGES.length - 1;

  // Update detail panel
  const stage = STAGES[currentStageIndex];
  document.getElementById('explanation').innerHTML = stage.description;

  const plotArea = document.getElementById('plot-area');
  if (stage.render && pipelineData) {
    stage.render(pipelineData, plotArea);
  } else {
    plotArea.innerHTML = '<p style="color:#8b949e;text-align:center;padding-top:160px">' +
      (pipelineData ? 'Visualization not yet implemented for this stage.' :
       'Click <strong>Run Pipeline</strong> to start.') + '</p>';
  }
}

// --- Signal config toggle ---
document.getElementById('config-toggle').addEventListener('click', () => {
  document.getElementById('signal-config').classList.toggle('collapsed');
});

// --- Nav ---
document.getElementById('prev-btn').addEventListener('click', () => goToStage(currentStageIndex - 1));
document.getElementById('next-btn').addEventListener('click', () => goToStage(currentStageIndex + 1));

// --- Run pipeline ---
document.getElementById('run-pipeline').addEventListener('click', async () => {
  const { runPipeline } = await import('./signals.js');
  pipelineData = runPipeline(document.getElementById('scenario-select').value);
  goToStage(0);
});

// --- Custom controls visibility ---
document.getElementById('scenario-select').addEventListener('change', (e) => {
  document.getElementById('custom-controls').style.display =
    e.target.value === 'custom' ? 'block' : 'none';
});

// --- Init ---
buildPipelineBar();
updateUI();
