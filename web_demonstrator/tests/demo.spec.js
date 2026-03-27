import { test, expect } from '@playwright/test';

test.describe('Page load', () => {
  test('renders header and pipeline bar', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('header h1')).toHaveText('LFR Signal Processing Pipeline');
    const badges = page.locator('.stage-badge');
    await expect(badges).toHaveCount(9);
  });

  test('first stage badge is active by default', async ({ page }) => {
    await page.goto('/');
    const first = page.locator('.stage-badge').first();
    await expect(first).toHaveClass(/active/);
    await expect(first).toHaveText('INPUT');
  });

  test('explanation panel shows INPUT description', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('#explanation')).toContainText('Input Signal');
  });

  test('plot area shows "Run Pipeline" prompt', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('#plot-area')).toContainText('Run Pipeline');
  });

  test('no console errors on load', async ({ page }) => {
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto('/');
    await page.waitForTimeout(500);
    expect(errors).toEqual([]);
  });
});

test.describe('Navigation', () => {
  test('clicking stage badges updates active state', async ({ page }) => {
    await page.goto('/');
    const badges = page.locator('.stage-badge');
    await badges.nth(2).click();
    await expect(badges.nth(2)).toHaveClass(/active/);
    await expect(badges.nth(0)).not.toHaveClass(/active/);
  });

  test('clicking stage badges updates explanation', async ({ page }) => {
    await page.goto('/');
    await page.locator('.stage-badge').nth(1).click();
    await expect(page.locator('#explanation')).toContainText('IIR');
  });

  test('next button advances stage', async ({ page }) => {
    await page.goto('/');
    await page.locator('#next-btn').click();
    await expect(page.locator('.stage-badge').nth(1)).toHaveClass(/active/);
    await expect(page.locator('#explanation')).toContainText('IIR');
  });

  test('prev button goes back', async ({ page }) => {
    await page.goto('/');
    await page.locator('#next-btn').click();
    await page.locator('#prev-btn').click();
    await expect(page.locator('.stage-badge').first()).toHaveClass(/active/);
  });

  test('prev button is disabled on first stage', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('#prev-btn')).toBeDisabled();
  });

  test('next button is disabled on last stage', async ({ page }) => {
    await page.goto('/');
    await page.locator('.stage-badge').last().click();
    await expect(page.locator('#next-btn')).toBeDisabled();
  });

  test('can navigate to all 9 stages', async ({ page }) => {
    await page.goto('/');
    const titles = ['INPUT', 'IIR', '÷4 → f0', 'f1/f2/f3', 'WINDOW', 'FFT', 'SM', 'AVG', 'BP1'];
    for (let i = 0; i < 9; i++) {
      await page.locator('.stage-badge').nth(i).click();
      await expect(page.locator('.stage-badge').nth(i)).toHaveClass(/active/);
    }
  });
});

test.describe('Signal config panel', () => {
  test('config panel starts collapsed', async ({ page }) => {
    await page.goto('/');
    await expect(page.locator('#signal-config')).toHaveClass(/collapsed/);
    await expect(page.locator('#config-panel')).not.toBeVisible();
  });

  test('toggle button expands config panel', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    await expect(page.locator('#config-panel')).toBeVisible();
    await expect(page.locator('#run-pipeline')).toBeVisible();
  });

  test('scenario dropdown has 5 options', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    const options = page.locator('#scenario-select option');
    await expect(options).toHaveCount(5);
  });
});

test.describe('Pipeline execution', () => {
  test('run pipeline produces plots on INPUT stage', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    await page.locator('#run-pipeline').click();
    // Wait for Plotly to render
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });
    expect(errors).toEqual([]);
  });

  test('run pipeline with each scenario produces no errors', async ({ page }) => {
    const scenarios = ['pure-tone', 'whistler', 'turbulence', 'multi-component'];
    for (const scenario of scenarios) {
      await page.goto('/');
      await page.locator('#config-toggle').click();
      await page.locator('#scenario-select').selectOption(scenario);

      const errors = [];
      page.on('pageerror', e => errors.push(e.message));

      await page.locator('#run-pipeline').click();
      await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });
      expect(errors, `Errors with scenario "${scenario}"`).toEqual([]);
    }
  });

  test('navigating through all stages after pipeline shows plots', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    // Stages 0-7 (skip BP1/WASM for now as it may not load)
    for (let i = 0; i < 8; i++) {
      await page.locator('.stage-badge').nth(i).click();
      // Wait for render
      await page.waitForTimeout(500);
      const plotCount = await page.locator('.js-plotly-plot').count();
      const hasPlot = plotCount > 0;
      const hasText = await page.locator('#plot-area').textContent();
      // Either a plot rendered or there's meaningful content (like the AVG message)
      expect(hasPlot || hasText.length > 10,
        `Stage ${i} should render content`).toBeTruthy();
    }
    expect(errors).toEqual([]);
  });

  test('backward navigation re-renders plots correctly', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    // Go forward to stage 3 (f1/f2/f3 — multiPlot)
    await page.locator('.stage-badge').nth(3).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const forwardPlots = await page.locator('.js-plotly-plot').count();
    expect(forwardPlots, 'Stage 3 should have plots').toBeGreaterThan(0);

    // Go back to stage 0 (INPUT — single timeDomain)
    await page.locator('.stage-badge').nth(0).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const backPlots = await page.locator('.js-plotly-plot').count();
    expect(backPlots, 'Stage 0 should have plots after going back').toBeGreaterThan(0);

    // Go forward to stage 5 (FFT — spectrum)
    await page.locator('.stage-badge').nth(5).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const fftPlots = await page.locator('.js-plotly-plot').count();
    expect(fftPlots, 'Stage 5 should have plots').toBeGreaterThan(0);

    // Back to stage 1 (IIR — multiPlot)
    await page.locator('.stage-badge').nth(1).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const iirPlots = await page.locator('.js-plotly-plot').count();
    expect(iirPlots, 'Stage 1 should have plots after going back').toBeGreaterThan(0);

    expect(errors).toEqual([]);
  });

  test('prev/next buttons re-render plots each time', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    // Navigate forward 5 stages then backward 5 using buttons
    for (let i = 0; i < 5; i++) {
      await page.locator('#next-btn').click();
      await page.waitForTimeout(300);
    }
    for (let i = 0; i < 5; i++) {
      await page.locator('#prev-btn').click();
      await page.waitForTimeout(300);
    }
    // Should be back at stage 0 with a working plot
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const plotCount = await page.locator('.js-plotly-plot').count();
    expect(plotCount).toBeGreaterThan(0);
    expect(errors).toEqual([]);
  });

  test('BP1 stage renders WASM output without errors', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    // Navigate to BP1 (last stage)
    await page.locator('.stage-badge').last().click();
    // BP1 is async (WASM load), wait for plot or content
    await page.waitForFunction(() => {
      const plotArea = document.getElementById('plot-area');
      return plotArea.querySelector('.js-plotly-plot') || plotArea.textContent.length > 20;
    }, { timeout: 15000 });

    const hasPlot = await page.locator('.js-plotly-plot').count() > 0;
    expect(hasPlot, 'BP1 should render WASM-computed plots').toBeTruthy();
    expect(errors).toEqual([]);
  });

  test('switching scenario re-runs pipeline and updates plots', async ({ page }) => {
    await page.goto('/');
    await page.locator('#config-toggle').click();

    const errors = [];
    page.on('pageerror', e => errors.push(e.message));

    // Run with pure-tone
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    // Navigate to FFT stage
    await page.locator('.stage-badge').nth(5).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });

    // Switch to turbulence and re-run
    await page.locator('#scenario-select').selectOption('turbulence');
    await page.locator('#run-pipeline').click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 10000 });

    // Navigate to FFT again — should have fresh plot
    await page.locator('.stage-badge').nth(5).click();
    await page.waitForSelector('.js-plotly-plot', { timeout: 5000 });
    const plotCount = await page.locator('.js-plotly-plot').count();
    expect(plotCount).toBeGreaterThan(0);

    expect(errors).toEqual([]);
  });
});
