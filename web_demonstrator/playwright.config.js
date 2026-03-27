import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './tests',
  timeout: 30000,
  use: {
    baseURL: 'http://localhost:3210',
    browserName: 'firefox',
  },
  webServer: {
    command: 'python3 -m http.server 3210',
    port: 3210,
    reuseExistingServer: true,
  },
});
