import { defineConfig } from '@playwright/test';

/**
 * Playwright config for RoboArm IK end-to-end tests.
 *
 * The tests connect to the live ESP32 web server. Set the ROBOARM_URL
 * environment variable to the ESP32's IP address, e.g.:
 *   ROBOARM_URL=http://192.168.1.42 npx playwright test
 *
 * Defaults to http://192.168.1.100 if not set.
 */
export default defineConfig({
  testDir: './tests',
  timeout: 30_000,
  retries: 0,
  use: {
    baseURL: process.env.ROBOARM_URL || 'http://192.168.1.100',
    // No headless default — show the browser for debugging against real hardware
    headless: true,
    viewport: { width: 520, height: 900 },
    // Longer action timeout for real hardware latency
    actionTimeout: 10_000,
  },
  projects: [
    {
      name: 'chromium',
      use: { browserName: 'chromium' },
    },
  ],
});
