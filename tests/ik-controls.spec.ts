import { test, expect, Page } from '@playwright/test';

/**
 * RoboArm IK End-to-End Tests
 *
 * These tests run against the live ESP32 web UI served over WiFi. They
 * exercise the Inverse Kinematics controls added in the IK completion work:
 *   - Point IK (free and fixed Ry)
 *   - Full RPY IK (Rx, Ry, Rz)
 *   - Tool-tip mode toggle
 *   - LINE mode (straight-line interpolated move)
 *   - Cartesian jog mode toggle
 *   - Unreachable target handling
 *
 * Pre-requisites:
 *   - ESP32 powered on and connected to WiFi
 *   - ROBOARM_URL env var set (or default 192.168.1.100 reachable)
 *   - `npm install` and `npx playwright install chromium` run
 */

// ── Helpers ─────────────────────────────────────────────────────────────────

/** Wait for WebSocket connection (green dot indicator) */
async function waitForConnection(page: Page) {
  await page.waitForSelector('#dt.ok', { timeout: 15_000 });
}

/** Wait for FK readout to update (non-placeholder) */
async function waitForFK(page: Page) {
  await page.waitForFunction(
    () => {
      const el = document.getElementById('fk');
      return el && el.textContent !== '--' && el.textContent!.includes('X:');
    },
    { timeout: 10_000 }
  );
}

/** Parse the FK readout into an object */
async function getFK(page: Page): Promise<Record<string, number>> {
  return page.evaluate(() => {
    const txt = document.getElementById('fk')?.textContent || '';
    const m: Record<string, number> = {};
    for (const token of txt.split(/\s+/)) {
      const match = token.match(/^([A-Za-z]+):$/);
      if (match) {
        // next token is the value
      }
    }
    // Parse "X: 39 mm Y: 168 mm Z: 0 mm | Rx: 0° Ry: 0° Rz: 13°"
    const xm = txt.match(/X:\s*(-?\d+)/);
    const ym = txt.match(/Y:\s*(-?\d+)/);
    const zm = txt.match(/Z:\s*(-?\d+)/);
    const rxm = txt.match(/Rx:\s*(-?\d+)/);
    const rym = txt.match(/Ry:\s*(-?\d+)/);
    const rzm = txt.match(/Rz:\s*(-?\d+)/);
    m.x = xm ? parseInt(xm[1]) : NaN;
    m.y = ym ? parseInt(ym[1]) : NaN;
    m.z = zm ? parseInt(zm[1]) : NaN;
    m.rx = rxm ? parseInt(rxm[1]) : NaN;
    m.ry = rym ? parseInt(rym[1]) : NaN;
    m.rz = rzm ? parseInt(rzm[1]) : NaN;
    return m;
  });
}

/** Wait for the arm to settle (FK stops changing for 1.5 seconds) */
async function waitForSettle(page: Page, timeoutMs = 12_000) {
  const start = Date.now();
  let lastFK = '';
  let stableMs = 0;
  while (Date.now() - start < timeoutMs) {
    const fk = await page.locator('#fk').textContent() || '';
    if (fk === lastFK) {
      stableMs += 200;
      if (stableMs >= 1500) return;
    } else {
      lastFK = fk;
      stableMs = 0;
    }
    await page.waitForTimeout(200);
  }
}

// ── Tests ───────────────────────────────────────────────────────────────────

test.describe('IK Controls', () => {

  test.beforeEach(async ({ page }) => {
    await page.goto('/');
    await waitForConnection(page);
    await waitForFK(page);
  });

  test('page loads with FK readout visible', async ({ page }) => {
    const fk = await page.locator('#fk').textContent();
    expect(fk).toContain('X:');
    expect(fk).toContain('Y:');
    expect(fk).toContain('Z:');
  });

  test('IK panel has all input fields', async ({ page }) => {
    // Position inputs
    await expect(page.locator('#mvx')).toBeVisible();
    await expect(page.locator('#mvy')).toBeVisible();
    await expect(page.locator('#mvz')).toBeVisible();
    // Orientation inputs
    await expect(page.locator('#mvrx')).toBeVisible();
    await expect(page.locator('#mvr')).toBeVisible();
    await expect(page.locator('#mvrz')).toBeVisible();
    // GO button
    await expect(page.locator('#mvgo')).toBeVisible();
    // LINE checkbox
    await expect(page.locator('#mvln')).toBeVisible();
    // Tool tip button
    await expect(page.locator('#mvtool')).toBeVisible();
    // Feedback span
    await expect(page.locator('#mvfb')).toBeVisible();
  });

  test('IK Move — free Ry (3 args)', async ({ page }) => {
    // Get current FK to know starting position
    const before = await getFK(page);

    // Send IK to a known-reachable point: X=0, Y=180, Z=0 (forward reach)
    await page.fill('#mvx', '0');
    await page.fill('#mvy', '180');
    await page.fill('#mvz', '0');
    // Leave Ry empty for free wrist pitch
    await page.click('#mvgo');

    // Wait for the arm to settle
    await waitForSettle(page);

    // Check FK updated — Y should be close to 180
    const after = await getFK(page);
    expect(after.y).toBeGreaterThan(150);
    expect(Math.abs(after.x)).toBeLessThan(30);

    // Check feedback span shows the command
    const fb = await page.locator('#mvfb').textContent();
    expect(fb).toContain('IK');
    expect(fb).toContain('free Ry');
  });

  test('IK Move — fixed Ry (4 args)', async ({ page }) => {
    await page.fill('#mvx', '0');
    await page.fill('#mvy', '150');
    await page.fill('#mvz', '50');
    await page.fill('#mvr', '20');  // Ry = 20°
    await page.click('#mvgo');

    await waitForSettle(page);

    const fk = await getFK(page);
    expect(fk.y).toBeGreaterThan(120);
    expect(Math.abs(fk.ry - 20)).toBeLessThan(10);

    const fb = await page.locator('#mvfb').textContent();
    expect(fb).toContain('Ry=20');
  });

  test('IK Move — full RPY (6 args)', async ({ page }) => {
    await page.fill('#mvx', '0');
    await page.fill('#mvy', '160');
    await page.fill('#mvz', '0');
    await page.fill('#mvrx', '0');
    await page.fill('#mvr', '10');
    await page.fill('#mvrz', '0');
    await page.click('#mvgo');

    await waitForSettle(page);

    const fb = await page.locator('#mvfb').textContent();
    expect(fb).toContain('RPY');
    expect(fb).toContain('Rx=0');
  });

  test('Tool-tip mode toggle', async ({ page }) => {
    const toolBtn = page.locator('#mvtool');

    // Initially OFF
    await expect(toolBtn).toContainText('OFF');

    // Click to enable
    await toolBtn.click();

    // Wait for status broadcast to reflect the change
    await page.waitForFunction(
      () => document.getElementById('mvtool')?.textContent?.includes('ON'),
      { timeout: 5000 }
    );
    await expect(toolBtn).toContainText('ON');
    await expect(toolBtn).toHaveClass(/on/);

    // Click again to disable
    await toolBtn.click();
    await page.waitForFunction(
      () => document.getElementById('mvtool')?.textContent?.includes('OFF'),
      { timeout: 5000 }
    );
    await expect(toolBtn).toContainText('OFF');
  });

  test('LINE mode checkbox sends LN command', async ({ page }) => {
    // Check the LINE checkbox
    await page.check('#mvln');
    expect(await page.isChecked('#mvln')).toBe(true);

    // Fill coordinates and click GO
    await page.fill('#mvx', '0');
    await page.fill('#mvy', '170');
    await page.fill('#mvz', '0');
    await page.click('#mvgo');

    // Check feedback shows LINE mode
    const fb = await page.locator('#mvfb').textContent();
    expect(fb).toContain('LINE');

    // Wait for settle
    await waitForSettle(page);

    // Uncheck LINE
    await page.uncheck('#mvln');
  });

  test('Cartesian jog toggle', async ({ page }) => {
    const cartBtn = page.locator('#cartbtn');

    // Initially OFF
    await expect(cartBtn).toContainText('OFF');

    // Click to enable
    await cartBtn.click();

    // Wait for status to reflect
    await page.waitForFunction(
      () => document.getElementById('cartbtn')?.textContent?.includes('ON'),
      { timeout: 5000 }
    );
    await expect(cartBtn).toContainText('ON');
    await expect(cartBtn).toHaveClass(/on/);

    // Verify stick labels changed to Cartesian
    const leftSub = page.locator('#sL .sub');
    await expect(leftSub).toContainText('X / Y');

    const rightSub = page.locator('#sR .sub');
    await expect(rightSub).toContainText('Z');

    // Click again to disable
    await cartBtn.click();
    await page.waitForFunction(
      () => document.getElementById('cartbtn')?.textContent?.includes('OFF'),
      { timeout: 5000 }
    );

    // Verify stick labels reverted to joint names
    await expect(leftSub).toContainText('BASE / SHOULDER');
    await expect(rightSub).toContainText('ELBOW / WRIST P.');
  });

  test('joint chips reflect current angles', async ({ page }) => {
    // All 6 joint chips should show angle values (not "--")
    for (let i = 0; i < 6; i++) {
      const text = await page.locator(`#v${i}`).textContent();
      expect(text).toMatch(/^\d+°$/);
    }
  });

  test('FSM state shows IDLE on connected arm', async ({ page }) => {
    const fsm = page.locator('#fsm');
    const text = await fsm.textContent();
    // After homing, should be IDLE or BUSY
    expect(['IDLE', 'BUSY']).toContain(text);
  });

  test('fine jog buttons exist for all joints', async ({ page }) => {
    for (let i = 0; i < 6; i++) {
      await expect(page.locator(`#fv${i}`)).toBeVisible();
    }
    // Check +1/-1 buttons exist (at least for first joint)
    const buttons = page.locator('#fjp button');
    expect(await buttons.count()).toBe(24); // 6 joints × 4 buttons each
  });

  test('preset cards are rendered', async ({ page }) => {
    const cards = page.locator('.ps');
    expect(await cards.count()).toBeGreaterThanOrEqual(4);
  });
});
