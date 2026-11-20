import { test, expect } from '@playwright/test'

test('loads sample mission with simulation warning and telemetry', async ({ page }) => {
  await page.goto('/')
  await expect(page.getByTestId('simulation-badge')).toContainText('NO REAL FLIGHT CONTROL')
  await expect(page.getByTestId('left-panel')).toContainText('Mission')
  await expect(page.getByTestId('right-panel')).toContainText('Vehicle')
  await expect(page.getByTestId('waypoint-list').locator('li')).not.toHaveCount(0)
  await expect(page.getByTestId('value-Battery')).toContainText('.')
})

test('play advances telemetry and reset returns to start', async ({ page }) => {
  await page.goto('/')
  const clock = page.locator('.clock')
  const before = await clock.textContent()
  await page.getByTestId('play').click()
  await page.waitForTimeout(1300)
  const after = await clock.textContent()
  expect(after).not.toBe(before)
  await page.getByTestId('play').click() // pause
  await page.getByTestId('reset').click()
  await expect(clock).toContainText('frame 1/')
})

test('selecting a waypoint highlights it in the list', async ({ page }) => {
  await page.goto('/')
  const wp = page.getByTestId('waypoint-1')
  await wp.click()
  await expect(wp).toHaveClass(/selected/)
  await expect(page.locator('.scene-caption')).toContainText('Selected')
})

test('low battery scenario shows RTL safety event', async ({ page }) => {
  await page.goto('/')
  await page.getByTestId('scenario-select').selectOption('low_battery')
  await expect(page.getByTestId('event-list')).toContainText('rtl')
  await expect(page.getByTestId('value-Battery')).toHaveText('15.0')
})

test('camera presets switch views', async ({ page }) => {
  await page.goto('/')
  await page.getByTestId('view-top').click()
  await expect(page.getByTestId('view-top')).toHaveClass(/active/)
  await expect(page.locator('canvas')).toHaveCount(1)
  await page.getByTestId('view-follow').click()
  await expect(page.getByTestId('view-follow')).toHaveClass(/active/)
})

test('scrub changes the rendered telemetry frame', async ({ page }) => {
  await page.goto('/')
  const scrub = page.getByTestId('scrub')
  await scrub.fill('3')
  await expect(page.locator('.clock')).toContainText('frame 4/')
})
