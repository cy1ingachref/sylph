import { defineConfig, devices } from '@playwright/test'

export default defineConfig({
  testDir: './tests',
  fullyParallel: true,
  reporter: 'list',
  use: {
    baseURL: 'http://127.0.0.1:8099',
    ...devices['Desktop Chrome'],
    launchOptions: { args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'] },
  },
  webServer: {
    command: 'npm run preview -- --port 8099',
    url: 'http://127.0.0.1:8099',
    reuseExistingServer: false,
    timeout: 30_000,
  },
})
