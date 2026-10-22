import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Dev: Vite on :5173 proxies /api and /ws to the core server (or the mock) on
// :8000, or on CORE_URL when set (e.g. CORE_URL=http://127.0.0.1:8001 bun run dev).
// Prod: the core server serves web/dist itself, so no proxy is needed.
const core = process.env.CORE_URL ?? "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    strictPort: false,
    proxy: {
      "/api": { target: core, changeOrigin: false },
      "/ws": { target: core.replace(/^http/, "ws"), ws: true },
    },
  },
  build: {
    outDir: "dist",
    sourcemap: false,
    chunkSizeWarningLimit: 1500,
  },
});
