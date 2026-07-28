import { defineConfig } from "vite";

export default defineConfig({
  base: process.env.GH_PAGES ? "/evtol-studio/" : "/",
});
