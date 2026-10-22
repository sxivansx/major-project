import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import { useStore } from "./store/store";
import { useOps } from "./store/ops";
import { cloud } from "./store/cloud";
import "./styles.css";

// Debug hook for the browser console: __obslam.store.getState(), __obslam.cloud.count, __obslam.ops.getState()
(window as unknown as { __obslam: unknown }).__obslam = { store: useStore, ops: useOps, cloud };

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
