import { useEffect, useRef, useState } from "react";
import { useStore, type State } from "./store";

// Subscribe to a slice of the store but re-render at most every `ms`.
// Used by panels that show 10 Hz data in the DOM (the raw table).
export function useThrottled<T>(selector: (s: State) => T, ms: number): T {
  const [value, setValue] = useState<T>(() => selector(useStore.getState()));
  const sel = useRef(selector);
  sel.current = selector;
  useEffect(() => {
    let last = 0;
    let timer: number | undefined;
    let latest: T = sel.current(useStore.getState());
    const flush = () => {
      timer = undefined;
      last = performance.now();
      setValue(latest);
    };
    const unsub = useStore.subscribe((s) => {
      const v = sel.current(s);
      if (Object.is(v, latest)) return;
      latest = v;
      const now = performance.now();
      if (now - last >= ms) flush();
      else if (timer === undefined) timer = window.setTimeout(flush, ms - (now - last));
    });
    return () => {
      unsub();
      window.clearTimeout(timer);
    };
  }, [ms]);
  return value;
}

// Re-render once a second (for countdowns / relative times).
export function useTick(ms = 1000): number {
  const [t, setT] = useState(0);
  useEffect(() => {
    const id = window.setInterval(() => setT((x) => x + 1), ms);
    return () => window.clearInterval(id);
  }, [ms]);
  return t;
}
