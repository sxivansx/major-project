import { useState, type ReactNode } from "react";

// The one layout every tab uses: a row of large numbers, the main view left
// with one plain-language column right, an action bar, and an Advanced drawer
// under it (own scroll, remembered per tab in localStorage).

export interface BigNumber {
  label: string;
  value: ReactNode;
  className?: string; // ok / warn / err on the value
}

export function useAdvancedFlag(key: string): [boolean, (v: boolean) => void] {
  const storageKey = `obslam.${key}.advanced`;
  const [open, setOpen] = useState<boolean>(() => {
    try {
      return localStorage.getItem(storageKey) === "1";
    } catch {
      return false;
    }
  });
  const set = (v: boolean) => {
    setOpen(v);
    try {
      localStorage.setItem(storageKey, v ? "1" : "0");
    } catch {
      // private window or storage blocked: the setting just does not persist
    }
  };
  return [open, set];
}

interface Props {
  numbers?: BigNumber[];
  numbersDim?: boolean; // grey the numbers (a sweep is running, values are stale)
  main: ReactNode;
  side: ReactNode;
  actions: ReactNode;
  actionsNote?: ReactNode; // one short line under the buttons
  banner?: ReactNode; // a plain full-width line above the numbers (urgent things)
  advanced?: ReactNode; // omit for tabs without a drawer
  advancedOpen?: boolean;
  onAdvanced?: (open: boolean) => void;
}

export function Shell({ numbers, numbersDim, main, side, actions, actionsNote, banner, advanced, advancedOpen, onAdvanced }: Props) {
  const hasDrawer = advanced !== undefined && onAdvanced !== undefined;
  return (
    <div className="ops-simple">
      {banner}
      {numbers && numbers.length > 0 && (
        <div
          className={"bignums ops-top" + (numbersDim ? " scanning" : "")}
          style={{ gridTemplateColumns: `repeat(${numbers.length}, minmax(0, 1fr))` }}
        >
          {numbers.map((n) => (
            <div className="bignum" key={n.label}>
              <div className="label">{n.label}</div>
              <div className={"value" + (n.className ? " " + n.className : "")}>{n.value}</div>
            </div>
          ))}
        </div>
      )}
      <div className="ops-main">
        {main}
        {side}
      </div>
      <div className="ops-actions">
        {actions}
        <span className="spacer" />
        {hasDrawer && (
          <button className="ops-adv-toggle" onClick={() => onAdvanced(!advancedOpen)} aria-expanded={!!advancedOpen}>
            Advanced {advancedOpen ? "▾" : "▸"}
          </button>
        )}
        {actionsNote && <div className="ops-actions-note">{actionsNote}</div>}
      </div>
      {hasDrawer && advancedOpen && <div className="ops-advanced">{advanced}</div>}
    </div>
  );
}
