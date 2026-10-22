import { PolarPlot } from "./PolarPlot";
import { RawValues } from "./RawValues";

export function LiveTab() {
  return (
    <div className="live-grid">
      <section className="panel">
        <div className="panel-head">
          <span>Live 2D scan</span>
          <span className="sub">native angle, clockwise from the cable mark</span>
        </div>
        <PolarPlot />
      </section>
      <div className="live-right">
        <RawValues />
      </div>
    </div>
  );
}
