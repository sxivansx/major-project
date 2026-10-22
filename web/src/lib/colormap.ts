// One height ramp, used by the 3D shader, the top-down map and the legend.
// Blue -> teal -> green -> yellow -> red, piecewise linear over 5 stops.

export const RAMP_STOPS: [number, number, number][] = [
  [0.12, 0.23, 0.58],
  [0.16, 0.62, 0.56],
  [0.54, 0.79, 0.15],
  [1.0, 0.79, 0.23],
  [1.0, 0.35, 0.37],
];

export function rampRgb(t: number, out: [number, number, number] = [0, 0, 0]): [number, number, number] {
  if (!(t >= 0)) t = 0;
  if (t > 1) t = 1;
  const s = t * (RAMP_STOPS.length - 1);
  const i = Math.min(RAMP_STOPS.length - 2, Math.floor(s));
  const f = s - i;
  const a = RAMP_STOPS[i];
  const b = RAMP_STOPS[i + 1];
  out[0] = a[0] + (b[0] - a[0]) * f;
  out[1] = a[1] + (b[1] - a[1]) * f;
  out[2] = a[2] + (b[2] - a[2]) * f;
  return out;
}

export function rampCss(t: number): string {
  const [r, g, b] = rampRgb(t);
  return `rgb(${Math.round(r * 255)},${Math.round(g * 255)},${Math.round(b * 255)})`;
}

export function rampCssGradient(): string {
  const parts = RAMP_STOPS.map((c, i) => {
    const pct = (i / (RAMP_STOPS.length - 1)) * 100;
    return `rgb(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)}) ${pct}%`;
  });
  return `linear-gradient(to right, ${parts.join(", ")})`;
}

// GLSL implementation of the same ramp.
export const RAMP_GLSL = `
vec3 rampColor(float t) {
  t = clamp(t, 0.0, 1.0);
  vec3 c0 = vec3(${RAMP_STOPS[0].join(", ")});
  vec3 c1 = vec3(${RAMP_STOPS[1].join(", ")});
  vec3 c2 = vec3(${RAMP_STOPS[2].join(", ")});
  vec3 c3 = vec3(${RAMP_STOPS[3].join(", ")});
  vec3 c4 = vec3(${RAMP_STOPS[4].join(", ")});
  float s = t * 4.0;
  if (s < 1.0) return mix(c0, c1, s);
  if (s < 2.0) return mix(c1, c2, s - 1.0);
  if (s < 3.0) return mix(c2, c3, s - 2.0);
  return mix(c3, c4, s - 3.0);
}
`;
