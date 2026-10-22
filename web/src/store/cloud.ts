// Point cloud accumulator. Lives outside React/zustand on purpose: the 3D
// view reads it in useFrame and only uploads the slice that changed. The
// store just gets a throttled count for the UI.

export const CLOUD_CAPACITY = 2_000_000;

export interface CloudState {
  positions: Float32Array; // CLOUD_CAPACITY * 3, base frame, metres (x, y, z)
  count: number; // points in use
  version: number; // bumps on every append/clear
  jobId: number; // job_id of the frames currently accumulated, -1 = none
  dirtyFrom: number; // first point index changed since the renderer last consumed it
  zMin: number;
  zMax: number;
  dropped: number; // points that did not fit
}

export const cloud: CloudState = {
  positions: new Float32Array(CLOUD_CAPACITY * 3),
  count: 0,
  version: 0,
  jobId: -1,
  dirtyFrom: 0,
  zMin: Infinity,
  zMax: -Infinity,
  dropped: 0,
};

export function cloudClear(): void {
  cloud.count = 0;
  cloud.dirtyFrom = 0;
  cloud.zMin = Infinity;
  cloud.zMax = -Infinity;
  cloud.dropped = 0;
  cloud.jobId = -1;
  cloud.version++;
}

// Append n points from xyz (n * 3 floats). Returns how many were kept.
export function cloudAppend(xyz: Float32Array, n: number, jobId: number): number {
  if (cloud.jobId !== -1 && jobId !== cloud.jobId) cloudClear();
  cloud.jobId = jobId;
  const room = CLOUD_CAPACITY - cloud.count;
  const keep = Math.min(n, room);
  if (keep <= 0) {
    cloud.dropped += n;
    return 0;
  }
  const start = cloud.count;
  cloud.positions.set(xyz.subarray(0, keep * 3), start * 3);
  let zmin = cloud.zMin;
  let zmax = cloud.zMax;
  for (let i = 2; i < keep * 3; i += 3) {
    const z = xyz[i];
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
  }
  cloud.zMin = zmin;
  cloud.zMax = zmax;
  if (cloud.dirtyFrom > start) cloud.dirtyFrom = start;
  cloud.count = start + keep;
  cloud.dropped += n - keep;
  cloud.version++;
  return keep;
}
