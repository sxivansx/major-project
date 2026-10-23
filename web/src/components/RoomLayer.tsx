import { useEffect, useLayoutEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { Html } from "@react-three/drei";
import type { RoomFrame, RoomStation, RoomState } from "../api/types";
import { useOps } from "../store/ops";
import { useRoom } from "../store/room";
import { useStore } from "../store/store";

// The handheld room map (kind 6) as a 3D layer in the cloud view, in the base
// frame (x = cable mark, y left, z up), drawn inside the cloud view's z-up group:
//  - occupied cells → one InstancedMesh of boxes from the floor up to an assumed wall height
//  - free cells (dark) and open-edge cells (amber) → one textured plane at the floor
//  - walked path → a line just above the floor, the pose → a flat triangle along theta
//  - next station → an amber ring on the floor with a short pole
// The geometry is rebuilt only when a new grid frame arrives (≤ 2 Hz) or a height changes.

const WALL = "#c8c8c8";
const FREE: [number, number, number] = [0x3a, 0x3a, 0x3a];
const EDGE: [number, number, number] = [0xe0, 0xa0, 0x40];
const ACCENT = "#4d9de0";
const AMBER = "#e0a040";
const DEFAULT_SCANNER_H = 1.0;

// Floor height in the base frame: the fitted floor when the analysis has one,
// otherwise the scan plane minus the scanner height above the floor.
export function useRoomFloorZ(): { floorZ: number; source: string } {
  const mount = useStore((s) => s.config?.mount);
  const manual = useStore((s) => s.roomScannerH);
  const floor = useOps((s) => s.findings?.floor);
  const height = useOps((s) => s.findings?.scanner.height_above_floor_m ?? null);
  const planeZ = ((mount?.axis_z_mm ?? 80) + (mount?.h_mm ?? 0)) / 1000;
  if (manual !== null && manual > 0) return { floorZ: planeZ - manual, source: `scanner ${manual.toFixed(2)} m above the floor (set)` };
  if (floor?.fitted && typeof floor.z_at_origin_m === "number") return { floorZ: floor.z_at_origin_m, source: "fitted floor" };
  if (height !== null && height > 0) return { floorZ: planeZ - height, source: `scanner ${height.toFixed(2)} m above the floor (findings)` };
  return { floorZ: planeZ - DEFAULT_SCANNER_H, source: `scanner assumed ${DEFAULT_SCANNER_H.toFixed(1)} m above the floor` };
}

// Bounds of the room grid in the base frame, for Fit.
export function roomBounds(g: RoomFrame | null): { cx: number; cy: number; extent: number } | null {
  if (!g || g.w === 0 || g.h === 0) return null;
  return { cx: g.x0_m + (g.w * g.cell_m) / 2, cy: g.y0_m + (g.h * g.cell_m) / 2, extent: Math.max(g.w, g.h) * g.cell_m };
}

function Walls({ grid, floorZ, wallH }: { grid: RoomFrame; floorZ: number; wallH: number }) {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const occupied = useMemo(() => {
    const out: number[] = [];
    for (let i = 0; i < grid.cells.length; i++) if (grid.cells[i] === 2) out.push(i);
    return out;
  }, [grid]);
  // capacity grows in powers of two so a growing map does not recreate the mesh every frame
  const capacity = Math.max(256, 2 ** Math.ceil(Math.log2(Math.max(1, occupied.length))));
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), []);
  const material = useMemo(() => new THREE.MeshLambertMaterial({ color: WALL, transparent: true, opacity: 0.9 }), []);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => material.dispose(), [material]);

  useLayoutEffect(() => {
    const mesh = meshRef.current;
    if (!mesh) return;
    const m = new THREE.Matrix4();
    const pos = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const scl = new THREE.Vector3(grid.cell_m, grid.cell_m, wallH);
    for (let k = 0; k < occupied.length; k++) {
      const i = occupied[k];
      const iy = Math.floor(i / grid.w);
      const ix = i - iy * grid.w;
      pos.set(grid.x0_m + (ix + 0.5) * grid.cell_m, grid.y0_m + (iy + 0.5) * grid.cell_m, floorZ + wallH / 2);
      m.compose(pos, q, scl);
      mesh.setMatrixAt(k, m);
    }
    mesh.count = occupied.length;
    mesh.instanceMatrix.needsUpdate = true;
    mesh.computeBoundingSphere();
  }, [grid, occupied, floorZ, wallH, capacity]);

  return <instancedMesh key={capacity} ref={meshRef} args={[geometry, material, capacity]} frustumCulled={false} />;
}

function Floor({ grid, floorZ }: { grid: RoomFrame; floorZ: number }) {
  const texture = useMemo(() => {
    const data = new Uint8Array(grid.w * grid.h * 4);
    for (let i = 0; i < grid.cells.length; i++) {
      const v = grid.cells[i];
      const c = v === 1 ? FREE : v === 3 ? EDGE : null;
      if (!c) continue;
      data[i * 4] = c[0];
      data[i * 4 + 1] = c[1];
      data[i * 4 + 2] = c[2];
      data[i * 4 + 3] = 255;
    }
    // row 0 = lowest y: a DataTexture's first row is at v = 0, which the plane maps to −y
    const t = new THREE.DataTexture(data, grid.w, grid.h, THREE.RGBAFormat);
    t.magFilter = THREE.NearestFilter;
    t.minFilter = THREE.NearestFilter;
    t.needsUpdate = true;
    return t;
  }, [grid]);
  useEffect(() => () => texture.dispose(), [texture]);
  const w = grid.w * grid.cell_m;
  const h = grid.h * grid.cell_m;
  return (
    <mesh position={[grid.x0_m + w / 2, grid.y0_m + h / 2, floorZ]}>
      <planeGeometry args={[w, h]} />
      <meshBasicMaterial map={texture} transparent alphaTest={0.5} side={THREE.DoubleSide} />
    </mesh>
  );
}

function PathAndPose({ room, floorZ }: { room: RoomState; floorZ: number }) {
  const line = useMemo(() => {
    const pts = room.path.map(([x, y]) => new THREE.Vector3(x, y, floorZ + 0.02));
    const g = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.Line(g, new THREE.LineBasicMaterial({ color: ACCENT }));
  }, [room.path, floorZ]);
  useEffect(
    () => () => {
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    },
    [line],
  );
  const tri = useMemo(() => {
    const s = new THREE.Shape();
    s.moveTo(0.22, 0);
    s.lineTo(-0.1, 0.1);
    s.lineTo(-0.1, -0.1);
    s.closePath();
    return new THREE.ShapeGeometry(s);
  }, []);
  useEffect(() => () => tri.dispose(), [tri]);
  const p = room.pose;
  return (
    <>
      {room.path.length > 1 && <primitive object={line} />}
      {p && (
        <mesh geometry={tri} position={[p.x, p.y, floorZ + 0.03]} rotation={[0, 0, (p.theta_deg * Math.PI) / 180]}>
          <meshBasicMaterial color={room.lost ? "#e06060" : ACCENT} side={THREE.DoubleSide} />
        </mesh>
      )}
    </>
  );
}

// Station mode: a numbered marker at each station with a heading tick along theta.
function Stations({ stations, floorZ }: { stations: RoomStation[]; floorZ: number }) {
  return (
    <>
      {stations.map((st) => (
        <group key={st.n} position={[st.x, st.y, floorZ + 0.025]} rotation={[0, 0, (st.theta_deg * Math.PI) / 180]}>
          <mesh>
            <ringGeometry args={[0.1, 0.14, 32]} />
            <meshBasicMaterial color={ACCENT} side={THREE.DoubleSide} />
          </mesh>
          {/* heading tick along local +x */}
          <mesh position={[0.24, 0, 0]}>
            <planeGeometry args={[0.18, 0.035]} />
            <meshBasicMaterial color={ACCENT} side={THREE.DoubleSide} />
          </mesh>
          <Html center position={[0, 0, 0.05]} zIndexRange={[10, 0]}>
            <div className="station-label">{st.n}</div>
          </Html>
        </group>
      ))}
    </>
  );
}

function NextStation({ x, y, floorZ }: { x: number; y: number; floorZ: number }) {
  const poleH = 1.0;
  return (
    <group position={[x, y, floorZ]}>
      <mesh position={[0, 0, 0.015]}>
        <ringGeometry args={[0.18, 0.24, 40]} />
        <meshBasicMaterial color={AMBER} side={THREE.DoubleSide} />
      </mesh>
      {/* cylinders run along y: turn it upright (z) */}
      <mesh position={[0, 0, poleH / 2]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.012, 0.012, poleH, 8]} />
        <meshBasicMaterial color={AMBER} />
      </mesh>
    </group>
  );
}

export function RoomLayer() {
  const grid = useRoom((s) => s.grid);
  const room = useRoom((s) => s.room);
  const wallH = useStore((s) => s.roomWallH);
  const { floorZ } = useRoomFloorZ();
  if (!grid || grid.w === 0 || grid.h === 0) return null;
  const st = room?.next?.station;
  return (
    <group>
      <Floor grid={grid} floorZ={floorZ} />
      <Walls grid={grid} floorZ={floorZ} wallH={wallH > 0 ? wallH : 2.4} />
      {room && <PathAndPose room={room} floorZ={floorZ} />}
      {room?.stations && room.stations.length > 0 && <Stations stations={room.stations} floorZ={floorZ} />}
      {st && <NextStation x={st.x} y={st.y} floorZ={floorZ} />}
    </group>
  );
}
