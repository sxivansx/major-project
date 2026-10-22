import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import type { GridFrame } from "../api/types";
import { cloud } from "../store/cloud";
import { useStore } from "../store/store";
import { RAMP_GLSL } from "../lib/colormap";

// Our data is z-up (base frame). three.js is y-up. Everything in the base
// frame lives inside <group rotation={[-PI/2, 0, 0]}>, which maps data
// (x, y, z) -> three (x, z, -y); no coordinates are rewritten.
const Z_UP: [number, number, number] = [-Math.PI / 2, 0, 0];
const DEG = Math.PI / 180;

const POINT_VERT = `
uniform float zMin;
uniform float zMax;
uniform float size;
varying vec3 vColor;
${RAMP_GLSL}
void main() {
  float t = (position.z - zMin) / max(zMax - zMin, 1e-6);
  vColor = rampColor(t);
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mv;
  gl_PointSize = size;
}
`;
const POINT_FRAG = `
varying vec3 vColor;
void main() {
  gl_FragColor = vec4(vColor, 1.0);
}
`;

const SURF_VERT = `
uniform float zMin;
uniform float zMax;
varying vec3 vColor;
varying vec3 vViewPos;
${RAMP_GLSL}
void main() {
  float t = (position.z - zMin) / max(zMax - zMin, 1e-6);
  vColor = rampColor(t);
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  vViewPos = mv.xyz;
  gl_Position = projectionMatrix * mv;
}
`;
const SURF_FRAG = `
varying vec3 vColor;
varying vec3 vViewPos;
void main() {
  vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
  float l = 0.5 + 0.5 * abs(n.z);
  gl_FragColor = vec4(vColor * l, 0.92);
}
`;

function PointCloud({ size, zRange }: { size: number; zRange: [number, number] }) {
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const attr = new THREE.BufferAttribute(cloud.positions, 3);
    attr.setUsage(THREE.DynamicDrawUsage);
    g.setAttribute("position", attr);
    g.setDrawRange(0, cloud.count);
    // never recompute bounds over 2M points; we disable culling anyway
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e3);
    return g;
  }, []);
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        uniforms: { zMin: { value: zRange[0] }, zMax: { value: zRange[1] }, size: { value: size } },
        vertexShader: POINT_VERT,
        fragmentShader: POINT_FRAG,
      }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => material.dispose(), [material]);
  useEffect(() => {
    material.uniforms.zMin.value = zRange[0];
    material.uniforms.zMax.value = zRange[1];
    material.uniforms.size.value = size;
  }, [material, size, zRange]);

  const uploaded = useRef(0); // points this geometry has uploaded so far
  const seen = useRef(cloud.version);

  useFrame(() => {
    if (cloud.version === seen.current) return;
    seen.current = cloud.version;
    const attr = geometry.getAttribute("position") as THREE.BufferAttribute;
    const count = cloud.count;
    if (count < uploaded.current) uploaded.current = 0; // cleared
    const from = Math.min(uploaded.current, cloud.dirtyFrom);
    if (count > from) {
      attr.addUpdateRange(from * 3, (count - from) * 3);
      attr.needsUpdate = true;
    }
    uploaded.current = count;
    cloud.dirtyFrom = count;
    geometry.setDrawRange(0, count);
  });

  return <points geometry={geometry} material={material} frustumCulled={false} />;
}

// Height map as a surface: one vertex per cell centre, two triangles per
// quad whose four corners are all finite (NaN cells leave holes). Quads whose
// corners differ by more than maxStep are skipped too, so a wall in a "max"
// layer does not become a floor-to-ceiling curtain.
function buildSurface(g: GridFrame, maxStep: number): THREE.BufferGeometry | null {
  const { w, h, cell_m: cell, x0_m: x0, y0_m: y0, data } = g;
  if (w < 2 || h < 2) return null;
  const pos = new Float32Array(w * h * 3);
  for (let iy = 0; iy < h; iy++) {
    for (let ix = 0; ix < w; ix++) {
      const i = iy * w + ix;
      const z = data[i];
      pos[i * 3] = x0 + (ix + 0.5) * cell;
      pos[i * 3 + 1] = y0 + (iy + 0.5) * cell;
      pos[i * 3 + 2] = Number.isFinite(z) ? z : 0;
    }
  }
  const idx: number[] = [];
  for (let iy = 0; iy < h - 1; iy++) {
    for (let ix = 0; ix < w - 1; ix++) {
      const a = iy * w + ix;
      const b = a + 1;
      const c = a + w;
      const d = c + 1;
      const za = data[a];
      const zb = data[b];
      const zc = data[c];
      const zd = data[d];
      if (!Number.isFinite(za) || !Number.isFinite(zb) || !Number.isFinite(zc) || !Number.isFinite(zd)) continue;
      if (maxStep > 0) {
        const lo = Math.min(za, zb, zc, zd);
        const hi = Math.max(za, zb, zc, zd);
        if (hi - lo > maxStep) continue;
      }
      idx.push(a, b, d, a, d, c);
    }
  }
  if (idx.length === 0) return null;
  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geom.setIndex(idx.length > 65535 ? new THREE.Uint32BufferAttribute(idx, 1) : new THREE.Uint16BufferAttribute(idx, 1));
  geom.computeBoundingSphere();
  return geom;
}

function HeightSurface({ zRange }: { zRange: [number, number] }) {
  const grid = useStore((s) => s.grid);
  const maxStep = useStore((s) => s.surfaceMaxStep);
  const geometry = useMemo(
    () => (grid && grid.layer !== "count" ? buildSurface(grid, maxStep) : null),
    [grid, maxStep],
  );
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        uniforms: { zMin: { value: zRange[0] }, zMax: { value: zRange[1] } },
        vertexShader: SURF_VERT,
        fragmentShader: SURF_FRAG,
        side: THREE.DoubleSide,
        transparent: true,
      }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );
  useEffect(() => () => geometry?.dispose(), [geometry]);
  useEffect(() => () => material.dispose(), [material]);
  useEffect(() => {
    material.uniforms.zMin.value = zRange[0];
    material.uniforms.zMax.value = zRange[1];
  }, [material, zRange]);
  if (!geometry) return null;
  return <mesh geometry={geometry} material={material} />;
}

function Rig({ axisZ, h, phiDeg }: { axisZ: number; h: number; phiDeg: number }) {
  return (
    <>
      {/* reticle at the tilt axis (0, 0, axis_z); the axis runs along x */}
      <group position={[0, 0, axisZ]}>
        <mesh>
          <sphereGeometry args={[0.012, 12, 12]} />
          <meshBasicMaterial color="#4d9de0" />
        </mesh>
        <mesh rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.002, 0.002, 0.3, 6]} />
          <meshBasicMaterial color="#4d9de0" />
        </mesh>
        {/* scan plane ring: rotated by φ about the tilt axis, offset by h */}
        <group rotation={[phiDeg * DEG, 0, 0]}>
          <group position={[0, 0, h]}>
            <mesh>
              <ringGeometry args={[0.085, 0.1, 48]} />
              <meshBasicMaterial color="#8c8c8c" side={THREE.DoubleSide} transparent opacity={0.6} />
            </mesh>
            {/* cable mark = +x = 0° */}
            <mesh position={[0.1, 0, 0]}>
              <sphereGeometry args={[0.008, 8, 8]} />
              <meshBasicMaterial color="#e0a040" />
            </mesh>
          </group>
        </group>
      </group>
    </>
  );
}

// Default view: looking down from above at about 60°, from the +x/+y side.
const CAMERA_HOME: [number, number, number] = [3.5, 9, 4.5];

// Puts the camera back to the default view whenever `nonce` changes (the Fit
// button). OrbitControls registers itself as the default controls.
function CameraHome({ nonce, targetY }: { nonce: number; targetY: number }) {
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as { target: THREE.Vector3; update: () => void } | null;
  useEffect(() => {
    if (nonce === 0) return;
    camera.position.set(...CAMERA_HOME);
    if (controls) {
      controls.target.set(0, targetY, 0);
      controls.update();
    } else camera.lookAt(0, targetY, 0);
  }, [nonce, camera, controls, targetY]);
  return null;
}

export function CloudView({ resetNonce = 0 }: { resetNonce?: number }) {
  const showSurface = useStore((s) => s.showSurface);
  const showHelpers = useStore((s) => s.showHelpers);
  const pointSize = useStore((s) => s.pointSize);
  const colorZ = useStore((s) => s.colorZ);
  const mount = useStore((s) => s.config?.mount);
  const phiDeg = useStore((s) => s.status?.servo.phi_deg ?? 0);
  const axisZ = (mount?.axis_z_mm ?? 80) / 1000;
  const h = (mount?.h_mm ?? 0) / 1000;

  return (
    <Canvas
      dpr={[1, 1.5]}
      gl={{ antialias: true, powerPreference: "high-performance" }}
      camera={{ position: CAMERA_HOME, fov: 45, near: 0.02, far: 300 }}
      onCreated={({ gl }) => gl.setClearColor("#1c1c1c")}
    >
      <OrbitControls makeDefault target={[0, axisZ, 0]} />
      <CameraHome nonce={resetNonce} targetY={axisZ} />
      {/* three's XZ plane at y=0 is our z=0 base plate */}
      {showHelpers && <gridHelper args={[10, 20, "#4a4a4a", "#2c2c2c"]} />}
      <group rotation={Z_UP}>
        {showHelpers && <axesHelper args={[0.5]} />}
        <Rig axisZ={axisZ} h={h} phiDeg={phiDeg} />
        <PointCloud size={pointSize} zRange={colorZ} />
        {showSurface && <HeightSurface zRange={colorZ} />}
      </group>
    </Canvas>
  );
}
