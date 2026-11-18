export const VERTICES = 18439;
export interface MeshMeta {
  mesh_sequence: number; topology?: string; session?: string;
  coordinates: string; units: string; model?: string; worker?: string;
}
export interface GatewayStatus {
  session: string; topology: string; source: string; source_error: string | null;
  mesh_available: boolean; stale: boolean; mesh_age_seconds: number | null;
  max_age_seconds: number; mesh_fps: number; capture_fps: number;
  mesh_sequence: number;
  control: {running: boolean; worker: string};
  workers: Record<string, {online: boolean; status?: string; error?: string}>;
}
export function decodeFaces(buffer: ArrayBuffer): Uint32Array {
  if (!buffer.byteLength || buffer.byteLength % 12 || buffer.byteLength > 2000000) {
    throw new Error('Invalid body topology size');
  }
  const view = new DataView(buffer), faces = new Uint32Array(buffer.byteLength / 4);
  for (let i = 0; i < faces.length; i++) {
    faces[i] = view.getUint32(i * 4, true);
    if (faces[i] >= VERTICES) { throw new Error('Body face index out of bounds'); }
  }
  return faces;
}
export function decodeMesh(buffer: ArrayBuffer): {positions: Float32Array; meta: MeshMeta} {
  const view = new DataView(buffer);
  if (buffer.byteLength < 8 || view.getUint32(0, true) !== 0x31304d48) { throw new Error('Expected HM01 body mesh'); }
  const n = view.getUint32(4, true);
  if (n > 65536 || n % 4 || buffer.byteLength !== 8 + n + VERTICES * 12) { throw new Error('Invalid body mesh length'); }
  const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 8, n))) as MeshMeta;
  if (!meta || !Number.isSafeInteger(meta.mesh_sequence) || meta.mesh_sequence < 1 ||
      meta.coordinates !== 'body-relative x-right y-down z-forward' || meta.units !== 'predicted metres') {
    throw new Error('Unsupported body coordinates or mesh sequence');
  }
  const positions = new Float32Array(VERTICES * 3);
  for (let i = 0; i < positions.length; i += 3) {
    for (let j = 0; j < 3; j++) {
      const value = view.getFloat32(8 + n + (i + j) * 4, true);
      if (!Number.isFinite(value)) { throw new Error('Non-finite body vertex'); }
      // Proper 180-degree X rotation: camera-style axes become a y-up display.
      positions[i + j] = j === 0 ? value : -value;
    }
  }
  return {positions, meta};
}
export function visible(status: GatewayStatus): boolean {
  if (status.source_error || !status.mesh_available) { return false; }
  if (status.source === 'demo') { return true; }
  return status.control.running && !status.stale && typeof status.mesh_age_seconds === 'number' &&
    Number.isFinite(status.mesh_age_seconds) && status.mesh_age_seconds >= 0 &&
    status.mesh_age_seconds <= status.max_age_seconds;
}
export function validateStatus(value: unknown): GatewayStatus {
  const s = value as GatewayStatus;
  if (!s || typeof s.session !== 'string' || !/^[a-f0-9]{64}$/.test(s.topology) ||
      !s.control || typeof s.control.running !== 'boolean' || !s.workers ||
      typeof s.source !== 'string' || typeof s.stale !== 'boolean' ||
      typeof s.mesh_available !== 'boolean' || !Number.isFinite(s.max_age_seconds) ||
      s.max_age_seconds <= 0 || s.max_age_seconds > 30) { throw new Error('Invalid gateway status'); }
  return s;
}
