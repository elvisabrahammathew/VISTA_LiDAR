import * as THREE from 'three';

export interface RoomMapMetadata {
  type: 'roommap';
  totalPoints: number;
  pointBudget: number;
  state: number;
  bounds: number[];
}

// Accept only finite, bounded metadata before changing the 3D camera.
export const decodeRoomMapMetadata = (text: string): RoomMapMetadata | undefined => {
  if (text.length > 4096) { return undefined; }
  const value = JSON.parse(text) as RoomMapMetadata;
  if (value.type !== 'roommap' || !Number.isSafeInteger(value.totalPoints) || value.totalPoints < 0 ||
      !Number.isInteger(value.pointBudget) || value.pointBudget < 1 || value.pointBudget > 2000000 ||
      !Number.isInteger(value.state) || value.state < 0 || value.state > 6 ||
      !Array.isArray(value.bounds) || value.bounds.length !== 6 ||
      !value.bounds.every(Number.isFinite) || value.bounds.slice(0, 3).some((v, i) => v > value.bounds[i + 3])) {
    return undefined;
  }
  return value;
};

// The server selects a non-overlapping octree cut for this camera/frustum.
// maxPoints is a rendering budget, never a cap on the stored room map.
export const makeRoomMapViewRequest = (camera: THREE.PerspectiveCamera, height: number, budget: number): string => {
  camera.updateMatrixWorld();
  const matrix = new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
  const frustum = new THREE.Frustum().setFromProjectionMatrix(matrix);
  const values = [camera.position.x, camera.position.y, camera.position.z];
  for (const plane of frustum.planes) {
    values.push(plane.normal.x, plane.normal.y, plane.normal.z, plane.constant);
  }
  return `VIEW ${Math.min(2000000, Math.max(1, Math.floor(budget)))} ${Math.min(8192, Math.max(1, Math.floor(height)))} ${values.map((v) => v.toFixed(6)).join(' ')}`;
};
