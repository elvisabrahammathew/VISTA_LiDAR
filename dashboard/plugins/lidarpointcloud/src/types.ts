export type PointCloudSourceMode = 'websocket' | 'dataframe';
export type PointCloudColorMode = 'intensity' | 'height' | 'solid';

export interface LidarPointCloudOptions {
  sourceMode: PointCloudSourceMode;
  websocketUrl: string;
  reconnectDelayMs: number;
  maxPoints: number;
  pointSize: number;
  colorMode: PointCloudColorMode;
  solidColor: string;
  backgroundColor: string;
  showGrid: boolean;
  showAxes: boolean;
  autoRotate: boolean;
}

export interface PointCloudFrame {
  positions: Float32Array;
  intensities?: Float32Array;
  pointCount: number;
  frameId?: bigint;
  timestampNs?: bigint;
  protocol: string;
  ground?: GroundStatus;
}

export type GroundState = 'calibrating' | 'valid' | 'static_fallback' | 'invalid';

export interface GroundStatus {
  state: GroundState;
  calibrated: boolean;
  usingImu: boolean;
  usingStaticFallback: boolean;
  mode: 'static' | 'ransac' | 'hybrid' | 'unknown';
  planeA: number;
  planeB: number;
  planeC: number;
  planeD: number;
  tiltDeg: number;
  sensorDistanceM: number;
  expectedDistanceM: number;
  heightErrorM: number;
  inlierRatio: number;
  meanResidualM: number;
  rmsResidualM: number;
  p95ResidualM: number;
  removedRatio: number;
  inputPointCount: number;
  removedPointCount: number;
}
