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
}
