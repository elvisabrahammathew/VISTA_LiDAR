import { GroundState, PointCloudFrame } from './types';

const LPC1_HEADER_SIZE = 32;
const LDR1_HEADER_SIZE = 24;
const MAX_ACCEPTED_POINTS = 10_000_000;
const LPC1_GROUND_HEADER_SIZE = 96;

const readMagic = (view: DataView): string => {
  if (view.byteLength < 4) {
    return '';
  }
  return String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
};

/** Decode the current LPC1 wire format and the older LDR1 test format. */
export const decodePointCloudPacket = (buffer: ArrayBuffer): PointCloudFrame => {
  const view = new DataView(buffer);
  const magic = readMagic(view);

  if (magic === 'LPC1') {
    return decodeLpc1(view);
  }
  if (magic === 'LDR1') {
    return decodeLegacyLdr1(view);
  }

  throw new Error(`Unsupported point-cloud packet magic: ${magic || '<empty>'}`);
};

/**
 * LPC1 little-endian packet:
 * magic[4], version u8, flags u8, headerSize u16, frameId u64,
 * timestampNs u64, pointCount u32, pointStride u32, then XYZ[I] float32 points.
 */
const decodeLpc1 = (view: DataView): PointCloudFrame => {
  if (view.byteLength < LPC1_HEADER_SIZE) {
    throw new Error('LPC1 packet is shorter than its 32-byte header');
  }

  const version = view.getUint8(4);
  if (version !== 1) {
    throw new Error(`Unsupported LPC1 version: ${version}`);
  }

  const flags = view.getUint8(5);
  const headerSize = view.getUint16(6, true);
  const frameId = view.getBigUint64(8, true);
  const timestampNs = view.getBigUint64(16, true);
  const pointCount = view.getUint32(24, true);
  const pointStride = view.getUint32(28, true);
  const hasIntensity = (flags & 0x01) !== 0;
  const hasGroundStatus = (flags & 0x02) !== 0;

  if (headerSize < LPC1_HEADER_SIZE || headerSize > view.byteLength) {
    throw new Error(`Invalid LPC1 header size: ${headerSize}`);
  }
  validatePointLayout(view.byteLength, headerSize, pointCount, pointStride, hasIntensity);

  const positions = new Float32Array(pointCount * 3);
  const intensities = hasIntensity ? new Float32Array(pointCount) : undefined;

  for (let index = 0; index < pointCount; index += 1) {
    const offset = headerSize + index * pointStride;
    const output = index * 3;
    positions[output] = view.getFloat32(offset, true);
    positions[output + 1] = view.getFloat32(offset + 4, true);
    positions[output + 2] = view.getFloat32(offset + 8, true);
    if (intensities) {
      intensities[index] = view.getFloat32(offset + 12, true);
    }
  }

  const states: GroundState[] = ['calibrating', 'valid', 'static_fallback', 'invalid'];
  const modes = ['static', 'ransac', 'hybrid'] as const;
  const stateCode = hasGroundStatus && headerSize >= LPC1_GROUND_HEADER_SIZE ? view.getUint8(32) : 3;
  const groundFlags = hasGroundStatus && headerSize >= LPC1_GROUND_HEADER_SIZE ? view.getUint8(33) : 0;
  const modeCode = hasGroundStatus && headerSize >= LPC1_GROUND_HEADER_SIZE ? view.getUint8(34) : 255;
  const ground = hasGroundStatus && headerSize >= LPC1_GROUND_HEADER_SIZE ? {
    state: states[stateCode] ?? 'invalid',
    calibrated: (groundFlags & 0x01) !== 0,
    usingImu: (groundFlags & 0x02) !== 0,
    usingStaticFallback: (groundFlags & 0x04) !== 0,
    mode: modes[modeCode] ?? 'unknown',
    planeA: view.getFloat32(36, true),
    planeB: view.getFloat32(40, true),
    planeC: view.getFloat32(44, true),
    planeD: view.getFloat32(48, true),
    tiltDeg: view.getFloat32(52, true),
    sensorDistanceM: view.getFloat32(56, true),
    expectedDistanceM: view.getFloat32(60, true),
    heightErrorM: view.getFloat32(64, true),
    inlierRatio: view.getFloat32(68, true),
    meanResidualM: view.getFloat32(72, true),
    rmsResidualM: view.getFloat32(76, true),
    p95ResidualM: view.getFloat32(80, true),
    removedRatio: view.getFloat32(84, true),
    inputPointCount: view.getUint32(88, true),
    removedPointCount: view.getUint32(92, true),
  } : undefined;

  return { positions, intensities, pointCount, frameId, timestampNs, protocol: 'LPC1', ground };
};

/** Decode packets produced by the earlier VISTA LDR1 point-cloud test sender. */
const decodeLegacyLdr1 = (view: DataView): PointCloudFrame => {
  if (view.byteLength < LDR1_HEADER_SIZE) {
    throw new Error('LDR1 packet is shorter than its 24-byte header');
  }

  const pointCount = view.getUint32(12, true);
  const timestampNs = view.getBigUint64(16, true);
  validatePointLayout(view.byteLength, LDR1_HEADER_SIZE, pointCount, 12, false);

  const positions = new Float32Array(pointCount * 3);
  for (let index = 0; index < pointCount; index += 1) {
    const offset = LDR1_HEADER_SIZE + index * 12;
    const output = index * 3;
    positions[output] = view.getFloat32(offset, true);
    positions[output + 1] = view.getFloat32(offset + 4, true);
    positions[output + 2] = view.getFloat32(offset + 8, true);
  }

  return {
    positions,
    pointCount,
    frameId: BigInt(view.getUint32(8, true)),
    timestampNs,
    protocol: 'LDR1',
  };
};

const validatePointLayout = (
  packetSize: number,
  headerSize: number,
  pointCount: number,
  pointStride: number,
  hasIntensity: boolean
): void => {
  const minimumStride = hasIntensity ? 16 : 12;
  if (pointCount > MAX_ACCEPTED_POINTS) {
    throw new Error(`Point count ${pointCount} exceeds the safety limit`);
  }
  if (pointStride < minimumStride) {
    throw new Error(`Point stride ${pointStride} is smaller than ${minimumStride}`);
  }
  const requiredSize = headerSize + pointCount * pointStride;
  if (requiredSize > packetSize) {
    throw new Error(`Incomplete point-cloud packet: expected ${requiredSize} bytes, received ${packetSize}`);
  }
};
