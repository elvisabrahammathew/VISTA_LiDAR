import { PointCloudFrame } from './types';

const LPC1_HEADER_SIZE = 32;
const LDR1_HEADER_SIZE = 24;
const MAX_ACCEPTED_POINTS = 10_000_000;

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

  return { positions, intensities, pointCount, frameId, timestampNs, protocol: 'LPC1' };
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
