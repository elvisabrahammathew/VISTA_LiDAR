export type LidarPosePacket = { mappingGeneration: number } & (
  { state: 'unavailable' } |
  { state: 'tracking' | 'fixed_mount' | 'last_known'; position: number[]; orientation: number[] }
);

type PositionPacket = Exclude<LidarPosePacket, { state: 'unavailable' }>;

const lastKnown = (pose?: PositionPacket): PositionPacket | undefined =>
  pose && pose.state !== 'last_known' ? { ...pose, state: 'last_known' } : pose;

/** Keep the displayed pose and the next map's pose separate. */
export class LidarPoseDisplay {
  private generation?: number;
  private awaitingMap = true;
  private current?: PositionPacket;
  private pending?: PositionPacket;

  get pose(): PositionPacket | undefined {
    return this.current;
  }

  receive(pose: LidarPosePacket): void {
    const generation = pose.mappingGeneration;
    if (!this.awaitingMap && this.generation !== undefined && generation < this.generation) {
      return;
    }
    if (!this.awaitingMap && generation === this.generation) {
      this.current = pose.state === 'unavailable' ? lastKnown(this.current) : pose;
      return;
    }

    this.current = lastKnown(this.current);
    if (this.pending && generation < this.pending.mappingGeneration) { return; }
    this.pending = pose.state !== 'unavailable' ? pose :
      this.pending?.mappingGeneration === generation ? lastKnown(this.pending) : undefined;
  }

  showMap(generation?: number): void {
    if (!this.awaitingMap && generation === this.generation) { return; }
    this.generation = generation;
    this.awaitingMap = false;
    // A fresh socket may reuse a previous process's generation number.
    this.current = this.pending?.mappingGeneration === generation ? this.pending : undefined;
    if (generation === undefined || (this.pending && this.pending.mappingGeneration <= generation)) {
      this.pending = undefined;
    }
  }

  disconnect(): void {
    this.awaitingMap = true;
    this.pending = undefined;
    this.current = lastKnown(this.current);
  }

  reset(): void {
    this.generation = undefined;
    this.current = undefined;
    this.disconnect();
  }
}

const finiteVector = (value: unknown, length: number): value is number[] =>
  Array.isArray(value) && value.length === length && value.every(Number.isFinite);

export const decodeLidarPoseMessage = (text: string): LidarPosePacket | undefined => {
  if (text.length > 1024) { return undefined; }
  const value = JSON.parse(text);
  if (!value || value.type !== 'lidar-pose' ||
      !Number.isSafeInteger(value.mappingGeneration) || value.mappingGeneration < 0) {
    return undefined;
  }
  const { state, mappingGeneration, position, orientation } = value;
  if (state === 'unavailable') { return { state, mappingGeneration }; }
  if (!['tracking', 'fixed_mount', 'last_known'].includes(state) ||
      !finiteVector(position, 3) || !finiteVector(orientation, 4)) {
    return undefined;
  }
  const length = Math.hypot(...orientation);
  if (length < 0.5 || length > 1.5) { return undefined; }
  return { state, mappingGeneration, position, orientation };
};
