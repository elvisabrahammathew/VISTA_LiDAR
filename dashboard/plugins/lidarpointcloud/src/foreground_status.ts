export interface ForegroundStatus {
  active: boolean;
  reason: string;
  inputPoints: number;
  backgroundPoints: number;
  unknownPoints: number;
  foregroundPoints: number;
}

export const decodeForegroundStatus = (text: string): ForegroundStatus | undefined => {
  const value = JSON.parse(text);
  if (value?.type !== 'foreground-status') { return undefined; }
  if (value.version !== 1 || typeof value.active !== 'boolean' ||
      typeof value.reason !== 'string' || value.reason.length > 8192) {
    throw new Error('Invalid foreground status');
  }
  for (const field of ['inputPoints', 'backgroundPoints', 'unknownPoints', 'foregroundPoints']) {
    if (!Number.isSafeInteger(value[field]) || value[field] < 0 || value[field] > 50000) {
      throw new Error('Invalid foreground counters');
    }
  }
  return value as ForegroundStatus;
};

export const foregroundStatusMessage = (status: ForegroundStatus): string => {
  if (!status.active) { return `Foreground paused: ${status.reason || 'waiting for valid input'}`; }
  if (status.foregroundPoints > 0) { return 'Receiving foreground points outside RoomMap'; }
  if (status.unknownPoints > 0) { return 'No foreground points; background query is warming or budget-limited'; }
  return 'No foreground points; current samples match RoomMap';
};
