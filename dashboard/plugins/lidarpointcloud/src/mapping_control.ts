export type MappingState =
  | { type: 'mapping-state'; enabled: false }
  | { type: 'mapping-state'; enabled: true; generation: number; pending: boolean; lioEnabled: boolean; mount: number[] };

export type MappingMessage = MappingState | { type: 'mapping-error'; message: string };
export type MappingChange = { mount: number[] } | { lioEnabled: boolean };

export const decodeMappingMessage = (text: string): MappingMessage | undefined => {
  if (text.length > 1024) { return undefined; }
  const value = JSON.parse(text) as MappingMessage | null;
  if (value?.type === 'mapping-error' && typeof value.message === 'string') { return value; }
  if (value?.type !== 'mapping-state') { return undefined; }
  if (value.enabled === false) { return value; }
  if (value.enabled !== true || !Number.isSafeInteger(value.generation) || value.generation < 0 ||
      typeof value.pending !== 'boolean' || typeof value.lioEnabled !== 'boolean' ||
      !Array.isArray(value.mount) || value.mount.length !== 6 || !value.mount.every(Number.isFinite)) {
    return undefined;
  }
  return value;
};

export const parseMount = (draft: string[]): number[] | undefined => {
  if (draft.length !== 6 || draft.some(value => value.trim() === '')) { return undefined; }
  const values = draft.map(Number);
  return values.every((value, index) => Number.isFinite(value) && Math.abs(value) <= (index < 3 ? 1000 : 360))
    ? values : undefined;
};

export const mappingCommand = (generation: number, token: string, change: MappingChange): string => {
  if (!/^[A-Za-z0-9_-]{24,128}$/.test(token)) {
    throw new Error('Enter the Room Map control token.');
  }
  return 'mount' in change
    ? 'MAPPING APPLY ' + generation + ' ' + token + ' ' + change.mount.join(' ')
    : 'MAPPING MODE ' + generation + ' ' + token + ' ' + (change.lioEnabled ? '1' : '0');
};
