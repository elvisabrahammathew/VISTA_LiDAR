export interface SettingsSnapshot {
  revision: number;
  busy: boolean;
  realSense: boolean;
  settings: Record<string, string>;
}

export interface SettingsView {
  phase: 'connecting' | 'token' | 'ready' | 'offline';
  snapshot?: SettingsSnapshot;
  pending?: string;
  notice?: { group?: string; ok: boolean; message: string };
  ports: string[];
}

export function encodeSettingsApply(revision: number, group: string, fields: Record<string, string>): string {
  const lines = Object.entries(fields);
  if (!Number.isSafeInteger(revision) || revision < 0 || !/^[a-z_]+$/.test(group) ||
      lines.length === 0 || lines.length > 16 ||
      lines.some(([key,value]) => !/^[A-Za-z][A-Za-z0-9]*$/.test(key) || /[\x00-\x1f\x7f#]/.test(value))) {
    throw new Error('Invalid configuration fields.');
  }
  const command = ['APPLY ' + revision + ' ' + group, ...lines.map(([key,value]) => key + '=' + value)].join('\n');
  if (new TextEncoder().encode(command).length > 2048) { throw new Error('Configuration request is too long.'); }
  return command;
}

const clients = new Map<string, SettingsClient>();

export function settingsClient(url: string): SettingsClient {
  const current = clients.get(url);
  if (current) { return current; }
  const client = new SettingsClient(url);
  clients.set(url, client);
  return client;
}

export class SettingsClient {
  private view: SettingsView = { phase: 'connecting', ports: [] };
  private listeners = new Set<() => void>();
  private socket?: WebSocket;
  private token = '';
  private retry?: ReturnType<typeof setTimeout>;
  private closeTimer?: ReturnType<typeof setTimeout>;

  constructor(private readonly url: string) {}

  getSnapshot = () => this.view;

  subscribe = (listener: () => void) => {
    this.listeners.add(listener);
    clearTimeout(this.closeTimer);
    if (!this.socket && !this.retry) { this.connect(); }
    return () => {
      this.listeners.delete(listener);
      if (!this.listeners.size) {
        this.closeTimer = setTimeout(() => {
          if (this.listeners.size) { return; }
          clearTimeout(this.retry); this.retry = undefined;
          const socket = this.socket; this.socket = undefined;
          socket?.close();
          this.token = '';
          this.view = { phase: 'connecting', ports: [] };
          clients.delete(this.url);
        }, 250);
      }
    };
  };

  authenticate(token: string) {
    if (!/^[A-Za-z0-9_-]{24,128}$/.test(token)) {
      this.update({ notice: { ok: false, message: 'Enter the VISTA Edge control token.' } });
      return;
    }
    this.token = token;
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.socket.send('AUTH ' + token);
      this.update({ notice: undefined });
    }
  }

  refreshPorts() {
    if (this.view.phase === 'ready' && this.socket?.readyState === WebSocket.OPEN) { this.socket.send('PORTS'); }
  }

  apply(group: string, fields: Record<string, string>) {
    const active = this.view.snapshot;
    if (!active || this.view.pending || active.busy || this.socket?.readyState !== WebSocket.OPEN) { return; }
    const command = encodeSettingsApply(active.revision, group, fields);
    this.socket.send(command);
    this.update({ pending: group, notice: undefined });
  }

  private update(patch: Partial<SettingsView>) {
    this.view = { ...this.view, ...patch };
    for (const listener of this.listeners) { listener(); }
  }

  private connect() {
    this.update({ phase: 'connecting', snapshot: undefined, pending: undefined, ports: [] });
    let socket: WebSocket;
    try {
      socket = new WebSocket(this.url);
    } catch {
      this.update({ phase: 'offline', notice: { ok: false, message: 'Invalid settings WebSocket URL.' } });
      return;
    }
    this.socket = socket;
    socket.onopen = () => {
      if (this.socket !== socket) { return; }
      this.update({ phase: 'token' });
      if (this.token) { socket.send('AUTH ' + this.token); }
    };
    socket.onmessage = event => {
      if (this.socket !== socket || typeof event.data !== 'string' || event.data.length > 128000) { return; }
      try {
        const data = JSON.parse(event.data);
        if (data?.type === 'state' && Number.isSafeInteger(data.revision) && data.revision >= 0 &&
            typeof data.busy === 'boolean' && typeof data.realSense === 'boolean' && data.settings &&
            Object.values(data.settings).every(value => typeof value === 'string')) {
          this.update({ phase: 'ready', snapshot: data });
        } else if (data?.type === 'ports' && Array.isArray(data.ports) &&
            data.ports.every((port: unknown) => typeof port === 'string')) {
          this.update({ ports: data.ports });
        } else if (data?.type === 'result' && typeof data.group === 'string' && typeof data.ok === 'boolean' &&
            typeof data.message === 'string') {
          this.update({ pending: undefined, notice: { group: data.group, ok: data.ok, message: data.message } });
        } else if (data?.type === 'error' && typeof data.message === 'string') {
          this.update({ pending: undefined, notice: { group: this.view.pending, ok: false, message: data.message } });
        }
      } catch {
        this.update({ pending: undefined, notice: { group: this.view.pending, ok: false, message: 'Invalid response from VISTA Edge.' } });
      }
    };
    socket.onclose = () => {
      if (this.socket !== socket) { return; }
      this.socket = undefined;
      this.update({ phase: 'offline', snapshot: undefined, ports: [], pending: undefined, notice: undefined });
      if (this.listeners.size) {
        this.retry = setTimeout(() => { this.retry = undefined; this.connect(); }, 1500);
      }
    };
  }
}
