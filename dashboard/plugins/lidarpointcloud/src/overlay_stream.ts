import { decodePointCloudPacket } from './protocol';
import { decodeRoomMapMetadata, RoomMapMetadata } from './room_map_view';
import { decodeForegroundStatus, ForegroundStatus } from './foreground_status';
import type { PointCloudFrame } from './types';

export type OverlayLayer = 'gaussian' | 'foreground';
export interface LayerCallbacks {
  frame: (frame: PointCloudFrame) => void;
  clear: (reason: string) => void;
  status: (message: string, error?: boolean) => void;
  map?: (meta: RoomMapMetadata) => void;
  foreground?: (meta: ForegroundStatus) => void;
}
export interface StreamEnvironment {
  socket: (url: string) => WebSocket;
  now: () => number;
  later: (run: () => void, ms: number) => number;
  cancel: (id: number) => void;
}
const browserEnvironment: StreamEnvironment = {
  socket: (url) => new WebSocket(url), now: () => performance.now(),
  later: (run, ms) => window.setTimeout(run, ms), cancel: (id) => window.clearTimeout(id),
};

// One isolated transport/lifecycle per layer. No Gaussian event can extend the
// foreground clock, and no foreground clear can remove persistent map geometry.
export class OverlayStream {
  private socket?: WebSocket;
  private retry?: number;
  private stopped = true;
  private lastFrame?: number;
  private cleared = true;
  private lastView = '';
  constructor(readonly layer: OverlayLayer, private url: string, private callbacks: LayerCallbacks,
    private reconnectMs = 2000, private expiryMs = 1000, private env = browserEnvironment) {}
  start(): void { if (!this.stopped) { return; } this.stopped = false; this.connect(); }
  stop(): void {
    this.stopped = true;
    if (this.retry !== undefined) { this.env.cancel(this.retry); this.retry = undefined; }
    if (this.socket) {
      this.socket.onopen = this.socket.onmessage = this.socket.onerror = this.socket.onclose = null;
      this.socket.close(); this.socket = undefined;
    }
    this.lastFrame = undefined;
  }
  expire(): void {
    if (this.stopped || this.cleared || this.layer !== 'foreground' || this.lastFrame === undefined) { return; }
    const timeout = Number.isFinite(this.expiryMs) ? Math.max(100, Math.min(60000, this.expiryMs)) : 1000;
    if (this.env.now() - this.lastFrame >= timeout) {
      this.clear('No fresh Foreground points'); this.callbacks.status('No fresh Foreground points');
    }
  }
  requestView(request: string): void {
    if (!this.stopped && this.layer === 'gaussian' && this.socket?.readyState === 1 && request !== this.lastView) {
      try { this.socket.send(request); this.lastView = request; }
      catch { this.callbacks.status('Gaussian view request failed', true); }
    }
  }
  private clear(reason: string): void {
    if (!this.cleared) { this.callbacks.clear(reason); }
    this.cleared = true;
  }
  private schedule(): void {
    if (this.stopped || this.retry !== undefined) { return; }
    const delay = Number.isFinite(this.reconnectMs) ? Math.max(250, Math.min(60000, this.reconnectMs)) : 2000;
    this.retry = this.env.later(() => { this.retry = undefined; if (!this.stopped) { this.connect(); } }, delay);
  }
  private connect(): void {
    if (this.stopped) { return; }
    this.callbacks.status(`Connecting to ${this.url}`);
    let socket: WebSocket;
    try { socket = this.env.socket(this.url); socket.binaryType = 'arraybuffer'; }
    catch (e) { this.callbacks.status(String(e), true); this.schedule(); return; }
    this.socket = socket; this.lastView = '';
    const current = () => !this.stopped && this.socket === socket;
    socket.onopen = () => { if (current()) { this.callbacks.status('Connected; waiting for data'); } };
    socket.onmessage = (event) => {
      if (!current()) { return; }
      try {
        if (typeof event.data === 'string') {
          if (this.layer === 'foreground') {
            const meta = decodeForegroundStatus(event.data);
            if (meta) {
              this.callbacks.foreground?.(meta);
              if (!meta.active) { this.clear(meta.reason); }
            }
          } else {
            const meta = decodeRoomMapMetadata(event.data);
            if (meta) {
              if (meta.state >= 4) { this.cleared = false; this.clear('Gaussian map unavailable'); }
              this.callbacks.map?.(meta);
            }
          }
          return;
        }
        if (!(event.data instanceof ArrayBuffer)) { throw new Error('Expected a binary layer frame'); }
        const view = new DataView(event.data);
        const magic = view.byteLength >= 4 ? String.fromCharCode(...new Uint8Array(event.data, 0, 4)) : '';
        const expected = this.layer === 'gaussian' ? 'GSP1' : 'LPC1';
        if (magic !== expected) { throw new Error(`${this.layer} requires ${expected}, received ${magic || 'empty'}`); }
        if (this.layer === 'foreground' && (view.byteLength < 32 || view.byteLength > 96+50000*16 || view.getUint32(24, true) > 50000)) {
          throw new Error('Foreground frame exceeds its 50000-point safety limit');
        }
        const frame = decodePointCloudPacket(event.data);
        if(this.layer === 'foreground' && !frame.positions.every(Number.isFinite)) { throw new Error('Nonfinite Foreground coordinates'); }
        this.lastFrame = this.env.now(); this.cleared = false;
        this.callbacks.frame(frame);
      } catch (e) {
        if (this.layer === 'foreground') { this.clear('Invalid Foreground frame'); }
        this.callbacks.status(e instanceof Error ? e.message : String(e), true);
      }
    };
    const disconnected = () => {
      if (!current()) { return; }
      if (this.layer === 'foreground') { this.clear('Foreground disconnected'); }
      this.socket = undefined;
      socket.onopen = socket.onmessage = socket.onerror = socket.onclose = null;
      if (socket.readyState < 2) { socket.close(); }
      this.callbacks.status(this.layer === 'gaussian' ? 'Disconnected; retaining last Gaussian map' : 'Foreground disconnected', true);
      this.schedule();
    };
    socket.onerror = disconnected; socket.onclose = disconnected;
  }
}
