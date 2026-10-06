// Wall-clock timestamps are supplied by the producer. Windows and the edge
// host must have synchronized calendar clocks (independent of LIO clocks).
export const DEFAULT_TIMEOUT_MS = 6000;
export class Freshness {
  private latest = Number.NEGATIVE_INFINITY;
  constructor(private readonly timeoutMs = DEFAULT_TIMEOUT_MS) {}
  observe(timestamp: number): void {
    if (Number.isFinite(timestamp)) { this.latest = Math.max(this.latest, timestamp); }
  }
  live(now: number): boolean {
    const age = now - this.latest;
    return Number.isFinite(age) && age >= -2000 && age <= this.timeoutMs;
  }
  reset(): void { this.latest = Number.NEGATIVE_INFINITY; }
}
