// Map views retain geometry by default; a foreground-only panel must never retain ghosts.
export const transientExpired = (lastFrameMs: number, nowMs: number, timeoutMs?: number): boolean =>
  Number.isFinite(timeoutMs) && (timeoutMs ?? 0) > 0 && nowMs - lastFrameMs >= timeoutMs!;
