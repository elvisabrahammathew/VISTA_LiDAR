import React, {useEffect, useRef, useState} from 'react';
import type {PanelProps} from '@grafana/data';
import {decodeFaces, decodeMesh, validateStatus, visible} from './protocol';
import {BodyRenderer} from './renderer';
export interface Options {bridgeUrl: string; pollMs: number; wireframe: boolean}

export function DigitalTwinPanel({width, height, options}: PanelProps<Options>) {
  const host = useRef<HTMLDivElement>(null), view = useRef<BodyRenderer | null>(null);
  const dimensions = useRef({width, height}); dimensions.current = {width, height};
  const [message, setMessage] = useState('Connecting to Windows bridge…');
  useEffect(() => {
    let body: BodyRenderer;
    try { body = new BodyRenderer(host.current!); view.current = body; }
    catch { setMessage('WebGL unavailable on this browser'); return; }
    body.resize(dimensions.current.width, Math.max(1, dimensions.current.height - 65));
    let disposed = false, timer: ReturnType<typeof setTimeout> | undefined;
    let sequence = '', identity = '', expires = 0;
    let active: AbortController | undefined;
    const interval = Math.max(100, Math.min(2000, Number(options.pollMs) || 250));
    const base = (options.bridgeUrl || 'http://127.0.0.1:8101').replace(/\/$/, '');
    const expiry = setInterval(() => {
      if (expires && performance.now() >= expires) {
        body.clear(); expires = 0; sequence = ''; setMessage('No fresh mesh — waiting for AI');
      }
    }, 100);
    const poll = async () => {
      const controller = new AbortController(); active = controller;
      const timeout = setTimeout(() => controller.abort(), 2000);
      const started = performance.now();
      const fetchRoute = async (route: string, after?: string) => {
        const response = await fetch(base + route, {signal: controller.signal, cache: 'no-store',
          credentials: 'omit', headers: after ? {'X-After': after} : {}});
        if (!response.ok) { throw new Error(`Bridge HTTP ${response.status}; check gateway/token/origin`); }
        return response;
      };
      try {
        const status = validateStatus(await (await fetchRoute('/api/status')).json());
        if (disposed) { return; }
        const current = status.session + ':' + status.topology;
        if (identity !== current) {
          body.clear(); sequence = ''; expires = 0;
          const raw = await (await fetchRoute('/api/faces')).arrayBuffer();
          const sha = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', raw)))
            .map(v => v.toString(16).padStart(2, '0')).join('');
          if (sha !== status.topology) { throw new Error('Topology changed; retrying gateway session'); }
          if (disposed) { return; }
          body.setTopology(decodeFaces(raw)); identity = current;
        }
        if (!visible(status)) {
          body.clear(); expires = 0; sequence = '';
          const worker = status.workers[status.control.worker];
          setMessage(status.source_error || (!status.control.running ? 'Inference stopped' :
            worker?.error || (worker?.online ? `Waiting for one visible person · ${worker.status || 'AI connected'}` : 'Waiting for Windows AI worker')));
        } else {
          const response = await fetchRoute('/api/mesh', sequence);
          if (response.status !== 204) {
            const mesh = decodeMesh(await response.arrayBuffer());
            if (disposed) { return; }
            if (status.source !== 'demo' && (mesh.meta.topology !== status.topology || mesh.meta.session !== status.session)) {
              throw new Error('Mesh session/topology differs from status');
            }
            // Age belongs to the gateway monotonic clock; add local round-trip time conservatively.
            const remaining = status.source === 'demo' ? 3000 :
              (status.max_age_seconds - (status.mesh_age_seconds || 0)) * 1000 - (performance.now() - started);
            if (remaining <= 0) { body.clear(); sequence = ''; expires = 0; setMessage('Mesh expired in transit'); }
            else {
              body.setMesh(mesh.positions, options.wireframe !== false);
              sequence = String(mesh.meta.mesh_sequence); expires = performance.now() + remaining;
            }
          } else if (status.source === 'demo' && expires) { expires = performance.now() + 3000; }
          if (expires) {
            const label = status.source === 'demo' ? 'DEMO — not live' : status.source === 'replay' ? 'REPLAY — not live camera' : 'LIVE';
            const fps = Number.isFinite(status.mesh_fps) ? status.mesh_fps.toFixed(1) : '?';
            const age = status.mesh_age_seconds == null ? '—' : `${status.mesh_age_seconds.toFixed(2)} s`;
            setMessage(`${label} · mesh ${fps}/s · age ${age} · worker ${status.control.worker}`);
          }
        }
      } catch (error) {
        if (!disposed) {
          body.clear(); sequence = ''; expires = 0;
          setMessage(error instanceof Error ? error.message : 'Digital Twin disconnected');
        }
      } finally {
        clearTimeout(timeout);
        if (!disposed) { timer = setTimeout(poll, interval); }
      }
    };
    void poll();
    return () => {
      disposed = true; active?.abort(); clearTimeout(timer); clearInterval(expiry);
      body.dispose(); view.current = null;
    };
  }, [options.bridgeUrl, options.pollMs, options.wireframe]);
  useEffect(() => { view.current?.resize(width, Math.max(1, height - 65)); }, [width, height]);
  return <div style={{width, height, background: '#080f1a', overflow: 'hidden'}}>
    <div style={{height: 35, padding: '5px 8px', display: 'flex', alignItems: 'center', gap: 10}}>
      <button type="button" onClick={() => view.current?.reset()}>Reset view</button>
      <span style={{fontSize: 12, color: '#9de3ff'}} role="status">{message}</span>
    </div>
    <div ref={host} style={{height: Math.max(1, height - 65)}} />
    <div style={{height: 30, padding: '4px 8px', color: '#a9b6c8', fontSize: 11}}>
      Inferred body · predicted metres · body-relative · grid is illustrative, not LiDAR room position
    </div>
  </div>;
}
