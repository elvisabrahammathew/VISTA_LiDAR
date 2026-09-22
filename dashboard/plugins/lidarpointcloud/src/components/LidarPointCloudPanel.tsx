import { Field, PanelProps } from '@grafana/data';
import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { decodePointCloudPacket } from '../protocol';
import { LidarPointCloudOptions, PointCloudFrame } from '../types';

type Props = PanelProps<LidarPointCloudOptions>;

interface SceneState {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  geometry: THREE.BufferGeometry;
  material: THREE.PointsMaterial;
  positions: Float32Array;
  colors: Float32Array;
  capacity: number;
  pointCount: number;
  inputPointCount: number;
  grid: THREE.GridHelper;
  axes: THREE.AxesHelper;
  fitted: boolean;
  animationId: number;
}

interface StatusState {
  state: 'waiting' | 'connecting' | 'connected' | 'error';
  message: string;
  shownPoints: number;
  inputPoints: number;
  protocol: string;
  frameId: string;
}

const initialStatus: StatusState = {
  state: 'waiting',
  message: 'Waiting for a point-cloud frame',
  shownPoints: 0,
  inputPoints: 0,
  protocol: '-',
  frameId: '-',
};

export const LidarPointCloudPanel: React.FC<Props> = ({ options, data, width, height }) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<SceneState | null>(null);
  const optionsRef = useRef(options);
  const statusUpdateTimeRef = useRef(0);
  const [status, setStatus] = useState<StatusState>(initialStatus);

  optionsRef.current = options;

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(55, Math.max(width, 1) / Math.max(height, 1), 0.01, 10000);
    camera.position.set(6, -6, 4);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(Math.max(width, 1), Math.max(height, 1), false);
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;

    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: optionsRef.current.pointSize || 2,
      sizeAttenuation: false,
      vertexColors: true,
    });
    scene.add(new THREE.Points(geometry, material));

    const grid = new THREE.GridHelper(20, 20, 0x547080, 0x26313a);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);

    const axes = new THREE.AxesHelper(2);
    scene.add(axes);

    const state: SceneState = {
      renderer,
      scene,
      camera,
      controls,
      geometry,
      material,
      positions: new Float32Array(0),
      colors: new Float32Array(0),
      capacity: 0,
      pointCount: 0,
      inputPointCount: 0,
      grid,
      axes,
      fitted: false,
      animationId: 0,
    };
    sceneRef.current = state;

    const animate = () => {
      const liveState = sceneRef.current;
      if (!liveState) {
        return;
      }
      liveState.controls.autoRotate = Boolean(optionsRef.current.autoRotate);
      liveState.controls.autoRotateSpeed = 0.8;
      liveState.controls.update();
      liveState.renderer.render(liveState.scene, liveState.camera);
      liveState.animationId = window.requestAnimationFrame(animate);
    };
    animate();

    return () => {
      window.cancelAnimationFrame(state.animationId);
      state.controls.dispose();
      state.geometry.dispose();
      state.material.dispose();
      state.renderer.dispose();
      state.renderer.domElement.remove();
      sceneRef.current = null;
    };
    // The Three.js scene is created once; resize and option changes are handled below.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    const state = sceneRef.current;
    if (!state) {
      return;
    }
    const safeWidth = Math.max(width, 1);
    const safeHeight = Math.max(height, 1);
    state.camera.aspect = safeWidth / safeHeight;
    state.camera.updateProjectionMatrix();
    state.renderer.setSize(safeWidth, safeHeight, false);
  }, [width, height]);

  useEffect(() => {
    const state = sceneRef.current;
    if (!state) {
      return;
    }
    state.material.size = Math.max(options.pointSize || 0.1, 0.1);
    state.material.needsUpdate = true;
    state.scene.background = new THREE.Color(options.backgroundColor || '#0b0f14');
    state.grid.visible = options.showGrid !== false;
    state.axes.visible = options.showAxes !== false;
  }, [options.pointSize, options.backgroundColor, options.showGrid, options.showAxes]);

  useEffect(() => {
    if (options.sourceMode !== 'dataframe') {
      return;
    }

    const frame = extractDataFrame(data.series);
    if (!frame) {
      setStatus((current) => ({
        ...current,
        state: 'waiting',
        message: 'Waiting for DataFrame fields named x, y and z',
      }));
      return;
    }

    displayFrame(sceneRef.current, frame, optionsRef.current);
    updateStatus(frame, 'Grafana DataFrame');
  }, [data.series, options.sourceMode, options.maxPoints, options.colorMode, options.solidColor]);

  useEffect(() => {
    if (options.sourceMode !== 'websocket') {
      return;
    }

    let socket: WebSocket | undefined;
    let reconnectTimer: number | undefined;
    let stopped = false;

    const connect = () => {
      if (stopped) {
        return;
      }
      setStatus((current) => ({
        ...current,
        state: 'connecting',
        message: `Connecting to ${options.websocketUrl}`,
      }));

      try {
        socket = new WebSocket(options.websocketUrl);
        socket.binaryType = 'arraybuffer';
      } catch (error) {
        scheduleReconnect(error instanceof Error ? error.message : String(error));
        return;
      }

      socket.onopen = () => {
        setStatus((current) => ({
          ...current,
          state: 'connected',
          message: `Connected to ${options.websocketUrl}; waiting for frames`,
        }));
      };

      socket.onmessage = (event) => {
        if (!(event.data instanceof ArrayBuffer)) {
          setStatus((current) => ({ ...current, state: 'error', message: 'Expected a binary LPC1/LDR1 frame' }));
          return;
        }
        try {
          const frame = decodePointCloudPacket(event.data);
          displayFrame(sceneRef.current, frame, optionsRef.current);
          updateStatus(frame, options.websocketUrl);
        } catch (error) {
          setStatus((current) => ({
            ...current,
            state: 'error',
            message: error instanceof Error ? error.message : String(error),
          }));
        }
      };

      socket.onerror = () => {
        setStatus((current) => ({ ...current, state: 'error', message: `WebSocket error at ${options.websocketUrl}` }));
      };

      socket.onclose = () => {
        if (!stopped) {
          scheduleReconnect('WebSocket disconnected');
        }
      };
    };

    const scheduleReconnect = (message: string) => {
      if (stopped || reconnectTimer !== undefined) {
        return;
      }
      const delay = Math.max(options.reconnectDelayMs || 2000, 250);
      setStatus((current) => ({ ...current, state: 'waiting', message: `${message}; retrying in ${delay} ms` }));
      reconnectTimer = window.setTimeout(() => {
        reconnectTimer = undefined;
        connect();
      }, delay);
    };

    connect();
    return () => {
      stopped = true;
      if (reconnectTimer !== undefined) {
        window.clearTimeout(reconnectTimer);
      }
      if (socket) {
        socket.onclose = null;
        socket.close();
      }
    };
  }, [options.sourceMode, options.websocketUrl, options.reconnectDelayMs]);

  const updateStatus = (frame: PointCloudFrame, source: string) => {
    const now = performance.now();
    if (now - statusUpdateTimeRef.current < 200) {
      return;
    }
    statusUpdateTimeRef.current = now;
    const shownPoints = sceneRef.current?.pointCount ?? 0;
    setStatus({
      state: 'connected',
      message: `Receiving from ${source}`,
      shownPoints,
      inputPoints: frame.pointCount,
      protocol: frame.protocol,
      frameId: frame.frameId?.toString() ?? '-',
    });
  };

  const resetView = () => {
    const state = sceneRef.current;
    if (!state || state.pointCount === 0) {
      return;
    }
    fitCameraToCloud(state);
  };

  const statusColor =
    status.state === 'connected' ? '#73bf69' : status.state === 'error' ? '#f2495c' : '#ffb357';

  return (
    <div ref={containerRef} style={{ width, height, position: 'relative', overflow: 'hidden' }}>
      <div
        style={{
          position: 'absolute',
          zIndex: 2,
          top: 8,
          left: 8,
          maxWidth: '70%',
          padding: '6px 9px',
          borderRadius: 4,
          color: '#d8d9da',
          background: 'rgba(10, 14, 20, 0.78)',
          fontSize: 11,
          pointerEvents: 'none',
        }}
      >
        <div style={{ color: statusColor, fontWeight: 600 }}>{status.message}</div>
        <div>
          Points {status.shownPoints.toLocaleString()} / {status.inputPoints.toLocaleString()} · {status.protocol} · frame{' '}
          {status.frameId}
        </div>
      </div>
      <button
        type="button"
        onClick={resetView}
        style={{
          position: 'absolute',
          zIndex: 2,
          top: 8,
          right: 8,
          padding: '5px 9px',
          borderRadius: 4,
          border: '1px solid #45505b',
          color: '#d8d9da',
          background: 'rgba(10, 14, 20, 0.78)',
          cursor: 'pointer',
        }}
      >
        Reset view
      </button>
    </div>
  );
};

const displayFrame = (
  state: SceneState | null,
  frame: PointCloudFrame,
  options: LidarPointCloudOptions
): void => {
  if (!state || frame.pointCount <= 0) {
    return;
  }

  const limit = Math.max(Math.floor(options.maxPoints || 100000), 1);
  const step = Math.max(Math.ceil(frame.pointCount / limit), 1);
  const displayedCount = Math.ceil(frame.pointCount / step);
  ensureCapacity(state, displayedCount);

  let minZ = Number.POSITIVE_INFINITY;
  let maxZ = Number.NEGATIVE_INFINITY;
  let maxIntensity = 0;
  for (let input = 0; input < frame.pointCount; input += step) {
    const z = frame.positions[input * 3 + 2];
    if (Number.isFinite(z)) {
      minZ = Math.min(minZ, z);
      maxZ = Math.max(maxZ, z);
    }
    if (frame.intensities) {
      maxIntensity = Math.max(maxIntensity, frame.intensities[input] || 0);
    }
  }

  const solid = new THREE.Color(options.solidColor || '#55d6ff');
  const color = new THREE.Color();
  const heightRange = Math.max(maxZ - minZ, 0.000001);
  const intensityScale = maxIntensity > 1 ? maxIntensity : 1;
  const minimum = new THREE.Vector3(Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY);
  const maximum = new THREE.Vector3(Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY);
  let outputPoint = 0;

  for (let inputPoint = 0; inputPoint < frame.pointCount; inputPoint += step) {
    const inputOffset = inputPoint * 3;
    const outputOffset = outputPoint * 3;
    const x = frame.positions[inputOffset];
    const y = frame.positions[inputOffset + 1];
    const z = frame.positions[inputOffset + 2];

    state.positions[outputOffset] = x;
    state.positions[outputOffset + 1] = y;
    state.positions[outputOffset + 2] = z;

    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      minimum.x = Math.min(minimum.x, x);
      minimum.y = Math.min(minimum.y, y);
      minimum.z = Math.min(minimum.z, z);
      maximum.x = Math.max(maximum.x, x);
      maximum.y = Math.max(maximum.y, y);
      maximum.z = Math.max(maximum.z, z);
    }

    if (options.colorMode === 'intensity' && frame.intensities) {
      const normalized = THREE.MathUtils.clamp((frame.intensities[inputPoint] || 0) / intensityScale, 0, 1);
      color.setHSL(0.66 - normalized * 0.66, 1, 0.5);
    } else if (options.colorMode === 'height') {
      const normalized = THREE.MathUtils.clamp((z - minZ) / heightRange, 0, 1);
      color.setHSL(0.66 - normalized * 0.66, 1, 0.5);
    } else {
      color.copy(solid);
    }

    state.colors[outputOffset] = color.r;
    state.colors[outputOffset + 1] = color.g;
    state.colors[outputOffset + 2] = color.b;
    outputPoint += 1;
  }

  state.pointCount = outputPoint;
  state.inputPointCount = frame.pointCount;
  state.geometry.setDrawRange(0, outputPoint);
  const positionAttribute = state.geometry.getAttribute('position') as THREE.BufferAttribute;
  const colorAttribute = state.geometry.getAttribute('color') as THREE.BufferAttribute;
  positionAttribute.needsUpdate = true;
  colorAttribute.needsUpdate = true;
  if (Number.isFinite(minimum.x)) {
    state.geometry.boundingBox = new THREE.Box3(minimum, maximum);
    state.geometry.boundingSphere = state.geometry.boundingBox.getBoundingSphere(new THREE.Sphere());
  }

  if (!state.fitted) {
    fitCameraToCloud(state);
    state.fitted = true;
  }
};

const ensureCapacity = (state: SceneState, requiredPoints: number): void => {
  if (requiredPoints <= state.capacity) {
    return;
  }
  const capacity = Math.max(requiredPoints, Math.ceil(Math.max(state.capacity, 1024) * 1.5));
  state.positions = new Float32Array(capacity * 3);
  state.colors = new Float32Array(capacity * 3);
  const positionAttribute = new THREE.BufferAttribute(state.positions, 3);
  const colorAttribute = new THREE.BufferAttribute(state.colors, 3);
  positionAttribute.setUsage(THREE.DynamicDrawUsage);
  colorAttribute.setUsage(THREE.DynamicDrawUsage);
  state.geometry.setAttribute('position', positionAttribute);
  state.geometry.setAttribute('color', colorAttribute);
  state.capacity = capacity;
};

const fitCameraToCloud = (state: SceneState): void => {
  const bounds = state.geometry.boundingBox;
  if (!bounds || bounds.isEmpty()) {
    return;
  }
  const center = bounds.getCenter(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const radius = Math.max(size.length() * 0.5, 0.5);
  state.controls.target.copy(center);
  state.camera.near = Math.max(radius / 1000, 0.001);
  state.camera.far = Math.max(radius * 100, 100);
  state.camera.position.set(center.x + radius * 1.4, center.y - radius * 1.4, center.z + radius);
  state.camera.updateProjectionMatrix();
  state.controls.update();
};

const extractDataFrame = (series: Props['data']['series']): PointCloudFrame | undefined => {
  for (let seriesIndex = series.length - 1; seriesIndex >= 0; seriesIndex -= 1) {
    const frame = series[seriesIndex];
    const x = findField(frame.fields, ['x']);
    const y = findField(frame.fields, ['y']);
    const z = findField(frame.fields, ['z']);
    if (!x || !y || !z) {
      continue;
    }

    const pointCount = Math.min(x.values.length, y.values.length, z.values.length);
    if (pointCount <= 0) {
      continue;
    }

    const intensity = findField(frame.fields, ['intensity', 'reflectivity', 'i']);
    const positions = new Float32Array(pointCount * 3);
    const intensities = intensity ? new Float32Array(pointCount) : undefined;

    for (let index = 0; index < pointCount; index += 1) {
      positions[index * 3] = Number(x.values[index]);
      positions[index * 3 + 1] = Number(y.values[index]);
      positions[index * 3 + 2] = Number(z.values[index]);
      if (intensities) {
        intensities[index] = Number(intensity?.values[index] ?? 0);
      }
    }

    return { positions, intensities, pointCount, protocol: 'DataFrame' };
  }
  return undefined;
};

const findField = (fields: Field[], names: string[]): Field | undefined => {
  const accepted = new Set(names.map((name) => name.toLowerCase()));
  return fields.find((field) => accepted.has(field.name.toLowerCase()));
};
