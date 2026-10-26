import { PanelPlugin } from '@grafana/data';
import { LidarPointCloudPanel } from './components/LidarPointCloudPanel';
import { LidarPointCloudOptions } from './types';

export const plugin = new PanelPlugin<LidarPointCloudOptions>(LidarPointCloudPanel)
  .setPanelOptions((builder) =>
    builder
      .addRadio({
        path: 'sourceMode',
        name: 'Point-cloud source',
        description: 'Use a dedicated WebSocket or x/y/z fields returned by the panel query.',
        defaultValue: 'websocket',
        settings: {
          options: [
            { value: 'websocket', label: 'WebSocket' },
            { value: 'dataframe', label: 'Grafana DataFrame' },
          ],
        },
      })
      .addTextInput({
        path: 'websocketUrl',
        name: 'WebSocket URL',
        description: 'Room Map / foreground points LPC1 or geometric Gaussian Map GSP1 stream.',
        defaultValue: 'ws://127.0.0.1:8766',
      })
      .addBooleanSwitch({
        path: 'overlayForeground', name: 'Overlay Foreground on Gaussian', defaultValue: false,
        description: 'Two independent sockets, one world-coordinate camera. Main URL must serve Gaussian GSP1.',
        showIf: (o) => o.sourceMode === 'websocket',
      })
      .addTextInput({
        path: 'foregroundWebsocketUrl', name: 'Foreground WebSocket URL', defaultValue: 'ws://127.0.0.1:8768',
        description: 'LPC1 foreground from the same VISTA application/world frame as the Gaussian map.',
        showIf: (o) => o.sourceMode === 'websocket' && Boolean(o.overlayForeground),
      })
      .addNumberInput({
        path: 'foregroundMaxPoints', name: 'Maximum Foreground points', defaultValue: 12000,
        settings: { min: 1, max: 50000, step: 1000 }, showIf: (o) => Boolean(o.overlayForeground),
      })
      .addNumberInput({
        path: 'foregroundPointSize', name: 'Foreground point size', defaultValue: 3,
        settings: { min: 0.1, max: 20, step: 0.1 }, showIf: (o) => Boolean(o.overlayForeground),
      })
      .addColorPicker({
        path: 'foregroundColor', name: 'Foreground color', defaultValue: '#ff7043',
        showIf: (o) => Boolean(o.overlayForeground),
      })
      .addNumberInput({
        path: 'foregroundStaleTimeoutMs', name: 'Foreground expiry (ms)', defaultValue: 1000,
        description: 'Only the Foreground layer expires. Gaussian geometry is retained on disconnect.',
        settings: { min: 100, max: 60000, step: 100 }, showIf: (o) => Boolean(o.overlayForeground),
      })
      .addNumberInput({
        path: 'reconnectDelayMs',
        name: 'Reconnect delay (ms)',
        defaultValue: 2000,
        settings: { min: 250, max: 60000, step: 250 },
      })
      .addNumberInput({
        path: 'maxPoints',
        name: 'Maximum displayed points',
        description: 'Camera-dependent Room Map / Gaussian Map view budget. Stored maps are not capped.',
        defaultValue: 100000,
        settings: { min: 1000, max: 2000000, step: 1000 },
      })
      .addNumberInput({
        path: 'staleTimeoutMs',
        name: 'Clear stale geometry after (ms)',
        description: '0 retains map views. Use 1000 for transient foreground points; disconnect also clears.',
        defaultValue: 0,
        settings: { min: 0, max: 60000, step: 100 },
      })
      .addNumberInput({
        path: 'pointSize',
        name: 'Point size',
        defaultValue: 2,
        settings: { min: 0.1, max: 20, step: 0.1 },
      })
      .addRadio({
        path: 'colorMode',
        name: 'Point color',
        defaultValue: 'height',
        settings: {
          options: [
            { value: 'height', label: 'Height' },
            { value: 'intensity', label: 'Intensity' },
            { value: 'solid', label: 'Solid' },
          ],
        },
      })
      .addColorPicker({
        path: 'solidColor',
        name: 'Solid color',
        defaultValue: '#55d6ff',
      })
      .addColorPicker({
        path: 'backgroundColor',
        name: 'Background',
        defaultValue: '#0b0f14',
      })
      .addBooleanSwitch({ path: 'showGrid', name: 'Show ground grid', defaultValue: true })
      .addBooleanSwitch({ path: 'showAxes', name: 'Show XYZ axes', defaultValue: true })
      .addBooleanSwitch({ path: 'autoRotate', name: 'Auto rotate', defaultValue: false })
  );
