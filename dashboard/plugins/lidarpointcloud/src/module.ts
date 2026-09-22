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
        description: 'Dedicated LPC1/LDR1 binary point-cloud stream.',
        defaultValue: 'ws://127.0.0.1:8765',
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
        description: 'Frames larger than this limit are uniformly sampled in the browser.',
        defaultValue: 100000,
        settings: { min: 1000, max: 2000000, step: 1000 },
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
