import {PanelPlugin} from '@grafana/data';
import {DigitalTwinPanel, Options} from './panel';
export const plugin = new PanelPlugin<Options>(DigitalTwinPanel).setPanelOptions(builder => builder
  .addTextInput({path: 'bridgeUrl', name: 'Windows bridge URL', defaultValue: 'http://127.0.0.1:8101',
    description: 'Read-only bridge on the Windows browser computer. No gateway token in dashboard JSON.'})
  .addNumberInput({path: 'pollMs', name: 'Update interval (ms)', defaultValue: 250, settings: {min: 100, max: 2000}})
  .addBooleanSwitch({path: 'wireframe', name: 'Wireframe', defaultValue: true}));
