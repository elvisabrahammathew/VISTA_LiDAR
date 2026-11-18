import React from 'react';
import {createRoot} from 'react-dom/client';
import {DigitalTwinPanel} from '../src/panel';
createRoot(document.getElementById('root')!).render(<DigitalTwinPanel {...({
  width: 1100, height: 680,
  options: {bridgeUrl: 'http://127.0.0.1:18101', pollMs: 150, wireframe: true}
} as any)} />);
