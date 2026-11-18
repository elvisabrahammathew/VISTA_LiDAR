import React, {useState} from 'react';
import {createRoot} from 'react-dom/client';
import {LidarPointCloudPanel} from '../src/components/LidarPointCloudPanel';
const defaults = {sourceMode: 'websocket', websocketUrl: 'ws://127.0.0.1:19998',
  foregroundWebsocketUrl: 'ws://127.0.0.1:19999', showGrid: true, showAxes: true,
  backgroundColor: '#0b0f14', reconnectDelayMs: 60000, maxPoints: 10000};
function App() {
  const [size, setSize] = useState({width: 600, height: 440});
  (window as any).resizePanels = (width: number, height: number) => setSize({width, height});
  return <div style={{display: 'flex', gap: 12}}>
    {[true, false].map(overlayForeground => <section key={String(overlayForeground)}>
      <h3>{overlayForeground ? 'Gaussian Room Map and Foreground' : 'Room Map - Out-of-Core LOD'}</h3>
      <div data-testid="panel" style={{width: size.width, height: size.height, background: '#181b1f'}}>
        <LidarPointCloudPanel {...({options: {...defaults, overlayForeground}, ...size, data: {series: []}} as any)}/>
      </div>
    </section>)}
  </div>;
}
createRoot(document.getElementById('root')!).render(<App/>);
