import { PanelProps } from '@grafana/data';
import React, { useEffect, useMemo, useState, useSyncExternalStore } from 'react';
import { SettingField, settingGroups } from '../settings_schema';
import { settingsClient } from '../runtime_settings';
import type { LidarPointCloudOptions } from '../types';

const control: React.CSSProperties = {
  all: 'revert', boxSizing: 'border-box', font: 'inherit', color: 'inherit', background: '#1d252e',
  border: '1px solid #566675', borderRadius: 4, colorScheme: 'dark', minHeight: 34, padding: '4px 8px',
  minWidth: 0, maxWidth: '100%',
};
const buttons: React.CSSProperties = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: 10 };
const button: React.CSSProperties = { ...control, cursor: 'pointer', whiteSpace: 'nowrap' };
const label: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: 5, minWidth: 0 };
const pairs = [
  { key: 'LioImuAccelerationCovariance', also: 'LioImuGyroscopeCovariance', label: 'IMU measurement covariance' },
  { key: 'LioImuAccelerationBiasCovariance', also: 'LioImuGyroscopeBiasCovariance', label: 'IMU bias covariance' },
];
const l515 = ['UsbSerial','DepthWidth','DepthHeight','DepthFps'];
const udp = ['SensorIP','SensorPort','LocalIP','LocalPort'];
const serial = ['SerialPort','BaudRate'];
const outliers = ['ForegroundOutlierRadiusMeters','ForegroundOutlierStartDistanceMeters','ForegroundOutlierEndDistanceMeters',
  'ForegroundOutlierMaxRadiusMeters','ForegroundOutlierMinNeighbors'];

function visible(key: string, values: Record<string,string>): boolean {
  if (l515.includes(key)) { return values.Lidar === 'realsense-l515'; }
  if (udp.includes(key)) {
    return values.Lidar === 'unitree-l2' ? values.ConnectionMode !== 'serial'
      : values.Lidar === 'quanergy-m8' && (key === 'SensorIP' || key === 'SensorPort');
  }
  if (serial.includes(key)) {
    return values.Lidar === 'unitree-l2' && (values.ConnectionMode !== 'udp' || values.InitializeOnConnect === '1');
  }
  if (key === 'WorkMode') { return values.Lidar === 'unitree-l2' && values.InitializeOnConnect === '1'; }
  if (key === 'ConnectionMode' || key === 'InitializeOnConnect') { return values.Lidar === 'unitree-l2'; }
  if (key === 'RoomMapFile') { return values.RoomMapLoadExisting === '1'; }
  return !outliers.includes(key) || values.ForegroundOutlierFilterEnabled === '1';
}

export const RuntimeSettingsPanel: React.FC<PanelProps<LidarPointCloudOptions>> = ({ options, width, height, replaceVariables }) => {
  const url = replaceVariables(options.settingsUrl || 'ws://127.0.0.1:8769');
  const group = options.settingsGroup || '';
  const definition = settingGroups[group];
  const client = useMemo(() => settingsClient(url), [url]);
  const state = useSyncExternalStore(client.subscribe, client.getSnapshot);
  const [token, setToken] = useState('');
  const [draft, setDraft] = useState<Record<string,{ value: string; base: string }>>({});
  const [error, setError] = useState('');
  const active = state.snapshot?.settings ?? {};
  const current = { ...active, ...Object.fromEntries(Object.entries(draft).map(([key,data]) => [key,data.value])) };
  const formNotice = state.notice?.group === group ? state.notice : undefined;
  const busy = !!state.snapshot?.busy || state.pending !== undefined;
  const connected = state.phase === 'ready';
  const isAuthPanel = group === 'logging';

  useEffect(() => {
    if (formNotice?.ok) { setDraft({}); setError(''); }
  }, [formNotice]);
  useEffect(() => { setDraft({}); setError(''); }, [client, group]);

  const change = (changes: Record<string,string>) => {
    setError('');
    setDraft(old => {
      const next = { ...old };
      for (const [key,value] of Object.entries(changes)) {
        next[key] = { value, base: old[key]?.base ?? active[key] };
      }
      return next;
    });
  };

  const apply = (event: React.FormEvent) => {
    event.preventDefault();
    setError('');
    const changes: Record<string,string> = {};
    for (const field of definition.fields) {
      const value = draft[field.key];
      if (!value || field.readOnly || !visible(field.key,current)) { continue; }
      if (value.base !== active[field.key] && value.value !== active[field.key]) {
        setError('A value changed on another dashboard. Use Discard then review the active values.');
        return;
      }
      let encoded = value.value;
      if (field.kind === 'i' || field.kind === 'f') {
        const number = Number(encoded);
        if (encoded.trim() === '' || !Number.isFinite(number) || (field.kind === 'i' && (!Number.isSafeInteger(number) || number < 0))) {
          setError(field.label + ' requires a valid number.');
          return;
        }
        if (active[field.key] !== '' && number === Number(active[field.key])) { continue; }
        encoded = String(number);
      }
      if (encoded !== active[field.key]) { changes[field.key] = encoded; }
    }
    if (!Object.keys(changes).length) { setError('Values are already active.'); return; }
    try { client.apply(group, changes); }
    catch (failure) { setError(failure instanceof Error ? failure.message : 'Could not send changes.'); }
  };

  const fieldControl = (field: SettingField) => {
    const value = current[field.key] ?? '';
    const disabled = busy || (!!field.readOnly && field.kind !== 's');
    return <label key={field.key} style={field.kind === 'b' ? { ...buttons, alignSelf: 'center' } : label}>
      {field.kind === 'b' ? <>
        <input type="checkbox" checked={value === '1'} disabled={disabled}
          onChange={event => change({ [field.key]: event.target.checked ? '1' : '0' })}
          style={{ appearance: 'auto', width: 16, height: 16, accentColor: '#6cafff' }} />
        {field.label}
      </> : <>
        <span>{field.label}</span>
        {field.kind === 's' || field.kind === 'p' ? (
          <select value={value} disabled={disabled} style={control}
            onChange={event => { if (!field.readOnly) change({ [field.key]: event.target.value }); }}>
            {field.kind === 'p' ? [...new Set([value,...state.ports].filter(Boolean))].map(port =>
              <option key={port} value={port}>{port}</option>) :
              field.options?.map(option => <option key={option} value={option}
                disabled={option === 'RealSense' || option === 'realsense-l515' && !state.snapshot?.realSense}>
                {option === 'realsense-l515' && !state.snapshot?.realSense ? option + ' (unavailable in this build)' : option}
              </option>)}
          </select>
        ) : (
          <input type={field.kind === 'i' || field.kind === 'f' ? 'number' : 'text'}
            step={field.kind === 'i' ? 1 : 'any'} value={value} readOnly={field.readOnly}
            disabled={busy} style={control} placeholder={field.key === 'RoomMapFile' ? 'RoomMap_YYYYMMDD_HHMMSS.pcd' : undefined}
            onChange={event => change({ [field.key]: event.target.value })} />
        )}
      </>}
    </label>;
  };

  return <div style={{ width,height,boxSizing:'border-box',overflow:'auto',padding:'12px',fontSize:13 }}>
    {!definition ? <div>Select a settings group in the panel options.</div> : <>
      {!connected && <div role="status" style={{ marginBottom: 8 }}>
        {state.phase === 'connecting' ? 'Connecting to VISTA Edge…' :
          state.phase === 'offline' ? 'VISTA Edge settings connection unavailable.' :
          isAuthPanel ? 'Enter the control token to load the active settings.' : 'Connect in Capture Logging to load the active settings.'}
      </div>}
      {isAuthPanel && !connected && <form style={buttons} onSubmit={event => {
        event.preventDefault(); client.authenticate(token.trim());
      }}>
        <input type="password" autoComplete="off" aria-label="Control token" placeholder="Control token"
          value={token} onChange={event => setToken(event.target.value)} style={control} />
        <button type="submit" style={button} disabled={state.phase !== 'token'}>Connect</button>
        {state.notice && <span role="alert">{state.notice.message}</span>}
      </form>}
      {connected && <form onSubmit={apply}>
        <div style={{ display:'grid',gridTemplateColumns:'repeat(auto-fit, minmax(min(100%, 170px), 1fr))',gap:'10px 14px' }}>
          {definition.fields.filter(field => visible(field.key,current)).map(field => {
            const linked = pairs.find(pair => pair.key === field.key);
            if (pairs.some(pair => pair.also === field.key)) { return null; }
            if (!linked) { return fieldControl(field); }
            const first = current[linked.key];
            const second = current[linked.also];
            return <label key={field.key} style={label}>
              <span>{linked.label}</span>
              <input type="number" step="any" value={first === second ? first : ''}
                placeholder={first === second ? '' : 'Enter a shared value'}
                disabled={busy} style={control}
                onChange={event => change({ [linked.key]: event.target.value,[linked.also]: event.target.value })} />
              {first !== second && <small>Acceleration {first} · Gyroscope {second}</small>}
            </label>;
          })}
        </div>
        {group === 'camera' && <p>Independent camera capture is unavailable in VISTA Edge.</p>}
        {group === 'radar' && <p>No radar driver is available.</p>}
        {group !== 'camera' && group !== 'radar' && <div style={{ ...buttons, marginTop: 12 }}>
          <button type="submit" disabled={busy || Object.keys(draft).length === 0} style={button}>
            {state.pending === group ? 'Applying…' : 'Apply'}
          </button>
          <button type="button" disabled={busy || Object.keys(draft).length === 0} style={button}
            onClick={() => { setDraft({}); setError(''); }}>
            Discard
          </button>
          {group === 'lidar' && visible('SerialPort',current) &&
            <button type="button" onClick={() => client.refreshPorts()} style={button}>Refresh ports</button>}
          {(error || formNotice) && <span role="status" style={{ color: error || formNotice?.ok === false ? '#ff8996' : '#73bf69' }}>
            {error || formNotice?.message}
          </span>}
        </div>}
      </form>}
    </>}
  </div>;
};
