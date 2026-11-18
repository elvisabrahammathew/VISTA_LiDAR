import React, { useEffect, useState } from 'react';
import { MappingChange, MappingState, mappingCommand, parseMount } from '../mapping_control';

type Props = {
  state: MappingState;
  error?: { message: string };
  send: (command: string) => void;
  resetView: () => void;
};

const fields = [
  { label: 'X (m)', name: 'Mount X, meters' }, { label: 'Y (m)', name: 'Mount Y, meters' },
  { label: 'Z (m)', name: 'Mount Z, meters' }, { label: 'Roll (°)', name: 'Mount Roll, degrees' },
  { label: 'Pitch (°)', name: 'Mount Pitch, degrees' }, { label: 'Yaw (°)', name: 'Mount Yaw, degrees' },
];

const control: React.CSSProperties = {
  all: 'revert', boxSizing: 'border-box', display: 'inline-block', font: 'inherit', color: '#d8d9da',
  height: 32, minHeight: 32, padding: '0 7px', margin: 0, background: '#25313c',
  colorScheme: 'dark', border: '1px solid #677889', borderRadius: 4,
};
const label: React.CSSProperties = {
  display: 'inline-flex', alignItems: 'center', gap: 5, whiteSpace: 'nowrap',
};
const toolbar: React.CSSProperties = {
  position: 'absolute', zIndex: 2, bottom: 8, right: 8, maxWidth: 'calc(100% - 16px)',
  boxSizing: 'border-box', display: 'flex', flexWrap: 'wrap', alignItems: 'center',
  justifyContent: 'flex-end', gap: '6px 10px', padding: '6px 9px', borderRadius: 4,
  color: '#d8d9da', background: 'rgba(10,14,20,.94)', border: '1px solid #354351', fontSize: 12,
};

export const RoomMapControls: React.FC<Props> = ({ state, error, send, resetView }) => {
  const [draft, setDraft] = useState<string[]>();
  const [token, setToken] = useState('');
  const [request, setRequest] = useState<{ generation: number; change: MappingChange }>();
  const [notice, setNotice] = useState('');
  const generation = state.enabled ? state.generation : undefined;

  useEffect(() => {
    if (!error) { return; }
    setRequest(undefined);
    setNotice(error.message);
  }, [error]);

  useEffect(() => {
    if (!request || generation === undefined) { return; }
    if (generation > request.generation) {
      if ('mount' in request.change) { setDraft(undefined); }
      setNotice('Pose reset applied for this session.');
      setRequest(undefined);
      return;
    }
    const timeout = window.setTimeout(() => {
      setNotice('No pose reset confirmation from VISTA Edge. Check the active values.');
      setRequest(undefined);
    }, 5000);
    return () => window.clearTimeout(timeout);
  }, [generation, request]);

  const submit = (change: MappingChange) => {
    if (!state.enabled || request || state.pending) { return; }
    try {
      send(mappingCommand(state.generation, token, change));
      setRequest({ generation: state.generation, change });
      setNotice('Resetting the mapping pose…');
    } catch (failure) {
      setNotice(failure instanceof Error ? failure.message : 'Unable to send the pose reset.');
    }
  };

  const values = draft ?? (state.enabled ? state.mount.map(value => String(Number(value.toPrecision(7)))) : []);
  const busy = (state.enabled && state.pending) || request !== undefined;
  const ready = state.enabled && !busy && /^[A-Za-z0-9_-]{24,128}$/.test(token);
  const buttonStyle: React.CSSProperties = { ...control, padding: '0 10px', cursor: 'pointer', whiteSpace: 'nowrap' };
  return <form style={toolbar} aria-label="Room Map controls" onSubmit={event => {
    event.preventDefault();
    const mount = parseMount(values);
    if (mount && ready) { submit({ mount }); }
  }}>
    {state.enabled ? <>
      <strong>Mount</strong>
      {fields.map(({ label: text, name }, index) => (
        <label style={label} key={name}>{text}
          <input aria-label={name} type="number" step="any" value={values[index]} required
            min={index < 3 ? -1000 : -360} max={index < 3 ? 1000 : 360} disabled={busy}
            onChange={event => {
              const next = [...values];
              next[index] = event.target.value;
              setDraft(next);
              setNotice('');
            }}
            style={{ ...control, width: 65 }} />
        </label>
      ))}
      <label style={label}>
        <input type="checkbox" checked={state.lioEnabled} disabled={!ready}
          onChange={event => submit({ lioEnabled: event.target.checked })}
          style={{ all: 'revert', appearance: 'auto', display: 'inline-block', width: 14, height: 14,
            margin: 0, accentColor: '#8ab8ff' }} />
        LIO Enabled
      </label>
      <input type="password" aria-label="Room Map control token" placeholder="Control token"
        title="Enter the VISTA Edge control token to change Mount or LIO"
        value={token} onChange={event => { setToken(event.target.value); setNotice(''); }}
        autoComplete="off" disabled={busy} style={{ ...control, width: 115 }} />
      <button type="submit" disabled={!ready || !parseMount(values)}
        style={{ ...buttonStyle, opacity: ready && parseMount(values) ? 1 : 0.5 }}>Apply</button>
    </> : <span>Remote mapping controls disabled</span>}
    <button type="button" onClick={resetView} style={buttonStyle}>Reset view</button>
    {notice && <span style={{ flexBasis: '100%', textAlign: 'right' }} role="status">{notice}</span>}
  </form>;
};
