import type { PanelProps } from '@grafana/data';
import React, { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { LidarPointCloudOptions } from '../types';
import { OverlayLayers, boundedCount } from '../overlay_layers';
import { OverlayStream } from '../overlay_stream';
import { defaultCornerRadiusMeters, gaussianModes, makeGaussianViewRequest, maximumCornerRadiusMeters,
  normalizeGaussianMode, RoomMapMetadata, supportsGaussianMode, validCornerRadius } from '../room_map_view';
import { ForegroundStatus, foregroundStatusMessage } from '../foreground_status';

import { LidarMarker, LidarPoseMarker } from './LidarPoseMarker';

interface View {
  renderer: THREE.WebGLRenderer; scene: THREE.Scene; camera: THREE.PerspectiveCamera;
  controls: OrbitControls; layers: OverlayLayers; grid: THREE.GridHelper; axes: THREE.AxesHelper;
  fitted: boolean; bounds?: THREE.Box3; animation: number;
}
interface Status {
  gaussian: string; foreground: string; gaussianError: boolean; foregroundError: boolean;
  gaussianCount: number; foregroundCount: number; detail?: ForegroundStatus;
}
const fit = (view: View, bounds: THREE.Box3) => {
  if(bounds.isEmpty()) { return; }
  const center=bounds.getCenter(new THREE.Vector3()),radius=Math.max(.25,bounds.getSize(new THREE.Vector3()).length()/2);
  const vertical=THREE.MathUtils.degToRad(view.camera.fov);
  const horizontal=2*Math.atan(Math.tan(vertical/2)*view.camera.aspect);
  const distance=radius/Math.sin(Math.min(vertical,horizontal)/2)*1.1;
  view.controls.target.copy(center);
  view.camera.position.copy(center).addScaledVector(new THREE.Vector3(1.4,-1.4,1).normalize(),distance);
  view.camera.near=Math.max(.001,radius/1000);view.camera.far=Math.max(100,distance+radius*100);
  view.camera.updateProjectionMatrix();view.controls.minDistance=.005;view.controls.maxDistance=Math.max(100,radius*100);
  view.controls.update();view.fitted=true;
};

// A separate optional component preserves the legacy single-socket/DataFrame
// path. Both world-coordinate streams share exactly one scene, camera and canvas.
export const GaussianForegroundPanel: React.FC<PanelProps<LidarPointCloudOptions>> = ({options,width,height,onOptionsChange}) => {
  const container=useRef<HTMLDivElement>(null),viewRef=useRef<View | null>(null),optionsRef=useRef(options);
  const lidarMarker = useMemo(() => new LidarMarker(), []);
  const markerElement = useRef<HTMLDivElement>(null);
  const [showLidar, setShowLidar] = useState(true);
  lidarMarker.visible = showLidar;
  optionsRef.current=options;
  const [showGaussian,setShowGaussian]=useState(true),[showForeground,setShowForeground]=useState(true);
  const [capabilities,setCapabilities]=useState<{edges:boolean;corners:boolean;defaultRadius?:number}>();
  const [radiusDraft,setRadiusDraft]=useState<string>();
  const radiusOverride=validCornerRadius(options.gaussianCornerRadiusMeters) ? options.gaussianCornerRadiusMeters : undefined;
  const displayedRadius=Number((radiusOverride ?? capabilities?.defaultRadius ?? defaultCornerRadiusMeters).toFixed(2));
  const radiusSupported=capabilities?.defaultRadius!==undefined;
  const commitRadius=(raw:string)=>{
    setRadiusDraft(undefined);
    if(raw.trim()==='') {
      onOptionsChange({...optionsRef.current,gaussianCornerRadiusMeters:undefined});
      return;
    }
    const entered=Number(raw);
    if(validCornerRadius(entered))
      onOptionsChange({...optionsRef.current,gaussianCornerRadiusMeters:Math.round(entered*100)/100});
  };
  const surfaceMode=normalizeGaussianMode(options.gaussianSurfaceMode);
  const [status,setStatus]=useState<Status>({gaussian:'Waiting for Gaussian map',foreground:'Waiting for Foreground',
    gaussianError:false,foregroundError:false,gaussianCount:0,foregroundCount:0});

  useEffect(()=>{
    if(!container.current) { return; }
    let renderer: THREE.WebGLRenderer;
    try { renderer=new THREE.WebGLRenderer({antialias:true,alpha:false}); }
    catch(e) {setStatus(s=>({...s,gaussian:String(e),gaussianError:true}));return;}
    const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(55,Math.max(width,1)/Math.max(height,1),.01,10000);
    camera.up.set(0,0,1);camera.position.set(6,-6,4);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));renderer.setSize(Math.max(width,1),Math.max(height,1),false);
    // Keep CSS size independent of the pixel-ratio-scaled drawing buffer.
    Object.assign(renderer.domElement.style,{display:'block',width:'100%',height:'100%'});
    container.current.appendChild(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.08;
    const layers=new OverlayLayers(scene),grid=new THREE.GridHelper(60,60,0x547080,0x26313a),axes=new THREE.AxesHelper(2);
    grid.rotation.x=Math.PI/2;scene.add(grid,axes);
    const view:View={renderer,scene,camera,controls,layers,grid,axes,fitted:false,animation:0};viewRef.current=view;
    const animate=()=>{
      controls.autoRotate=Boolean(optionsRef.current.autoRotate);controls.autoRotateSpeed=.8;controls.update();
      if(layers.gaussian.mesh.visible) {
        const size=renderer.getDrawingBufferSize(new THREE.Vector2());layers.gaussian.update(camera,size.x,size.y,performance.now());
      }
      renderer.render(scene,camera);
      lidarMarker.draw(markerElement.current, camera, renderer);
      view.animation=window.requestAnimationFrame(animate);
    };animate();
    return ()=>{
      window.cancelAnimationFrame(view.animation);controls.dispose();layers.dispose();
      grid.geometry.dispose();(grid.material as THREE.Material).dispose();
      axes.geometry.dispose();(axes.material as THREE.Material).dispose();
      renderer.dispose();renderer.domElement.remove();viewRef.current=null;
    };
    // Create a single canvas; sizing/styling/layer connections are independent.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  },[]);

  useEffect(()=>{
    const view=viewRef.current;if(!view){return;}
    view.camera.aspect=Math.max(width,1)/Math.max(height,1);view.camera.updateProjectionMatrix();
    view.renderer.setSize(Math.max(width,1),Math.max(height,1),false);
  },[width,height]);
  useEffect(()=>{
    const view=viewRef.current;if(!view){return;}
    view.scene.background=new THREE.Color(options.backgroundColor||'#0b0f14');
    view.grid.visible=options.showGrid!==false;view.axes.visible=options.showAxes!==false;
    view.layers.configure(optionsRef.current);
    setStatus(s=>({...s,gaussianCount:view.layers.gaussianCount,foregroundCount:view.layers.foregroundCount}));
  },[options.backgroundColor,options.showGrid,options.showAxes,options.colorMode,options.solidColor,options.maxPoints,
    options.foregroundColor,options.foregroundPointSize,options.foregroundMaxPoints]);
  useEffect(()=>{
    const view=viewRef.current;if(view){view.layers.gaussian.mesh.visible=showGaussian;view.layers.foreground.visible=showForeground;}
  },[showGaussian,showForeground]);

  useEffect(()=>{
    const view=viewRef.current;if(!view){return;}
    const layers=view.layers;let metadata:RoomMapMetadata|undefined,foregroundMeta:ForegroundStatus|undefined;
    layers.clearGaussian();layers.clearForeground();view.bounds=undefined;view.fitted=false;
    setStatus(s=>({...s,gaussianCount:0,foregroundCount:0,detail:undefined}));
    setCapabilities(undefined);
    const unavailable = () => {
      layers.clearGaussian();
      setStatus(s=>s.gaussian==='Selected display mode requires an updated vista_edge' ? s :
        {...s, gaussian:'Selected display mode requires an updated vista_edge', gaussianError:true, gaussianCount:0});
    };
    const gaussian=new OverlayStream('gaussian',options.websocketUrl,{
      frame:(frame)=>{
        const requested=normalizeGaussianMode(optionsRef.current.gaussianSurfaceMode);
        if(!supportsGaussianMode(requested,metadata)) {
          unavailable();
          return;
        }
        // A previous mode may still be in flight. Metadata and its binary
        // replacement are ordered on this socket; never label Full as Edges.
        if(metadata?.surfaceMode && metadata.surfaceMode!==requested){return;}
        if(requested==='corners' && metadata?.defaultCornerRadiusMeters!==undefined) {
          const override=optionsRef.current.gaussianCornerRadiusMeters;
          const expected=validCornerRadius(override) ? override : metadata.defaultCornerRadiusMeters;
          if (metadata.cornerRadiusMeters!==expected) {return;}
        }
        layers.setGaussian(frame,optionsRef.current);
        lidarMarker.display.showMap(metadata?.mappingGeneration);
        if(!view.fitted && frame.pointCount) {
          const bounds=new THREE.Box3().setFromArray(frame.positions);fit(view,bounds);
        }
        const label=gaussianModes.find(mode=>mode.value===requested)?.label;
        setStatus(s=>({...s,gaussian:`Receiving ${label}`,gaussianError:false,gaussianCount:layers.gaussianCount}));
      },
      clear:(reason)=>{layers.clearGaussian();setStatus(s=>({...s,gaussian:reason,gaussianCount:0}));},
      status:(message,error=false)=>{
        if(message.startsWith('Connecting to ') || message.startsWith('Disconnected;')) {
          metadata=undefined;
          setCapabilities(undefined);
        }
        setStatus(s=>({...s,gaussian:message,gaussianError:error}));
      },
      pose: pose => lidarMarker.display.receive(pose),
      disconnected: () => lidarMarker.display.disconnect(),
      map:(meta)=>{
        metadata=meta;
        const edges=meta.supportsEdges===true,corners=meta.supportsCorners===true;
        const defaultRadius=meta.defaultCornerRadiusMeters;
        setCapabilities(previous=>previous?.edges===edges && previous.corners===corners &&
          previous.defaultRadius===defaultRadius ? previous : {edges,corners,defaultRadius});
        if(meta.totalPoints>0 && meta.state<4) {
          view.bounds=new THREE.Box3(new THREE.Vector3(...meta.bounds.slice(0,3)),new THREE.Vector3(...meta.bounds.slice(3,6)));
          if(!view.fitted){fit(view,view.bounds);}
        }
      },
    },options.reconnectDelayMs);
    const foreground=new OverlayStream('foreground',options.foregroundWebsocketUrl||'ws://127.0.0.1:8768',{
      frame:(frame)=>{
        layers.setForeground(frame,optionsRef.current);
        setStatus(s=>({...s,foreground:foregroundMeta?foregroundStatusMessage(foregroundMeta):'Receiving Foreground',
          foregroundError:false,foregroundCount:layers.foregroundCount,detail:foregroundMeta}));
      },
      clear:(reason)=>{layers.clearForeground();setStatus(s=>({...s,foreground:reason,foregroundCount:0}));},
      status:(message,error=false)=>setStatus(s=>({...s,foreground:message,foregroundError:error})),
      foreground:(meta)=>{
        foregroundMeta=meta;
        setStatus(s=>({...s,foreground:foregroundStatusMessage(meta),foregroundError:false,detail:meta}));
      },
    },options.reconnectDelayMs,options.foregroundStaleTimeoutMs??1000);
    gaussian.start();foreground.start();let lastView=0;
    const timer=window.setInterval(()=>{
      foreground.expire();const now=performance.now();
      if(metadata && now-lastView>=400 && layers.gaussian.mesh.visible) {
        lastView=now;const viewport=view.renderer.getSize(new THREE.Vector2());
        const mode=normalizeGaussianMode(optionsRef.current.gaussianSurfaceMode);
        if(!supportsGaussianMode(mode,metadata)) unavailable();
        else gaussian.requestView(makeGaussianViewRequest(view.camera,viewport.y,
          Math.min(boundedCount(optionsRef.current.maxPoints,100000,2000000),metadata.pointBudget),mode,
          metadata.defaultCornerRadiusMeters!==undefined ? optionsRef.current.gaussianCornerRadiusMeters : undefined));
      }
    },100);
    return ()=>{
      window.clearInterval(timer);
      gaussian.stop();
      foreground.stop();
      layers.clearForeground();
      lidarMarker.display.reset();
    };
  },[options.websocketUrl,options.foregroundWebsocketUrl,options.reconnectDelayMs,options.foregroundStaleTimeoutMs,lidarMarker]);

  const reset=()=>{
    const view=viewRef.current;if(!view){return;}
    const bounds=view.bounds??view.layers.foregroundBounds();if(bounds){fit(view,bounds);}
  };
  const box:React.CSSProperties={position:'absolute',zIndex:2,padding:'6px 9px',borderRadius:4,
    color:'#d8d9da',background:'rgba(10,14,20,.82)',fontSize:11};
  // Native form controls also inherit Grafana's global element styles. Reset
  // and size each control explicitly so the select cannot collapse to a line.
  const control:React.CSSProperties={all:'revert',boxSizing:'border-box',font:'inherit',
    color:'#d8d9da',margin:0,verticalAlign:'middle'};
  const label:React.CSSProperties={...control,display:'inline-flex',alignItems:'center',gap:6,
    minHeight:32,lineHeight:'20px',padding:0,flex:'0 0 auto',whiteSpace:'nowrap'};
  const checkbox:React.CSSProperties={...control,appearance:'auto',display:'inline-block',
    width:14,height:14,minWidth:14,minHeight:14,padding:0,flex:'0 0 14px',cursor:'pointer',accentColor:'#8ab8ff'};
  const select:React.CSSProperties={...control,appearance:'auto',WebkitAppearance:'menulist',
    display:'inline-block',height:32,minHeight:32,maxHeight:32,width:144,minWidth:144,
    flex:'0 0 144px',lineHeight:'normal',padding:'0 8px',backgroundColor:'#25313c',
    border:'1px solid #677889',borderRadius:4,cursor:'pointer',colorScheme:'dark'};
  const button:React.CSSProperties={...control,display:'inline-flex',alignItems:'center',justifyContent:'center',
    height:32,minHeight:32,flex:'0 0 auto',lineHeight:'20px',padding:'0 10px',
    backgroundColor:'#25313c',border:'1px solid #677889',borderRadius:4,cursor:'pointer',whiteSpace:'nowrap'};
  return <div ref={container} style={{width,height,position:'relative',overflow:'hidden'}}>
    <div style={{...box,top:8,left:8,maxWidth:'calc(100% - 16px)',maxHeight:110,overflow:'auto'}} role="status">
      <div style={{color:status.gaussianError?'#f2495c':'#a6b4c4'}}>Gaussian: {status.gaussian} · {status.gaussianCount.toLocaleString()} splats</div>
      <div style={{color:status.foregroundError?'#f2495c':options.foregroundColor||'#ff7043'}}>Foreground: {status.foreground} · {status.foregroundCount.toLocaleString()} points</div>
      {status.detail && <div>Processed {status.detail.inputPoints.toLocaleString()} · Background {status.detail.backgroundPoints.toLocaleString()} · Unresolved {status.detail.unknownPoints.toLocaleString()}</div>}
    </div>
    <LidarPoseMarker elementRef={markerElement} />
    <div style={{...box,bottom:8,right:8,display:'flex',gap:'6px 12px',alignItems:'center',justifyContent:'flex-end',
      flexWrap:'wrap',maxWidth:'calc(100% - 16px)',boxSizing:'border-box',fontSize:12,lineHeight:'20px',
      background:'rgba(10,14,20,.94)',border:'1px solid #354351'}} role="toolbar" aria-label="Map layers">
      <label style={label}><input style={checkbox} type="checkbox" checked={showGaussian} onChange={e=>setShowGaussian(e.target.checked)}/> Gaussian</label>
      <label style={label}>Display <select aria-label="Gaussian display" value={surfaceMode}
        onChange={e=>onOptionsChange({...optionsRef.current,gaussianSurfaceMode:normalizeGaussianMode(e.target.value)})}
        style={select}>
        {gaussianModes.map(({value,label})=>
          <option key={value} style={{backgroundColor:'#25313c',color:'#d8d9da'}} value={value}
            disabled={value!=='full_surface' && capabilities?.[value]===false}>{label}</option>)}
      </select></label>
      {surfaceMode==='corners' && <>
        <label style={label} title={radiusSupported ? 'Press Enter to apply the radius.' : 'Requires vista_edge with corner radius support.'}>
          Corner radius (m)
          <input type="number" min={0} max={maximumCornerRadiusMeters} step={0.01}
            value={radiusDraft ?? displayedRadius} disabled={!radiusSupported}
            onChange={event=>setRadiusDraft(event.target.value)}
            onBlur={event=>{ if(radiusDraft!==undefined) commitRadius(event.target.value); }}
            onKeyDown={event=>{ if(event.key==='Enter') {event.preventDefault();event.currentTarget.blur();} }}
            style={{...select,appearance:'auto',WebkitAppearance:undefined,width:68,minWidth:68,flex:'0 0 68px'}} />
        </label>
        {radiusOverride!==undefined && radiusSupported && <button type="button" style={button}
          onClick={()=>commitRadius('')}>Use device default</button>}
      </>}
      <label style={label}><input style={checkbox} type="checkbox" checked={showForeground} onChange={e=>setShowForeground(e.target.checked)}/> Foreground</label>
      <label style={label}><input style={checkbox} type="checkbox" checked={showLidar}
        onChange={event=>setShowLidar(event.target.checked)} /> Show LiDAR</label>
      <button type="button" onClick={reset} style={button}>Reset view</button>
    </div>
  </div>;
};
