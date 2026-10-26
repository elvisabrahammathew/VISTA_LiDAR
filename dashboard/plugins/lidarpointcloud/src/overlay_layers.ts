import * as THREE from 'three';
import { GaussianRenderer } from './gaussian_renderer';
import type { LidarPointCloudOptions, PointCloudFrame } from './types';

export const boundedCount = (value: number | undefined, fallback: number, maximum: number): number =>
  Number.isFinite(value) ? Math.max(1, Math.min(maximum, Math.floor(value!))) : fallback;

// Two disjoint render buffers in one scene. Never pass a foreground replacement
// through the old single-stream displayFrame(), which clears the Gaussian mesh.
export class OverlayLayers {
  readonly gaussian = new GaussianRenderer();
  readonly foregroundGeometry = new THREE.BufferGeometry();
  readonly foregroundMaterial = new THREE.PointsMaterial({color: '#ff7043', size: 3, sizeAttenuation: false});
  readonly foreground = new THREE.Points(this.foregroundGeometry, this.foregroundMaterial);
  gaussianCount = 0;
  foregroundCount = 0;
  private positions = new Float32Array(0);
  private gaussianFrame?: PointCloudFrame;
  constructor(readonly scene: THREE.Scene) {
    scene.add(this.gaussian.mesh, this.foreground);
    this.foregroundGeometry.setDrawRange(0, 0);
  }
  setGaussian(frame: PointCloudFrame, options: LidarPointCloudOptions): void {
    if (frame.protocol !== 'GSP1' || !frame.covariances || !frame.opacities) { throw new Error('Gaussian layer requires GSP1 covariance/opacity'); }
    const limit = boundedCount(options.maxPoints, 100000, 2000000);
    const step = Math.max(1, Math.ceil(frame.pointCount / limit)), count = Math.ceil(frame.pointCount / step);
    const positions = new Float32Array(count * 3), covariance = new Float32Array(count * 6);
    const intensities = new Float32Array(count), opacity = new Float32Array(count), colors = new Float32Array(count * 3);
    let low = Infinity, high = -Infinity, maxIntensity = 1;
    for (let i = 0, n = 0; i < frame.pointCount; i += step, ++n) {
      positions.set(frame.positions.subarray(i*3,i*3+3),n*3);
      covariance.set(frame.covariances.subarray(i*6,i*6+6),n*6);
      intensities[n] = frame.intensities?.[i] ?? 0; opacity[n] = frame.opacities[i];
      low = Math.min(low, positions[n*3+2]); high = Math.max(high, positions[n*3+2]);
      maxIntensity = Math.max(maxIntensity,intensities[n]);
    }
    const solid = new THREE.Color(options.solidColor || '#a6b4c4'), color = new THREE.Color();
    for (let n = 0; n < count; ++n) {
      if (options.colorMode === 'height') { color.setHSL(.66 - THREE.MathUtils.clamp((positions[n*3+2]-low)/Math.max(high-low,1e-6),0,1)*.66,1,.5); }
      else if (options.colorMode === 'intensity') { color.setHSL(.66 - THREE.MathUtils.clamp(intensities[n]/maxIntensity,0,1)*.66,1,.5); }
      else { color.copy(solid); }
      colors.set([color.r,color.g,color.b],n*3);
    }
    this.gaussian.set(positions,covariance,colors,opacity);this.gaussianCount=count;
    // Retain only the bounded displayed sample for restyling, never a full map.
    this.gaussianFrame={...frame,positions,covariances:covariance,intensities,opacities:opacity,pointCount:count};
  }
  setForeground(frame: PointCloudFrame, options: LidarPointCloudOptions): void {
    if (frame.protocol !== 'LPC1') { throw new Error('Foreground layer requires LPC1'); }
    const limit = boundedCount(options.foregroundMaxPoints,12000,50000);
    const step = Math.max(1,Math.ceil(frame.pointCount/limit)),count=Math.ceil(frame.pointCount/step);
    if(this.positions.length < count*3 || this.positions.length > limit*3) {
      const capacity=Math.min(limit,Math.max(count,1024,Math.ceil(this.positions.length/3*1.5)));
      this.foregroundGeometry.dispose();this.positions=new Float32Array(capacity*3);
      const attr=new THREE.BufferAttribute(this.positions,3);attr.setUsage(THREE.DynamicDrawUsage);
      this.foregroundGeometry.setAttribute('position',attr);
    }
    for(let i=0,n=0;i<frame.pointCount;i+=step,++n) {this.positions.set(frame.positions.subarray(i*3,i*3+3),n*3);}
    const attr=this.foregroundGeometry.getAttribute('position');if(attr){attr.needsUpdate=true;}
    this.foregroundCount=count;this.foregroundGeometry.setDrawRange(0,count);
    if(count) {this.foregroundGeometry.computeBoundingSphere();} else {this.foregroundGeometry.boundingSphere=null;}
    this.styleForeground(options);
  }
  clearGaussian(): void {
    this.gaussian.set(new Float32Array(0),new Float32Array(0),new Float32Array(0),new Float32Array(0));
    this.gaussianFrame=undefined;this.gaussianCount=0;
  }
  clearForeground(): void {this.foregroundCount=0;this.foregroundGeometry.setDrawRange(0,0);}
  foregroundBounds(): THREE.Box3 | undefined {
    return this.foregroundCount?new THREE.Box3().setFromArray(this.positions.subarray(0,this.foregroundCount*3)):undefined;
  }
  configure(options: LidarPointCloudOptions): void {
    this.styleForeground(options);
    if(this.gaussianFrame){this.setGaussian(this.gaussianFrame,options);}
    const limit=boundedCount(options.foregroundMaxPoints,12000,50000);
    if(this.foregroundCount>limit){this.clearForeground();} // Next fresh frame uses the new budget.
  }
  private styleForeground(options: LidarPointCloudOptions): void {
    this.foregroundMaterial.color.set(options.foregroundColor || '#ff7043');
    this.foregroundMaterial.size=Number.isFinite(options.foregroundPointSize)?Math.max(.1,Math.min(20,options.foregroundPointSize!)):3;
  }
  dispose(): void {
    this.scene.remove(this.gaussian.mesh,this.foreground);
    this.gaussian.dispose();this.foregroundGeometry.dispose();this.foregroundMaterial.dispose();
  }
}
