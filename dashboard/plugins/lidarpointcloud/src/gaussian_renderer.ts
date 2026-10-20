import * as THREE from 'three';

// Project the full world-space covariance into a screen-space ellipse. Quads
// carry a three-sigma Gaussian alpha, rather than displaying centre point dots.
export const gaussianVertexShader = `
attribute vec3 center;
attribute vec3 covarianceA;
attribute vec3 covarianceB;
attribute vec3 splatColor;
attribute float opacity;
uniform vec2 viewport;
varying vec2 gaussianUV;
varying vec4 gaussianColor;
void main() {
  vec4 eye = modelViewMatrix * vec4(center, 1.0);
  vec4 clip = projectionMatrix * eye;
  gaussianUV = position.xy * 3.0;
  gaussianColor = vec4(splatColor, opacity);
  if (eye.z >= -0.001 || clip.w <= 0.0) {
    gaussianColor.a = 0.0; gl_Position = vec4(2.0,2.0,2.0,1.0); return;
  }
  mat3 C = mat3(covarianceA.x,covarianceA.y,covarianceA.z,
                covarianceA.y,covarianceB.x,covarianceB.y,
                covarianceA.z,covarianceB.y,covarianceB.z);
  float depth = -eye.z;
  float fx = projectionMatrix[0][0] * viewport.x * 0.5;
  float fy = projectionMatrix[1][1] * viewport.y * 0.5;
  vec3 jx = vec3(fx/depth,0.0,fx*eye.x/(depth*depth)) * mat3(modelViewMatrix);
  vec3 jy = vec3(0.0,fy/depth,fy*eye.y/(depth*depth)) * mat3(modelViewMatrix);
  float a = dot(jx,C*jx) + 0.3;
  float b = dot(jx,C*jy);
  float c = dot(jy,C*jy) + 0.3;
  float mid = (a+c)*0.5;
  float delta = sqrt(max(0.0, (a-c)*(a-c)*0.25+b*b));
  float major = max(mid+delta,0.3), minor = max(mid-delta,0.3);
  vec2 axis = abs(b)>0.00001 ? normalize(vec2(b,major-a)) : (a>=c ? vec2(1.0,0.0) : vec2(0.0,1.0));
  vec2 offset = 3.0 * (position.x*sqrt(major)*axis + position.y*sqrt(minor)*vec2(-axis.y,axis.x));
  clip.xy += 2.0 * offset / viewport * clip.w;
  gl_Position = clip;
}`;
export const gaussianFragmentShader = `
varying vec2 gaussianUV;
varying vec4 gaussianColor;
void main() {
  float radius2=dot(gaussianUV,gaussianUV);
  if(radius2>9.0) discard;
  float alpha=gaussianColor.a*exp(-0.5*radius2);
  if(alpha<0.003) discard;
  gl_FragColor=vec4(gaussianColor.rgb,alpha);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

export class GaussianRenderer {
  readonly geometry = new THREE.InstancedBufferGeometry();
  readonly material = new THREE.ShaderMaterial({
    vertexShader: gaussianVertexShader, fragmentShader: gaussianFragmentShader,
    uniforms: { viewport: { value: new THREE.Vector2(1, 1) } },
    transparent: true, depthWrite: false, depthTest: true, side: THREE.DoubleSide,
  });
  readonly mesh = new THREE.Mesh(this.geometry, this.material);
  private centers: Float32Array = new Float32Array(0);
  private covariance: Float32Array = new Float32Array(0);
  private colors: Float32Array = new Float32Array(0);
  private opacity: Float32Array = new Float32Array(0);
  private signature = '';
  private nextSort = 0;
  constructor() {
    this.geometry.setAttribute('position', new THREE.Float32BufferAttribute([-1,-1,0, 1,-1,0, 1,1,0, -1,1,0], 3));
    this.geometry.setIndex([0,1,2,0,2,3]);
    this.geometry.instanceCount = 0;
    this.mesh.frustumCulled = false; // Centres alone do not bound projected ellipses.
  }
  set(centers: Float32Array, covariance: Float32Array, colors: Float32Array, opacity: Float32Array): void {
    this.centers=centers; this.covariance=covariance; this.colors=colors; this.opacity=opacity;
    this.geometry.dispose(); // Release replaced GPU buffers.
    for (const [name, width] of [['center',3],['covarianceA',3],['covarianceB',3],['splatColor',3],['opacity',1]] as const) {
      this.geometry.setAttribute(name, new THREE.InstancedBufferAttribute(new Float32Array(opacity.length*width), width));
    }
    this.geometry.instanceCount=opacity.length;this.signature='';this.nextSort=0;
  }
  update(camera: THREE.PerspectiveCamera, width: number, height: number, now: number): void {
    this.material.uniforms.viewport.value.set(width,height);
    if (!this.opacity.length || now<this.nextSort) { return; }
    camera.updateMatrixWorld();
    const matrix=camera.matrixWorldInverse.elements;
    const signature=matrix.map((v)=>v.toFixed(5)).join(',');
    if (signature===this.signature) { return; }
    this.signature=signature;this.nextSort=now+100;
    const depths=new Float32Array(this.opacity.length);
    const order=new Uint32Array(this.opacity.length);
    for(let i=0;i<order.length;++i) {
      order[i]=i;depths[i]=matrix[2]*this.centers[3*i]+matrix[6]*this.centers[3*i+1]+matrix[10]*this.centers[3*i+2]+matrix[14];
    }
    order.sort((a,b)=>depths[a]-depths[b]); // Back-to-front alpha blending.
    const attrs=['center','covarianceA','covarianceB','splatColor','opacity'].map((name)=>this.geometry.getAttribute(name) as THREE.InstancedBufferAttribute);
    for(let i=0;i<order.length;++i) {
      const source=order[i];
      attrs[0].setXYZ(i,this.centers[source*3],this.centers[source*3+1],this.centers[source*3+2]);
      attrs[1].setXYZ(i,this.covariance[source*6],this.covariance[source*6+1],this.covariance[source*6+2]);
      attrs[2].setXYZ(i,this.covariance[source*6+3],this.covariance[source*6+4],this.covariance[source*6+5]);
      attrs[3].setXYZ(i,this.colors[source*3],this.colors[source*3+1],this.colors[source*3+2]);
      attrs[4].setX(i,this.opacity[source]);
    }
    attrs.forEach((attr)=>{attr.needsUpdate=true;});
  }
  dispose(): void { this.geometry.dispose();this.material.dispose(); }
}
