import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls';

export class BodyRenderer {
  readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(40, 1, .01, 100);
  private readonly geometry = new THREE.BufferGeometry();
  private readonly material = new THREE.MeshBasicMaterial({color: '#55d6ff', wireframe: true, side: THREE.DoubleSide});
  private readonly mesh = new THREE.Mesh(this.geometry, this.material);
  private readonly grid = new THREE.GridHelper(6, 24, '#284969', '#162737');
  private readonly controls: OrbitControls;
  private animation = 0;
  private fitted = false;
  constructor(private readonly host: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({antialias: true});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor('#080f1a');
    host.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.mesh.visible = false;
    this.scene.add(this.mesh, this.grid);
    this.reset();
    const render = () => {
      this.animation = requestAnimationFrame(render);
      this.controls.update(); this.renderer.render(this.scene, this.camera);
    };
    render();
  }
  resize(width: number, height: number): void {
    this.renderer.setSize(Math.max(1, width), Math.max(1, height));
    this.camera.aspect = Math.max(1, width) / Math.max(1, height); this.camera.updateProjectionMatrix();
  }
  reset(): void {
    this.camera.position.set(0, .4, 3.8); this.controls.target.set(0, 0, 0); this.controls.update();
    this.fitted = false;
  }
  setTopology(faces: Uint32Array): void { this.clear(); this.geometry.setIndex(new THREE.BufferAttribute(faces, 1)); }
  setMesh(positions: Float32Array, wireframe: boolean): void {
    let attribute = this.geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    if (!attribute || attribute.array.length !== positions.length) {
      attribute = new THREE.BufferAttribute(positions, 3); attribute.setUsage(THREE.DynamicDrawUsage);
      this.geometry.setAttribute('position', attribute);
    } else { (attribute.array as Float32Array).set(positions); attribute.needsUpdate = true; }
    this.material.wireframe = wireframe;
    this.geometry.computeBoundingBox(); this.geometry.computeBoundingSphere();
    const box = this.geometry.boundingBox!;
    this.grid.position.y = box.min.y - .02;
    if (!this.fitted) {
      const centre = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3());
      const distance = Math.max(size.x, size.y, size.z, .5) / (2 * Math.tan(THREE.MathUtils.degToRad(20))) * 1.3;
      this.controls.target.copy(centre); this.camera.position.copy(centre).add(new THREE.Vector3(0, .1, distance));
      this.controls.update(); this.fitted = true;
    }
    this.mesh.visible = true;
  }
  clear(): void { this.mesh.visible = false; }
  dispose(): void {
    cancelAnimationFrame(this.animation); this.controls.dispose();
    this.geometry.dispose(); this.material.dispose(); this.grid.geometry.dispose();
    (this.grid.material as THREE.Material).dispose();
    this.renderer.dispose(); this.renderer.domElement.remove();
  }
}
