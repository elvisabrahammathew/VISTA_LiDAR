import React from 'react';
import * as THREE from 'three';
import { LidarPoseDisplay } from '../lidar_pose';

const color = '#f472d0';

// The overlay shares the map's camera and stays legible at any zoom.
export class LidarMarker {
  visible = true;
  readonly display = new LidarPoseDisplay();
  private readonly world = new THREE.Vector3();
  private readonly center = new THREE.Vector3();
  private readonly heading = new THREE.Vector3();
  private readonly orientation = new THREE.Quaternion();
  private readonly size = new THREE.Vector2();

  draw(element: HTMLDivElement | null, camera: THREE.PerspectiveCamera, renderer: THREE.WebGLRenderer) {
    if (!element) { return; }
    const pose = this.display.pose;
    if (!this.visible || !pose) {
      element.hidden = true;
      return;
    }

    this.world.fromArray(pose.position);
    this.center.copy(this.world).project(camera);
    const { x, y, z } = this.center;
    if (!(Math.abs(x) <= 1 && Math.abs(y) <= 1 && Math.abs(z) <= 1)) {
      element.hidden = true;
      return;
    }

    renderer.getSize(this.size);
    const px = (x + 1) * this.size.x * 0.5;
    const py = (1 - y) * this.size.y * 0.5;
    element.style.transform = `translate3d(${px}px, ${py}px, 0)`;
    element.hidden = false;

    const children = element.children as HTMLCollectionOf<HTMLElement>;
    const ring = children[0];
    const arrow = children[1];
    const caption = children[2];
    const lastKnown = pose.state === 'last_known';
    const label = lastKnown ? 'Last known pose' : pose.state === 'fixed_mount' ? 'LiDAR · Mount' : 'LiDAR';
    if (caption.textContent !== label) {
      caption.textContent = label;
      element.style.opacity = lastKnown ? '0.80' : '1';
      ring.style.borderStyle = lastKnown ? 'dashed' : 'solid';
    }

    this.orientation.fromArray(pose.orientation).normalize();
    const headingLength = Math.min(1, this.world.distanceTo(camera.position) * 0.02);
    this.heading.set(headingLength, 0, 0).applyQuaternion(this.orientation).add(this.world).project(camera);
    const dx = (this.heading.x - x) * this.size.x;
    const dy = (y - this.heading.y) * this.size.y;
    arrow.hidden = !(Math.hypot(dx, dy) > 0.25 && Math.abs(this.heading.z) <= 1);
    if (!arrow.hidden) {
      arrow.style.transform = `rotate(${Math.atan2(dy, dx)}rad)`;
    }
  }
}

export const LidarPoseMarker = ({ elementRef }: { elementRef: React.Ref<HTMLDivElement> }) => (
  <div ref={elementRef} hidden data-lidar-pose-marker style={{
    position: 'absolute', zIndex: 1, top: 0, left: 0, pointerEvents: 'none', color,
  }}>
    <span style={{
      position: 'absolute', left: -9, top: -9, boxSizing: 'border-box', width: 18, height: 18,
      border: `2px solid ${color}`, borderRadius: '50%', background: 'rgba(73, 12, 56, .85)',
      boxShadow: '0 0 0 1px #0b0f14',
    }} />
    <span style={{ position: 'absolute', top: -3, left: 0, width: 22, height: 6, transformOrigin: '0 50%' }}>
      <span style={{
        position: 'absolute', left: 9, top: 0, width: 13, height: 6, background: color,
        clipPath: 'polygon(0 33%, 65% 33%, 65% 0, 100% 50%, 65% 100%, 65% 67%, 0 67%)',
      }} />
    </span>
    <span style={{
      position: 'absolute', top: 23, left: 0, transform: 'translateX(-50%)', whiteSpace: 'nowrap',
      padding: '2px 5px', borderRadius: 3, background: 'rgba(10, 14, 20, .92)', fontSize: 11, fontWeight: 600,
    }} />
  </div>
);
