import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

import rocketModelUrl from "../../../assets/3d.stl?url";
import type { FlightStage } from "../../types";

type RocketModelViewerProps = {
  stage: FlightStage;
};

export function RocketModelViewer({ stage }: RocketModelViewerProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const stageRef = useRef(stage);

  useEffect(() => {
    stageRef.current = stage;
  }, [stage]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 100);
    camera.position.set(0.7, 0.2, 5.2);

    const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    renderer.setClearColor(0x000000, 0);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.enableZoom = false;
    controls.rotateSpeed = 0.45;
    controls.target.set(0, 0, 0);

    scene.add(new THREE.HemisphereLight(0xd9f4ff, 0x17120a, 2.7));

    const keyLight = new THREE.DirectionalLight(0xffffff, 3.6);
    keyLight.position.set(3, 5, 4);
    scene.add(keyLight);

    const rimLight = new THREE.DirectionalLight(0x52d7ff, 2.1);
    rimLight.position.set(-3, 1, -4);
    scene.add(rimLight);

    const rocketGroup = new THREE.Group();
    scene.add(rocketGroup);

    const material = new THREE.MeshStandardMaterial({
      color: 0xf4f0dc,
      metalness: 0.18,
      roughness: 0.48,
    });

    const flameMaterial = new THREE.MeshBasicMaterial({
      color: 0xffc247,
      transparent: true,
      opacity: 0,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    const flame = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.95, 32), flameMaterial);
    flame.position.y = -1.88;
    flame.rotation.x = Math.PI;
    rocketGroup.add(flame);

    let disposed = false;
    let model: THREE.Mesh | null = null;
    let frameId = 0;

    new STLLoader().load(rocketModelUrl, (geometry) => {
      if (disposed) {
        geometry.dispose();
        return;
      }

      geometry.computeVertexNormals();
      geometry.center();

      const bounds = new THREE.Box3().setFromBufferAttribute(geometry.getAttribute("position") as THREE.BufferAttribute);
      const size = new THREE.Vector3();
      bounds.getSize(size);
      const maxAxis = Math.max(size.x, size.y, size.z) || 1;
      geometry.scale(3.2 / maxAxis, 3.2 / maxAxis, 3.2 / maxAxis);
      geometry.rotateX(-Math.PI / 2);

      model = new THREE.Mesh(geometry, material);
      rocketGroup.add(model);
    });

    const setSize = () => {
      const width = Math.max(1, container.clientWidth);
      const height = Math.max(1, container.clientHeight);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height, false);
    };

    const resizeObserver = new ResizeObserver(setSize);
    resizeObserver.observe(container);
    setSize();

    const animate = () => {
      const boost = stageRef.current === "BOOST";
      const targetFlameOpacity = boost ? 0.78 : 0;
      flameMaterial.opacity += (targetFlameOpacity - flameMaterial.opacity) * 0.08;
      flame.scale.y = boost ? 1 + Math.sin(performance.now() * 0.018) * 0.12 : 0.55;

      rocketGroup.rotation.y += 0.006;
      rocketGroup.rotation.z = boost ? Math.sin(performance.now() * 0.012) * 0.025 : 0;
      controls.update();
      renderer.render(scene, camera);
      frameId = requestAnimationFrame(animate);
    };
    animate();

    return () => {
      disposed = true;
      cancelAnimationFrame(frameId);
      resizeObserver.disconnect();
      controls.dispose();
      renderer.dispose();
      material.dispose();
      flameMaterial.dispose();
      flame.geometry.dispose();
      model?.geometry.dispose();
      container.removeChild(renderer.domElement);
    };
  }, []);

  return <div ref={containerRef} className="rocket-model-viewer" aria-label="3D rocket model viewer" />;
}
