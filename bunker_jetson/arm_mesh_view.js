/* URDF joint tree for fafu_follower. Meshes are the link STLs; cylinders if they 404. */
(function (global) {
  "use strict";

  var JOINTS = [
    { xyz: [0, 0, 0.0584], axis: [0, 0, 1], mesh: "link1.STL", r: 0.028, color: 0x8b9aab },
    { xyz: [0.018199, 0, 0.053], axis: [0, 1, 0], mesh: "link2.STL", r: 0.022, color: 0x5ec8ff },
    { xyz: [-0.26, 0, 0], axis: [0, -1, 0], mesh: "link3.STL", r: 0.020, color: 0x9aa8b8 },
    { xyz: [0.23, 0, 0.06], axis: [0, -1, 0], mesh: "link4.STL", r: 0.018, color: 0x5ec8ff },
    { xyz: [0.07, 0, 0.036319], axis: [0, 0, -1], mesh: "link5.STL", r: 0.014, color: 0x9aa8b8 },
    { xyz: [0.02345, 0, -0.039], axis: [1, 0, 0], mesh: "link6.STL", r: 0.012, color: 0x5ec8ff },
  ];
  var BASE = "base_link.STL";
  var TOOL = [0.165, 0, 0];
  var GRIP_LO = 8;
  var GRIP_HI = 75;

  var host = null;
  var overlay = null;
  var renderer = null;
  var scene = null;
  var camera = null;
  var robot = null;
  var nodes = [];
  var jawL = null;
  var jawR = null;
  var raf = 0;
  var ready = false;
  var active = false;
  var started = false;
  var az = 0.9;
  var el = 0.35;
  var dist = 1.15;
  var dragging = false;
  var lastX = 0;
  var lastY = 0;
  var lastQ = [0, 0, 0, 0, 0, 0];
  var lastGrip = GRIP_HI;

  function $(id) {
    return document.getElementById(id);
  }

  function setOverlay(text, hide) {
    overlay = overlay || $("arm3dOverlay");
    if (!overlay) return;
    overlay.textContent = text || "";
    overlay.classList.toggle("gone", !!hide);
  }

  function parseSTL(buffer) {
    var view = new DataView(buffer);
    if (buffer.byteLength < 84) throw new Error("stl short");
    var n = view.getUint32(80, true);
    if (n <= 0 || n > 2000000 || 84 + n * 50 > buffer.byteLength) {
      throw new Error("stl header");
    }
    var pos = new Float32Array(n * 9);
    var off = 84;
    var i;
    var k = 0;
    for (i = 0; i < n; i += 1) {
      off += 12;
      var a = 0;
      for (a = 0; a < 9; a += 1) {
        pos[k] = view.getFloat32(off, true);
        k += 1;
        off += 4;
      }
      off += 2;
    }
    var geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    geo.computeVertexNormals();
    return geo;
  }

  function meshMaterial(color) {
    return new THREE.MeshLambertMaterial({
      color: color,
      side: THREE.DoubleSide,
    });
  }

  function addMesh(parent, geo, color) {
    var mesh = new THREE.Mesh(geo, meshMaterial(color));
    parent.add(mesh);
    return mesh;
  }

  function bone(parent, to, radius, color) {
    var dir = new THREE.Vector3(to[0], to[1], to[2]);
    var len = dir.length();
    if (len < 1e-5) return;
    var geo = new THREE.CylinderGeometry(radius * 0.88, radius, len, 14);
    var mesh = new THREE.Mesh(geo, meshMaterial(color));
    mesh.position.copy(dir).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    parent.add(mesh);
  }

  function addJaws(parent) {
    var jawMat = meshMaterial(0xf0c14b);
    var jawGeo = new THREE.BoxGeometry(0.034, 0.008, 0.016);
    jawL = new THREE.Mesh(jawGeo, jawMat);
    jawR = new THREE.Mesh(jawGeo, jawMat);
    jawL.position.set(0.02, 0.02, 0);
    jawR.position.set(0.02, -0.02, 0);
    parent.add(jawL);
    parent.add(jawR);
  }

  function finishRobot() {
    scene.add(robot);
    applyJoints(lastQ, lastGrip);
    placeCamera();
    ready = true;
    resize();
    setActive(active);
  }

  function buildCylinders(note) {
    robot = new THREE.Group();
    robot.rotation.x = -Math.PI / 2;
    var base = new THREE.Mesh(
      new THREE.CylinderGeometry(0.055, 0.062, 0.036, 24),
      meshMaterial(0x3a4654)
    );
    base.rotation.x = Math.PI / 2;
    base.position.z = 0.018;
    robot.add(base);
    var parent = robot;
    nodes = [];
    var i;
    for (i = 0; i < JOINTS.length; i += 1) {
      var spec = JOINTS[i];
      bone(parent, spec.xyz, spec.r, spec.color);
      var node = new THREE.Group();
      node.position.set(spec.xyz[0], spec.xyz[1], spec.xyz[2]);
      parent.add(node);
      var hub = new THREE.Mesh(
        new THREE.SphereGeometry(spec.r * 1.15, 16, 12),
        meshMaterial(i % 2 ? 0xd7f3ff : 0xc5d0db)
      );
      node.add(hub);
      nodes.push(node);
      parent = node;
    }
    bone(parent, TOOL, 0.011, 0x5ec8ff);
    var tool = new THREE.Group();
    tool.position.set(TOOL[0], TOOL[1], TOOL[2]);
    parent.add(tool);
    tool.add(new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12), meshMaterial(0xff4d5a)));
    addJaws(tool);
    finishRobot();
    setOverlay(note || "未找到 STL，当前是关节骨架", false);
  }

  function buildFromGeometries(geos) {
    robot = new THREE.Group();
    robot.rotation.x = -Math.PI / 2;
    addMesh(robot, geos[0], 0xb7c0ca);
    var parent = robot;
    nodes = [];
    var i;
    for (i = 0; i < JOINTS.length; i += 1) {
      var spec = JOINTS[i];
      var node = new THREE.Group();
      node.position.set(spec.xyz[0], spec.xyz[1], spec.xyz[2]);
      parent.add(node);
      addMesh(node, geos[i + 1], 0xc5cdd6);
      nodes.push(node);
      parent = node;
    }
    var tool = new THREE.Group();
    tool.position.set(TOOL[0], TOOL[1], TOOL[2]);
    parent.add(tool);
    addJaws(tool);
    finishRobot();
    setOverlay("", true);
  }

  function loadMeshes() {
    var names = [BASE].concat(JOINTS.map(function (j) { return j.mesh; }));
    var done = 0;
    setOverlay("正在加载模型 0/" + names.length, false);
    return Promise.all(names.map(function (name) {
      return fetch("/arm-static/meshes/" + name, { credentials: "same-origin" }).then(function (res) {
        if (!res.ok) throw new Error(name);
        return res.arrayBuffer();
      }).then(function (buf) {
        done += 1;
        setOverlay("正在加载模型 " + done + "/" + names.length, false);
        return parseSTL(buf);
      });
    }));
  }

  function applyJoints(qDeg, gripDeg) {
    var i;
    for (i = 0; i < nodes.length; i += 1) {
      var q = ((Number(qDeg[i]) || 0) * Math.PI) / 180;
      var ax = JOINTS[i].axis;
      nodes[i].quaternion.setFromAxisAngle(
        new THREE.Vector3(ax[0], ax[1], ax[2]).normalize(),
        q
      );
    }
    var t = (Number(gripDeg) - GRIP_LO) / (GRIP_HI - GRIP_LO);
    if (!(t >= 0)) t = 1;
    t = Math.max(0, Math.min(1, t));
    var spread = 0.008 + t * 0.026;
    if (jawL) jawL.position.y = spread;
    if (jawR) jawR.position.y = -spread;
  }

  function lookAtTarget() {
    return new THREE.Vector3(0, 0.28, 0.05);
  }

  function placeCamera() {
    if (!camera) return;
    var e = Math.max(-1.2, Math.min(1.35, el));
    var target = lookAtTarget();
    camera.position.set(
      target.x + dist * Math.cos(e) * Math.sin(az),
      target.y + dist * Math.sin(e),
      target.z + dist * Math.cos(e) * Math.cos(az)
    );
    camera.lookAt(target);
  }

  function resize() {
    if (!host || !renderer || !camera) return;
    var w = host.clientWidth;
    var h = host.clientHeight;
    if (w < 8 || h < 8) return;
    renderer.setPixelRatio(Math.min(global.devicePixelRatio || 1, 2));
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  function tick() {
    raf = 0;
    if (!ready || !active) return;
    if (renderer && scene && camera && host && host.clientWidth > 8) {
      renderer.render(scene, camera);
    }
    raf = requestAnimationFrame(tick);
  }

  function setActive(on) {
    active = !!on;
    if (active && ready && !raf) raf = requestAnimationFrame(tick);
  }

  function onPointerDown(ev) {
    if (ev.button !== 0) return;
    dragging = true;
    lastX = ev.clientX;
    lastY = ev.clientY;
    try { host.setPointerCapture(ev.pointerId); } catch (e) {}
  }
  function onPointerMove(ev) {
    if (!dragging) return;
    az -= (ev.clientX - lastX) * 0.008;
    el += (ev.clientY - lastY) * 0.008;
    lastX = ev.clientX;
    lastY = ev.clientY;
    placeCamera();
  }
  function onPointerUp(ev) {
    dragging = false;
    try { host.releasePointerCapture(ev.pointerId); } catch (e) {}
  }
  function onWheel(ev) {
    ev.preventDefault();
    dist = Math.max(0.35, Math.min(2.8, dist * (ev.deltaY > 0 ? 1.08 : 0.92)));
    placeCamera();
  }

  function bootScene() {
    host = $("arm3dHost");
    overlay = $("arm3dOverlay");
    if (!host || typeof THREE === "undefined") {
      setOverlay("未加载 Three.js", false);
      return;
    }
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070b10);
    camera = new THREE.PerspectiveCamera(42, 1, 0.01, 30);
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (err) {
      setOverlay("本机无法创建 WebGL", false);
      return;
    }
    renderer.setClearColor(0x070b10, 1);
    renderer.domElement.style.display = "block";
    host.appendChild(renderer.domElement);
    scene.add(new THREE.AmbientLight(0xffffff, 0.62));
    var key = new THREE.DirectionalLight(0xffffff, 0.9);
    key.position.set(0.8, 1.4, 0.6);
    scene.add(key);
    var fill = new THREE.DirectionalLight(0x8eb4d4, 0.35);
    fill.position.set(-1.0, 0.3, -0.4);
    scene.add(fill);
    var grid = new THREE.GridHelper(1.4, 14, 0x334556, 0x1b2733);
    scene.add(grid);
    host.addEventListener("pointerdown", onPointerDown);
    host.addEventListener("pointermove", onPointerMove);
    host.addEventListener("pointerup", onPointerUp);
    host.addEventListener("pointercancel", onPointerUp);
    host.addEventListener("wheel", onWheel, { passive: false });
    global.addEventListener("resize", resize);
    if (typeof ResizeObserver !== "undefined") new ResizeObserver(resize).observe(host);
    loadMeshes().then(buildFromGeometries).catch(function () {
      buildCylinders("未找到 STL，当前是关节骨架");
    });
  }

  function show() {
    setActive(true);
    if (!started) {
      started = true;
      bootScene();
    }
    resize();
  }

  function hide() {
    setActive(false);
  }

  function setPose(qDeg, gripDeg) {
    if (qDeg && qDeg.length) lastQ = qDeg;
    if (gripDeg != null && !isNaN(Number(gripDeg))) lastGrip = Number(gripDeg);
    if (ready) applyJoints(lastQ, lastGrip);
  }

  global.ArmMesh = {
    show: show,
    hide: hide,
    setPose: setPose,
    resize: resize,
  };
})(window);
