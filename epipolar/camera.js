// Pinhole camera with OpenCV conventions: pure maths, no DOM and no three.js, so it runs under node too.
// A classic script (not an ES module) so the page also works opened straight from disk (file://): in the browser
// it defines the global Cam, under node it is require('./camera.js').
//
// World: right-handed, Z up, metres. Camera: x right, y down, z forward (the optical axis).
//   X_c = R X_w + t,  t = -R C           (R: world -> camera, C: camera centre in the world)
//   x = X_c / Z_c,  y = Y_c / Z_c        (normalised image coordinates, the image plane at Z_c = 1)
//   r² = x² + y²
//   x_d = x (1 + k1 r² + k2 r⁴ + k3 r⁶) + 2 p1 x y + p2 (r² + 2 x²)
//   y_d = y (1 + k1 r² + k2 r⁴ + k3 r⁶) + p1 (r² + 2 y²) + 2 p2 x y
//   [u v 1]ᵀ = K [x_d y_d 1]ᵀ,  K = [[fx, s, cx], [0, fy, cy], [0, 0, 1]]
// Pixel coordinates run from (0, 0) at the top-left corner of the image to (W, H) at the bottom-right one.

(function (root) {
  'use strict';
  // ---------------------------------------------------------------------------- small linear algebra
  const mul = (A, B) => A.map(row => B[0].map((_, j) => row.reduce((s, a, k) => s + a * B[k][j], 0)));
  const mulv = (A, v) => A.map(row => row.reduce((s, a, k) => s + a * v[k], 0));
  const transpose = A => A[0].map((_, j) => A.map(row => row[j]));
  const sub = (a, b) => a.map((x, i) => x - b[i]);
  const add = (a, b) => a.map((x, i) => x + b[i]);
  const scale = (a, k) => a.map(x => x * k);
  const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
  const norm = a => Math.sqrt(dot(a, a));
  const angleBetween = (a, b) => Math.acos(Math.max(-1, Math.min(1, dot(a, b) / (norm(a) * norm(b)))));
  const DEG = Math.PI / 180;

  // ---------------------------------------------------------------------------- pose
  const rotX = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
  const rotZ = a => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
  // camera axes in the world at rest: x_c = +X, y_c = -Z (down), z_c = +Y (the camera looks along +Y)
  const REST = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];

  // R (world -> camera) from yaw (about the world Z axis, + turns left), pitch (+ looks up) and roll (about the
  // optical axis), all in degrees
  function rotationFromYawPitchRoll(yaw, pitch, roll) {
    const camToWorld = mul(mul(mul(rotZ(yaw * DEG), rotX(pitch * DEG)), REST), rotZ(roll * DEG));
    return transpose(camToWorld);
  }

  // yaw and pitch (degrees) that point a camera at C towards the target
  function lookAt(C, target) {
    const [dx, dy, dz] = sub(target, C);
    return { yaw: Math.atan2(-dx, dy) / DEG, pitch: Math.atan2(dz, Math.hypot(dx, dy)) / DEG };
  }

  const translationFromCentre = (R, C) => scale(mulv(R, C), -1);

  // ---------------------------------------------------------------------------- intrinsics
  // fx from the focal length and the sensor width: fx = f W / sensor width (pixels per mm times mm)
  function intrinsics({ fx, fy, cx, cy, skew = 0 }) {
    return [[fx, skew, cx], [0, fy, cy], [0, 0, 1]];
  }
  const fxFromFocal = (f_mm, sensorW_mm, W) => f_mm * W / sensorW_mm;

  function projectionMatrix(K, R, t) {
    return mul(K, R.map((row, i) => [...row, t[i]]));
  }

  // pixel -> normalised coordinates on the (distorted) image plane: K⁻¹ [u v 1]ᵀ
  function pixelToNormalised([u, v], K) {
    const [[fx, s, cx], [, fy, cy]] = K;
    const y = (v - cy) / fy;
    return [(u - cx - s * y) / fx, y];
  }
  function normalisedToPixel([x, y], K) {
    const [[fx, s, cx], [, fy, cy]] = K;
    return [fx * x + s * y + cx, fy * y + cy];
  }

  // ---------------------------------------------------------------------------- distortion
  const NO_DISTORTION = { k1: 0, k2: 0, k3: 0, p1: 0, p2: 0 };
  const hasDistortion = d => !!(d && (d.k1 || d.k2 || d.k3 || d.p1 || d.p2));

  function radialFactor(r2, d) {
    return 1 + r2 * (d.k1 + r2 * (d.k2 + r2 * d.k3));
  }
  function distort([x, y], d) {
    const r2 = x * x + y * y, k = radialFactor(r2, d);
    return [x * k + 2 * d.p1 * x * y + d.p2 * (r2 + 2 * x * x),
            y * k + d.p1 * (r2 + 2 * y * y) + 2 * d.p2 * x * y];
  }

  // the fixed-point iteration cv2.undistortPoints uses; NaN where it does not converge (outside the model's reach)
  function undistort([xd, yd], d, iterations = 200) {
    let x = xd, y = yd;
    for (let i = 0; i < iterations; i++) {
      const r2 = x * x + y * y, k = radialFactor(r2, d);
      const dx = 2 * d.p1 * x * y + d.p2 * (r2 + 2 * x * x), dy = d.p1 * (r2 + 2 * y * y) + 2 * d.p2 * x * y;
      const nx = (xd - dx) / k, ny = (yd - dy) / k;
      if (!isFinite(nx) || !isFinite(ny)) return [NaN, NaN];
      const done = Math.abs(nx - x) + Math.abs(ny - y) < 1e-15;
      x = nx; y = ny;
      if (done) break;
    }
    const [cx, cy] = distort([x, y], d);
    return Math.abs(cx - xd) + Math.abs(cy - yd) < 1e-9 ? [x, y] : [NaN, NaN];
  }

  // The radial polynomial r (1 + k1 r² + k2 r⁴ + k3 r⁶) stops growing at some radius when the coefficients turn it
  // down (strong barrel). Beyond it the model folds the scene back into the picture, which no lens does, so
  // projections are cut there. Returns the smallest such r, or Infinity.
  function maxRadius(d) {
    const slope = q => 1 + 3 * d.k1 * q + 5 * d.k2 * q * q + 7 * d.k3 * q * q * q;   // d(r·factor)/dr with q = r²
    if (!(d.k1 || d.k2 || d.k3)) return Infinity;
    let lo = 0, hi = 1e-4;
    while (hi < 1e6 && slope(hi) > 0) { lo = hi; hi *= 1.05; }
    if (hi >= 1e6) return Infinity;
    for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; if (slope(m) > 0) lo = m; else hi = m; }
    return Math.sqrt(lo);
  }

  // ---------------------------------------------------------------------------- the camera and projection
  // cam = { K, R, t, dist, near }; makeCamera fills in the derived values. view = [W, H], when given, limits the
  // distorted curves to a generous area around the image (see pathToPolylines).
  function makeCamera({ K, R, t, dist = NO_DISTORTION, near = 0.05, view = null }) {
    return { K, R, t, dist, near, rMax: maxRadius(dist), P: projectionMatrix(K, R, t), view };
  }

  // every stage of the pipeline for one world point
  function projectPoint(Xw, cam) {
    const Xc = add(mulv(cam.R, Xw), cam.t);
    const inFront = Xc[2] > 0;
    const xn = [Xc[0] / Xc[2], Xc[1] / Xc[2]];
    const r2 = xn[0] * xn[0] + xn[1] * xn[1];
    const radial = radialFactor(r2, cam.dist);
    const xd = distort(xn, cam.dist);
    const uv = normalisedToPixel(xd, cam.K);
    const ideal = normalisedToPixel(xn, cam.K);   // where a distortion-free lens would put it
    return { Xw, Xc, inFront, xn, r: Math.sqrt(r2), radial, xd, uv, ideal, inModel: Math.sqrt(r2) <= cam.rMax };
  }

  // camera-frame points (consecutive along a path) -> pixel polylines, broken where the path goes behind the
  // camera or beyond the distortion model's reach
  function pathToPolylines(camPoints, cam, useDist) {
    // with zero coefficients take the straight path: a segment is then only its two endpoints, and the cut of far-off
    // samples below would drop it whole
    useDist = useDist && hasDistortion(cam.dist);
    const lines = [];
    let cur = null;
    for (const p of camPoints) {
      let uv = null;
      if (p[2] > 1e-3 * norm(p)) {   // within ~89.9° of the axis: further out the pixel coordinates explode
        const xn = [p[0] / p[2], p[1] / p[2]];
        if (!useDist) uv = normalisedToPixel(xn, cam.K);
        else if (Math.hypot(xn[0], xn[1]) <= cam.rMax) {
          uv = normalisedToPixel(distort(xn, cam.dist), cam.K);
          // without a turning point (pincushion, tangential) the polynomial flings points beside the camera
          // millions of pixels away, and a canvas takes ages to stroke such curves: drop samples far off the image
          const v = cam.view;
          if (v && (uv[0] < -v[0] || uv[0] > 2 * v[0] || uv[1] < -v[1] || uv[1] > 2 * v[1])) uv = null;
        }
      }
      if (uv) { if (!cur) lines.push(cur = []); cur.push(uv); }
      else cur = null;
    }
    return lines.filter(l => l.length > 1);
  }

  // clip the camera-frame segment a-b to Z_c >= near; null when it is wholly behind
  function clipNear(a, b, near) {
    const ia = a[2] >= near, ib = b[2] >= near;
    if (ia && ib) return [a, b];
    if (!ia && !ib) return null;
    const s = (near - a[2]) / (b[2] - a[2]), m = add(a, scale(sub(b, a), s));
    return ia ? [a, m] : [m, b];
  }

  // a straight world segment -> pixel polylines. Without distortion its image is straight too; with it, the segment
  // is sampled finely enough (by the angle it spans from the camera) that the curve looks smooth.
  function projectSegment(A, B, cam, useDist = true) {
    const c = clipNear(add(mulv(cam.R, A), cam.t), add(mulv(cam.R, B), cam.t), cam.near);
    if (!c) return [];
    const [a, b] = c;
    const curved = useDist && hasDistortion(cam.dist);
    const n = curved ? Math.max(1, Math.min(240, Math.ceil(angleBetween(a, b) / 0.008))) : 1;
    const pts = [];
    for (let i = 0; i <= n; i++) pts.push(add(a, scale(sub(b, a), i / n)));
    return pathToPolylines(pts, cam, useDist);
  }

  // a closed polygon -> one closed pixel outline, or null when any part of it cannot be drawn (behind the camera or
  // beyond the distortion model) – used for filled faces, which are skipped rather than drawn torn
  function projectPolygon(verts, cam, useDist = true) {
    const out = [];
    for (let i = 0; i < verts.length; i++) {
      const a = add(mulv(cam.R, verts[i]), cam.t), b = add(mulv(cam.R, verts[(i + 1) % verts.length]), cam.t);
      if (a[2] < cam.near || b[2] < cam.near) return null;
      const lines = projectSegment(verts[i], verts[(i + 1) % verts.length], cam, useDist);
      if (lines.length !== 1) return null;
      out.push(...lines[0].slice(0, -1));
    }
    return out;
  }

  // directions (points at infinity) along a path, e.g. the horizon: the image of the plane at infinity
  function projectDirections(dirs, cam, useDist = true) {
    return pathToPolylines(dirs.map(d => mulv(cam.R, d)), cam, useDist);
  }

  // angle (degrees) between the rays through two pixels; with the distortion, the rays are the undistorted ones
  function rayAngle(p1, p2, K, dist) {
    const ray = p => {
      let n = pixelToNormalised(p, K);
      if (hasDistortion(dist)) n = undistort(n, dist);
      return [n[0], n[1], 1];
    };
    const a = ray(p1), b = ray(p2);
    return isFinite(a[0]) && isFinite(b[0]) ? angleBetween(a, b) / DEG : NaN;
  }

  // horizontal, vertical and diagonal field of view (degrees), measured through the principal point
  function fieldOfView(K, W, H, dist = NO_DISTORTION) {
    const [cx, cy] = [K[0][2], K[1][2]];
    return {
      h: rayAngle([0, cy], [W, cy], K, dist),
      v: rayAngle([cx, 0], [cx, H], K, dist),
      d: rayAngle([0, 0], [W, H], K, dist),
    };
  }

  // ---------------------------------------------------------------------------- scene primitives
  // Each returns { vertices, edges, faces? } in the object's own frame: origin at the centre of its base on the
  // ground, Z up. faces: [[vertex indices], shade] (shade 0 dark, 1 light), used by the checkerboard.
  function cube(size = 1) {
    const h = size / 2, vertices = [];
    for (const z of [0, size]) for (const [x, y] of [[-h, -h], [h, -h], [h, h], [-h, h]]) vertices.push([x, y, z]);
    const edges = [];
    for (let i = 0; i < 4; i++) edges.push([i, (i + 1) % 4], [i + 4, (i + 1) % 4 + 4], [i, i + 4]);
    return { vertices, edges };
  }

  function pyramid(base = 1.2, height = 1.5) {
    const h = base / 2;
    const vertices = [[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0], [0, 0, height]];
    return { vertices, edges: [[0, 1], [1, 2], [2, 3], [3, 0], [0, 4], [1, 4], [2, 4], [3, 4]] };
  }

  function sphere(radius = 0.6, rings = 7, segments = 12) {
    const vertices = [[0, 0, 0]], edges = [];   // bottom pole, then the rings from the bottom up, then the top pole
    for (let i = 1; i < rings; i++) {
      const phi = Math.PI * i / rings, z = radius * (1 - Math.cos(phi)), rr = radius * Math.sin(phi);
      for (let j = 0; j < segments; j++) {
        const a = 2 * Math.PI * j / segments;
        vertices.push([rr * Math.cos(a), rr * Math.sin(a), z]);
      }
    }
    vertices.push([0, 0, 2 * radius]);
    const top = vertices.length - 1, idx = (i, j) => 1 + (i - 1) * segments + (j % segments);
    for (let i = 1; i < rings; i++) for (let j = 0; j < segments; j++) {
      edges.push([idx(i, j), idx(i, j + 1)]);
      edges.push(i === 1 ? [0, idx(i, j)] : [idx(i - 1, j), idx(i, j)]);
      if (i === rings - 1) edges.push([idx(i, j), top]);
    }
    return { vertices, edges };
  }

  function cylinder(radius = 0.4, height = 1.6, segments = 16) {
    const vertices = [], edges = [];
    for (const z of [0, height]) for (let j = 0; j < segments; j++) {
      const a = 2 * Math.PI * j / segments;
      vertices.push([radius * Math.cos(a), radius * Math.sin(a), z]);
    }
    for (let j = 0; j < segments; j++) {
      edges.push([j, (j + 1) % segments], [segments + j, segments + (j + 1) % segments]);
      if (j % 2 === 0) edges.push([j, segments + j]);
    }
    return { vertices, edges };
  }

  // a calibration target lying on the ground: nx × ny squares of the given size, centred on the origin
  function checkerboard(nx = 7, ny = 5, size = 0.25) {
    const vertices = [], edges = [], faces = [];
    const id = (i, j) => j * (nx + 1) + i;
    for (let j = 0; j <= ny; j++) for (let i = 0; i <= nx; i++) vertices.push([(i - nx / 2) * size, (j - ny / 2) * size, 0]);
    for (let j = 0; j <= ny; j++) edges.push([id(0, j), id(nx, j)]);
    for (let i = 0; i <= nx; i++) edges.push([id(i, 0), id(i, ny)]);
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++)
      faces.push([[id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)], (i + j) % 2]);
    return { vertices, edges, faces };
  }

  // the world axes: origin and the tips of X, Y, Z (edges 0-1, 0-2, 0-3)
  function axes(length = 1) {
    return { vertices: [[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]], edges: [[0, 1], [0, 2], [0, 3]] };
  }

  function groundGrid(extent = 10, step = 1) {
    const vertices = [], edges = [];
    for (let v = -extent; v <= extent + 1e-9; v += step) {
      vertices.push([v, -extent, 0], [v, extent, 0]); edges.push([vertices.length - 2, vertices.length - 1]);
      vertices.push([-extent, v, 0], [extent, v, 0]); edges.push([vertices.length - 2, vertices.length - 1]);
    }
    return { vertices, edges };
  }

  // object frame -> world: turn by yaw (degrees) about Z, then move to the position
  function placeVertices(vertices, [x, y, z], yaw = 0) {
    const c = Math.cos(yaw * DEG), s = Math.sin(yaw * DEG);
    return vertices.map(([a, b, h]) => [x + c * a - s * b, y + s * a + c * b, z + h]);
  }

  const Cam = { mul, mulv, transpose, sub, add, scale, dot, norm, angleBetween, rotationFromYawPitchRoll, lookAt, translationFromCentre, intrinsics, fxFromFocal, projectionMatrix, pixelToNormalised, normalisedToPixel, NO_DISTORTION, hasDistortion, radialFactor, distort, undistort, maxRadius, makeCamera, projectPoint, projectSegment, projectPolygon, projectDirections, rayAngle, fieldOfView, cube, pyramid, sphere, cylinder, checkerboard, axes, groundGrid, placeVertices };
  root.Cam = Cam;
  if (typeof module !== 'undefined' && module.exports) module.exports = Cam;
})(typeof globalThis !== 'undefined' ? globalThis : this);
