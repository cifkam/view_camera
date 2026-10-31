// Two-view (epipolar) geometry of two pinhole cameras: pure maths, no DOM, so it runs under node too. A classic script
// like camera.js: in the browser it defines the global Epi, under node it is require('./epipolar.js').
//
// Each camera maps the world by X_c = R X_w + t (t = −R C) and pixels by x = K X_c / Z_c (camera.js's conventions).
//   relative pose        X_2 = R X_1 + t,   R = R_2 R_1ᵀ,   t = t_2 − R t_1
//   essential matrix     E = [t]× R           x̂_2ᵀ E x̂_1 = 0 for normalised coordinates x̂ = K⁻¹ x
//   fundamental matrix   F = K_2⁻ᵀ E K_1⁻¹    x_2ᵀ F x_1 = 0 for pixels (homogeneous)
//   epipolar lines       l_2 = F x_1 in image 2, l_1 = Fᵀ x_2 in image 1   (a u + b v + c = 0)
//   epipoles             e_1 = the image of C_2 in camera 1 (F e_1 = 0), e_2 = the image of C_1 in camera 2 (Fᵀ e_2 = 0)

(function (root) {
  'use strict';
  const mul = (A, B) => A.map(row => B[0].map((_, j) => row.reduce((s, a, k) => s + a * B[k][j], 0)));
  const mulv = (A, v) => A.map(row => row.reduce((s, a, k) => s + a * v[k], 0));
  const transpose = A => A[0].map((_, j) => A.map(row => row[j]));
  const sub = (a, b) => a.map((x, i) => x - b[i]);
  const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  // [a]×: the matrix with [a]× b = a × b
  const skew = ([x, y, z]) => [[0, -z, y], [z, 0, -x], [-y, x, 0]];

  // the inverse of an intrinsic matrix [[fx, s, cx], [0, fy, cy], [0, 0, 1]]
  function invK(K) {
    const [[fx, s, cx], [, fy, cy]] = K;
    return [[1 / fx, -s / (fx * fy), (s * cy - cx * fy) / (fx * fy)], [0, 1 / fy, -cy / fy], [0, 0, 1]];
  }

  // camera 2 relative to camera 1: X_2 = R X_1 + t
  function relativePose(cam1, cam2) {
    const R = mul(cam2.R, transpose(cam1.R));
    return { R, t: sub(cam2.t, mulv(R, cam1.t)) };
  }
  const essential = (R, t) => mul(skew(t), R);
  function fundamental(cam1, cam2) {
    const { R, t } = relativePose(cam1, cam2), E = essential(R, t);
    return { R, t, E, F: mul(mul(transpose(invK(cam2.K)), E), invK(cam1.K)) };
  }

  // the epipole of camera a: the image of the other camera's centre C_b, homogeneous [u, v, w] (w = 0: at infinity,
  // the direction (u, v) in the picture)
  const centreOf = cam => mulv(transpose(cam.R), cam.t.map(x => -x));
  const epipole = (camA, camB) => mulv(camA.K, mulv(camA.R, sub(centreOf(camB), centreOf(camA))));

  const lineInImage2 = (F, [u, v]) => mulv(F, [u, v, 1]);
  const lineInImage1 = (F, [u, v]) => mulv(transpose(F), [u, v, 1]);
  // distance in pixels of a point from a line a u + b v + c = 0
  const pointLineDistance = ([a, b, c], [u, v]) => (a * u + b * v + c) / Math.hypot(a, b);
  const residual = (F, x1, x2) => dot([x2[0], x2[1], 1], mulv(F, [x1[0], x1[1], 1]));

  // the part of the line a u + b v + c = 0 inside the rectangle [x0, x1] × [y0, y1], as two points, or null
  function clipLine([a, b, c], x0, y0, x1, y1) {
    const pts = [];
    if (Math.abs(b) > 1e-12) for (const u of [x0, x1]) { const v = -(a * u + c) / b; if (v >= y0 - 1e-9 && v <= y1 + 1e-9) pts.push([u, v]); }
    if (Math.abs(a) > 1e-12) for (const v of [y0, y1]) { const u = -(b * v + c) / a; if (u >= x0 - 1e-9 && u <= x1 + 1e-9) pts.push([u, v]); }
    let best = null, bd = -1;
    for (let i = 0; i < pts.length; i++) for (let j = i + 1; j < pts.length; j++) {
      const d = Math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]);
      if (d > bd) { bd = d; best = [pts[i], pts[j]]; }
    }
    return bd > 1e-9 ? best : null;
  }

  // a pixel's ray: the camera centre and a unit direction in the world
  function pixelRay(cam, [u, v]) {
    const d = mulv(transpose(cam.R), mulv(invK(cam.K), [u, v, 1])), n = Math.hypot(...d);
    return { C: centreOf(cam), dir: d.map(x => x / n) };
  }

  const Epi = { skew, invK, relativePose, essential, fundamental, centreOf, epipole, lineInImage1, lineInImage2,
    pointLineDistance, residual, clipLine, pixelRay, cross, dot };
  root.Epi = Epi;
  if (typeof module !== 'undefined' && module.exports) module.exports = Epi;
})(typeof globalThis !== 'undefined' ? globalThis : this);
