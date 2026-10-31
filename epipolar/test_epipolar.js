// Checks for epipolar.js. Run: node epipolar/test_epipolar.js
const Cam = require('./camera.js'), Epi = require('./epipolar.js');
let fails = 0;
const check = (name, got, expected, tol = 1e-9) => {
  const ok = got === expected || Math.abs(got - expected) <= tol * Math.max(1, Math.abs(expected));
  if (!ok) fails++;
  console.log((ok ? 'PASS ' : 'FAIL ') + name + (ok ? '' : ` (got ${got}, expected ${expected})`));
};
const det3 = M => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
  + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
const frob = M => Math.hypot(...M.flat());
const scaled = (M, k) => M.map(r => r.map(x => x * k));

// two cameras looking at a scene from different places, with different intrinsics (skew included)
const K1 = Cam.intrinsics({ fx: 1000, fy: 1000, cx: 640, cy: 360 }), K2 = Cam.intrinsics({ fx: 1100, fy: 1080, cx: 600, cy: 380, skew: 2 });
const c1 = Cam.makeCamera({ K: K1, R: Cam.rotationFromYawPitchRoll(-20, -8, 3), t: Cam.translationFromCentre(Cam.rotationFromYawPitchRoll(-20, -8, 3), [-1.5, -4, 1.4]) });
const c2 = Cam.makeCamera({ K: K2, R: Cam.rotationFromYawPitchRoll(25, -10, -2), t: Cam.translationFromCentre(Cam.rotationFromYawPitchRoll(25, -10, -2), [2, -3.5, 1.6]) });
const { E, F } = Epi.fundamental(c1, c2);

check('K⁻¹ K = I', Math.hypot(...Cam.mul(Epi.invK(K2), K2).flat().map((x, i) => x - (i % 4 === 0 ? 1 : 0))), 0, 1e-12);
check('E has rank 2 (det 0)', det3(E) / frob(E) ** 3, 0, 1e-12);
check('F has rank 2 (det 0)', det3(F) / frob(F) ** 3, 0, 1e-12);
let worst = 0, worstPx = 0;
for (let i = 0; i < 30; i++) {
  const X = [Math.sin(i) * 1.5, 1 + (i % 7) * 0.4, (i % 5) * 0.3];
  const x1 = Cam.projectPoint(X, c1).uv, x2 = Cam.projectPoint(X, c2).uv;
  worst = Math.max(worst, Math.abs(Epi.residual(F, x1, x2)) / frob(F));
  worstPx = Math.max(worstPx, Math.abs(Epi.pointLineDistance(Epi.lineInImage2(F, x1), x2)), Math.abs(Epi.pointLineDistance(Epi.lineInImage1(F, x2), x1)));
}
check('x₂ᵀ F x₁ = 0 for 30 scene points', worst, 0, 1e-12);
check('each image point lies on the epipolar line of its partner (px)', worstPx, 0, 1e-8);
const e1 = Epi.epipole(c1, c2), e2 = Epi.epipole(c2, c1);
check('F e₁ = 0', Math.hypot(...Cam.mulv(F, e1)) / (frob(F) * Math.hypot(...e1)), 0, 1e-12);
check('Fᵀ e₂ = 0', Math.hypot(...Cam.mulv(Cam.transpose(F), e2)) / (frob(F) * Math.hypot(...e2)), 0, 1e-12);
check('e₁ is the projection of C₂', e1[0] / e1[2], Cam.projectPoint(Epi.centreOf(c2), c1).uv[0], 1e-9);
// a point anywhere on the ray of a pixel of camera 1 lands on that pixel's epipolar line in camera 2
const ray = Epi.pixelRay(c1, [300, 200]), l2 = Epi.lineInImage2(F, [300, 200]);
for (const d of [1, 3, 10]) {
  const X = Cam.add(ray.C, Cam.scale(ray.dir, d));
  check(`a point ${d} m along a pixel's ray lies on its epipolar line`, Epi.pointLineDistance(l2, Cam.projectPoint(X, c2).uv), 0, 1e-8);
}

// against OpenCV: cv2.findFundamentalMat (8-point) from 40 exact correspondences of two cameras, scaled to unit norm
// with F[2][2] > 0 (opencv-python 4; rvecs (0.05, −0.1, 0.02) and (−0.04, 0.25, −0.03))
const R1 = [[0.9948055875968536, -0.02245434138184177, -0.09928567590134285], [0.017459714071124073, 0.9985515580798918, -0.050891494778350845], [0.10028460136348638, 0.04889364385406377, 0.9937567158616029]];
const R2 = [[0.9684713367301254, 0.024703079337642236, 0.24789721217351804], [-0.03464902989596228, 0.99875675618021, 0.035838341363032995], [-0.24670369810651963, -0.043297804281773024, 0.9681232284605843]];
const Fcv = [[-1.9714716375401282e-07, 1.1368490563589246e-06, 0.00015805383686338836], [8.200775403021273e-07, 5.163335846487206e-07, -0.006469419953682928], [-0.0009479412514202389, 0.005249706080123731, 0.9999648311893939]];
const cv1 = Cam.makeCamera({ K: [[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], R: R1, t: [0.2, -0.1, 4.0] });
const cv2 = Cam.makeCamera({ K: [[1100, 2, 600], [0, 1080, 380], [0, 0, 1]], R: R2, t: [-0.9, 0.05, 4.2] });
const Fo = Epi.fundamental(cv1, cv2).F, Fn = scaled(Fo, Math.sign(Fo[2][2]) / frob(Fo));
check('F matches cv2.findFundamentalMat', Math.hypot(...Fn.flat().map((x, i) => x - Fcv.flat()[i])), 0, 1e-6);

// rectified stereo: same orientation, camera 2 to the right: horizontal epipolar lines, epipoles at infinity
const R0 = Cam.rotationFromYawPitchRoll(0, 0, 0);
const s1 = Cam.makeCamera({ K: K1, R: R0, t: Cam.translationFromCentre(R0, [0, 0, 1]) });
const s2 = Cam.makeCamera({ K: K1, R: R0, t: Cam.translationFromCentre(R0, [0.5, 0, 1]) });
const Fs = Epi.fundamental(s1, s2).F, ls = Epi.lineInImage2(Fs, [100, 250]);
check('rectified: an epipolar line is horizontal (a = 0)', ls[0] / Math.hypot(...ls), 0, 1e-12);
check('rectified: at the same row', -ls[2] / ls[1], 250, 1e-9);
check('rectified: the epipole is at infinity (w = 0)', Epi.epipole(s1, s2)[2], 0, 1e-9);
// forward motion: the epipole is the principal point (the focus of expansion)
const f2 = Cam.makeCamera({ K: K1, R: R0, t: Cam.translationFromCentre(R0, [0, 1, 1]) });
const ef = Epi.epipole(s1, f2);
check('forward motion: the epipole is the principal point (u)', ef[0] / ef[2], 640, 1e-9);
check('forward motion: (v)', ef[1] / ef[2], 360, 1e-9);

// clipping a line to the picture
const seg = Epi.clipLine([0, 1, -100], 0, 0, 640, 480);
check('clip a horizontal line: from u = 0', seg[0][0] + seg[1][0], 640, 1e-12);
check('a line missing the picture is null', Epi.clipLine([0, 1, -900], 0, 0, 640, 480), null, 0);

console.log(fails ? `${fails} FAILED` : 'all passed');
process.exitCode = fails ? 1 : 0;
