# Camera tools

Interactive camera optics, as static web pages (no build step): thin-lens models of focus, depth of field and blur,
and the pinhole model of computer vision. Published with GitHub Pages; the root `index.html` links to the individual
tools.

Thin lens:

- [`3d/`](3d/) – **3D thin lens: focus and depth of field in 3D**: the plane of sharp focus, depth of field, field of
  view and lens coverage; optionally a view camera with tilt, swing, rise and shift and the Scheimpflug geometry.
  See [`3d/README.md`](3d/README.md).
- [`2d/`](2d/) – **2D thin lens: focus and depth of field, side view**: the same in the vertical plane, with lens and
  film tilt, the hinge and Scheimpflug points and the depth-of-field wedge; the focus point and the hinge can be
  dragged (a visual Scheimpflug calculator).
- [`defocus/`](defocus/) – **Thin lens: defocus blur visualisation**: a thin lens photographing a 3D scene, each
  point's light cone, the blur the sensor cuts from it, the depth of field and the blurred picture. Self-contained
  apart from the shared favicon and the link back to the start page. See [`defocus/README.md`](defocus/README.md).
- [`calculators/`](calculators/) – calculators and tables (depth of field with draggable charts, hyperfocal and
  DoF tables); the formulas are in `calculators/optics.js`. See [`calculators/README.md`](calculators/README.md).

Pinhole:

- [`pinhole_model/`](pinhole_model/) – **Pinhole camera: projection**: the pinhole camera model of computer vision,
  intrinsics K, pose [R | t], the projection matrix and OpenCV distortion, with a 3D scene and its image.
  Self-contained like `defocus/`. See [`pinhole_model/README.md`](pinhole_model/README.md).
- [`epipolar/`](epipolar/) – **Pinhole cameras: epipolar geometry**: two cameras on one scene, the epipolar plane,
  lines and epipoles, the essential and fundamental matrices. Self-contained like `defocus/`.
  See [`epipolar/README.md`](epipolar/README.md).

## Checks

- `uv run python check_optics.py` – numpy cross-check of the thin-lens / Scheimpflug / depth-of-field
  algebra used by the 3D page (the uv project in the root exists for this).
- `node calculators/test_optics.js` – checks the calculator formulas against textbook values.
- `node pinhole_model/test_camera.js` – checks the projection maths, including values from OpenCV's `cv2.projectPoints`.
- `node defocus/test_lens.js` – checks the thin-lens and blur-circle maths.
- `node epipolar/test_epipolar.js` – checks the two-view maths, including F against OpenCV's `cv2.findFundamentalMat`.

## Publishing

`.github/workflows/pages.yml` publishes only the pages themselves (`index.html`, `3d/index.html`, `2d/index.html`,
`calculators/index.html`, `calculators/optics.js`, `pinhole_model/index.html`, `pinhole_model/camera.js`,
`defocus/`'s `index.html`, `camera.js` and `lens.js`, and `epipolar/`'s `index.html`, `camera.js` and `epipolar.js`)
to GitHub Pages on every push to `main`, plus `favicon.ico`,
`404.html` and uppercase alias folders (`2D/`, `3D/`) that redirect to the real directories, since Pages paths are
case-sensitive.
