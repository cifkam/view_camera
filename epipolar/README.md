# Pinhole cameras: epipolar geometry

Two pinhole cameras looking at the same scene, and the geometry that ties their pictures together. Self-contained
apart from the shared favicon and the link back to the start page: open `index.html` directly or from any static
web server (three.js comes from a CDN).

## What is on the page

- **3D view**: the scene, both cameras (C₁ orange, C₂ teal) with their image planes carrying their pictures, the
  baseline through both centres (dashed) and the epipoles e₁, e₂ where it pierces the image planes. For the selection,
  the rays to both centres and the epipolar plane through them (magenta). Drag a camera's centre to move it across the
  ground, with Shift up and down; it keeps the direction it looks in.
- **Pictures**: what each camera sees, with the selection's epipolar line and the epipoles (an arrow at the edge
  when one lies outside the picture or at infinity). Tap a vertex in either view to select it: its two images lie on
  each other's epipolar lines. Tap or drag anywhere else in a picture to pick a pixel: the 3D view shows its ray, and
  the points along it (the dots) all land on its epipolar line in the other picture, which is why a match is searched
  for along that line. A point on the ray (orange, at first the last but one) can be dragged along it in the 3D view,
  its image sliding along the line. A vertex tapped in a picture works the same, the point nearest it on the ray being
  the vertex itself (its image ringed on the line). Optionally a set of epipolar lines in matching colours, all
  through the epipoles.
- **Maths**: the relative pose R, t, the essential matrix E = [t]× R, the fundamental matrix F = K₂⁻ᵀ E K₁⁻¹, the
  epipoles, and for the selection its epipolar lines and x₂ᵀ F x₁ = 0.
- **Presets**: a converging pair; a rectified stereo pair (same orientation, side by side: the epipolar lines are
  horizontal, the epipoles at infinity); and forward motion (the epipole is the focus of expansion).

## Model

Camera.js's conventions: world Z up, OpenCV camera frame (x right, y down, z forward), X_c = R X_w + t, t = −R C,
x = K X_c / Z_c, both cameras with the same K. Camera 2 relative to camera 1: X₂ = R X₁ + t with R = R₂R₁ᵀ and
t = t₂ − R t₁; E = [t]× R, F = K₂⁻ᵀ E K₁⁻¹; l₂ = F x₁, l₁ = Fᵀ x₂; e₁ is the image of C₂ in camera 1 (F e₁ = 0),
e₂ that of C₁ in camera 2 (Fᵀ e₂ = 0). No lens distortion: epipolar lines are straight only without it.

## Files

- `index.html` – the page.
- `epipolar.js` – the two-view maths (classic script, also loadable under node).
- `camera.js` – pose, projection and the scene's primitives; a copy of the one in the pinhole page.
- `test_epipolar.js` – checks, including F against OpenCV's `cv2.findFundamentalMat`: `node epipolar/test_epipolar.js`.
