# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# Configure with CMake preset (from cpp_eyetracker/)
cmake --preset vs2022-vcpkg -B build

# Build all targets
cmake --build build --config Release

# Build a specific target
cmake --build build --config Release --target test_multi_cam
```

Requires vcpkg (with `VCPKG_ROOT` env var), Basler Pylon SDK (`PYLON_DEV_DIR` env var), and optionally HALCON (`HALCONROOT` env var). Key vcpkg packages: opencv4, ceres, eigen3, yaml-cpp, pugixml.

## Architecture

The project is a C++ gaze-estimation system using Basler cameras and the PCCR (Pupil Center Corneal Reflection) method. The top-level `CMakeLists.txt` in `cpp_eyetracker/` is the sole entry point; all libraries are static and linked into test executables (there is no production binary — tests are the executables).

### Module dependency graph (top-down)

```
core (math_types: Eigen3 wrappers)
 ├─> logger (interface-only, no .cpp)
 ├─> cfg (YAML config reader/writer, depends on core + logger + yaml-cpp)
 │    └─ config reads cpp_eyetracker/cfg/default.yaml
 ├─> cam_model (interface-only, pinhole camera model)
 ├─> utils (gaze_estimation_types, shared_calculations, intersection, visualize)
 │    ├─ depends on core, cfg, cam_model, OpenCV, pugixml
 │    ├─ subdir: glint_detection (corneal reflection detection via OpenCV)
 │    ├─ subdir: pupil_center (pupil localization)
 │    └─ subdir: cam (Basler Pylon SDK wrapper)
 ├─> inference (gaze estimation: cornea center + optical/visual axis, depends on utils + Ceres)
 └─> calib (personal calibration via Ceres nonlinear optimization, depends on utils + inference)
```

### Key data flow

1. **Image acquisition**: `cam` (Basler) → raw eye images
2. **Feature extraction**: `glint_detection` → corneal reflections + `pupil_center` → pupil center → produces `PupilCenterGlintInputs` (pairs of 2D glints + pupil center per eye)
3. **Gaze estimation**: `GazeTracker::estimate()` computes 3D cornea center and optical/visual axis from glints + pupil → produces `DefaultGazeEstimationResult`
4. **Calibration**: `Calibration::calibrate()` optimizes eye parameters (alpha, beta, R, K, n1, n2, D) via Ceres to fit gaze predictions to known screen targets

### Configuration

All runtime settings come from `cpp_eyetracker/cfg/default.yaml`. The `Cfg` class preserves YAML structure and supports reading via `cfg["section"]["key"].as<T>()` and writing back with precision control (`setScalar`, `setVector`, `setVector2D`).

### Test executables

**Currently built** (active in CMakeLists):
- `tests/utils/cam/` — `test_multi_cam`, `test_multi_cam_multi_host` (multi-camera recording with network sync)

**Commented out** (temporarily disabled in parent CMakeLists — re-enable by uncommenting `add_subdirectory`):
- `tests/calib/` — `test_calib_single_eye`, `test_calib_screen`, `test_jitter`
- `tests/inference/` — `test_two_eye`
- `tests/cfg/` — `test_cfg`
- `tests/utils/cam_calib/` — camera calibration executables (requires HALCON)
- `tests/utils/glint_detection/` — glint detection tests

## HALCON Pose Convention (rotation order)

`calib_cam_chain` / `calib_cam_intrinsics` write camera poses to `{SN}_Data.xml` with `PoseTypeCode=0` (Rp+T), `OrderOfRotation=gba`, `ViewOfTransform=point`. The XML `<Rotation>` fields `Alpha/Beta/Gamma` follow HALCON's `create_pose` semantics:

- **`gba` rotation matrix: `R = Rx(Alpha) · Ry(Beta) · Rz(Gamma)`** — Alpha rotates around X, Beta around Y, Gamma around Z. Read right-to-left in the fixed (reference) coordinate system: first around fixed z by Gamma, then old y by Beta, then old x by Alpha. Source: HALCON `create_pose` operator reference (`R('gba') = Rx(RotX) * Ry(RotY) * Rz(RotZ)`, RotX=alpha, RotY=beta, RotZ=gamma).
- Pose transforms camera coordinates into the reference (center-camera) frame: `p_ref = R · p_cam + T`. Camera frame: x right, y down, z forward out of the lens.
- C++ code does **no manual angle math** — all rebasing goes through HALCON (`PoseCompose`); raw gba values are exported to XML unchanged.
- `viz_calib_chain.py` reimplements the convention. If axes look wrong in a new visualizer, check this formula first — a `Rz(α)·Ry(β)·Rz(γ)` implementation produces flipped Z and wrong X axes.

## Orphaned / Unused files

These source files exist but are **not compiled by any CMakeLists** and will not be used again:

| File | Reason |
|------|--------|
| `tests/calib/test_calib.cpp` | Replaced by `test_calib_single_eye.cpp` + `test_calib_screen.cpp` |
| `tests/inference/test_gaze.cpp` | Replaced by `test_two_eye.cpp` (old style: hardcoded paths, raw Win32 API, wrong `#include <iomanip>`) |
| `tests/utils/cam/test_net.cpp` | Network sync test superseded by `test_multi_cam_multi_host.cpp` |
| `tests/utils/cam/test_single_cam.cpp` | Single-cam test superseded by `test_multi_cam.cpp` |

**Orphaned test directories** (have `CMakeLists.txt` but no parent `add_subdirectory` references them — never built):
- `tests/utils/pupil_center/` — `test_pupil`
- `tests/utils/cam_model/` — `test_cam_model`
- `tests/calib/record_data/` — `test_record`
- `tests/calib/set_image/` — `test_image`

**Python visualization scripts** (all standalone, run manually — not referenced by build):
- `tests/calib/viz_calib_result.py` — 3D gaze calibration result viewer
- `tests/calib/viz_jitter_tendency.py` — Jitter sensitivity analysis (scans output files for pupil/glint jitter metrics, exports CSV + plots)
- `tests/inference/viz.py` — 3D gaze ray visualizer with screen intersection
- `tests/utils/glint_detection/viz_jitter_glint.py` — Creates eye-crop videos (uses dlib face landmarks)
- `tests/utils/cam/green_screen.py` — Theoretical green-screen size calculator for a 9-camera spherical array; detached from any C++ pipeline
