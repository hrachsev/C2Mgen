# C2Mgen — Point Cloud to Mesh Generator

C2Mgen is a desktop application that loads **PLY** point clouds, reconstructs a **triangle mesh** with several algorithms implemented from scratch in C++, and exports **OBJ**. The UI uses a split 3D view (point cloud on the left, mesh on the right), an ImGui control panel, and background jobs with a progress bar.

Third-party dependencies are limited to **GLFW**, **Dear ImGui**, **GLM**, and system **OpenGL** (fixed-function pipeline for rendering). GLFW, GLM, and ImGui are fetched automatically by CMake on first configure.

---

## Features

### Reconstruction (point cloud → mesh)


| Mode                      | Description                                                                                                                                   |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| **Automatic**             | Analyses cloud statistics (density, normal consistency, scale) and picks an algorithm with tuned parameters.                                  |
| **Ball Pivoting**         | Rolling-ball triangulation over oriented samples; good for modest, fairly uniform clouds.                                                     |
| **Screened Poisson**      | Uniform-grid Poisson solve on a splatted normal field, then iso-surface extraction; tends toward watertight results on dense, oriented scans. |
| **Marching Cubes (TSDF)** | Builds a truncated signed distance field from kNN tangent-plane projections, then marching cubes; useful for noisy or non-uniform sampling.   |
| **Greedy Projection**     | PCL-style gp3 fringe growth with angle and normal filters; strong on smooth manifolds with reliable normals.                                  |


If the PLY has no `nx`/`ny`/`nz`, the app estimates PCA normals and orients them consistently before reconstruction.

### Post-process (mesh → mesh)


| Action       | Description                                                                                                                                                           |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Fix mesh** | Voxel remesh: rasterize the current mesh, optionally dilate the shell to close small gaps, flood-fill exterior air, extract a watertight surface with marching cubes. |


### I/O and UI

- Load **ASCII or binary little-endian PLY** (`x`, `y`, `z`; optional `nx`, `ny`, `nz`).
- Export **Wavefront OBJ** (vertices, normals, faces).
- Native file dialogs on Windows (COMDLG).
- Per-viewport orbit camera, wireframe toggle, point size and colours, status log, and tooltips on parameters.

---

## Prerequisites

### Windows 

- **Visual Studio 2022** or newer with the **Desktop development with C++** workload.
- **CMake** 3.16 or newer on your `PATH`.
- **Internet** on the first `cmake` configure (downloads GLFW, GLM, and ImGui via `FetchContent`).
- A GPU/driver that supports **legacy OpenGL** (fixed-function). Most Windows desktop GPUs do; some VMs do not.

### macOS / Linux

The project builds with CMake on other platforms where GLFW and OpenGL are available; file dialogs are fully implemented on Windows. On macOS, link flags for Cocoa/IOKit/CoreVideo are set in `CMakeLists.txt`.

---

## Build steps

From PowerShell (adjust the path if your clone lives elsewhere):

```powershell
cmake -S . -B build
cmake --build build --config Release
```

**MSVC note:** Multi-config generators ignore `-DCMAKE_BUILD_TYPE`. Always pass `--config Release` to `cmake --build`.

Release executable (typical MSVC layout):

```text
.\build\Release\C2Mgen.exe
```

To reconfigure from a clean tree, delete the `build` folder and run the commands again.

---

## Project layout

```text
C2Mgen/
├── CMakeLists.txt          # Build + FetchContent for GLFW, GLM, ImGui
├── README.md
└── src/
    ├── main.cpp            # Entry point
    ├── app/
    │   └── Application.*   # GLFW loop, ImGui UI, job threading
    ├── ui/
    │   └── OrbitCamera.*   # Orbit / pan / zoom camera
    ├── render/
    │   └── Viewport3D.*    # OpenGL draw for cloud and mesh
    ├── io/
    │   ├── PlyLoader.*     # PLY import
    │   └── ObjWriter.*     # OBJ export
    ├── system/
    │   └── NativeFileDialogs.*  # Win32 open/save dialogs
    ├── core/
    │   ├── GeometryTypes.h # Point cloud, mesh, parameter structs
    │   ├── KdTree.*        # k-nearest neighbours
    │   ├── Normals.*       # PCA normals + orientation
    │   ├── MarchingCubes.* # Lorensen–Cline iso-surface extraction
    │   └── JobProgress.h   # Background job progress reporting
    └── recon/
        ├── CloudStats.*         # Sampling / normal statistics
        ├── AutoSelector.*       # Heuristic algorithm + parameter choice
        ├── BallPivoting.*       # Ball pivoting reconstruction
        ├── PoissonRecon.*       # Screened Poisson on a uniform grid
        ├── MarchingCubesRecon.* # TSDF + marching cubes
        ├── GreedyProjection.*   # Greedy projection triangulation
        └── VoxelRemesh.*        # Voxel remesh / fix mesh
```

---

## How to run

1. Start `**C2Mgen.exe**` from `build\Release\` (or your chosen config output directory).
2. Click **Load PLY…** and choose a `.ply` file.
3. Optionally click **Apply auto-tuned parameters** after loading (or pick an algorithm and edit sliders under **Parameters**).
4. Click **Generate mesh** and wait for the progress bar to finish. The mesh appears in the **right** viewport; the **left** viewport stays the point cloud.
5. Use **Export OBJ…** to save the mesh for Blender, MeshLab, etc.
6. If the mesh has holes or is not watertight, tune **Fix mesh settings** and click **Fix mesh** (requires an existing mesh).

### Camera (in either 3D viewport, not over the control panel)


| Action | Input                            |
| ------ | -------------------------------- |
| Rotate | Left mouse drag                  |
| Pan    | Middle mouse or right mouse drag |
| Zoom   | Mouse wheel                      |


**Fit to view** reframes both viewports from the loaded cloud and/or mesh bounds.

### Tips

- Large clouds: lower **Octree depth**, **Grid resolution**, or **Ball radius** to reduce memory and time.
- **Automatic** mode only selects the reconstruction algorithm; **Fix mesh** is always run manually when needed.
- Status text and the **Log** collapsible panel at the bottom of the sidebar report counts, timing, and auto-selector rationale.

