# Copilot Instructions for DiSCo-SLAM Codebase

## Project Overview
- **DiSCo-SLAM** is a distributed, multi-robot SLAM system built on a modified version of LIO-SAM, using 3D LiDAR and IMU data.
- The system maintains two factor graphs: one for global optimization (mapOptimization.cpp) and one for real-time odometry (imuPreintegration.cpp).
- Multi-robot support is implemented in `src/DiSCo-SLAM/`, leveraging Scan Context for efficient inter-robot data exchange and PCM for outlier detection.

## Key Components & Structure
- `src/` — Main C++ source files for feature extraction, projection, IMU preintegration, map optimization, and DiSCo-SLAM logic.
- `src/DiSCo-SLAM/` — Multi-robot SLAM logic, global/local optimization, and integration with Scan Context and PCM.
- `config/` — YAML and other config files for sensors, parameters, and dataset preparation.
- `launch/` — ROS launch files for various experiment setups.
- `msg/` and `srv/` — Custom ROS message and service definitions.
- `third_parties/` — Embedded dependencies (e.g., fast_max-clique_finder, scanContext).

## Developer Workflows
- **Build:**
  - Use `catkin_make` from the workspace root after any code or config change.
- **Run:**
  - Launch with `roslaunch disco_double run.launch` (see `launch/` for variants).
  - Play datasets with `rosbag play your-bag.bag -r 3` (adjust rate as needed).
- **Config:**
  - Sensor and algorithm parameters are in `config/params.yaml` and related files.
  - For different datasets, update launch files and config paths as described in `src/DiSCo-SLAM/README.md`.
- **Debugging:**
  - IMU and LiDAR data handling is in `imageProjection.cpp`—uncomment debug lines in `imuHandler()` for sensor alignment checks.
  - For Ouster or other sensors, follow the code comments and README for point type and timestamp/ring field adjustments.

## Project-Specific Patterns & Conventions
- **ROS-centric:** All major components are ROS nodes; launch and config files are essential for correct operation.
- **Parameterization:** Most runtime behavior is controlled via YAML files loaded in launch scripts.
- **Multi-robot:** DiSCo-SLAM logic expects multiple robots and datasets; see `src/DiSCo-SLAM/README.md` for dataset-specific launch/config changes.
- **Embedded third-party code:** Use the provided versions in `third_parties/` for compatibility.
- **Custom messages:** Use `msg/` and `srv/` for inter-node communication; rebuild after changes.

## Integration Points
- **Scan Context:** For place recognition and loop closure (see `src/DiSCo-SLAM/scanContext/`).
- **PCM:** For outlier rejection in global optimization.
- **LIO-SAM:** Core local SLAM logic; DiSCo-SLAM extends this for distributed operation.

## References
- See `README.md` (root and `src/DiSCo-SLAM/`) for detailed setup, dataset, and usage instructions.
- For dataset preparation, see `config/DATASET_PREPARATION.md`.
- For third-party details, see `src/DiSCo-SLAM/third_parties/` and their READMEs.

---

**Example: To run KITTI08, update `launch/run.launch` to load `params_k.yaml` and `mapfusion_k.yaml` as described in `src/DiSCo-SLAM/README.md`.**

**For Ouster LiDAR, follow the code comments in `imageProjection.cpp` and adjust config as per the main `README.md`.**

---

*Keep instructions concise, actionable, and project-specific. Update this file as project structure or conventions evolve.*
