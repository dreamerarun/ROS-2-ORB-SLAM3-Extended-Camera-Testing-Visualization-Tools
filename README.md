# ROS 2 ORB-SLAM3 — Extended Camera, Testing & Visualization Tools

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Research%20%2F%20Development-yellow)]()

## Overview

This repository contains a **ROS 2 integration and extension of an existing ORB-SLAM3-based implementation**.

The original ORB-SLAM3 system was developed by **Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, and Juan D. Tardós**. ORB-SLAM3 is an open-source visual, visual-inertial, and multi-map SLAM system released under the GNU General Public License v3 (GPLv3).

This repository builds upon existing ORB-SLAM3/ROS work and adds experimental and practical tools for:

* Monocular camera input
* Live webcam/camera testing
* Manual frame transmission
* Automated camera-motion experiments
* ORB-SLAM3 initialization experiments
* Camera pose monitoring
* OpenCV-based visualization
* RViz2 integration
* Dataset-based testing
* Debugging and restart utilities

The purpose of this repository is to provide a practical environment for **ROS 2-based ORB-SLAM3 experimentation, testing, visualization, and further development**.

> **Important:** This repository should not be interpreted as a reimplementation of ORB-SLAM3 from scratch. It contains existing upstream components together with additional modifications, scripts, integrations, and experimental tools.

---

# 1. Original Work and Attribution

This project incorporates and/or builds upon existing open-source work.

### ORB-SLAM3

The underlying SLAM technology is based on:

**C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, and J. D. Tardós,
"ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM,"
IEEE Transactions on Robotics, vol. 37, no. 6, pp. 1874–1890, 2021.**

Official ORB-SLAM3 repository:

https://github.com/UZ-SLAMLab/ORB_SLAM3

ORB-SLAM3 is released under **GPLv3**. The original repository also requests that academic users cite the ORB-SLAM3 publication.

### ROS / ROS 2 integration

Parts of the ROS integration are based on existing ORB-SLAM3 ROS implementations and related open-source work.

One relevant upstream project is:

**thien94/orb_slam3_ros**

https://github.com/thien94/orb_slam3_ros

That project provides a ROS implementation of ORB-SLAM3 and is also distributed under GPL-3.0.

Where applicable, the original authors and projects remain credited and their license requirements are retained.

---

# 2. What Has Been Added / Modified

The main purpose of this repository is to extend the existing ORB-SLAM3/ROS environment with additional tools for experimentation.

The repository includes custom scripts for:

### Live camera input

* `simple_live_camera.py`
* `live_mono_camera.py`
* `live_mono_camera_with_init_helper.py`

These scripts provide camera frames to the ORB-SLAM3 ROS pipeline for real-time monocular SLAM experiments.

### Manual frame control

`space_bar.py`

Provides a camera interface where image transmission can be manually controlled using the keyboard.

This is useful for:

* Testing initialization
* Controlling when frames are sent
* Debugging tracking behaviour
* Performing controlled experiments

### Automated camera-motion testing

`auto_motion_camera.py`

Provides experimental motion patterns for camera input, including patterns such as:

* Horizontal motion
* Vertical motion
* Circular motion
* Figure-eight motion
* Random motion

These modes are intended primarily for experimentation and testing.

### Visualization

`1.py` / `orbslam3_visualizer.py`

Provides an interface for monitoring ORB-SLAM3 camera pose and visualizing trajectory information.

For a cleaner public repository, it is recommended to rename `1.py` to something descriptive such as:

```text
orbslam3_visualizer.py
```

### RViz2 integration

`slam_to_rviz2.py`

Provides ROS 2 visualization support for:

* Camera trajectory
* Camera pose
* Keyframe-related visualization
* RViz2 visualization

**Important:** Some map-point visualization in the current implementation is experimental/simulated and should not be interpreted as the actual ORB-SLAM3 reconstructed map unless it is explicitly connected to the real ORB-SLAM3 map data.

### Additional utilities

The repository also contains:

* `debug_detection.py`
* `restart_slam.sh`
* `best_frame.png`

These files are intended to support debugging, testing, and development.

---

# 3. Repository Structure

```text
ros2_orb_slam3/
│
├── README.md
├── LICENSE
├── CHANGELOGS.md
├── CMakeLists.txt
├── package.xml
│
├── config/
│   └── Camera and ORB-SLAM3 configuration files
│
├── include/
│   └── C++ header files
│
├── launch/
│   └── ROS 2 launch files
│
├── orb_slam3/
│   └── ORB-SLAM3 related components
│
├── ros2_orb_slam3/
│   └── ROS 2 package components
│
├── src/
│   └── C++ source files
│
├── scripts/
│   ├── auto_motion_camera.py
│   ├── debug_detection.py
│   ├── live_mono_camera.py
│   ├── live_mono_camera_with_init_helper.py
│   ├── mono_driver_node.py
│   ├── restart_slam.sh
│   ├── simple_live_camera.py
│   ├── slam_to_rviz2.py
│   └── space_bar.py
│
└── TEST_DATASET/
    └── Local testing data
```

---

# 4. Requirements

The project is primarily intended for:

* Ubuntu 22.04
* ROS 2 Humble
* Python 3
* OpenCV
* `rclpy`
* `cv_bridge`
* ROS 2 image messages
* ORB-SLAM3 dependencies

Depending on the selected configuration, additional dependencies may be required.

---

# 5. ROS 2 Workspace Setup

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
```

Clone or copy this repository into the workspace:

```bash
cd ~/ros2_test/src
git clone <YOUR-GITHUB-REPOSITORY-URL> ros2_orb_slam3
```

Then build the workspace:

```bash
cd ~/ros2_test
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/ros2_test/install/setup.bash
```

---

# 6. Configuration

Before running the system, review the configuration files inside:

```text
config/
```

Camera calibration and ORB-SLAM3 settings should correspond to the camera and environment being tested.

Do not commit machine-specific absolute paths to the repository.

For example, avoid configurations such as:

```text
/home/username/Desktop/...
```

Use relative paths or ROS 2 package paths where possible.

---

# 7. Live Camera Testing

The repository contains multiple camera drivers for experimentation.

For example:

```bash
python3 scripts/simple_live_camera.py
```

The live camera driver publishes camera images to the ROS 2 system and communicates with the ORB-SLAM3 node.

Typical topics include:

```text
/mono_py_driver/img_msg
/mono_py_driver/timestep_msg
/mono_py_driver/experiment_settings
```

The camera pose is received through:

```text
/orb_slam3/camera_pose
```

The exact topics may depend on the selected implementation and configuration.

---

# 8. Manual Camera / Spacebar Mode

To perform controlled experiments:

```bash
python3 scripts/space_bar.py
```

The script provides keyboard-controlled frame transmission.

The general workflow is:

```text
Camera
   │
   ▼
Python Camera Driver
   │
   ▼
ROS 2 Image Topic
   │
   ▼
ORB-SLAM3
   │
   ▼
Camera Pose
```

This mode can be useful when investigating initialization and tracking behaviour.

---

# 9. Automated Motion Experiments

The experimental camera-motion utility can be run using:

```bash
python3 scripts/auto_motion_camera.py
```

Example:

```bash
python3 scripts/auto_motion_camera.py \
    --camera_id 0 \
    --motion_pattern circular
```

Available motion patterns include:

```text
horizontal
vertical
circular
figure8
random
```

Additional parameters can be adjusted for motion amplitude and speed.

These modes are intended for **experimental testing** and should not be interpreted as physically accurate camera-motion simulation.

---

# 10. Dataset Testing

The repository can also be used with dataset-based experiments.

If using a local dataset, configure the appropriate dataset path and camera settings before execution.

Do **not** upload large datasets to GitHub unless their redistribution license explicitly permits it.

Instead, document the dataset source and provide instructions for users to download it independently.

---

# 11. RViz2 Visualization

The repository includes:

```text
scripts/slam_to_rviz2.py
```

This provides a ROS 2 interface for visualizing SLAM-related information in RViz2.

The implementation can publish information such as:

```text
/orb_slam3/camera_path
/orb_slam3/map_points
/orb_slam3/keyframe_points
```

It can also provide a TF relationship such as:

```text
map → camera_link
```

### Important limitation

The current RViz2 bridge contains experimental visualization logic.

In particular, the map-point visualization should **not automatically be considered a visualization of the true ORB-SLAM3 reconstructed map**.

If actual ORB-SLAM3 map data is not being extracted directly, the visualization should be described as experimental/simulated.

---

# 12. Important ROS 2 Topics

The project may use topics such as:

| Topic                                 | Purpose                       |
| ------------------------------------- | ----------------------------- |
| `/mono_py_driver/img_msg`             | Camera image                  |
| `/mono_py_driver/timestep_msg`        | Image timestamp               |
| `/mono_py_driver/experiment_settings` | Experiment configuration      |
| `/mono_py_driver/exp_settings_ack`    | Configuration acknowledgement |
| `/orb_slam3/camera_pose`              | Estimated camera pose         |
| `/orb_slam3/camera_path`              | Camera trajectory             |
| `/orb_slam3/map_points`               | Visualization map points      |
| `/orb_slam3/keyframe_points`          | Keyframe visualization        |

The exact topic availability depends on the selected launch/configuration.

---

# 13. Typical Workflow

A typical experiment can follow:

```text
1. Start ROS 2
       │
       ▼
2. Start ORB-SLAM3
       │
       ▼
3. Start camera driver
       │
       ▼
4. Camera images → ROS 2
       │
       ▼
5. ORB-SLAM3 processes frames
       │
       ▼
6. Camera pose is published
       │
       ├──────────────► OpenCV visualization
       │
       └──────────────► RViz2 visualization
```

---

# 14. Debugging

Useful scripts include:

```text
scripts/debug_detection.py
scripts/restart_slam.sh
```

When debugging camera initialization or tracking, check:

1. Camera device availability
2. Image resolution
3. Camera frame rate
4. Camera calibration
5. ROS 2 topic publication
6. ORB-SLAM3 configuration
7. Image encoding
8. Timestamp synchronization
9. Camera movement
10. ORB-SLAM3 terminal output

Check available topics with:

```bash
ros2 topic list
```

Check image messages with:

```bash
ros2 topic echo /mono_py_driver/img_msg
```

Check camera pose with:

```bash
ros2 topic echo /orb_slam3/camera_pose
```

---

# 15. Development Status

This repository is intended for:

* Research
* Academic projects
* Robotics experimentation
* ROS 2 development
* ORB-SLAM3 testing
* Camera integration experiments
* Visualization experiments

The additional scripts should be considered **experimental/research software** unless a specific component is documented as stable.

No claim is made that every component provides production-ready SLAM performance.

---

# 16. Known Limitations

Current limitations may include:

* Camera calibration must be configured correctly for each camera.
* Monocular SLAM can require suitable camera motion and scene texture for initialization and tracking.
* Experimental camera-motion scripts do not represent physically accurate motion.
* RViz2 map-point visualization may use simulated/experimental data rather than the complete ORB-SLAM3 internal map.
* Performance depends on hardware, camera characteristics, configuration, and scene conditions.
* Some scripts may require modification for a user's specific camera device or ROS 2 environment.
* Dataset files are not intended to be redistributed through this repository unless their licenses permit it.

---

# 17. Attribution and Credits

This repository would not exist without the original work of the ORB-SLAM3 authors and the broader open-source robotics community.

### ORB-SLAM3 Authors

**Carlos Campos**
**Richard Elvira**
**Juan J. Gómez Rodríguez**
**José M. M. Montiel**
**Juan D. Tardós**

ORB-SLAM3:

https://github.com/UZ-SLAMLab/ORB_SLAM3

### Related ROS Implementation

This project also acknowledges existing ROS integration work, including:

**thien94/orb_slam3_ros**

https://github.com/thien94/orb_slam3_ros

The original projects retain their respective copyrights, authorship, and license terms.

---

# 18. Citation

If you use the underlying ORB-SLAM3 system in academic work, please cite the original publication:

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual,
         Visual-Inertial and Multi-Map {SLAM}},
  author={Campos, Carlos and Elvira, Richard and
          G{\'o}mez, Juan J. and Montiel, Jos{\'e} M. M. and
          Tard{\'o}s, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874--1890},
  year={2021}
}
```

The official ORB-SLAM3 repository provides this citation and identifies the project as GPLv3 licensed.

---

# 19. License

This repository includes the GNU General Public License Version 3 (`GPL-3.0`).

See:

```text
LICENSE
```

for the complete license text.

Because this repository contains or builds upon existing open-source software, users should review the license and attribution requirements of the original components and third-party dependencies before redistributing modified versions or using the software commercially.

The presence of a GPLv3 license in this repository does **not** remove or replace the copyright and attribution requirements of upstream components.

---

# 20. Third-Party Software

This project may depend on third-party libraries and software distributed under their own licenses.

Users should review the licenses of:

* ORB-SLAM3
* ROS 2
* OpenCV
* Eigen
* Pangolin
* DBoW2
* g2o
* `cv_bridge`
* Other dependencies included or referenced by the project

Where third-party source code is redistributed, its original license and attribution information should be preserved.

---

# 21. Contribution

Contributions, bug reports, experiments, and improvements are welcome.

When submitting modifications, please:

1. Clearly identify new functionality.
2. Preserve upstream copyright notices.
3. Preserve applicable third-party licenses.
4. Document significant changes.
5. Avoid committing datasets or generated files unnecessarily.
6. Do not remove required attribution from upstream projects.

---

# 22. Recommended `.gitignore`

The following generated/local files should generally not be committed:

```gitignore
# ROS 2
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo

# Virtual environments
.venv/
venv/

# IDE
.vscode/
.idea/

# OS
.DS_Store

# Generated trajectories/logs
trajectory_*.txt
*.log

# ROS bags
*.bag
*.db3

# Local datasets
datasets/
data/
TEST_DATASET/

# Local configuration
*.local
```

Review this list before committing because some files may be intentionally required by your project.

---

# 23. Disclaimer

This repository is provided for research, educational, and experimental purposes.

The additional scripts and modifications are provided without any guarantee of accuracy, robustness, or suitability for a particular application.

Users are responsible for validating the software, configurations, camera calibration, datasets, and dependencies for their own environment.

---

## Acknowledgement

**This repository is an extension/modification of existing open-source ORB-SLAM3 and ROS-related work. Full credit is given to the original authors and upstream projects. The additional scripts, integrations, experiments, and modifications in this repository are intended to build upon that work while preserving the applicable open-source licenses and attribution.**
