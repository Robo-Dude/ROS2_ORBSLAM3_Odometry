# ROS 2 ORB-SLAM3 Odometry Wrapper

This package provides a **ROS 2 interface for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**.  
It links directly against the compiled **`libORB_SLAM3.so`** library and exposes SLAM-estimated **camera pose** as standard ROS 2 messages:

- Publishes **`/odom`** (`nav_msgs/Odometry`)
- Broadcasts TF **`odom → base_link`**
- Supports **Stereo**, **Stereo-Inertial**, **Monocular**, and **RGB-D** camera modes

> Unlike the original ORB-SLAM3 examples that rely on the **Pangolin viewer**,  
> this wrapper is **headless** and bi-directional:
>
> *ROS 2 → SLAM → ROS 2*
>
> Incoming ROS 2 image/IMU topics are fed into SLAM, and the SLAM-estimated
> camera pose is published back to ROS 2 in real-time.

---

## ✨ Features

- 🔗 **Direct API call:** uses ORB-SLAM3 C++ API (not ROS 1 bridge)
- 🎥 **Multi-sensor:** stereo, mono, RGB-D, optional IMU fusion
- 📤 **ROS 2-native output:** odometry + TF frames
- ⚙️ **Axis-conversion:** converts ORB-SLAM3 coordinate system  
  *(X = forward, Y = left, Z = up per ROS REP-103)*
- 🟢 **CUDA-ready:** works with **OpenCV 4.12 / 4.6 + CUDA**
- 🔀 **Relative pose accumulation** option for smoother odom stream

---

## 🖥️ Tested Environment

| Component                | Version / Notes                          |
|--------------------------|------------------------------------------|
| OS                        | **Ubuntu 24.04 LTS (Noble)**             |
| ROS 2 distro             | **Jazzy** (desktop-full + dev-tools)     |
| CMake / GCC              | CMake ≥ 3.22, GCC ≥ 11                   |
| OpenCV                   | **4.12 (CUDA-enabled, included)** or 4.6  |
| Pangolin                 | Latest master (needed only for SLAM build) |
| Eigen                    | 3.4+                                     |
| Sophus                   | Included with ORB-SLAM3                  |
| GPU                      | NVIDIA RTX-series (CUDA 12.x)            |

---

## ⚠️ OpenCV Version Setup

- **OpenCV 4.12 (custom / CUDA-enabled)**  
  The repository already includes all required vision packages stacked along with ORB-SLAM3:
  - `cv_bridge`  
  - `image_transport`  
  Users only need to clone this repository and build — **no extra source fetching required**.

- **OpenCV 4.6 (ROS default)**  
  The included vision packages can be **skipped**, as ROS 2’s default `cv_bridge` and `image_transport` will work without modification.

---

## 🚀 1. Setup ROS 2 Workspace and Clone Repository

1. Create a workspace (for example `ros2_ws`) in your home directory:

```bash
mkdir -p ~/projects/ros2_ws/src
cd ~/projects/ros2_ws/
git clone https://github.com/Robo-Dude/ROS2_ORB-SLAM3_Odometry.git src
```

---

## ⚙️ 2. OpenCV Version Setup

Once the repository is cloned, you have **two options** depending on your OpenCV version:

---

### **Option 1: OpenCV 4.12 (custom / CUDA-enabled)**

- Keep the stacked vision packages (`cv_bridge` and `image_transport`) in the workspace.
- No changes are required inside ORB-SLAM3 or the wrapper CMake files.
- Users can directly proceed to the mandatory ORB-SLAM3 library configuration step.

---

### **Option 2: OpenCV 4.6 (ROS default)**

- **Remove the stacked vision packages** from the workspace (`cv_bridge` and `image_transport`) if present.  
- Install ROS 2’s default packages via `apt`:

```bash
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-image-pipeline
```
> ⚠️  Edit ORB-SLAM3 CMake file (ORB_SLAM3/CMakeLists.txt) to change OpenCV version 4.12 → 4.6.
>
> $ROS_WS_PATH/src/orbslam3_ros2/CMakeLists.txt
```bash
find_package(OpenCV 4.12 REQUIRED) ---> find_package(OpenCV 4.6 REQUIRED)
```

---

### 🔧 3. Mandatory Step: ORB-SLAM3 Library Configuration
You must point the ROS 2 wrapper to the ORB-SLAM3 shared library:
1. Locate the libORB_SLAM3.so file, typically in:
   ```bash
   ~/ORB_SLAM3/lib
   ```
2. Open the ROS 2 wrapper CMake file:
   ```bash
   ~/projects/ros2_ws/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake
   ```
3. Set the ORB-SLAM3 root path:
   ```bash
   set(ORB_SLAM3_ROOT "/full/path/to/ORB_SLAM3")
   ```
   > Replace /full/path/to/ORB_SLAM3 with your actual ORB-SLAM3 folder path.
---

### 🏗️ 4. Build Instructions
```bash
# Navigate to workspace root
cd ~/projects/ros2_ws

# Build the workspace
colcon build 

# again build the orbslam3_ros2 with -symlink-install
colcon build --packages-select orbslam3_ros2 --symlink-install

# Source the setup file
source install/setup.bash
```
