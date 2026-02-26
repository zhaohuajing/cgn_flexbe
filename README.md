# FlexBE States and Behaviors for Contact-GraspNet

FlexBE service states and behavior pipelines for integrating **Contact-GraspNet (CGN)** into a ROS 2 manipulation workflow.

This repository provides:

- **FlexBE service states** for calling a ROS 2 Contact-GraspNet server
- **Behavior pipelines** that connect perception/segmentation to grasp planning and motion execution
- **FlexBE utilities** for utility scripts different from `uml-robotics/compare_flexbe` upstream
- A recommended pipeline using:
  - **Unseen Object Clustering (UOC)** for segmentation
  - **Contact-GraspNet with RGB-D inputs** for grasp generation

## Overview

This package is a **FlexBE-based perception-to-action integration layer** for Contact-GraspNet.

It supports two CGN service states:

1. **`cgn_grasp_rgbd_service_state.py`** (recommended)
   - Uses an **RGB-D-based CGN workflow**
   - Works well with **Unseen Object Clustering (UOC)** as the upstream segmentation step
   - Calls the ROS 2 Contact-GraspNet server (`/get_grasps`) using a `scene_name` convention (server-side scene loading)

2. **`cgn_grasp_cloud_service_state.py`** (less recommended)
   - Uses **point cloud input** (`sensor_msgs/PointCloud2`)
   - Sends flattened XYZ points + mask/indices to the CGN ROS 2 server
   - Works with PointCloud-based perception modules `GetPointCloudServiceState` and `EuclideanClusteringServiceState`, but is generally less robust than the RGB-D/UOC pipeline

## Recommended Pipeline

**Most recommended:**  
**UOC segmentation + CGN (RGB-D) + MoveIt OMPL motion planning**

Implemented in:

- `unseenobjclustercontactgraspnetpipeine_sm.py`

High-level flow:

1. RGB-D segmentation (UOC)
2. Target instance selection
3. Contact-GraspNet grasp generation (RGB-D mode)
4. MoveIt-based motion planning to candidate grasp poses

## Repository Structure

```text
├── cgn_flexbe
├── cgn_flexbe_behaviors
│   ├── bin
│   ├── cgn_flexbe_behaviors
│   │   ├── __init__.py
│   │   ├── euclideanclustercontactgraspnetpipeine_sm.py
│   │   ├── pointcloudcontactgraspnetpipeine_sm.py
│   │   └── unseenobjclustercontactgraspnetpipeine_sm.py
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── config
│   │   └── example.yaml
│   ├── manifest
│   │   ├── euclideanclustercontactgraspnetpipeine.xml
│   │   ├── pointcloudcontactgraspnetpipeine.xml
│   │   └── unseenobjclustercontactgraspnetpipeine.xml
│   ├── package.xml
│   ├── resource
│   │   └── cgn_flexbe_behaviors
│   ├── setup.cfg
│   └── setup.py
├── cgn_flexbe_states
│   ├── cgn_flexbe_states
│   │   ├── __init__.py
│   │   ├── cgn_grasp_cloud_service_state.py
│   │   ├── cgn_grasp_rgbd_service_state.py
│   │   ├── reach_to_grasp_service_state.py
│   │   └── select_instance_to_cgn_indices_state.py
│   ├── CHANGELOG.rst
│   ├── package.xml
│   ├── resource
│   │   └── cgn_flexbe_states
│   ├── setup.cfg
│   └── setup.py
```

## Quick Start

This section is tailored to the service names used in the uploaded FlexBE states/behaviors.

### 1) Build the workspace

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

### 2) Setup required ROS 2 servers (recommended UOC + RGB-D CGN pipeline)

The recommended behavior (`UnseenObjClusterContactGraspnetPipeine`) expects these services to be set up:

- `/segmentation_rgbd` (UOC segmentation server): setup unseen_object_clustering_ros2 server through https://github.com/zhaohuajing/unseen_object_clustering_ros2
- `/get_grasps` (Contact-GraspNet server): setup contact_graspnet_ros2 server through https://github.com/zhaohuajing/contact_graspnet_ros2
- `/move_to_pose` (MoveIt/OMPL execution server): setup moveit and armada_ros2 servers through https://github.com/flynnbm/armada_ros2

Consider adding the following nodes to your launch file:
```text
uoc_rgbd_bringup = Node(
    package="unseen_obj_clst_ros2",
    executable="segmentation_rgbd_server",
    name="segmentation_rgbd_server",
    output="screen",
)

cgn_rgbd_bringup = Node(
    package="contact_graspnet_ros2",
    executable="grasp_executor_rgbd_server",
    name="grasp_executor_rgbd_server",
    output="screen",
)


cgn_cloud_bringup = Node(
    package="contact_graspnet_ros2",
    executable="grasp_executor_cloud_server",
    name="grasp_executor_cloud_server",
    output="screen",
)
```

### 3) Start FlexBE and run a behavior

Open your FlexBE app / onboard execution, then run one of:

- `UnseenObjClusterContactGraspnetPipeine` (**recommended**)
- `EuclideanClusterContactGraspnetPipeine`
- `PointCloudContactGraspnetPipeine`

### 4) Optional services for alternate pipelines

Depending on the behavior you choose, additional services may be required:

- `/get_point_cloud`
- `/euclidean_clustering`
- `/filter_by_indices`
- `/reach_to_grasp`

You can verify everything is up with:

```bash
ros2 service list | grep -E "segmentation_rgbd|get_grasps|move_to_pose|get_point_cloud|euclidean_clustering|filter_by_indices|reach_to_grasp"
```

## Provided FlexBE States

### `CGNGraspRGBDServiceState` (recommended)
**File:** `cgn_flexbe_states/cgn_grasp_rgbd_service_state.py`

Calls the Contact-GraspNet ROS 2 service in **RGB-D scene-name mode** (server loads data internally using `scene_name`).

**Inputs**
- `scene_name` (`string`) — CGN scene identifier (e.g., `scene_from_ucn`)

**Outputs**
- `grasp_target_poses` (`geometry_msgs/Pose[]`)
- `grasp_scores` (`float[]`)
- `grasp_samples` (`geometry_msgs/Point[]`)
- `grasp_object_ids` (`int[]`)

**Default service**
- `/get_grasps`

---

### `CGNGraspCloudServiceState` (point cloud mode, less recommended)
**File:** `cgn_flexbe_states/cgn_grasp_cloud_service_state.py`

Calls the Contact-GraspNet ROS 2 service using `PointCloud2`, converting the cloud to flattened XYZ points and optionally applying index-based masking / Z filtering before the request.

**Inputs**
- `cloud_in` (`sensor_msgs/PointCloud2`) — target/filtered point cloud
- `indices` (`list[int]`, optional) — optional point indices used for mask generation

**Outputs**
- `grasp_target_poses` (`geometry_msgs/Pose[]`)
- `grasp_scores` (`float[]`)
- `grasp_samples` (`geometry_msgs/Point[]`)
- `grasp_object_ids` (`int[]`)

**Default service**
- `/get_grasps`

---

### Other related states (used in pipelines)
This repository also includes utility states for integration and execution, such as:

- `reach_to_grasp_service_state.py`
- `select_instance_to_cgn_indices_state.py`

The behaviors also rely on additional states from companion packages (e.g., `compare_flexbe_states`) such as:
- `UnseenObjSegRGBDServiceState`
- `GetPointCloudServiceState`
- `EuclideanClusteringServiceState`
- `FilterByIndicesServiceState`
- `MoveToPoseServiceState`
- `PublishPointCloudState`

## Provided FlexBE Behaviors (Pipelines)

### 1) `UnseenObjClusterContactGraspnetPipeine` (recommended)
**File:** `cgn_flexbe_behaviors/cgn_flexbe_behaviors/unseenobjclustercontactgraspnetpipeine_sm.py`

Pipeline:
1. `UnseenObjSegRGBDServiceState` (`/segmentation_rgbd`)
2. `SelectInstanceToSceneNameState`
3. `CGNGraspRGBDServiceState` (`/get_grasps`)
4. `MoveToPoseServiceState` (`/move_to_pose`)

Why recommended:
- Best compatibility in this integration
- Clean segmentation-to-scene-name handoff
- More stable grasp generation in the current setup

---

### 2) `EuclideanClusterContactGraspnetPipeine`
**File:** `cgn_flexbe_behaviors/cgn_flexbe_behaviors/euclideanclustercontactgraspnetpipeine_sm.py`

Pipeline:
1. `GetPointCloudServiceState` (`/get_point_cloud`)
2. `EuclideanClusteringServiceState` (`/euclidean_clustering`)
3. `FilterByIndicesServiceState` (`/filter_by_indices`)
4. `CGNGraspServiceState` (`/get_grasps`, point cloud mode)
5. `PublishPointCloudState` (publishes `/filtered_cloud/target_object`)
6. `MoveToPoseServiceState` (`/move_to_pose`)

Use case:
- Full point-cloud pipeline with clustering
- Useful fallback when UOC is unavailable

---

### 3) `PointCloudContactGraspnetPipeine`
**File:** `cgn_flexbe_behaviors/cgn_flexbe_behaviors/pointcloudcontactgraspnetpipeine_sm.py`

Pipeline:
1. `GetPointCloudServiceState` (`/get_point_cloud`)
2. `CGNGraspServiceState` (`/get_grasps`, point cloud mode)
3. `MoveToPoseServiceState` (`/move_to_pose`)
4. `ReachToGraspServiceState` (`/reach_to_grasp`)

Use case:
- Minimal direct point-cloud-to-grasp pipeline
- Good for debugging and simpler demos

## Tables for Easier Documentation

### State summary

| State file | Main class | Inputs | Outputs | Service called | Notes |
|---|---|---|---|---|---|
| `cgn_grasp_rgbd_service_state.py` | `CGNGraspRGBDServiceState` | `scene_name` | `grasp_target_poses`, `grasp_scores`, `grasp_samples`, `grasp_object_ids` | `/get_grasps` | Recommended. RGB-D scene-name mode. |
| `cgn_grasp_cloud_service_state.py` | `CGNGraspServiceState` | `cloud_in`, `indices` (optional) | `grasp_target_poses`, `grasp_scores`, `grasp_samples`, `grasp_object_ids` | `/get_grasps` | Point-cloud mode, less recommended. |
| `reach_to_grasp_service_state.py` | `ReachToGraspServiceState` | typically `grasp_poses`, `grasp_index` | behavior-dependent | `/reach_to_grasp` | Used as final approach/closure in point-cloud pipeline. |
| `select_instance_to_cgn_indices_state.py` | `SelectInstanceToSceneNameState` or related selector | segmentation results (`seg_json`, ids, masks) | selected target / `scene_name` or indices | none (local mapping) | Bridges segmentation output to CGN input convention. |

### Behavior summary

| Behavior (FlexBE) | Main file | Pipeline type | Services used | Recommended |
|---|---|---|---|---|
| `UnseenObjClusterContactGraspnetPipeine` | `unseenobjclustercontactgraspnetpipeine_sm.py` | UOC (RGB-D) -> CGN RGB-D -> MoveIt | `/segmentation_rgbd`, `/get_grasps`, `/move_to_pose` | Yes (primary) |
| `EuclideanClusterContactGraspnetPipeine` | `euclideanclustercontactgraspnetpipeine_sm.py` | Point cloud -> Euclidean clustering -> CGN cloud -> MoveIt | `/get_point_cloud`, `/euclidean_clustering`, `/filter_by_indices`, `/get_grasps`, `/move_to_pose` | Secondary |
| `PointCloudContactGraspnetPipeine` | `pointcloudcontactgraspnetpipeine_sm.py` | Point cloud -> CGN cloud -> MoveIt -> Reach | `/get_point_cloud`, `/get_grasps`, `/move_to_pose`, `/reach_to_grasp` | Debug/fallback |

## Architecture

### Recommended architecture (UOC + RGB-D CGN)

```text
RGB-D Camera
   |
   v
Unseen Object Clustering (ROS 2 service: /segmentation_rgbd)
   |
   v
Target instance selection / scene mapping
   |
   v
CGNGraspRGBDServiceState
   |
   v
Contact-GraspNet ROS 2 Server (/get_grasps)
   |
   v
Grasp poses (Pose[])
   |
   v
MoveIt / OMPL (MoveToPoseServiceState -> /move_to_pose)
   |
   v
Robot motion execution
```

### Point-cloud architecture (less recommended)

```text
PointCloud2 (/get_point_cloud)
   |
   v
(optional) Euclidean clustering / filtering
   |
   v
CGNGraspServiceState (cloud mode)
   |
   v
Contact-GraspNet ROS 2 Server (/get_grasps)
   |
   v
Grasp poses (Pose[])
   |
   v
MoveIt / OMPL
   |
   v
(optional) ReachToGrasp (/reach_to_grasp)
```

## Dependencies

This repository assumes you already have the following in your ROS 2 workspace:

- **FlexBE** (core + onboard/app tooling)
- **Contact-GraspNet ROS 2 server**
  - service: `/get_grasps`
  - service/message types from `contact_graspnet_ros2`
- **MoveIt / motion execution service(s)**
  - `/move_to_pose`
  - optionally `/reach_to_grasp`
- **Perception/segmentation services** depending on pipeline:
  - `/segmentation_rgbd` (UOC, recommended)
  - `/get_point_cloud`
  - `/euclidean_clustering`
  - `/filter_by_indices`

## Installation

Clone into your ROS 2 workspace `src/` folder:

```bash
cd ~/your_ws/src
git clone <your_repo_url>
```

Build and source:

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

## Notes and Recommendations

- **Use `CGNGraspRGBDServiceState` + UOC whenever possible.**  
  This is the most reliable configuration in the current integration.

- The point-cloud CGN state is still useful for debugging or fallback integration, but grasp quality and robustness may be lower depending on scene preprocessing.

- These behavior files are generated FlexBE state machines. Manual edits inside generated sections may be overwritten if the behavior is regenerated.

- Some behavior imports reference companion states from `compare_flexbe_states` (e.g., segmentation, clustering, move-to-pose). Keep those packages in the same workspace.

## Acknowledgments

This repository builds on:

- Contact-GraspNet
- FlexBE
- ROS 2
- MoveIt
- Upstream perception modules (Unseen Object Clustering / PCL clustering)
