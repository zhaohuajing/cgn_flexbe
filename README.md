# FlexBE States and Behaviors for Contact-GraspNet (ROS 2) 

FlexBE service states and behavior pipelines for integrating **Contact-GraspNet (CGN)** into a ROS 2 manipulation workflow.

This repository provides:

- **FlexBE service states** for calling a ROS 2 Contact-GraspNet server
- **Behavior pipelines** that connect perception/segmentation to grasp planning and motion execution
- A recommended pipeline using:
  - **Unseen Object Clustering (UOC)** for segmentation
  - **Contact-GraspNet with RGB-D inputs** for grasp generation

## Overview

This package is designed as a **FlexBE-based perception-to-action integration layer** for Contact-GraspNet.

It supports two CGN service states:

1. **`cgn_grasp_rgbd_service_state.py`** (recommended)
   - Uses **RGB-D-based CGN workflow**
   - Works well with **Unseen Object Clustering (UOC)** as the upstream segmentation step
   - Calls the ROS 2 Contact-GraspNet server (`/get_grasps`) using a `scene_name` / precomputed RGB-D scene representation

2. **`cgn_grasp_cloud_service_state.py`** (less recommended)
   - Uses **point cloud input** (`PointCloud2`)
   - Sends flattened points + mask to the CGN ROS 2 server
   - Can work, but is generally less robust than the RGB-D/UOC pipeline in our setup

## Recommended Pipeline

**Most recommended:**  
**UOC segmentation + CGN (RGB-D) + MoveIt OMPL motion planning**

This pipeline is implemented in:

- `unseenobjclustercontactgraspnetpipeine_sm.py`

It performs:

1. RGB-D segmentation (Unseen Object Clustering)
2. Target instance selection
3. Contact-GraspNet grasp generation (RGB-D mode)
4. MoveIt-based motion planning to candidate grasp poses

## Repository Structure

```text
├── cgn_flexbe
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   └── package.xml
├── cgn_flexbe_behaviors
│   ├── bin
│   │   └── copy_behavior
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
----

This raw repository has several folders and files with the generic name `cgn`.

---

This repository is used by the FlexBE widget 
[`create_repo`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_widget/bin/create_repo) 
script to create an example project that you can build off of to add your own states and behaviors.  

Using `ros2 run flexbe_widget create_repo <my_new_project_name>` will clone this repository, 
and change the relevant `cgn` text to `my_new_project_name` as needed.

It sets up the `package.xml` files with proper FlexBE export tags.
It is maintained at version `0.0.1` as the starting point for your work.

We have provided a license file to conform to ROS guidelines; however, you are free to replace the 
`LICENSE` file, and apply whatever license you choose to states and behaviors that you create.

This repository contains an example behavior and examples for writing your own state implementations.

## Example States in `cgn_flexbe_states`

Packages providing FlexBE states are identified by an export tag in the `package.xml`:

```xml
  <export>
      <flexbe_states />
      <build_type>ament_cmake</build_type>
  </export>
```

* `example_state.py `
  * Example state implementation with extra console logging to show the state life cycle.

* `example_action_state.py`

> Note: These example states are defined with extra console logging that is useful when learning FlexBE, 
> but you will typically not include so much of the `Logger.info` commands as in these examples.

> Note: You are free to copy and modify these files to create your own files and publish under your own license terms.
> As per the existing licenses, no warranty is implied.

## Example Behaviors in `cgn_flexbe_behaviors`

Packages providing FlexBE behaviors are identified by an export tag in the `package.xml`:

```xml
  <export>
      <flexbe_behaviors />
      <build_type>ament_cmake</build_type>
  </export>
```

  * `example_behavior_sm.py`
    * Most basic example state machine

  * `example_action_behavior_sm.py` 
    * Uses the `ExampleActionState` with the standard action tutorials 

        [Understanding ROS2 Actions](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

        [Introducing Turtlesim](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
        
        To execute the associated behavior in FlexBE, you need to first run the turtlesim node that provdes the action server

        `ros2 run turtlesim turtlesim_node`
        
        To display the available actions:

        `ros2 action list`
        
        The action is defined by:

        `/turtle1/rotate_absolute:` [`turtlesim/action/RotateAbsolute`](https://docs.ros2.org/latest/api/turtlesim/action/RotateAbsolute.html)

Behaviors typically edited and generated by the FlexBE UI.  
These generated files are stored in the root workspace `install` folder.
Presuming a `WORKSPACE_ROOT` environment variable exists, we provide a simple 
[`copy_behavior`](cgn_flexbe_behaviors/bin/copy_behavior) script to copy a saved behavior 
&mdash; both the Python implementation and manifest `.xml` file &mdash; 
to the project source folder for long term storage.
Use `ros2 run cgn_flexbe_behavior copy_behavior` to see the usage guide. 
The script should be run from this repository's base folder.

For a Quick-start and more comprehensive introduction to FlexBE, 
see the [FlexBE Turtlesim Demonstrations](https://github.com/FlexBE/flexbe_turtlesim_demo).

# cgn_flexbe
