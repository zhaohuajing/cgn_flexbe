#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from typing import Iterable, Optional

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64

# Read PCL points from PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# Contact-GraspNet ROS2 service & msg
from contact_graspnet_ros2.srv import GetGrasps as SrvType
from contact_graspnet_ros2.msg import Grasps


class CGNGraspServiceState(EventState):
    """
    Calls the Contact-GraspNet `get_grasps` service and returns grasp poses.

    -- service_timeout   float     Timeout for service discovery (sec, default: 5.0)
    -- service_name      str       Service name (default: '/get_grasps')
    -- use_scene_name      bool      If True, only send `scene_name` (server loads NPZ/JSON internally)
    -- field_names       list[str] Point fields to read from PointCloud2 (default: ['x','y','z'])

    ># cloud_in                    sensor_msgs/PointCloud2   Input (filtered) cloud of the target object
    ># scene_name                    int                       Optional: server-side dataset id (if using offline results)
    ># indices                     list[int] or None         Optional: indices mask (1 = keep) to match CGN 'mask'
    <# grasp_target_poses          geometry_msgs/Pose[]      Target pose list from CGN
    <# grasp_scores                float[]                   Confidence scores
    <# grasp_samples               geometry_msgs/Point[]     Contact points
    <# grasp_object_ids            int[]                     Object ids (per-grasp)

    <= done                        Service call succeeded
    <= failed                      Service call failed or timed out
    """

    def __init__(self,
                 service_timeout: float = 5.0,
                 service_name: str = '/get_grasps',
                 use_scene_name: bool = False,
                 field_names: Optional[list] = None,
                 z_min: float = 0.28):
        super().__init__(
            outcomes=['done', 'failed'],
            input_keys=['cloud_in', 'indices'], #, 'scene_name'],
            output_keys=['grasp_target_poses', 'grasp_scores', 'grasp_samples', 'grasp_object_ids']
        )
        self._service_timeout = float(service_timeout)
        self._service_name = str(service_name)
        self._use_scene_name = bool(use_scene_name)
        self._fields = field_names if field_names is not None else ['x', 'y', 'z']
        self._z_min = float(z_min)

        # Proxy service
        self._srv = ProxyServiceCaller({self._service_name: SrvType})

        # Result bookkeeping
        self._res = None
        self._had_error = False

    # ---------------------------- helpers ----------------------------

    @staticmethod
    def _ensure_int_list(seq: Iterable) -> list[int]:
        return [int(v) for v in seq]

    @staticmethod
    def _ensure_uint32_list(seq: Iterable) -> list[int]:
        # Python int is unbounded, service will coerce to uint32
        return [int(v) & 0xFFFFFFFF for v in seq]

    @staticmethod
    def _pc2_to_xyz_list(cloud: PointCloud2, fields: list[str]) -> list[float]:
        """Returns flattened [x0, y0, z0, x1, y1, z1, ...] in float32-like Python floats."""
        # Read generator of tuples (x,y,z,...) using only requested fields
        pts_iter = pc2.read_points(cloud, field_names=fields, skip_nans=True)
        flat = []
        for t in pts_iter:
            # t is (x, y, z) when fields=['x','y','z']
            flat.extend((float(t[0]), float(t[1]), float(t[2])))
        return flat

    # ---------------------------- lifecycle ----------------------------

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # Check for error or no response
        if self._had_error or self._res is None:
            return 'failed'

        try:
            grasps = self._res.grasps
            userdata.grasp_target_poses = list(grasps.poses)
            userdata.grasp_scores = list(grasps.scores)
            userdata.grasp_samples = list(grasps.samples)
            userdata.grasp_object_ids = list(grasps.object_ids)
            Logger.loginfo(f"[{type(self).__name__}] Received {len(grasps.poses)} CGN grasp poses.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to copy result to userdata: {e}")
            return 'failed'

        return 'done'

    def on_enter(self, userdata):
        self._res = None
        self._had_error = False

        # Build request
        request = SrvType.Request()

        # If the server supports loading precomputed NPZ/JSON by scene id,
        # you can skip sending points/mask and just pass scene_name.
        # if self._use_scene_name:
        #     try:
        #         request.scene_name = int(userdata.scene_name)
        #     except Exception:
        #         request.scene_name = 0  # default fallback
        # else:
        # Prepare points from PointCloud2
        try:
            cloud = userdata.cloud_in
            request.points = self._pc2_to_xyz_list(cloud, self._fields)

            # Optional indices -> boolean-ish mask (uint32). If no indices are given, keep-all.
            if userdata.indices:
                # Make a dense keep-mask of size N; indices mark the ones to keep.
                # Since we used skip_nans=True and projected to a *filtered* cloud already,
                # we simply mark all as 1 unless a sparse index list is explicitly provided.
                # If you want exact alignment with original cloud, pre-filter before this state.
                idx = self._ensure_int_list(userdata.indices)
                # We'll just pass a trivial same-length mask of ones; the server ignores 0s anyway
                # if you pre-filtered the cloud to target object.
                # (If you truly need sparse selection, pass a binary mask the same length as points/3.)
            # Default: "keep all"
            # # (mask length in server is expected to match number of points; 1 = keep)
            # num_pts = len(request.points) // 3
            # request.mask = [1] * num_pts

            # Build Z > z_min mask (1 = keep)
            num_pts = len(request.points) // 3
            if num_pts == 0:
                raise RuntimeError("Input cloud produced zero points")
            Logger.loginfo(f"[{type(self).__name__}] num_pts = {num_pts}")

            # z values live at indices 2, 5, 8, ...
            z_vals = (request.points[2::3])
            z_mask = [1 if float(z) > self._z_min else 0 for z in z_vals]

            # Optional indices -> combine with z_mask (AND)
            # If indices is a list of positions to keep, turn into a dense mask.
            if hasattr(userdata, 'indices') and userdata.indices:
                idx_list = self._ensure_int_list(userdata.indices)
                idx_mask = [0] * num_pts
                for i in idx_list:
                    if 0 <= i < num_pts:
                        idx_mask[i] = 1
                # AND-combine: keep only those above the table AND in indices
                request.mask = [1 if (z_mask[i] and idx_mask[i]) else 0 for i in range(num_pts)]
            else:
                # Only Z-filter
                request.mask = z_mask

            kept = sum(request.mask)
            Logger.loginfo(f"[{type(self).__name__}] Z-filter: z_min={self._z_min:.3f}m "
                           f"→ kept {kept}/{num_pts} points")

            if kept == 0:
                Logger.logwarn(f"[{type(self).__name__}] No points above z_min={self._z_min:.3f}m; aborting.")
                self._had_error = True
                return

        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to prepare request from cloud: {e}")
            self._had_error = True
            return

        # # Optional scene id (used by your server for bookkeeping)
        # try:
        #     request.scene_name = int(userdata.scene_name)
        # except Exception:
        #     request.scene_name = 0


        # inside on_enter(), before using self._srv:
        import time
        deadline = time.time() + self._service_timeout
        while time.time() < deadline:
            if self._srv.is_available(self._service_name):
                break
            time.sleep(0.2)

        # Wait for service
        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[{type(self).__name__}] Service '{self._service_name}' not available after {self._service_timeout}s.")
            self._had_error = True
            return

        # Call service
        try:
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(f"[{type(self).__name__}] Called service '{self._service_name}'.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")
            self._had_error = True
            self._res = None


    def on_exit(self, userdata):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # No-op: template hook
        pass

    def on_start(self):
        # Call this method when the behavior is instantiated on board.
        # If possible, it is generally better to initialize used resources in the constructor
        #   because if anything failed, the behavior would not even be started.

        # No-op: template hook
        pass

    def on_stop(self):
        # Call this method whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # No-op: template hook
        pass
