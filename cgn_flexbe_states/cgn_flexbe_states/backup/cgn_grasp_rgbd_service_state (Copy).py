#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from contact_graspnet_ros2.srv import GetGrasps as SrvType
from contact_graspnet_ros2.msg import Grasps

import subprocess, os



class CGNGraspRGBDServiceState(EventState):
    """
    Calls the Contact-GraspNet `get_grasps_rgbd` service using only a precomputed
    scene name (RGBD pipeline).

    Assumes the server:
      - Expects `scene_name` to select results/<scene_name>.npy
      - Ignores `points` and `mask` fields (or they can be left empty)

    This matches your `grasp_executor_rgbd_server.py`, which uses:
        self.scene_name = request.scene_name
        np_path = f"results/{scene_name}.npy"
    and loads 'results/predictions_<scene_name>.npz'. :contentReference[oaicite:1]{index=1}

    ># scene_name              string                 CGN scene name (without .npy)
    <# grasp_target_poses      geometry_msgs/Pose[]   Grasp poses
    <# grasp_scores            float[]                Grasp scores
    <# grasp_samples           geometry_msgs/Point[]  Contact points
    <# grasp_object_ids        int[]                  Object IDs per grasp

    <= done                    Service call succeeded
    <= failed                  Service call failed or no grasps received
    """

    def __init__(self,
                 service_timeout: float = 10.0,
                 # service_name: str = '/get_grasps'):
                 service_name: str = '/get_grasps_rgbd'):
        super().__init__(
            outcomes=['done', 'failed'],
            input_keys=['scene_name', 'target_instance_id'],
            output_keys=['grasp_target_poses', 'grasp_scores', 'grasp_samples', 'grasp_object_ids']
        )

        self._service_timeout = float(service_timeout)
        self._service_name = str(service_name)

        self._srv = ProxyServiceCaller({self._service_name: SrvType})
        self._res = None
        self._had_error = False

    def on_enter(self, userdata):
        self._res = None
        self._had_error = False

        try:
            scene_name = str(userdata.scene_name)
        except Exception:
            scene_name = "scene_from_ucn"

        Logger.loginfo(f"[CGNGraspRGBDServiceState] Requesting grasps for scene '{scene_name}'")

        request = SrvType.Request()
        # Leave points/mask empty; server loads data from scene_name internally.
        request.points = []
        request.mask = []
        request.scene_name = scene_name

        # Wait for service to be available
        deadline = time.time() + self._service_timeout
        while time.time() < deadline:
            if self._srv.is_available(self._service_name):
                break
            time.sleep(0.2)

        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[CGNGraspRGBDServiceState] Service '{self._service_name}' not available after "
                          f"{self._service_timeout}s.")
            self._had_error = True
            return


        # Call service
        try:
            self._res = self._srv.call(self._service_name, request)

            cmd = [
                "python3",
                "/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/result_plotter.py",
            ]
            subprocess.check_call(cmd)

            Logger.loginfo(f"[CGNGraspRGBDServiceState] Called service '{self._service_name}'.")
        except Exception as e:
            Logger.logerr(f"[CGNGraspRGBDServiceState] Service call failed: {e}")
            self._had_error = True
            self._res = None

    def execute(self, userdata):
        if self._had_error or self._res is None:
            return 'failed'

        try:
            grasps = self._res.grasps  # type: Grasps

            poses = list(grasps.poses)
            scores = list(grasps.scores)
            samples = list(grasps.samples)
            object_ids = list(grasps.object_ids)

            target_id = getattr(userdata, 'target_instance_id', -1)
            try:
                target_id = int(target_id)
            except Exception:
                target_id = -1

            if target_id >= 0 and object_ids:
                keep = [i for i, oid in enumerate(object_ids) if int(oid) == target_id]
                if keep:
                    poses = [poses[i] for i in keep]
                    scores = [scores[i] for i in keep]
                    samples = [samples[i] for i in keep]
                    object_ids = [object_ids[i] for i in keep]
                    Logger.loginfo(
                        f"[CGNGraspRGBDServiceState] Filtered grasps to target_instance_id={target_id}: "
                        f"{len(keep)} poses kept."
                    )
                else:
                    Logger.logwarn(
                        f"[CGNGraspRGBDServiceState] No returned grasps had object_id={target_id}. "
                        f"Returned object_ids={sorted(set(int(x) for x in object_ids))}. "
                        "Keeping all returned grasps."
                    )

            userdata.grasp_target_poses = poses
            userdata.grasp_scores = scores
            userdata.grasp_samples = samples
            userdata.grasp_object_ids = object_ids

            Logger.loginfo(
                f"[CGNGraspRGBDServiceState] Received {len(grasps.poses)} grasp poses; "
                f"using {len(userdata.grasp_target_poses)} poses. "
                f"object_ids={sorted(set(int(x) for x in userdata.grasp_object_ids)) if userdata.grasp_object_ids else []}"
            )
        except Exception as e:
            Logger.logerr(f"[CGNGraspRGBDServiceState] Failed to copy result to userdata: {e}")
            return 'failed'

        if len(userdata.grasp_target_poses) == 0:
            Logger.logwarn("[CGNGraspRGBDServiceState] No grasps in response.")
            return 'failed'

        return 'done'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
