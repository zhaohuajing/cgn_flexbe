#!/usr/bin/env python3

import rclpy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

# Use the same srv type as move_to_pose
from compare_flexbe_utilities.srv import MoveToPose as SrvType
from geometry_msgs.msg import Pose


class ReachToGraspServiceState(EventState):
    """
    Call /reach_to_grasp (MoveToPose.srv) to execute the grasp sequence:
      - open gripper
      - move along EE-Z
      - close gripper
      - lift in base-Z
      - (optionally) open again, etc.

    We pass the selected grasp pose in the request as `target_pose`.

    -- service_name      string   Absolute service name (default: '/reach_to_grasp')
    -- timeout_sec       float    [currently not used by ProxyServiceCaller] (default: 10.0)

    ># grasp_poses       list     List[geometry_msgs/Pose] of candidate grasp poses
    ># grasp_index       int      Index of the grasp pose to use

    <= done                      Sequence executed successfully
    <= failed                    Service call failed or service reported failure
    """

    def __init__(self,
                 service_name='/reach_to_grasp',
                 timeout_sec=10.0):
        super().__init__(
            outcomes=['done', 'failed'],
            input_keys=['grasp_poses', 'grasp_index'],
            output_keys=['grasp_index'])

        self._service_name = service_name
        self._timeout_sec = timeout_sec

        # Do not block at construction time; service may not exist yet
        self._srv = ProxyServiceCaller({self._service_name: SrvType},
                                       wait_duration=0.0)

        self._res = None
        self._failed = False

    def on_enter(self, userdata):
        self._failed = False
        self._res = None

        Logger.loginfo(f"[{type(self).__name__}] Preparing to call service "
                       f"'{self._service_name}'")

        # Validate userdata
        grasp_poses = userdata.grasp_poses
        idx = userdata.grasp_index

        if (not isinstance(grasp_poses, list) or
                not isinstance(idx, int) or
                idx < 0 or
                idx >= len(grasp_poses)):
            Logger.logerr(
                f"[{type(self).__name__}] grasp_index {idx} out of range for "
                f"{len(grasp_poses)} poses or invalid userdata."
            )
            self._failed = True
            return

        request = SrvType.Request()
        request.target_pose = grasp_poses[idx]

        if not self._srv.is_available(self._service_name):
            Logger.logwarn(
                f"[{type(self).__name__}] Service '{self._service_name}' not "
                f"reported available yet, calling anyway."
            )

        try:
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(
                f"[{type(self).__name__}] Called service '{self._service_name}'.")
        except Exception as e:
            Logger.logerr(
                f"[{type(self).__name__}] Service call to "
                f"'{self._service_name}' failed: {e}"
            )
            self._failed = True

    def execute(self, userdata):
        if self._failed or self._res is None:
            return 'failed'

        # MoveToPose.srv uses an integer success flag (usually 1 == success)
        try:
            if getattr(self._res, 'success', 0) == 1:
                Logger.loginfo(
                    f"[{type(self).__name__}] Reach-to-grasp OK: "
                    f"{getattr(self._res, 'message', '')}"
                )
                return 'done'
            else:
                Logger.logwarn(
                    f"[{type(self).__name__}] Reach-to-grasp failed: "
                    f"{getattr(self._res, 'message', '')}"
                )
                return 'failed'
        except Exception as e:
            Logger.logerr(
                f"[{type(self).__name__}] Error checking response: {e}")
            return 'failed'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
