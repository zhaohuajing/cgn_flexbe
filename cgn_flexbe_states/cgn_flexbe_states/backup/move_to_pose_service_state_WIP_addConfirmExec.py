#!/usr/bin/env python3

# Copyright 2023 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
FlexBE state for calling /move_to_pose.

This version intentionally creates the ProxyServiceCaller lazily in on_enter()
instead of in __init__().  That avoids FlexBE's behavior-preparation stage
emitting the default 1-second service-availability warning before all robot
servers have finished launching.
"""

import time

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from cgn_flexbe_utilities.srv import MoveToPose as SrvType


class MoveToPoseServiceState(EventState):
    """
    Calls a service to move the robot to a specific pose using MoveIt.

    -- timeout_sec       float          Timeout for waiting for service (default: 5.0)
    -- service_name      str            Service name (default: '/move_to_pose')

    ># grasp_poses       list           List of geometry_msgs/Pose messages
    ># grasp_index       int            Index of the pose to try

    #> grasp_index       int            Updated index when trying next pose

    <= done                             Service call succeeded
    <= next                             Current pose failed/rejected; try next pose
    <= failed                           Service unavailable, invalid data, or all poses failed
    """

    def __init__(self, timeout_sec=5.0, service_name='/move_to_pose'):
        super().__init__(
            outcomes=['done', 'next', 'failed'],
            input_keys=['grasp_poses', 'grasp_index'],
            output_keys=['grasp_index']
        )
        self._timeout_sec = float(timeout_sec)
        self._service_name = str(service_name)

        # IMPORTANT: create this lazily in on_enter().  Creating it in __init__()
        # lets FlexBE check the service during behavior preparation with its
        # default 1-second wait, which causes a confusing warning if MoveIt is
        # still starting.
        self._srv = None

        self._res = None
        self._had_error = False

    def _ensure_service_caller(self):
        if self._srv is None:
            self._srv = ProxyServiceCaller({self._service_name: SrvType})

    def _wait_for_service(self):
        self._ensure_service_caller()
        start = time.time()
        last_log = 0.0

        while time.time() - start < self._timeout_sec:
            try:
                if self._srv.is_available(self._service_name):
                    return True
            except Exception:
                # ProxyServiceCaller may raise while ROS graph is still updating.
                pass

            now = time.time()
            if now - last_log > 1.0:
                remaining = max(0.0, self._timeout_sec - (now - start))
                Logger.loginfo(
                    f"[{type(self).__name__}] Waiting for service '{self._service_name}' "
                    f"({remaining:.1f}s remaining)..."
                )
                last_log = now
            time.sleep(0.2)

        return False

    def on_enter(self, userdata):
        self._res = None
        self._had_error = False

        grasp_poses = userdata.grasp_poses
        idx = userdata.grasp_index

        if not isinstance(grasp_poses, list) or not isinstance(idx, int) or idx < 0 or idx >= len(grasp_poses):
            Logger.logerr(
                f"[{type(self).__name__}] grasp_index {idx} out of range for "
                f"{len(grasp_poses) if isinstance(grasp_poses, list) else 'invalid'} poses "
                "or invalid/missing userdata type."
            )
            self._had_error = True
            return

        if not self._wait_for_service():
            Logger.logerr(
                f"[{type(self).__name__}] Service '{self._service_name}' not available "
                f"after {self._timeout_sec:.1f}s."
            )
            self._had_error = True
            return

        request = SrvType.Request()
        request.target_pose = grasp_poses[idx]

        try:
            Logger.loginfo(
                f"[{type(self).__name__}] Calling service '{self._service_name}' "
                f"with grasp_index={idx}."
            )
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(f"[{type(self).__name__}] Service '{self._service_name}' returned.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")
            self._had_error = True

    def execute(self, userdata):
        if self._had_error or self._res is None:
            return 'failed'

        try:
            Logger.loginfo(f"[{type(self).__name__}] Finished plan with result: {self._res.success}.")
            if self._res.success == 1:
                return 'done'

            if (userdata.grasp_index + 1) < len(userdata.grasp_poses):
                userdata.grasp_index = userdata.grasp_index + 1
                return 'next'

            return 'failed'
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed while evaluating response: {e}")
            return 'failed'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
