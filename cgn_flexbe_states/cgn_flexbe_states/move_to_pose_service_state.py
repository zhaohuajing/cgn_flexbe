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

import rclpy
from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from compare_flexbe_utilities.srv import MoveToPose as SrvType
from geometry_msgs.msg import Pose

class MoveToPoseServiceState(EventState):
    """
    Calls a service to move the robot to a specific pose using the he setPoseTarget, plan and execute C++ functions wrapped into a service server.

    -- timeout_sec       float          Timeout for waiting for service (default: 5.0)
    -- service_name      str            Service name (default: '/move_to_pose')

    ># grasp_poses       list           A list of geometry_msgs/Pose messages to choose from (uses the last one)

    <= finished                         Service call succeeded
    <= failure                          Service call failed or pose was invalid
    """

    def __init__(self, timeout_sec=5.0, service_name='/move_to_pose'):
        super().__init__(outcomes=['done', 'next', 'failed'],
                            input_keys=['grasp_poses', 'grasp_index'],
                            output_keys=['grasp_index']
        )
        self._timeout_sec = timeout_sec
        self._service_name = service_name
        self._client = None

        # Create proxy service caller to handle rclpy node
        self._srv = ProxyServiceCaller({self._service_name: SrvType})

        # result storage
        self._res = None
        self._had_error = False

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # Check for error or no response
        if self._had_error or self._res is None:
            return 'failed'

        try:
            Logger.loginfo(f"[{type(self).__name__}] Finished plan with result: {self._res.success}.")
            # if self._res.success == 1:
            if self._res.success == 1 and userdata.grasp_index >= 10:
                return 'done'
            if (userdata.grasp_index + 1) < len(userdata.grasp_poses):
                userdata.grasp_index = userdata.grasp_index + 1
                return 'next'
            else:
                # This was the last set of waypoints
                return 'failed'
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {str(e)}")
            return 'failed'

        # Return outcome failed if no other outcomes occur
        return 'failed'
    
    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # reset state
        self._res = None
        self._had_error = False

        # check for correct data
        grasp_poses = userdata.grasp_poses
        idx = userdata.grasp_index

        # check for correct data
        if not isinstance(grasp_poses, list) or not isinstance(idx, int) or idx < 0 or idx >= len(grasp_poses):
            Logger.logerr(f"[{type(self).__name__}] grasp_index {idx} out of range for {len(grasp_poses)} sets or invalid or missing data type in userdata.")
            return

        # construct request
        request = SrvType.Request()
        request.target_pose = grasp_poses[idx]

        # wait for availability (once per entry)
        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[{type(self).__name__}] Service '{self._service_name}' not available after {self._service_timeout}s.")
            self._had_error = True
            return

        # send request
        try:
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(f"[{type(self).__name__}] Called service '{self._service_name}'.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")

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