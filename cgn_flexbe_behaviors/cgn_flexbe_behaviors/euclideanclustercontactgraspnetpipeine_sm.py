#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Huajing Zhao
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define EuclideanClusterContactGraspnetPipeine.

A perception-to-action pipeline which employs cluster extraction in order to
cluster pickable objects from a
scene and choose a target for grasping, then contact-graspnet for grasp
planning and OMPL for manipulation via MoveIt

Created on Oct 22 2025
@author: Huajing Zhao
"""


from cgn_flexbe_states.cgn_grasp_cloud_service_state import CGNGraspCloudServiceState
from compare_flexbe_states.euclidean_clustering_service_state import EuclideanClusteringServiceState
from compare_flexbe_states.filter_by_indices_service_state import FilterByIndicesServiceState
from compare_flexbe_states.get_point_cloud_service_state import GetPointCloudServiceState
from compare_flexbe_states.move_to_pose_service_state import MoveToPoseServiceState
from compare_flexbe_states.publish_point_cloud_state import PublishPointCloudState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class EuclideanClusterContactGraspnetPipeineSM(Behavior):
    """
    Define EuclideanClusterContactGraspnetPipeine.

    A perception-to-action pipeline which employs cluster extraction in order to
    cluster pickable objects from a
    scene and choose a target for grasping, then contact-graspnet for grasp
    planning and OMPL for manipulation via MoveIt
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'EuclideanClusterContactGraspnetPipeine'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:1576 y:418, x:251 y:389
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['target_cluster_indexed', 'scene_pointcloud'])
        _state_machine.userdata.scene_pointcloud = 0
        _state_machine.userdata.camera_pose = 0
        _state_machine.userdata.target_cluster_indexed = 0
        _state_machine.userdata.obstacle_clusters_indexed = 0
        _state_machine.userdata.cluster_count = 0
        _state_machine.userdata.point_cloud_visual = 0
        _state_machine.userdata.camera_source = 0
        _state_machine.userdata.grasp_poses = []
        _state_machine.userdata.test_indices = []
        _state_machine.userdata.grasp_waypoints = []
        _state_machine.userdata.waypoint_index = 0
        _state_machine.userdata.grasp_target_poses = []
        _state_machine.userdata.grasp_index = 0
        _state_machine.userdata.ready_pose = 'ready'
        _state_machine.userdata.scene_id = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:119 y:63
            OperatableStateMachine.add('GetPointCloud',
                                       GetPointCloudServiceState(service_timeout=5.0,
                                                                 service_name='/get_point_cloud',
                                                                 camera_topic='/rgbd_camera/points',
                                                                 target_frame='simple_pedestal'),
                                       transitions={'finished': 'EuclideanClustering'  # 306 84 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 236 225 228 116 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

            # x:831 y:53
            OperatableStateMachine.add('CgnGrasp',
                                       CGNGraspCloudServiceState(service_name='/get_grasps',
                                                            use_scene_id=False,
                                                            service_timeout=5.0,
                                                            field_names=None),
                                       transitions={'done': 'PublishPointCloud',
                                                    'failed': 'failed'  # 688 269 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual',
                                                  'indices': 'test_indices',
                                                  'scene_id': 'scene_id',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_scores': 'grasp_scores',
                                                  'grasp_samples': 'grasp_samples',
                                                  'grasp_object_ids': 'grasp_object_ids'})

            # x:344 y:61
            OperatableStateMachine.add('EuclideanClustering',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'FilterByIndices'  # 557 80 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 400 213 406 114 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:600 y:56
            OperatableStateMachine.add('FilterByIndices',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'CgnGrasp',
                                                    'failed': 'failed'  # 648 229 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'point_cloud_visual'})

            # x:1316 y:56
            OperatableStateMachine.add('MoveOMPL',
                                       MoveToPoseServiceState(timeout_sec=5.0,
                                                              service_name='/move_to_pose'),
                                       transitions={'done': 'finished'  # 1455 349 -1 -1 -1 -1
                                                    , 'next': 'MoveOMPL'  # 1376 147 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1142 394 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_poses': 'grasp_target_poses',
                                                  'grasp_index': 'grasp_index'})

            # x:1055 y:53
            OperatableStateMachine.add('PublishPointCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'MoveOMPL',
                                                    'failed': 'failed'  # 1099 242 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
