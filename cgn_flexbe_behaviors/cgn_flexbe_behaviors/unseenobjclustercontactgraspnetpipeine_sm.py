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
Define UnseenObjClusterContactGraspnetPipeine.

A perception-to-action pipeline which employs unseen-object-clustering for
object segmentation from a
scene and choose a target for grasping, then contact-graspnet for grasp
planning and OMPL for manipulation via MoveIt

Created on Dec 07 2025
@author: Huajing Zhao
"""


from cgn_flexbe_states.cgn_grasp_rgbd_service_state import CGNGraspRGBDServiceState
from cgn_flexbe_states.move_to_pose_service_state import MoveToPoseServiceState
from cgn_flexbe_states.select_instance_to_cgn_indices_state import SelectInstanceToSceneNameState
from uoc_flexbe_states.unseen_obj_seg_rgbd_service_state import UnseenObjSegRGBDServiceState
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


class UnseenObjClusterContactGraspnetPipeineSM(Behavior):
    """
    Define UnseenObjClusterContactGraspnetPipeine.

    A perception-to-action pipeline which employs unseen-object-clustering for
    object segmentation from a
    scene and choose a target for grasping, then contact-graspnet for grasp
    planning and OMPL for manipulation via MoveIt
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'UnseenObjClusterContactGraspnetPipeine'

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
        # x:1154 y:332, x:141 y:356
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['im_name'])
        _state_machine.userdata.im_name = 'from_rgbd'
        _state_machine.userdata.seg_json = {}
        _state_machine.userdata.result_dir = ''
        _state_machine.userdata.instance_ids_2d = []
        _state_machine.userdata.instance_id_list = []
        _state_machine.userdata.target_instance_id = 0
        _state_machine.userdata.scene_name = 'scene_from_ucn'
        _state_machine.userdata.message = ''
        _state_machine.userdata.grasp_target_poses = []
        _state_machine.userdata.grasp_scores = []
        _state_machine.userdata.grasp_samples = []
        _state_machine.userdata.grasp_object_ids = []
        _state_machine.userdata.grasp_index = 0
        _state_machine.userdata.manual_target_instance_id = -1

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('UnseenObjSegRGBD',
                                       UnseenObjSegRGBDServiceState(service_name='/segmentation_rgbd',
                                                                    service_timeout=5.0,
                                                                    default_im_name='from_rgbd',
                                                                    background_id=0),
                                       transitions={'finished': 'SelectInstanceToScene',
                                                    'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'im_name': 'im_name',
                                                  'seg_json': 'seg_json',
                                                  'result_dir': 'result_dir',
                                                  'instance_ids_2d': 'instance_ids_2d',
                                                  'instance_id_list': 'instance_id_list',
                                                  'instance_masks': 'instance_masks',
                                                  'message': 'message'})

            # x:762 y:41
            OperatableStateMachine.add('CgnGraspRGBD',
                                       CGNGraspRGBDServiceState(service_timeout=20.0,
                                                                service_name='/get_grasps_rgbd'),
                                       transitions={'done': 'MoveOMPL', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'scene_name': 'scene_name',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_scores': 'grasp_scores',
                                                  'grasp_samples': 'grasp_samples',
                                                  'grasp_object_ids': 'grasp_object_ids'})

            # x:1087 y:38
            OperatableStateMachine.add('MoveOMPL',
                                       MoveToPoseServiceState(timeout_sec=5.0,
                                                              service_name='/move_to_pose'),
                                       transitions={'done': 'finished',
                                                    'next': 'MoveOMPL',
                                                    'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_poses': 'grasp_target_poses',
                                                  'grasp_index': 'grasp_index'})

            # x:419 y:38
            OperatableStateMachine.add('SelectInstanceToScene',
                                       SelectInstanceToSceneNameState(default_scene_name='scene_from_ucn',
                                                                     selection_mode='manual'),
                                       transitions={'finished': 'CgnGraspRGBD', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'seg_json': 'seg_json',
                                                  'result_dir': 'result_dir',
                                                  'instance_ids_2d': 'instance_ids_2d',
                                                  'instance_id_list': 'instance_id_list',
                                                  'im_name': 'im_name',
                                                  'target_instance_id': 'target_instance_id',
                                                  'scene_name': 'scene_name',
                                                  'manual_target_instance_id': 'manual_target_instance_id',
                                                  'message': 'message'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
