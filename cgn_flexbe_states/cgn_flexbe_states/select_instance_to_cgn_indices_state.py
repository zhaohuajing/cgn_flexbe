#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from flexbe_core import EventState, Logger
import subprocess
import numpy as np

class SelectInstanceToSceneNameState(EventState):
    def __init__(self,
                 default_scene_name: str = 'scene_from_ucn',
                 selection_mode: str = 'manual',  # 'largest' | 'manual' | 'largest_or_manual'
                 allow_background: bool = False,
                 manual_sentinel: int = -1):
        super().__init__(
            outcomes=['finished', 'failed'],
            input_keys=[
                'seg_json', 'result_dir', 'instance_ids_2d', 'instance_id_list', 'im_name',
                'manual_target_instance_id'   # NEW
            ],
            output_keys=['target_instance_id', 'scene_name', 'message']
        )
        self._default_scene_name = str(default_scene_name)
        self._selection_mode = str(selection_mode).lower().strip()
        self._allow_background = bool(allow_background)
        self._manual_sentinel = int(manual_sentinel)

        self._had_error = False
        self._target_id = None
        self._msg = ""

    def _pick_largest(self, instance_ids, instance_ids_2d):
        best_id = None
        best_area = -1
        areas = {}

        for inst_id in instance_ids:
            mask = (instance_ids_2d == inst_id)
            area = int(mask.sum())
            areas[int(inst_id)] = area
            Logger.loginfo(f"[SelectInstanceToSceneNameState] Instance {inst_id} has area {area}.")
            if area > best_area:
                best_area = area
                best_id = inst_id

        return (None, None, areas) if best_id is None else (int(best_id), int(best_area), areas)

    def _get_manual_id(self, userdata):
        # Accept None, missing, sentinel => "not provided"
        if not hasattr(userdata, 'manual_target_instance_id'):
            return None
        v = userdata.manual_target_instance_id
        if v is None:
            return None
        try:
            v = int(v)
        except Exception:
            return None
        if v == self._manual_sentinel:
            return None
        return v

    def on_enter(self, userdata):
        self._had_error = False
        self._target_id = None
        self._msg = ""

        try:
            seg = userdata.seg_json
            if isinstance(seg, str):
                seg = json.loads(seg)

            instance_ids = list(userdata.instance_id_list) if userdata.instance_id_list else []
            if not self._allow_background:
                instance_ids = [i for i in instance_ids if int(i) != 0]

            if not instance_ids:
                self._msg = "[SelectInstanceToSceneNameState] No instance ids found."
                Logger.logwarn(self._msg)
                self._had_error = True
                return

            instance_ids_2d = np.array(userdata.instance_ids_2d, dtype=np.int32)

            best_id, best_area, areas = self._pick_largest(instance_ids, instance_ids_2d)
            if best_id is None or best_area is None or best_area <= 0:
                self._msg = "[SelectInstanceToSceneNameState] Failed to find a non-empty instance mask."
                Logger.logwarn(self._msg)
                self._had_error = True
                return

            # NEW: manual id from userdata
            manual_id = self._get_manual_id(userdata)

            # Decide
            if self._selection_mode == 'largest':
                chosen_id = best_id
            elif self._selection_mode == 'manual':
                if manual_id is None:
                    self._msg = ("[SelectInstanceToSceneNameState] selection_mode='manual' but "
                                 "manual_target_instance_id was not provided.")
                    Logger.logwarn(self._msg)
                    self._had_error = True
                    return
                chosen_id = manual_id
            else:  # 'largest_or_manual' default
                chosen_id = manual_id if manual_id is not None else best_id

            # Validate chosen_id
            valid_ids = [int(x) for x in instance_ids]
            if int(chosen_id) not in valid_ids:
                self._msg = (f"[SelectInstanceToSceneNameState] Chosen instance id {chosen_id} "
                             f"is not in instance_id_list {valid_ids}.")
                Logger.logwarn(self._msg)
                self._had_error = True
                return

            chosen_area = areas.get(int(chosen_id), -1)
            self._target_id = int(chosen_id)
            self._msg = (f"[SelectInstanceToSceneNameState] Selected instance {self._target_id} "
                         f"(area={chosen_area}) → scene_name='{self._default_scene_name}'")
            Logger.loginfo(self._msg)

            cmd = [
                "python3",
                "/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/test_data/ucn_to_cgn_scene.py",
                # IMPORTANT: if your script supports args, you should pass them here:
                # "--im_name", userdata.im_name,
                # "--seg_dir", userdata.result_dir,
                # "--scene_name", self._default_scene_name,
                # "--target_id", str(self._target_id),
            ]
            subprocess.check_call(cmd)

            Logger.loginfo("[SelectInstanceToSceneNameState] Generated contact_graspnet/test_data/scene_from_ucn.npy using ucn_to_cgn_scene.py")

        except Exception as e:
            self._msg = f"[SelectInstanceToSceneNameState] Exception: {e}"
            Logger.logerr(self._msg)
            self._had_error = True

    def execute(self, userdata):
        if self._had_error:
            userdata.message = self._msg
            return 'failed'

        userdata.target_instance_id = self._target_id
        userdata.scene_name = self._default_scene_name
        userdata.message = self._msg
        return 'finished'