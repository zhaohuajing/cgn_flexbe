#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import subprocess

import numpy as np
from flexbe_core import EventState, Logger


class SelectInstanceToSceneNameState(EventState):
    """
    Build a Contact-GraspNet scene from the latest UOC RGB-D result using a
    target object id already chosen by an upstream state.

    This state is still CGN-specific because it creates test_data/<scene>.npy.
    It no longer performs human selection. For a manual pipeline, put
    ManualSelectObjectIDState before this state and remap its target_instance_id
    into this state's target_instance_id input.
    """

    def __init__(self,
                 default_scene_name: str = 'scene_from_ucn',
                 selection_mode: str = 'manual',  # 'manual' | 'largest' | 'largest_or_manual'
                 allow_background: bool = False,
                 manual_sentinel: int = -1,
                 converter_script: str = '/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/ucn_to_cgn_scene.py',
                 cgn_test_data_dir: str = '/home/csrobot/graspnet_ws/src/contact_graspnet_ros2/contact_graspnet/results',
                 filter_scene_to_selected: bool = True):
        super().__init__(
            outcomes=['finished', 'failed'],
            input_keys=[
                'seg_json',
                'result_dir',
                'instance_ids_2d',
                'instance_id_list',
                'im_name',
                'target_instance_id',
            ],
            output_keys=['target_instance_id', 'scene_name', 'message']
        )
        self._default_scene_name = str(default_scene_name)
        self._selection_mode = str(selection_mode).lower().strip()
        self._allow_background = bool(allow_background)
        self._manual_sentinel = int(manual_sentinel)
        self._converter_script = os.path.expanduser(str(converter_script))
        self._cgn_test_data_dir = os.path.expanduser(str(cgn_test_data_dir))
        self._filter_scene_to_selected = bool(filter_scene_to_selected)

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
        if not hasattr(userdata, 'target_instance_id'):
            return None
        v = userdata.target_instance_id
        if v is None:
            return None
        try:
            v = int(v)
        except Exception:
            return None
        if v == self._manual_sentinel:
            return None
        return v

    def _run_ucn_to_cgn_scene(self):
        if not os.path.exists(self._converter_script):
            raise RuntimeError(f"CGN scene converter not found: {self._converter_script}")

        # Keep this no-argument call compatible with your existing script.
        # The selected object filtering is applied to the generated .npy below.
        cmd = ["python3", self._converter_script]
        env = os.environ.copy()
        env["PYTHONNOUSERSITE"] = "1"
        env.setdefault("MPLBACKEND", "Agg")

        Logger.loginfo(
            "[SelectInstanceToSceneNameState] Running UOC->CGN scene converter:\n"
            f"  {' '.join(cmd)}"
        )
        subprocess.check_call(cmd, env=env)

    def _filter_scene_npy_to_target(self, target_id: int):
        scene_path = os.path.join(self._cgn_test_data_dir, f"{self._default_scene_name}.npy")
        if not os.path.exists(scene_path):
            raise RuntimeError(f"CGN scene file was not generated: {scene_path}")

        data = np.load(scene_path, allow_pickle=True)
        if isinstance(data, np.ndarray) and data.shape == () and data.dtype == object:
            scene = data.item()
        elif isinstance(data, dict):
            scene = data
        else:
            raise RuntimeError(
                f"Unsupported CGN scene .npy format in {scene_path}: "
                f"type={type(data)}, shape={getattr(data, 'shape', None)}, dtype={getattr(data, 'dtype', None)}"
            )

        seg_key = None
        for key in ('seg', 'segmap', 'mask', 'labels', 'instance_ids'):
            if key in scene:
                seg_key = key
                break
        if seg_key is None:
            raise RuntimeError(
                f"CGN scene file {scene_path} has no segmentation key. "
                f"Available keys: {list(scene.keys())}"
            )

        seg = np.asarray(scene[seg_key])
        unique_before = np.unique(seg)
        selected_mask = (seg.astype(np.int64) == int(target_id))
        selected_area = int(selected_mask.sum())
        if selected_area <= 0:
            raise RuntimeError(
                f"Selected target id {target_id} has zero pixels in generated CGN scene. "
                f"Scene unique ids before filtering: {unique_before.tolist()}"
            )

        # Preserve the original target id instead of renumbering to 1 so the
        # returned CGN object_ids remain comparable to the UOC instance id.
        filtered = np.zeros_like(seg)
        filtered[selected_mask] = np.asarray(target_id, dtype=seg.dtype)
        scene[seg_key] = filtered.astype(seg.dtype, copy=False)

        np.save(scene_path, scene)
        unique_after = np.unique(scene[seg_key])
        Logger.loginfo(
            f"[SelectInstanceToSceneNameState] Filtered {scene_path} to selected object id {target_id}. "
            f"selected_area={selected_area}, unique ids before={unique_before.tolist()}, "
            f"after={unique_after.tolist()}"
        )

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

            manual_id = self._get_manual_id(userdata)

            if self._selection_mode == 'largest':
                chosen_id = best_id
            elif self._selection_mode == 'manual':
                if manual_id is None:
                    self._msg = (
                        "[SelectInstanceToSceneNameState] selection_mode='manual' but "
                        "target_instance_id was not provided."
                    )
                    Logger.logwarn(self._msg)
                    self._had_error = True
                    return
                chosen_id = manual_id
            else:  # 'largest_or_manual'
                chosen_id = manual_id if manual_id is not None else best_id

            valid_ids = [int(x) for x in instance_ids]
            if int(chosen_id) not in valid_ids:
                self._msg = (
                    f"[SelectInstanceToSceneNameState] Chosen instance id {chosen_id} "
                    f"is not in instance_id_list {valid_ids}."
                )
                Logger.logwarn(self._msg)
                self._had_error = True
                return

            chosen_area = areas.get(int(chosen_id), -1)
            self._target_id = int(chosen_id)
            self._msg = (
                f"[SelectInstanceToSceneNameState] Using target instance {self._target_id} "
                f"(area={chosen_area}) → scene_name='{self._default_scene_name}'"
            )
            Logger.loginfo(self._msg)

            self._run_ucn_to_cgn_scene()

            if self._filter_scene_to_selected:
                self._filter_scene_npy_to_target(self._target_id)

            Logger.loginfo(
                "[SelectInstanceToSceneNameState] Generated filtered "
                f"{self._cgn_test_data_dir}/{self._default_scene_name}.npy"
            )

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
