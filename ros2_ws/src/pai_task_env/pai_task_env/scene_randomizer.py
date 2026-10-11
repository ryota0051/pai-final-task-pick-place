"""Gazebo のシーンをランダム化する（物体色・机の色味・照明・ダミーの箱）. 検出器の学習データ生成と方策評価で共用（ADR 0008）.

Gazebo（Ignition Fortress）のサービスを ign コマンドで呼ぶ。1 回 0.3 秒程度かかるので、変更はまとめて並列に送る。
ダミーの箱は実行中に大きさを変えられないため、大きさ違いを最初に作って使い回す（位置と色だけ変える）。
箱は見た目だけで当たり判定はない。
"""
from __future__ import annotations

import json
import math
import random
import re
import subprocess
from dataclasses import replace
from typing import List, Mapping, Sequence, Tuple

from pai_task_env.domain_randomization import SceneParams, SceneSampler

BASE_Z = 1.015           # world → crane_plus_base（task.yaml の base_origin と一致）
OFFSCREEN = (2.0, 2.0)   # 使わない箱の置き場所（カメラ視野外, world）
DEFAULT_OBJECT_RGB = (0.9, 0.1, 0.1)     # worlds/pick_place.sdf の target_cube
DEFAULT_LIGHT = (0.8, (-0.5, 0.1, -0.9))  # Fuel の Sun モデルの diffuse と direction


def ign(service: str, reqtype: str, req: str, reptype: str = 'ignition.msgs.Boolean', timeout: int = 3000) -> str:
    return subprocess.run(['ign', 'service', '-s', service, '--reqtype', reqtype, '--reptype', reptype,
                           '--timeout', str(timeout), '--req', req], capture_output=True, text=True).stdout


def ign_parallel(calls) -> None:
    """(service, reqtype, req) のリストを並列に呼ぶ."""
    procs = [subprocess.Popen(['ign', 'service', '-s', s, '--reqtype', t, '--reptype', 'ignition.msgs.Boolean',
                               '--timeout', '3000', '--req', r], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
             for s, t, r in calls]
    for proc in procs:
        proc.wait()


def color(rgb) -> str:
    r, g, b = rgb
    return f'{{r: {r:.3f}, g: {g:.3f}, b: {b:.3f}, a: 1}}'


class SceneRandomizer:
    def __init__(self, cfg: Mapping, world: str = 'default', object_model: str = 'target_cube',
                 table_model: str = 'Table', table_visual: str = 'visual1', light_name: str = 'sun'):
        """cfg: domain_randomization.yaml の randomize 以下（object, distractors, table_tint, light）."""
        self.world, self.object_model = world, object_model
        self.table_model, self.table_visual, self.light = table_model, table_visual, light_name
        d = cfg['distractors']
        self.pool_sizes: List[float] = list(d['pool_sizes'])
        self.sampler = SceneSampler({
            'object': cfg['object'],
            'distractors': {'count': d['count'], 'size': [min(self.pool_sizes), max(self.pool_sizes)],
                            'min_dist': d['min_dist'], 'x': d['x'], 'y': d['y']},
            'table_tint': cfg['table_tint'], 'light': cfg['light'],
        })
        self.ids = None

    def setup(self):
        """ダミーの箱を作り、色を変える visual の ID を調べる（create は応答が遅いので後で ID を引く）."""
        for k, s in enumerate(self.pool_sizes):
            sdf = (f'<sdf version="1.8"><model name="dr_box_{k}"><static>true</static>'
                   f'<pose>{OFFSCREEN[0]} {OFFSCREEN[1] + k} {BASE_Z} 0 0 0</pose><link name="link">'
                   f'<visual name="v"><geometry><box><size>{s} {s} {s}</size></box></geometry>'
                   f'<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>'
                   f'</link></model></sdf>')
            ign(f'/world/{self.world}/create', 'ignition.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf))
        scene = ign(f'/world/{self.world}/scene/info', 'ignition.msgs.Empty', '', 'ignition.msgs.Scene', 5000)
        ids = {}
        for blk in re.split(r'\nmodel \{', scene)[1:]:
            name = re.search(r'name: "([^"]+)"', blk).group(1)
            ids[name] = dict(re.findall(r'visual \{\s*name: "([^"]+)"\s*id: (\d+)', blk))
        self.ids = {'object': ids[self.object_model]['visual'], 'table': ids[self.table_model][self.table_visual],
                    'boxes': [ids[f'dr_box_{k}']['v'] for k in range(len(self.pool_sizes))]}

    def sample(self, rng: random.Random, object_xy: Sequence[float]) -> Tuple[SceneParams, List[int]]:
        """シーンを決め、ダミーに使い回しの箱を割り当てる（大きさは割り当てた箱のものに置き換える）."""
        scene = self.sampler.sample(rng, object_xy)
        free = list(range(len(self.pool_sizes)))
        rng.shuffle(free)
        pairs = [(free[i], replace(d, size=self.pool_sizes[free[i]])) for i, d in enumerate(scene.distractors)]
        return replace(scene, distractors=tuple(d for _, d in pairs)), [k for k, _ in pairs]

    # ---- 呼び出しの組み立て ----
    def visual_call(self, vid, rgb, ambient=None):
        amb = rgb if ambient is None else ambient
        return (f'/world/{self.world}/visual_config', 'ignition.msgs.Visual',
                f'id: {vid}, material: {{ambient: {color(amb)}, diffuse: {color(rgb)}}}')

    def light_call(self, intensity, direction):
        d = intensity
        return (f'/world/{self.world}/light_config', 'ignition.msgs.Light',
                f'name: "{self.light}", type: DIRECTIONAL, diffuse: {color((d, d, d))}, '
                f'specular: {color((0.2, 0.2, 0.2))}, direction: {{x: {direction[0]:.3f}, y: {direction[1]:.3f}, '
                f'z: {direction[2]:.3f}}}, cast_shadows: true, intensity: 1, range: 1000, '
                # 元の sun の設定（省略すると 0 になり暗くなる）
                f'attenuation_constant: 0.9, attenuation_linear: 0.01, attenuation_quadratic: 0.001, '
                f'pose: {{position: {{z: 10}}}}')

    def pose_call(self, name, x, y, z, yaw=0.0):
        return (f'/world/{self.world}/set_pose', 'ignition.msgs.Pose',
                f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                f'orientation: {{z: {math.sin(yaw / 2):.4f}, w: {math.cos(yaw / 2):.4f}}}')

    def hide_boxes_calls(self):
        return [self.pose_call(f'dr_box_{k}', OFFSCREEN[0], OFFSCREEN[1] + k, BASE_Z)
                for k in range(len(self.pool_sizes))]

    # ---- 適用 ----
    def apply(self, scene: SceneParams, box_ids: Sequence[int]):
        calls = [self.visual_call(self.ids['object'], scene.object_rgb),
                 self.visual_call(self.ids['table'], scene.table_tint, ambient=(0.0, 0.0, 0.0)),  # 天板は環境光なし（元の設定）
                 self.light_call(scene.light_intensity, scene.light_direction)]
        used = dict(zip(box_ids, scene.distractors))
        for k in range(len(self.pool_sizes)):
            if k in used:
                d = used[k]
                calls.append(self.pose_call(f'dr_box_{k}', d.xy[0], d.xy[1], BASE_Z + d.size / 2, d.yaw))
                calls.append(self.visual_call(self.ids['boxes'][k], d.rgb))
            else:
                calls.append(self.pose_call(f'dr_box_{k}', OFFSCREEN[0], OFFSCREEN[1] + k, BASE_Z))
        ign_parallel(calls)

    def restore(self, remove_boxes: bool = True):
        calls = [self.visual_call(self.ids['object'], DEFAULT_OBJECT_RGB),
                 self.visual_call(self.ids['table'], (1.0, 1.0, 1.0), ambient=(0.0, 0.0, 0.0)),
                 self.light_call(*DEFAULT_LIGHT)]
        if remove_boxes:
            calls += [(f'/world/{self.world}/remove', 'ignition.msgs.Entity', f'name: "dr_box_{k}", type: MODEL')
                      for k in range(len(self.pool_sizes))]
        ign_parallel(calls)
