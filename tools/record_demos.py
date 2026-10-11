"""웹버전 데모용 녹화: 미션 자동 풀이를 서버 없이 실행해 static/demo/{robot}.json으로 저장.

    .venv\\Scripts\\python.exe tools\\record_demos.py            (4개 모두)
    .venv\\Scripts\\python.exe tools\\record_demos.py go2 g1     (일부만)

파일 구성: 모델(메시 0.1mm 반올림) + 15fps 프레임(움직이는 형상의 위치·쿼터니언, 관절, 미션 상태) + 보낸 JSON 명령.
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tests'))
from app import World            # noqa: E402
import mission_helpers           # noqa: E402

OUTPUT_DIR = ROOT / 'static' / 'demo'
STEPS_PER_SECOND = 30            # World.step 1회 = 1/30초
CAPTURE_EVERY_STEPS = 2          # 15fps로 저장(웹에서 보간 재생)
DECIMALS = 4
MOVING_THRESHOLD = 1e-5


def round_tree(value):
    """중첩 dict/list의 실수를 일괄 반올림해 JSON 크기 축소"""
    if isinstance(value, float):
        return round(value, DECIMALS)
    if isinstance(value, dict):
        return {key: round_tree(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [round_tree(item) for item in value]
    return value


class DemoRecorder:
    def __init__(self, robot):
        self.world = World(robot)
        self.model_message = self.world.description()
        self.rendered_geoms = [geom['id'] for geom in self.model_message['geoms']]
        self.frames, self.poses, self.commands = [], [], []
        self.step_count = 0
        self.capture()

    def capture(self):
        world, data = self.world, self.world.data
        state = world.state()
        quats = np.zeros((len(self.rendered_geoms), 4))
        for index, geom in enumerate(self.rendered_geoms):
            mujoco.mju_mat2Quat(quats[index], data.geom_xmat[geom])
        self.poses.append(np.hstack([data.geom_xpos[self.rendered_geoms], quats]))
        self.frames.append({
            't': round(state['time'], 3), 'run': state['running'], 'anc': state['anchored'], 'con': state['contacts'],
            'j': [[round(j[key], DECIMALS) for key in ('q', 'dq', 'target', 'torque')] for j in state['joints']],
            'm': round_tree(state['mission']),
        })

    # mission_helpers의 run_seconds/send를 대체해 실행 중 프레임과 명령을 기록
    def run_seconds(self, _world, seconds):
        for _ in range(int(round(seconds * STEPS_PER_SECOND))):
            self.world.step()
            self.step_count += 1
            if self.step_count % CAPTURE_EVERY_STEPS == 0:
                self.capture()

    def send(self, _world, **msg):
        result = self.world.command(msg)
        # 실행 전에는 시뮬레이션 시간이 멈춰 있으므로 재생 기준은 프레임 번호(f)
        self.commands.append({'f': len(self.frames), 'msg': round_tree(msg), 'result': round_tree(result)})
        return result

    def compact_model(self):
        model = dict(self.model_message)
        model['meshes'] = {str(mesh_id): {'vertices': [round(v, DECIMALS) for v in mesh['vertices']], 'faces': mesh['faces']}
                           for mesh_id, mesh in model['meshes'].items()}
        model['geoms'] = round_tree(model['geoms'])
        return model

    def export(self, path):
        poses = np.array(self.poses)                       # (프레임, 형상, 7)
        moving = np.where(np.abs(poses - poses[0]).max(axis=(0, 2)) > MOVING_THRESHOLD)[0]
        payload = {
            'version': 1, 'robot': self.world.robot, 'fps': STEPS_PER_SECOND / CAPTURE_EVERY_STEPS,
            'model': self.compact_model(),
            'geom_ids': self.rendered_geoms,
            'initial_poses': np.round(poses[0], DECIMALS).tolist(),
            'moving': moving.tolist(),
            'frames': [{**frame, 'p': np.round(poses[index, moving], DECIMALS).flatten().tolist()}
                       for index, frame in enumerate(self.frames)],
            'commands': self.commands,
        }
        path.write_text(json.dumps(payload, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
        return path.stat().st_size, len(moving), len(self.frames)


def play_allegro(recorder):
    send, run = mission_helpers.send, mission_helpers.run_seconds
    send(recorder.world, type='play')
    run(recorder.world, 1.5)
    mission_helpers.close_hand(recorder.world)
    run(recorder.world, 2.5)


def play_go2(recorder):
    mission_helpers.run_seconds(recorder.world, 0.5)
    mission_helpers.walk_to_goal(recorder.world, timeout=20)
    mission_helpers.run_seconds(recorder.world, 1.5)


def play_rby1(recorder):
    world, send, run = recorder.world, mission_helpers.send, mission_helpers.run_seconds
    send(world, type='conveyor', speed=0.0)
    send(world, type='play')
    for _ in range(3):
        window = [i for i, c in enumerate(world.mission.status(world)['cubes']) if -0.35 < c[1] < -0.15 and c[2] > 0.75]
        if not window:
            send(world, type='conveyor', speed=0.12)
            for _ in range(150):
                run(world, 0.1)
                window = [i for i, c in enumerate(world.mission.status(world)['cubes']) if -0.35 < c[1] < -0.15 and c[2] > 0.75]
                if window:
                    break
            send(world, type='conveyor', speed=0.0)
            run(world, 0.5)
        if not window:
            raise RuntimeError('RB-Y1 데모: 작업 구간에 큐브가 들어오지 않음')
        mission_helpers.pick_cube_on_conveyor(world, window[0], arm='right', log=lambda *_: None)
    send(world, type='conveyor', speed=0.05)
    run(world, 2.0)


def play_g1(recorder):
    mission_helpers.carry_cargo_to_storage(recorder.world, log=lambda *_: None)
    mission_helpers.run_seconds(recorder.world, 1.0)


SCENARIOS = {'go2': play_go2, 'rby1': play_rby1, 'g1': play_g1, 'allegro': play_allegro}


def main(robots):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for robot in robots:
        if robot not in SCENARIOS:
            raise SystemExit(f'[오류] 알 수 없는 로봇: {robot} (가능: {", ".join(SCENARIOS)})')
        recorder = DemoRecorder(robot)
        original_run, original_send = mission_helpers.run_seconds, mission_helpers.send
        mission_helpers.run_seconds, mission_helpers.send = recorder.run_seconds, recorder.send
        try:
            SCENARIOS[robot](recorder)
        finally:
            mission_helpers.run_seconds, mission_helpers.send = original_run, original_send
        size, moving_count, frame_count = recorder.export(OUTPUT_DIR / f'{robot}.json')
        success = recorder.world.mission.status(recorder.world)['success']
        print(f'{robot}: {size / 1e6:.1f} MB, 프레임 {frame_count}, 움직이는 형상 {moving_count}, 미션 성공 {success}', flush=True)


if __name__ == '__main__':
    main(sys.argv[1:] or list(SCENARIOS))
