"""시뮬레이션 실행 데이터 누적 수집.

실행(run) 하나 = 로봇 선택(또는 초기화)부터 다음 선택·초기화·접속 종료까지.
저장 구조: <데이터 폴더>/<로봇>/<run_id>/
  meta.json       로봇·라벨·시작/종료 시각·프레임 수·미션 성공 여부
  states.csv      실행 중 30 Hz 상태(몸체 자세, 관절 q/dq/target/torque, 센서, 미션 상태)
  commands.jsonl  받은 JSON 명령과 결과/오류(상태 행 번호 frame으로 states.csv와 맞춤)
"""
import csv
import json
import os
import re
import secrets
import tempfile
import threading
import zipfile
from datetime import datetime
from pathlib import Path
import mujoco

DATA_FILES = ('meta.json', 'states.csv', 'commands.jsonl')
RUN_ID_PATTERN = re.compile(r'^\d{8}-\d{6}-[0-9a-f]{6}$')
FLUSH_EVERY_FRAMES = 30          # 1초마다 디스크 반영(중간 다운로드·비정상 종료 대비)
MAX_PENDING_COMMANDS = 200       # 실행 전 명령은 첫 프레임이 기록될 때까지 보관
MAX_LABEL_LENGTH = 60


def now_iso():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def compact_number(value):
    """CSV 용량을 줄이기 위해 유효숫자 7자리로 기록"""
    return f'{value:.7g}'


class DataStore:
    """수집 폴더 관리: 용량 한도, 실행 목록·요약, 파일 경로 검증, ZIP 내보내기"""

    def __init__(self, root, robots, max_bytes, max_frames_per_run):
        self.root = Path(root)
        self.robots = set(robots)
        self.max_bytes = max_bytes
        self.max_frames_per_run = max_frames_per_run
        self.active_runs = set()
        self.lock = threading.Lock()
        self.used_bytes = None   # 처음 필요할 때 한 번만 폴더 전체 크기를 계산

    def ensure_usage_loaded(self):
        with self.lock:
            if self.used_bytes is None:
                self.used_bytes = sum(f.stat().st_size for f in self.root.rglob('*') if f.is_file()) if self.root.exists() else 0

    def add_usage(self, byte_count):
        self.ensure_usage_loaded()
        with self.lock:
            self.used_bytes += byte_count

    def is_full(self):
        self.ensure_usage_loaded()
        return self.used_bytes >= self.max_bytes

    def run_dir(self, robot, run_id):
        """외부 입력(로봇·run_id)으로 경로를 만들 때 폴더 밖 접근(../)을 차단"""
        if robot not in self.robots or not RUN_ID_PATTERN.fullmatch(run_id or ''):
            raise FileNotFoundError('존재하지 않는 실행 기록입니다')
        return self.root / robot / run_id

    def file_path(self, robot, run_id, filename):
        if filename not in DATA_FILES:
            raise FileNotFoundError('지원하지 않는 파일입니다')
        path = self.run_dir(robot, run_id) / filename
        if not path.is_file():
            raise FileNotFoundError('파일이 없습니다')
        return path

    def read_meta(self, run_dir):
        try:
            meta = json.loads((run_dir / 'meta.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None   # 쓰는 중이거나 손상된 기록은 목록에서 제외
        if meta.get('status') == 'recording' and meta.get('run_id') not in self.active_runs:
            meta['status'] = 'interrupted'   # 서버가 비정상 종료돼 마무리되지 못한 기록
        meta['bytes'] = sum(f.stat().st_size for f in run_dir.iterdir() if f.is_file())
        return meta

    def list_runs(self, robot=None, limit=None):
        if robot is not None and robot not in self.robots:
            raise ValueError('지원하지 않는 로봇입니다')
        runs = []
        for robot_name in ([robot] if robot else sorted(self.robots)):
            robot_dir = self.root / robot_name
            if not robot_dir.is_dir():
                continue
            for run_dir in robot_dir.iterdir():
                if run_dir.is_dir() and RUN_ID_PATTERN.fullmatch(run_dir.name):
                    meta = self.read_meta(run_dir)
                    if meta:
                        runs.append(meta)
        runs.sort(key=lambda meta: meta.get('started_at', ''), reverse=True)
        return runs[:limit] if limit else runs

    def summary(self):
        robots = {name: {'runs': 0, 'frames': 0, 'sim_seconds': 0.0, 'successes': 0, 'commands': 0, 'bytes': 0}
                  for name in sorted(self.robots)}
        for meta in self.list_runs():
            totals = robots.get(meta.get('robot'))
            if totals is None:
                continue
            totals['runs'] += 1
            totals['frames'] += int(meta.get('frames', 0))
            totals['sim_seconds'] += float(meta.get('sim_seconds', 0.0))
            totals['successes'] += 1 if meta.get('success') else 0
            totals['commands'] += int(meta.get('commands', 0))
            totals['bytes'] += int(meta.get('bytes', 0))
        for totals in robots.values():
            totals['sim_seconds'] = round(totals['sim_seconds'], 2)
        self.ensure_usage_loaded()
        return {'robots': robots, 'used_bytes': self.used_bytes, 'max_bytes': self.max_bytes}

    def export_zip(self, robot=None):
        """임시 ZIP 파일 경로를 반환(호출한 쪽이 전송 후 삭제)"""
        runs = self.list_runs(robot)
        handle, zip_path = tempfile.mkstemp(prefix='robot-lab-data-', suffix='.zip')
        os.close(handle)
        try:
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as archive:
                for meta in runs:
                    run_dir = self.root / meta['robot'] / meta['run_id']
                    for filename in DATA_FILES:
                        path = run_dir / filename
                        if path.is_file():
                            archive.write(path, f"{meta['robot']}/{meta['run_id']}/{filename}")
                archive.writestr('summary.json', json.dumps(self.summary(), ensure_ascii=False, indent=2))
        except Exception:
            os.remove(zip_path)
            raise
        return zip_path


class SessionRecorder:
    """WebSocket 세션 하나의 기록기. 실제 파일은 첫 '실행 중' 프레임에서 만들어 빈 기록을 남기지 않음"""

    def __init__(self, store, world, enabled=True, client='script'):
        self.store = store
        self.enabled = enabled
        self.label = ''
        self.client = client
        self.bind(world)

    # ---------------------------------------------------------------- 실행 경계
    def bind(self, world):
        """새 실행 시작(로봇 선택·초기화 시). 이전 실행은 마무리"""
        self.close()
        self.world = world
        self.run_id = None
        self.run_dir = None
        self.states_file = self.commands_file = self.states_writer = None
        self.frames = self.command_count = 0
        self.first_time = self.last_time = 0.0
        self.success = False
        self.first_success_time = None
        self.last_summary = ''
        self.pending_commands = []
        self.flushed_bytes = 0
        self.started_at = None
        self.stop_reason = ''   # 길이·용량 한도로 멈춘 경우도 새 실행에서는 다시 시도

    def configure(self, msg):
        """{"type":"record","enabled":bool,"label":"선택"} 처리"""
        enabled, label = msg.get('enabled'), msg.get('label', self.label)
        if type(enabled) is not bool:
            raise ValueError('enabled는 boolean입니다')
        if not isinstance(label, str):
            raise ValueError('label은 문자열입니다')
        label = label.strip()[:MAX_LABEL_LENGTH]
        if label != self.label or not enabled or self.stop_reason:
            self.bind(self.world)   # 라벨 변경·기록 끄기·한도 정지 후 재개는 새 실행으로 구분
        self.enabled, self.label = enabled, label
        return self.status()

    def status(self):
        if not self.enabled:
            state = 'off'
        elif self.stop_reason:
            state = 'stopped'
        elif self.run_id:
            state = 'recording'
        else:
            state = 'waiting'   # 켜져 있지만 아직 실행(play) 전
        return {'state': state, 'run_id': self.run_id, 'frames': self.frames, 'label': self.label,
                'reason': self.stop_reason}

    # ---------------------------------------------------------------- 기록
    def record_state(self, state):
        if not self.enabled or self.stop_reason or not state.get('running'):
            return
        try:
            if self.run_id is None and not self.open_run():
                return
            self.states_writer.writerow(self.state_row(state))
            self.frames += 1
            self.last_time = state['time']
            mission = state.get('mission') or {}
            self.last_summary = mission.get('summary', '')
            if mission.get('success') and not self.success:
                self.success, self.first_success_time = True, round(self.last_time - self.first_time, 3)
            if self.frames % FLUSH_EVERY_FRAMES == 0:
                self.flush()
            if self.frames >= self.store.max_frames_per_run:
                self.stop('한 실행의 최대 기록 길이에 도달했습니다. 초기화하거나 로봇을 다시 선택하면 새 실행으로 기록됩니다')
        except OSError as error:
            self.stop(f'저장 실패: {error}')

    def record_command(self, msg, result=None, error=None):
        if not self.enabled or self.stop_reason or not isinstance(msg, dict) or msg.get('type') == 'record':
            return
        entry = {'frame': self.frames, 'time': round(float(self.world.data.time), 4), 'command': msg}
        if result is not None:
            entry['result'] = result
        if error is not None:
            entry['error'] = error
        if self.run_id is None:
            # 실행 전 명령(목표값 설정 등)도 재현에 필요하므로 첫 프레임 때 함께 기록
            if len(self.pending_commands) < MAX_PENDING_COMMANDS:
                self.pending_commands.append(entry)
            return
        try:
            self.write_command(entry)
        except OSError as error:
            self.stop(f'저장 실패: {error}')

    def write_command(self, entry):
        self.commands_file.write(json.dumps(entry, ensure_ascii=False, default=str) + '\n')
        self.command_count += 1

    def stop(self, reason):
        self.stop_reason = reason
        self.close()

    def close(self):
        """현재 실행을 마무리(파일 닫기 + meta 확정). 여러 번 호출해도 안전"""
        if getattr(self, 'run_id', None) is None or self.states_file is None:
            return
        try:
            self.flush(status='stopped' if self.stop_reason else 'finished')
        except OSError:
            pass   # 디스크 오류여도 세션 종료는 계속 진행
        finally:
            for handle in (self.states_file, self.commands_file):
                try:
                    handle.close()
                except OSError:
                    pass
            self.states_file = self.commands_file = None
            self.store.active_runs.discard(self.run_id)

    # ---------------------------------------------------------------- 내부
    def open_run(self):
        if self.store.is_full():
            self.stop_reason = '데이터 저장 한도(ROBOT_LAB_DATA_MAX_MB)를 넘어 기록을 멈췄습니다'
            return False
        self.run_id = f"{datetime.now():%Y%m%d-%H%M%S}-{secrets.token_hex(3)}"
        self.run_dir = self.store.root / self.world.robot / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.store.active_runs.add(self.run_id)
        self.started_at = now_iso()
        self.first_time = float(self.world.data.time)
        self.columns = self.build_columns()
        # utf-8-sig: 엑셀에서 열어도 한글 미션 요약이 깨지지 않음
        self.states_file = open(self.run_dir / 'states.csv', 'w', newline='', encoding='utf-8-sig')
        self.commands_file = open(self.run_dir / 'commands.jsonl', 'w', encoding='utf-8')
        self.states_writer = csv.writer(self.states_file)
        self.states_writer.writerow(self.columns)
        for entry in self.pending_commands:
            self.write_command(entry)
        self.pending_commands = []
        self.flush()
        return True

    def build_columns(self):
        model, world = self.world.model, self.world
        columns = ['frame', 'time', 'anchored', 'contacts']
        if world.robot_free_joints:
            columns += ['base_x', 'base_y', 'base_z', 'base_qw', 'base_qx', 'base_qy', 'base_qz']
        for joint in world.joints:
            columns += [f"{joint['name']}_{field}" for field in ('q', 'dq', 'target', 'torque')]
        for sensor_id in range(model.nsensor):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id) or f'sensor{sensor_id}'
            dimension = int(model.sensor_dim[sensor_id])
            columns += [name] if dimension == 1 else [f'{name}_{axis}' for axis in range(dimension)]
        return columns + ['mission_success', 'mission_summary', 'mission_json']

    def state_row(self, state):
        world = self.world
        row = [self.frames, compact_number(state['time']), int(state['anchored']), state['contacts']]
        if world.robot_free_joints:
            address = world.model.jnt_qposadr[world.robot_free_joints[0]]
            row += [compact_number(v) for v in world.data.qpos[address:address + 7]]
        for joint in state['joints']:
            row += [compact_number(joint[field]) for field in ('q', 'dq', 'target', 'torque')]
        row += [compact_number(v) for v in world.data.sensordata]
        mission = dict(state.get('mission') or {})
        summary = mission.pop('summary', '')
        row += [int(bool(mission.get('success'))), summary,
                json.dumps(mission, ensure_ascii=False, separators=(',', ':'), default=str)]
        return row

    def flush(self, status='recording'):
        self.states_file.flush()
        self.commands_file.flush()
        meta = {
            'run_id': self.run_id, 'robot': self.world.robot, 'label': self.label, 'client': self.client,
            'mission': self.world.mission.title, 'status': status, 'stop_reason': self.stop_reason,
            'started_at': self.started_at, 'ended_at': None if status == 'recording' else now_iso(),
            'frames': self.frames, 'commands': self.command_count,
            'sim_seconds': round(self.last_time - self.first_time, 3) if self.frames else 0.0,
            'success': self.success, 'first_success_seconds': self.first_success_time,
            'final_summary': self.last_summary, 'columns': len(self.columns),
            'joints': [joint['name'] for joint in self.world.joints],
        }
        (self.run_dir / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
        # 용량 한도는 실제 파일 크기 증가분으로 추적
        written_bytes = sum(f.stat().st_size for f in self.run_dir.iterdir() if f.is_file())
        self.store.add_usage(written_bytes - self.flushed_bytes)
        self.flushed_bytes = written_bytes
        if status == 'recording' and self.store.is_full():
            self.stop('데이터 저장 한도(ROBOT_LAB_DATA_MAX_MB)를 넘어 기록을 멈췄습니다')
