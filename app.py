"""Local robot training server: one isolated MuJoCo world per WebSocket."""
import asyncio
import json
import math
import os
import secrets
from pathlib import Path
import mujoco
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from missions import MISSIONS

ROOT = Path(__file__).parent
# 로봇 ID → (Menagerie 폴더, 장면 파일)
MODELS = {
    'go2': ('unitree_go2', 'scene.xml'),
    'rby1': ('rainbow_robotics_rby1', 'scene_rby1a_1.2.xml'),
    'g1': ('unitree_g1', 'scene.xml'),
    'allegro': ('wonik_allegro', 'scene_right.xml'),
}

def is_velocity_actuator(model, actuator_id):
    """위치 바이어스 없이 속도 바이어스만 있는 구동기(예: RB-Y1 바퀴)는 관절 각도 제어 대상이 아님"""
    return model.actuator_biasprm[actuator_id, 1] == 0 and model.actuator_biasprm[actuator_id, 2] != 0

class World:
    def __init__(self, robot='go2'):
        if robot not in MODELS:
            raise ValueError('지원하지 않는 로봇')
        self.robot = robot
        model_folder, scene_file = MODELS[robot]
        # 원본 장면은 수정하지 않고 MjSpec으로 미션 환경(컨베이어·상자·목표 등)을 덧붙여 컴파일
        spec = mujoco.MjSpec.from_file(str(ROOT/'vendor'/'menagerie'/model_folder/scene_file))
        self.mission = MISSIONS[robot]()
        self.mission.build(spec)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        mission_bodies = set(self.mission.object_bodies)
        # 몸체 고정은 로봇 자유 관절에만 적용(미션 물체는 자유롭게 떨어지고 움직여야 함)
        self.robot_free_joints = [
            j for j in range(self.model.njnt)
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
            and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.jnt_bodyid[j]) not in mission_bodies
        ]
        self.joints = []
        self.actuators = {}
        for a in range(self.model.nu):
            if self.model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT or is_velocity_actuator(self.model, a):
                continue
            j = int(self.model.actuator_trnid[a, 0])
            if int(self.model.jnt_type[j]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                continue
            self.actuators[a] = j
        for j in sorted(set(self.actuators.values())):
            limits = self.model.jnt_range[j].tolist() if self.model.jnt_limited[j] else [-math.pi, math.pi]
            self.joints.append({'id': j, 'name': mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or str(j), 'min': limits[0], 'max': limits[1], 'unit': 'm' if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE else 'rad'})
        # PD 토크 제어(Go2) 기본 게인: 보행 시 몸통이 처지지 않는 값
        self.kp, self.kd = 70., 3.
        self.anchored = True
        self.running = False
        self.seq = 0
        self.mission.attach(self)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.targets = {j['id']: float(self.data.qpos[self.model.jnt_qposadr[j['id']]]) for j in self.joints}
        self.base = self.data.qpos.copy()
        self.running = False
        self.mission.reset(self)

    def command(self, msg):
        """명령 처리. 미션 전용 명령이면 결과 dict를 반환(ack에 포함)"""
        handled, result = self.mission.command(self, msg)
        if handled:
            return result
        kind = msg.get('type')
        if kind == 'joint':
            j, target = msg.get('joint'), msg.get('target')
            if type(j) is not int or j not in self.targets or type(target) not in (int, float) or not math.isfinite(target):
                raise ValueError('유효한 관절 ID와 유한한 목표값이 필요합니다')
            meta = next(x for x in self.joints if x['id'] == j)
            if not meta['min'] <= target <= meta['max']:
                raise ValueError('관절 제한 범위를 벗어났습니다')
            self.targets[j] = float(target)
        elif kind in ('play', 'pause', 'stop'):
            self.running = kind == 'play'
            if kind == 'stop':
                self.data.ctrl[:] = 0
                self.targets = {j: float(self.data.qpos[self.model.jnt_qposadr[j]]) for j in self.targets}
                self.mission.stop(self)
        elif kind == 'reset':
            self.reset()
        elif kind == 'anchor':
            if type(msg.get('enabled')) is not bool:
                raise ValueError('enabled는 boolean입니다')
            if msg['enabled'] and not self.anchored:
                # 다시 고정할 때 처음 위치로 순간이동하지 않도록 현재 자세를 고정 기준으로 사용
                self.base = self.data.qpos.copy()
                self.mission.stop(self)
            self.anchored = msg['enabled']
        elif kind == 'gains':
            kp, kd = msg.get('kp'), msg.get('kd')
            if any(type(x) not in (int, float) or not math.isfinite(x) for x in (kp, kd)) or not 0 <= kp <= 150 or not 0 <= kd <= 20:
                raise ValueError('kp 0~150, kd 0~20')
            self.kp, self.kd = float(kp), float(kd)
        else:
            raise ValueError('알 수 없는 명령')

    def step(self):
        if not self.running:
            return
        for _ in range(max(1, round((1/30)/self.model.opt.timestep))):
            self.mission.before_substep(self)
            for a, j in self.actuators.items():
                target = self.targets[j]
                q = self.data.qpos[self.model.jnt_qposadr[j]]
                dq = self.data.qvel[self.model.jnt_dofadr[j]]
                # Menagerie position actuators already include their own servo gains.
                if self.model.actuator_biasprm[a, 1] != 0:
                    u = target
                else:
                    gear = float(self.model.actuator_gear[a, 0])
                    u = (self.kp*(target-q)-self.kd*dq)/(gear or 1.)
                if self.model.actuator_ctrllimited[a]:
                    u = np.clip(u, *self.model.actuator_ctrlrange[a])
                self.data.ctrl[a] = u
            mujoco.mj_step(self.model, self.data)
            if self.anchored:
                for j in self.robot_free_joints:
                    q, v = self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
                    self.data.qpos[q:q+7] = self.base[q:q+7]
                    self.data.qvel[v:v+6] = 0
            if not np.isfinite(self.data.qpos).all():
                self.reset()
                raise ValueError('물리 계산 불안정: 초기화했습니다')
        mujoco.mj_forward(self.model, self.data)
        self.mission.update(self)

    def description(self):
        m = self.model
        meshes = {}
        geoms = []
        for g in range(m.ngeom):
            # Prefer visual group 2; retain ordinary primitives and environment.
            if m.geom_group[g] == 3:
                continue
            mid = int(m.geom_dataid[g])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and mid not in meshes:
                va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
                meshes[mid] = {'vertices': m.mesh_vert[va:va+vn].flatten().tolist(), 'faces': m.mesh_face[fa:fa+fn].flatten().tolist()}
            color = m.geom_rgba[g].tolist()
            mat = int(m.geom_matid[g])
            if mat >= 0:
                color = m.mat_rgba[mat].tolist()
            geoms.append({'id': g, 'type': int(m.geom_type[g]), 'mesh': mid, 'size': m.geom_size[g].tolist(), 'color': color})
        return {'type':'model', 'robot':self.robot, 'joints':self.joints, 'geoms':geoms, 'meshes':meshes, 'timestep':float(m.opt.timestep), 'actuation':'position' if np.any(m.actuator_biasprm[:,1]) else 'PD torque', 'mission':self.mission.describe()}

    def state(self):
        self.seq += 1
        return {'type':'state', 'seq':self.seq, 'time':float(self.data.time), 'running':self.running, 'anchored':self.anchored, 'contacts':int(self.data.ncon), 'joints':[{'id':j['id'], 'q':float(self.data.qpos[self.model.jnt_qposadr[j['id']]]), 'dq':float(self.data.qvel[self.model.jnt_dofadr[j['id']]]), 'target':self.targets[j['id']], 'torque':float(self.data.qfrc_actuator[self.model.jnt_dofadr[j['id']]])} for j in self.joints], 'positions':self.data.geom_xpos.tolist(), 'rotations':self.data.geom_xmat.tolist(), 'sensors':self.data.sensordata.tolist(), 'mission':self.mission.status(self)}

app = FastAPI(title='Hyoki Robot Lab')

@app.middleware('http')
async def revalidate_static_files(request, call_next):
    # 코드 수정 후 브라우저가 옛 app.js 모듈을 캐시로 쓰지 않도록 매번 ETag 재검증
    response = await call_next(request)
    response.headers['Cache-Control'] = 'no-cache'
    return response
@app.get('/api/robots')
def robots():
    return {'robots':list(MODELS), 'mode':'local-mujoco', 'hardware_connected':False}

def read_positive_int_env(name, default):
    raw_value = os.environ.get(name, '').strip()
    if not raw_value:
        return default
    if not raw_value.isdigit() or int(raw_value) < 1:
        raise SystemExit(f'[오류] {name} 값은 1 이상의 정수여야 합니다: {raw_value}')
    return int(raw_value)

# 공개(터널) 모드: ROBOT_LAB_TOKEN이 있으면 WebSocket 접속 시 ?token= 일치 필요
ACCESS_TOKEN = os.environ.get('ROBOT_LAB_TOKEN', '').strip()
# 다른 출처에서 이 서버로 붙을 수 있는 웹 페이지(GitHub Pages 웹버전)
ALLOWED_ORIGINS = {origin.strip().rstrip('/').lower()
                   for origin in os.environ.get('ROBOT_LAB_ALLOWED_ORIGINS', 'https://metasongi.github.io').split(',')
                   if origin.strip()}
# 접속마다 MuJoCo 세계를 따로 계산하므로 동시 접속 수를 제한해 PC 과부하 방지
MAX_SESSIONS = read_positive_int_env('ROBOT_LAB_MAX_SESSIONS', 4)
active_sessions = 0

def is_origin_allowed(origin, host):
    """Origin 없음(CLI)·같은 주소·허용 목록만 통과"""
    if not origin:
        return True
    normalized_origin = origin.rstrip('/').lower()
    return normalized_origin.split('://', 1)[-1] == (host or '').lower() or normalized_origin in ALLOWED_ORIGINS

def is_token_valid(provided_token):
    if not ACCESS_TOKEN:
        return True
    return secrets.compare_digest(provided_token.encode('utf-8'), ACCESS_TOKEN.encode('utf-8'))

@app.websocket('/ws')
async def socket(ws: WebSocket):
    global active_sessions
    if not is_origin_allowed(ws.headers.get('origin'), ws.headers.get('host')):
        await ws.close(code=1008)
        return
    await ws.accept()
    # 브라우저가 이유를 알 수 있도록 accept 후 전용 코드로 종료
    if not is_token_valid(ws.query_params.get('token', '')):
        await ws.close(code=4401, reason='invalid token')
        return
    if active_sessions >= MAX_SESSIONS:
        await ws.close(code=4429, reason='too many sessions')
        return
    active_sessions += 1
    try:
        await run_session(ws)
    finally:
        active_sessions -= 1

async def run_session(ws: WebSocket):
    world = World()
    lock = asyncio.Lock()
    async def send(payload):
        async with lock:
            await ws.send_json(payload)
    await send(world.description())
    async def pump():
        while True:
            try:
                world.step()
                await send(world.state())
            except ValueError as e:
                await send({'type':'error', 'message':str(e)})
            await asyncio.sleep(1/30)
    task = asyncio.create_task(pump())
    try:
        while True:
            raw = await ws.receive_text()
            if len(raw) > 4096:
                await send({'type':'error','message':'명령은 4096바이트 이하입니다'})
                continue
            try:
                msg = json.loads(raw)
                if not isinstance(msg, dict):
                    raise ValueError('JSON object가 필요합니다')
                result = None
                if msg.get('type') == 'select':
                    world = World(msg.get('robot'))
                    await send(world.description())
                else:
                    result = world.command(msg)
                ack = {'type':'ack', 'command':msg, 'time':float(world.data.time)}
                if result is not None:
                    ack['result'] = result
                await send(ack)
            except (ValueError, TypeError) as e:
                await send({'type':'error','message':str(e)})
    except WebSocketDisconnect:
        pass
    finally:
        task.cancel()
        try:
            await task
        except (asyncio.CancelledError, WebSocketDisconnect, RuntimeError):
            pass

app.mount('/', StaticFiles(directory=ROOT/'static', html=True), name='web')

# 다른 프로그램이 8000번을 점유한 경우를 대비한 후보 포트 목록
CANDIDATE_PORTS = (8000, 8010, 8080, 8888, 9000)

def is_port_available(host, port):
    """해당 포트에 바인딩 가능한지 실제로 열어 보고 판단"""
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe_socket:
        try:
            probe_socket.bind((host, port))
            return True
        except OSError:
            return False

def choose_server_port(host):
    """ROBOT_LAB_PORT 환경변수가 있으면 그 포트를, 없으면 후보 중 첫 빈 포트를 사용"""
    import os
    requested_port = os.environ.get('ROBOT_LAB_PORT')
    if requested_port:
        if not requested_port.isdigit() or not 1 <= int(requested_port) <= 65535:
            raise SystemExit(f'[오류] ROBOT_LAB_PORT 값이 올바르지 않습니다: {requested_port}')
        if not is_port_available(host, int(requested_port)):
            raise SystemExit(f'[오류] 포트 {requested_port}를 이미 다른 프로그램이 사용 중입니다.')
        return int(requested_port)
    for candidate_port in CANDIDATE_PORTS:
        if is_port_available(host, candidate_port):
            return candidate_port
    raise SystemExit(f'[오류] 사용 가능한 포트가 없습니다: {CANDIDATE_PORTS}. ROBOT_LAB_PORT로 직접 지정하세요.')

if __name__ == '__main__':
    import uvicorn
    server_host = '127.0.0.1'
    server_port = choose_server_port(server_host)
    if server_port != 8000:
        print(f'[안내] 8000번 포트가 사용 중이라 {server_port}번 포트로 실행합니다.', flush=True)
    print(f'브라우저 주소: http://{server_host}:{server_port}', flush=True)
    print(f'Python 예제: python examples/control.py --port {server_port}', flush=True)
    uvicorn.run(app, host=server_host, port=server_port)
