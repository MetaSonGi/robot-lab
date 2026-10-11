"""로봇별 훈련 미션.

각 미션은 원본 Menagerie 장면에 환경 물체를 추가(MjSpec)하고, 미션 전용 JSON 명령과
성공 판정을 제공합니다. '보조(assist)' 기능은 실제 로봇에는 없는 학습용 장치이며
상태 메시지에 항상 켜짐/꺼짐을 표시합니다.
"""
import math
import mujoco
import numpy as np

GEOM_BOX = mujoco.mjtGeom.mjGEOM_BOX
GEOM_CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
WORLD_UP = np.array([0.0, 0.0, 1.0])


# ---------------------------------------------------------------- 공용 도우미
def find_id(model, object_type, name):
    identifier = mujoco.mj_name2id(model, object_type, name)
    if identifier < 0:
        raise RuntimeError(f'모델에 {name} 이(가) 없습니다')
    return identifier


def read_number(msg, key, low, high, default=None):
    """JSON 숫자 필드 검증: 누락 시 default, 범위·유한성 위반 시 ValueError"""
    value = msg.get(key, default)
    if value is None:
        raise ValueError(f'{key} 값이 필요합니다')
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{key}는 유한한 숫자여야 합니다')
    if not low <= value <= high:
        raise ValueError(f'{key}는 {low}~{high} 범위여야 합니다')
    return float(value)


def read_bool(msg, key):
    value = msg.get(key)
    if type(value) is not bool:
        raise ValueError(f'{key}는 true/false 입니다')
    return value


def read_position(msg, key='pos'):
    value = msg.get(key)
    if not isinstance(value, list) or len(value) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{key}는 [x, y, z] 숫자 3개입니다')
    if any(abs(v) > 5 for v in value):
        raise ValueError(f'{key} 좌표는 ±5m 이내여야 합니다')
    return np.array(value, dtype=float)


def yaw_from_quat(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_from_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def add_static_box(body, size, pos, rgba, collide=True, name=None):
    kwargs = dict(type=GEOM_BOX, size=list(size), pos=list(pos), rgba=list(rgba))
    if name:
        kwargs['name'] = name
    if not collide:
        kwargs.update(contype=0, conaffinity=0)
    return body.add_geom(**kwargs)


def free_joint_address(model, body):
    joint = model.body_jntadr[body]
    if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError('자유 관절이 없는 바디입니다')
    return int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])


def place_free_body(world, body, position, quat=(1, 0, 0, 0)):
    qpos_address, dof_address = free_joint_address(world.model, body)
    world.data.qpos[qpos_address:qpos_address + 3] = position
    world.data.qpos[qpos_address + 3:qpos_address + 7] = quat
    world.data.qvel[dof_address:dof_address + 6] = 0


# ---------------------------------------------------------------- 팔 IK
class Arm:
    """IK 대상 팔: 관절 이름 목록, 끝단 바디, 파지점 계산 방법"""

    def __init__(self, name, joint_names, end_body, finger_bodies=None, palm_offset=None, pad_depth=0.0):
        self.name = name
        self.joint_names = joint_names
        self.end_body_name = end_body
        self.finger_body_names = finger_bodies or []
        self.palm_offset = np.array(palm_offset if palm_offset is not None else [0, 0, 0], dtype=float)
        # 손가락 관절 원점에서 실제 접촉 패드 중심까지 거리(파지 방향)
        self.pad_depth = pad_depth

    def attach(self, model):
        self.joint_ids = [find_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names]
        self.qpos_addresses = np.array([model.jnt_qposadr[j] for j in self.joint_ids])
        self.dof_addresses = np.array([model.jnt_dofadr[j] for j in self.joint_ids])
        self.lower = np.array([model.jnt_range[j, 0] for j in self.joint_ids])
        self.upper = np.array([model.jnt_range[j, 1] for j in self.joint_ids])
        self.end_body = find_id(model, mujoco.mjtObj.mjOBJ_BODY, self.end_body_name)
        self.finger_bodies = [find_id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in self.finger_body_names]

    def reference_point(self, data):
        # 그리퍼는 두 손가락 관절 중점, 손바닥형은 손목 바디 기준 오프셋
        if self.finger_bodies:
            return np.mean([data.xpos[b] for b in self.finger_bodies], axis=0)
        return data.xpos[self.end_body] + data.xmat[self.end_body].reshape(3, 3) @ self.palm_offset

    def approach_axis(self, data):
        direction = self.reference_point(data) - data.xpos[self.end_body]
        return direction / (np.linalg.norm(direction) + 1e-9)

    def grip_point(self, data):
        return self.reference_point(data) + self.pad_depth * self.approach_axis(data)


def solve_arm_ik(world, arm, target, point_down=False):
    """감쇠 최소제곱(DLS) IK. 현재 목표 자세에서 먼저 풀고, 특이자세 등으로 수렴하지 않으면
    정해진 난수 시드 자세들에서 재시도해 가장 오차가 작은 해를 관절 목표값으로 반환"""
    seed = np.array([world.targets.get(j, world.data.qpos[a]) for j, a in zip(arm.joint_ids, arm.qpos_addresses)])
    best_solution, best_error = None, math.inf
    random = np.random.default_rng(7)
    for attempt in range(8):
        if attempt:
            seed = arm.lower + random.random(len(seed)) * (arm.upper - arm.lower)
            seed = 0.5 * seed + 0.5 * np.clip(0, arm.lower, arm.upper)  # 극단 자세를 피하도록 중앙 쪽으로
        solution, position_error, rotation_error = _dls_ik(world, arm, target, seed, point_down)
        score = position_error + 0.05 * rotation_error
        if score < best_error:
            best_solution, best_error, best_position_error = solution, score, position_error
        if position_error < 0.005 and rotation_error < 0.15:
            break
    return best_solution, best_position_error


def _dls_ik(world, arm, target, seed, point_down, iterations=150):
    model, scratch = world.model, world.ik_data
    scratch.qpos[:] = world.data.qpos
    scratch.qpos[arm.qpos_addresses] = np.clip(seed, arm.lower, arm.upper)
    position_jacobian = np.zeros((3, model.nv))
    rotation_jacobian = np.zeros((3, model.nv))
    damping = 0.03
    for _ in range(iterations):
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        point = arm.grip_point(scratch)
        position_error = target - point
        mujoco.mj_jac(model, scratch, position_jacobian, rotation_jacobian, point, arm.end_body)
        jacobian = position_jacobian[:, arm.dof_addresses]
        error = position_error
        rotation_error = np.zeros(3)
        if point_down:
            # 파지 방향(손목→파지점)을 -z로 맞추는 보조 과제(가중치 0.3)
            rotation_error = np.cross(arm.approach_axis(scratch), -WORLD_UP)
            jacobian = np.vstack([jacobian, 0.3 * rotation_jacobian[:, arm.dof_addresses]])
            error = np.concatenate([position_error, 0.3 * rotation_error])
        if np.linalg.norm(position_error) < 0.002 and np.linalg.norm(rotation_error) < 0.02:
            break
        step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping ** 2 * np.eye(len(error)), error)
        step = np.clip(step, -0.15, 0.15)
        scratch.qpos[arm.qpos_addresses] = np.clip(scratch.qpos[arm.qpos_addresses] + step, arm.lower, arm.upper)
    mujoco.mj_kinematics(model, scratch)
    position_error = float(np.linalg.norm(target - arm.grip_point(scratch)))
    # 최종 평가는 각도(rad)로: 외적 크기는 정반대(180°)일 때도 0이 되므로 사용하지 않음
    rotation_error = math.acos(float(np.clip(arm.approach_axis(scratch) @ -WORLD_UP, -1, 1))) if point_down else 0.0
    solution = {joint_id: float(scratch.qpos[address]) for joint_id, address in zip(arm.joint_ids, arm.qpos_addresses)}
    return solution, position_error, rotation_error


# ---------------------------------------------------------------- 흡착 보조(weld)
class WeldAssist:
    """잡는 바디-물체 쌍마다 비활성 weld 구속을 미리 만들어 두고, 파지 시 현재 상대 자세로 고정"""

    def __init__(self, spec, holder_body, object_names, tag):
        self.holder_name = holder_body
        self.object_names = object_names
        self.names = [f'assist_{tag}_{i}' for i in range(len(object_names))]
        for name, object_name in zip(self.names, object_names):
            spec.add_equality(name=name, type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                              name1=holder_body, name2=object_name, active=False)
        self.held_index = None

    def attach(self, model):
        self.holder = find_id(model, mujoco.mjtObj.mjOBJ_BODY, self.holder_name)
        self.objects = [find_id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in self.object_names]
        self.equalities = [find_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, n) for n in self.names]

    def grab(self, world, index):
        model, data = world.model, world.data
        equality, holder, target = self.equalities[index], self.holder, self.objects[index]
        # eq_data: [anchor(3), body1(잡는 쪽) 좌표계에서 본 body2(물체) 상대 위치(3), 상대 쿼터니언(4), torquescale]
        relative_position = data.xmat[holder].reshape(3, 3).T @ (data.xpos[target] - data.xpos[holder])
        relative_quat = np.zeros(4)
        holder_conjugate = np.zeros(4)
        mujoco.mju_negQuat(holder_conjugate, data.xquat[holder])
        mujoco.mju_mulQuat(relative_quat, holder_conjugate, data.xquat[target])
        model.eq_data[equality, 0:3] = 0
        model.eq_data[equality, 3:6] = relative_position
        model.eq_data[equality, 6:10] = relative_quat
        model.eq_data[equality, 10] = 1
        data.eq_active[equality] = 1
        self.held_index = index

    def release(self, world):
        for equality in self.equalities:
            world.data.eq_active[equality] = 0
        self.held_index = None


# ---------------------------------------------------------------- 미션 기본형
class Mission:
    title = ''
    goal = ''
    steps = []
    examples = []
    quick = []              # 웹 빠른 실행 버튼: {'label', 'commands': [JSON 명령...]}
    object_bodies = []      # 몸체 고정(anchor) 대상에서 제외할 자유 물체
    has_assist = False

    def __init__(self):
        self.assist = self.has_assist
        self.success = False

    def build(self, spec):
        pass

    def attach(self, world):
        pass

    def reset(self, world):
        self.success = False

    def stop(self, world):
        pass

    def before_substep(self, world):
        pass

    def update(self, world):
        pass

    def command(self, world, msg):
        """처리했으면 (True, 결과 dict|None), 아니면 (False, None)"""
        if msg.get('type') == 'assist':
            if not self.has_assist:
                raise ValueError('이 미션에는 보조 기능이 없습니다')
            self.assist = read_bool(msg, 'enabled')
            self.on_assist_changed(world)
            return True, {'assist': self.assist}
        return False, None

    def on_assist_changed(self, world):
        pass

    def describe(self):
        return {'title': self.title, 'goal': self.goal, 'steps': self.steps, 'examples': self.examples,
                'quick': self.quick, 'has_assist': self.has_assist}

    def status(self, world):
        return {'success': self.success, 'assist': self.assist}


# ---------------------------------------------------------------- Allegro: 물체 쥐기
class AllegroGraspMission(Mission):
    title = '물체 쥐기'
    goal = '엄지를 포함한 손가락 3개 이상으로 물체를 1초 이상 감싸 쥐기'
    steps = ['실행 → 물체가 손바닥에 내려앉을 때까지 대기', '손가락 굽힘 관절(…j1~j3)과 엄지 회전(thj0) 조절', '접촉 손가락 표시를 보며 1초 유지']
    examples = [{'type': 'joint', 'joint': 1, 'target': 1.2}, {'type': 'play'}]
    quick = [{'label': '물체 다시 놓기', 'commands': [{'type': 'reset'}, {'type': 'play'}]}]
    object_bodies = ['object']
    FINGERS = {'ff': '검지', 'mf': '중지', 'rf': '약지', 'th': '엄지'}

    def attach(self, world):
        model = world.model
        self.object_body = find_id(model, mujoco.mjtObj.mjOBJ_BODY, 'object')
        self.finger_of_body = {}
        for body in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ''
            if name[:2] in self.FINGERS:
                self.finger_of_body[body] = name[:2]
        self.hold_seconds = 0.0
        self.last_time = 0.0
        self.touching = []

    def reset(self, world):
        super().reset(world)
        self.hold_seconds = 0.0
        self.last_time = 0.0
        self.touching = []

    def update(self, world):
        model, data = world.model, world.data
        touching = set()
        for index in range(data.ncon):
            contact = data.contact[index]
            bodies = (model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2])
            if self.object_body in bodies:
                other = bodies[1] if bodies[0] == self.object_body else bodies[0]
                if other in self.finger_of_body:
                    touching.add(self.finger_of_body[other])
        self.touching = sorted(touching)
        elapsed = data.time - self.last_time
        self.last_time = data.time
        gripping = 'th' in touching and len(touching) >= 3
        self.hold_seconds = self.hold_seconds + elapsed if gripping else 0.0
        if self.hold_seconds >= 1.0:
            self.success = True

    def status(self, world):
        return {**super().status(world), 'touching': [self.FINGERS[f] for f in self.touching],
                'hold_seconds': round(self.hold_seconds, 2),
                'summary': f"접촉: {', '.join(self.FINGERS[f] for f in self.touching) or '없음'} · 유지 {self.hold_seconds:.1f}s"}


# ---------------------------------------------------------------- RB-Y1: 컨베이어 피킹
class ConveyorPickMission(Mission):
    title = '컨베이어 피킹'
    goal = '컨베이어로 지나가는 큐브를 집어 오른쪽 수거함에 3개 넣기'
    steps = ['ee_target으로 그리퍼를 큐브 위로 이동(그리퍼는 아래를 향하도록 자동 IK)',
             'grasp enabled=true로 파지', '수거함 위로 이동 후 grasp enabled=false로 놓기',
             '컨베이어 속도를 0으로 두고 연습한 뒤 속도를 올려 보세요']
    examples = [{'type': 'conveyor', 'speed': 0.0}, {'type': 'ee_target', 'arm': 'right', 'pos': [0.42, -0.1, 0.9]},
                {'type': 'grasp', 'arm': 'right', 'enabled': True}, {'type': 'assist', 'enabled': False}]
    quick = [
        {'label': '벨트 정지', 'commands': [{'type': 'conveyor', 'speed': 0.0}]},
        {'label': '벨트 0.05', 'commands': [{'type': 'conveyor', 'speed': 0.05}]},
        {'label': '벨트 0.12', 'commands': [{'type': 'conveyor', 'speed': 0.12}]},
        {'label': '오른팔 대기', 'commands': [{'type': 'ee_target', 'arm': 'right', 'pos': [0.42, -0.25, 0.95]}, {'type': 'play'}]},
        {'label': '오른손 잡기', 'commands': [{'type': 'grasp', 'arm': 'right', 'enabled': True}]},
        {'label': '오른손 놓기', 'commands': [{'type': 'grasp', 'arm': 'right', 'enabled': False}]},
        {'label': '수거함 위로', 'commands': [{'type': 'ee_target', 'arm': 'right', 'pos': [0.18, -0.52, 0.9]}]},
    ]
    has_assist = True
    CUBE_COUNT = 3
    CUBE_HALF = 0.022
    BELT_CENTER = np.array([0.42, 0.0])
    BELT_HALF = np.array([0.12, 0.95])
    BELT_TOP = 0.78
    BIN_CENTER = np.array([0.18, -0.52])
    BIN_HALF = 0.12
    BIN_FLOOR = 0.70
    SPAWN_Y = 0.9
    GRIPPER_OPEN, GRIPPER_CLOSED = -0.05, 0.0
    CUBE_COLORS = [[0.93, 0.33, 0.33, 1], [0.98, 0.8, 0.25, 1], [0.35, 0.6, 0.98, 1]]

    def __init__(self):
        super().__init__()
        self.object_bodies = [f'cube_{i}' for i in range(self.CUBE_COUNT)]
        self.arms = {
            'right': Arm('right', [f'right_arm_{i}' for i in range(7)], 'EE_BODY_R', finger_bodies=['ee_finger_r1', 'ee_finger_r2'], pad_depth=0.03),
            'left': Arm('left', [f'left_arm_{i}' for i in range(7)], 'EE_BODY_L', finger_bodies=['ee_finger_l1', 'ee_finger_l2'], pad_depth=0.03),
        }
        self.gripper_joint_names = {'right': 'gripper_finger_r1', 'left': 'gripper_finger_l1'}

    def build(self, spec):
        world = spec.worldbody
        conveyor = world.add_body(name='conveyor', pos=[*self.BELT_CENTER, 0])
        add_static_box(conveyor, [*self.BELT_HALF, 0.02], [0, 0, self.BELT_TOP - 0.02], [0.18, 0.2, 0.24, 1], name='belt')
        for side in (-1, 1):
            add_static_box(conveyor, [0.01, self.BELT_HALF[1], 0.03], [side * (self.BELT_HALF[0] + 0.01), 0, self.BELT_TOP], [0.85, 0.6, 0.15, 1])
        for leg_y in (-0.8, 0, 0.8):
            add_static_box(conveyor, [0.03, 0.03, (self.BELT_TOP - 0.04) / 2], [0, leg_y, (self.BELT_TOP - 0.04) / 2], [0.4, 0.42, 0.45, 1], collide=False)
        bin_body = world.add_body(name='bin', pos=[*self.BIN_CENTER, 0])
        half, floor = self.BIN_HALF, self.BIN_FLOOR
        add_static_box(bin_body, [half + 0.01, half + 0.01, 0.01], [0, 0, floor - 0.01], [0.2, 0.55, 0.4, 1])
        for dx, dy, sx, sy in ((half, 0, 0.01, half), (-half, 0, 0.01, half), (0, half, half, 0.01), (0, -half, half, 0.01)):
            add_static_box(bin_body, [sx, sy, 0.05], [dx, dy, floor + 0.05], [0.25, 0.7, 0.5, 0.9])
        add_static_box(bin_body, [0.03, 0.03, (floor - 0.02) / 2], [0, 0, (floor - 0.02) / 2], [0.4, 0.42, 0.45, 1], collide=False)
        for index, name in enumerate(self.object_bodies):
            cube = world.add_body(name=name, pos=[self.BELT_CENTER[0], self.SPAWN_Y - index * 0.6, self.BELT_TOP + self.CUBE_HALF])
            cube.add_freejoint()
            cube.add_geom(type=GEOM_BOX, size=[self.CUBE_HALF] * 3, rgba=self.CUBE_COLORS[index], mass=0.08,
                          friction=[1.2, 0.01, 0.001], condim=4)
        self.welds = {side: WeldAssist(spec, arm.end_body_name, self.object_bodies, side) for side, arm in self.arms.items()}

    def attach(self, world):
        model = world.model
        for arm in self.arms.values():
            arm.attach(model)
        for weld in self.welds.values():
            weld.attach(model)
        self.cubes = [find_id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in self.object_bodies]
        self.gripper_joints = {side: find_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for side, n in self.gripper_joint_names.items()}
        self.belt_speed = 0.05

    def reset(self, world):
        super().reset(world)
        for weld in self.welds.values():
            weld.release(world)
        self.belt_speed = 0.05
        self.picked = 0
        self.missed = 0
        self.counted = set()
        for side, joint in self.gripper_joints.items():
            world.targets[joint] = self.GRIPPER_OPEN

    def stop(self, world):
        self.belt_speed = 0.0

    def held_cubes(self):
        return {weld.held_index for weld in self.welds.values() if weld.held_index is not None}

    def on_belt(self, position):
        inside_x = abs(position[0] - self.BELT_CENTER[0]) <= self.BELT_HALF[0] + 0.005
        inside_y = abs(position[1] - self.BELT_CENTER[1]) <= self.BELT_HALF[1]
        # 접촉 침투(수 mm)로 약간 가라앉아도 벨트 위로 판정
        resting_height = -0.006 <= position[2] - (self.BELT_TOP + self.CUBE_HALF) < 0.012
        return inside_x and inside_y and resting_height

    def before_substep(self, world):
        # 벨트 마찰 모사: 벨트 위에 놓인 큐브의 수평 속도를 벨트 속도로 맞춤(-y 방향 이동)
        held = self.held_cubes()
        for index, cube in enumerate(self.cubes):
            if index in held:
                continue
            _, dof = free_joint_address(world.model, cube)
            if self.on_belt(world.data.xpos[cube]):
                world.data.qvel[dof:dof + 2] = [0.0, -self.belt_speed]

    def respawn(self, world, index):
        start = np.array([self.BELT_CENTER[0], self.SPAWN_Y, self.BELT_TOP + self.CUBE_HALF])
        others = [world.data.xpos[c] for i, c in enumerate(self.cubes) if i != index]
        if any(np.linalg.norm(o[:2] - start[:2]) < 0.15 for o in others):
            return False  # 출발점이 막혀 있으면 다음 프레임에 재시도
        place_free_body(world, self.cubes[index], start)
        return True

    def update(self, world):
        held = self.held_cubes()
        for index, cube in enumerate(self.cubes):
            if index in held:
                continue
            position = world.data.xpos[cube]
            _, dof = free_joint_address(world.model, cube)
            speed = float(np.linalg.norm(world.data.qvel[dof:dof + 3]))
            in_bin = (np.all(np.abs(position[:2] - self.BIN_CENTER) < self.BIN_HALF)
                      and self.BIN_FLOOR < position[2] < self.BIN_FLOOR + 0.1)
            fell = position[2] < 0.3 or position[1] < -self.BELT_HALF[1] - 0.2
            # 점수는 한 번만 집계하고, 재투입은 출발점이 비었을 때(벨트 정지 시 대기) 수행
            if (in_bin and speed < 0.05) or fell:
                if index not in self.counted:
                    self.counted.add(index)
                    if in_bin:
                        self.picked += 1
                    else:
                        self.missed += 1
                if self.respawn(world, index):
                    self.counted.discard(index)
        if self.picked >= 3:
            self.success = True

    def command(self, world, msg):
        handled, result = super().command(world, msg)
        if handled:
            return handled, result
        kind = msg.get('type')
        if kind == 'conveyor':
            self.belt_speed = read_number(msg, 'speed', 0.0, 0.2)
            return True, {'conveyor_speed': self.belt_speed}
        if kind == 'ee_target':
            arm = self.read_arm(msg)
            target = read_position(msg)
            solution, residual = solve_arm_ik(world, arm, target, point_down=msg.get('point_down', True) is not False)
            world.targets.update(solution)
            return True, {'arm': arm.name, 'ik_error_m': round(residual, 4), 'reachable': residual < 0.02}
        if kind == 'grasp':
            arm = self.read_arm(msg)
            enabled = read_bool(msg, 'enabled')
            weld = self.welds[arm.name]
            world.targets[self.gripper_joints[arm.name]] = self.GRIPPER_CLOSED if enabled else self.GRIPPER_OPEN
            if not enabled:
                weld.release(world)
                return True, {'arm': arm.name, 'holding': None}
            if self.assist:
                grip_point = arm.grip_point(world.data)
                distances = [np.linalg.norm(world.data.xpos[c] - grip_point) for c in self.cubes]
                nearest = int(np.argmin(distances))
                if distances[nearest] < 0.05 and weld.held_index is None:
                    # 양팔 협동: 다른 팔이 들고 있는 큐브면 넘겨받기(handover)
                    handed_over = False
                    for other in self.welds.values():
                        if other is not weld and other.held_index == nearest:
                            other.release(world)
                            handed_over = True
                    weld.grab(world, nearest)
                    return True, {'arm': arm.name, 'holding': self.object_bodies[nearest], 'assist': True, 'handover': handed_over}
                return True, {'arm': arm.name, 'holding': None, 'nearest_m': round(float(distances[nearest]), 3)}
            return True, {'arm': arm.name, 'assist': False}
        return False, None

    def on_assist_changed(self, world):
        if not self.assist:
            for weld in self.welds.values():
                weld.release(world)

    def read_arm(self, msg):
        side = msg.get('arm')
        if side not in self.arms:
            raise ValueError('arm은 "right" 또는 "left" 입니다')
        return self.arms[side]

    def status(self, world):
        holding = {side: (self.object_bodies[w.held_index] if w.held_index is not None else None) for side, w in self.welds.items()}
        grip_points = {side: [round(float(v), 3) for v in arm.grip_point(world.data)] for side, arm in self.arms.items()}
        cubes = [[round(float(v), 3) for v in world.data.xpos[c]] for c in self.cubes]
        return {**super().status(world), 'picked': self.picked, 'missed': self.missed, 'conveyor_speed': self.belt_speed,
                'holding': holding, 'grip_points': grip_points, 'cubes': cubes,
                'bin': [*map(float, self.BIN_CENTER), self.BIN_FLOOR],
                'summary': f'수거 {self.picked}/3 · 놓침 {self.missed} · 벨트 {self.belt_speed:.2f} m/s'}


# ---------------------------------------------------------------- Go2: 목표 지점 이동
class QuadrupedGoalMission(Mission):
    title = '목표 지점까지 이동'
    goal = '보행 명령(gait)으로 초록 원 안(반경 0.3m)까지 몸통을 이동'
    steps = ['gait 명령을 보내면 몸체 고정이 자동 해제됩니다', 'vx(전진 m/s)와 wz(회전 rad/s)로 방향을 맞추기',
             'state.mission.goal_local로 목표의 상대 위치를 읽어 폐루프 제어해 보기', 'assist=false로 균형 보조 없이 도전']
    examples = [{'type': 'gait', 'vx': 0.4, 'wz': 0.0}, {'type': 'play'}, {'type': 'gait', 'vx': 0.0, 'wz': 0.0},
                {'type': 'goal', 'x': 2.0, 'y': -1.0}]
    quick = [
        {'label': '↑ 전진', 'commands': [{'type': 'gait', 'vx': 0.4, 'wz': 0.0}, {'type': 'play'}]},
        {'label': '↖ 좌회전', 'commands': [{'type': 'gait', 'vx': 0.2, 'wz': 0.6}, {'type': 'play'}]},
        {'label': '↗ 우회전', 'commands': [{'type': 'gait', 'vx': 0.2, 'wz': -0.6}, {'type': 'play'}]},
        {'label': '↓ 후진', 'commands': [{'type': 'gait', 'vx': -0.2, 'wz': 0.0}, {'type': 'play'}]},
        {'label': '제자리 서기', 'commands': [{'type': 'gait', 'vx': 0.0, 'wz': 0.0}]},
        {'label': '목표 이동(-1.5, 2)', 'commands': [{'type': 'goal', 'x': -1.5, 'y': 2.0}]},
    ]
    has_assist = True
    STAND = (0.0, 0.9, -1.8)
    LEGS = ('FL', 'FR', 'RL', 'RR')
    PHASE_OFFSET = {'FL': 0.0, 'RR': 0.0, 'FR': math.pi, 'RL': math.pi}
    SIDE = {'FL': 1, 'RL': 1, 'FR': -1, 'RR': -1}
    GAIT_FREQUENCY = 2.2
    STRIDE_PER_SPEED = 0.8    # 전진 1 m/s당 허벅지 스윙 진폭(rad), kp=70 기준 명령≈실제 속도
    STRIDE_PER_TURN = 0.3     # 회전 1 rad/s당 좌우 보폭 차이(rad)
    THIGH_LIFT, CALF_LIFT = 0.25, 0.55
    GOAL_RADIUS = 0.3

    def build(self, spec):
        goal = spec.worldbody.add_body(name='goal_marker', pos=[2.5, 1.0, 0.0], mocap=True)
        goal.add_geom(type=GEOM_CYLINDER, size=[self.GOAL_RADIUS, 0.004], pos=[0, 0, 0.004], rgba=[0.3, 0.9, 0.6, 0.55], contype=0, conaffinity=0)
        goal.add_geom(type=GEOM_CYLINDER, size=[0.02, 0.25], pos=[0, 0, 0.25], rgba=[0.3, 0.9, 0.6, 0.9], contype=0, conaffinity=0)

    def attach(self, world):
        model = world.model
        self.base = find_id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        self.mocap = int(model.body_mocapid[find_id(model, mujoco.mjtObj.mjOBJ_BODY, 'goal_marker')])
        self.leg_joints = {leg: [find_id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{part}_joint') for part in ('hip', 'thigh', 'calf')] for leg in self.LEGS}
        self.base_mass = float(model.body_subtreemass[self.base])
        self.goal_xy = np.array([2.5, 1.0])
        self.velocity_command = (0.0, 0.0)
        self.phase = 0.0

    def reset(self, world):
        super().reset(world)
        self.velocity_command = (0.0, 0.0)
        self.was_walking = False
        self.phase = 0.0
        world.data.mocap_pos[self.mocap] = [*self.goal_xy, 0.0]

    def stop(self, world):
        self.velocity_command = (0.0, 0.0)

    def command(self, world, msg):
        handled, result = super().command(world, msg)
        if handled:
            return handled, result
        kind = msg.get('type')
        if kind == 'gait':
            forward = read_number(msg, 'vx', -0.3, 0.6, 0.0)
            turn = read_number(msg, 'wz', -1.0, 1.0, 0.0)
            self.velocity_command = (forward, turn)
            released = world.anchored
            world.anchored = False
            return True, {'vx': forward, 'wz': turn, 'anchor_released': released}
        if kind == 'goal':
            self.goal_xy = np.array([read_number(msg, 'x', -4, 4), read_number(msg, 'y', -4, 4)])
            world.data.mocap_pos[self.mocap] = [*self.goal_xy, 0.0]
            self.success = False
            return True, {'goal': self.goal_xy.tolist()}
        return False, None

    def before_substep(self, world):
        data, timestep = world.data, world.model.opt.timestep
        forward, turn = self.velocity_command
        walking = abs(forward) > 1e-3 or abs(turn) > 1e-3
        stopped_now = self.was_walking and not walking
        self.was_walking = walking
        if walking:
            self.phase = (self.phase + 2 * math.pi * self.GAIT_FREQUENCY * timestep) % (2 * math.pi)
        # 보행 중이거나 막 멈췄을 때만 다리 목표를 덮어씀(평소엔 수동 관절 제어 유지)
        for leg in self.LEGS if walking or stopped_now else ():
            hip, thigh, calf = self.STAND
            if walking:
                leg_phase = self.phase + self.PHASE_OFFSET[leg]
                # 보폭(rad): 전진 성분 + 회전 시 좌우 보폭 차이
                stride = self.STRIDE_PER_SPEED * forward - self.STRIDE_PER_TURN * turn * self.SIDE[leg]
                lift = max(0.0, math.sin(leg_phase))
                thigh += stride * math.cos(leg_phase) - self.THIGH_LIFT * lift
                calf -= self.CALF_LIFT * lift
            for joint, value in zip(self.leg_joints[leg], (hip, thigh, calf)):
                world.targets[joint] = value
        data.xfrc_applied[self.base] = 0
        if self.assist and not world.anchored:
            # 균형 보조: 몸통 기울기(roll/pitch)만 복원 토크로 잡고 yaw는 자유
            rotation = data.xmat[self.base].reshape(3, 3)
            up = rotation[:, 2]
            _, dof = free_joint_address(world.model, self.base)
            angular_velocity = rotation @ data.qvel[dof + 3:dof + 6]
            tilt_rate = angular_velocity - angular_velocity.dot(WORLD_UP) * WORLD_UP
            torque = 120.0 * np.cross(up, WORLD_UP) - 8.0 * tilt_rate
            height_error = 0.27 - data.xpos[self.base][2]
            lift_force = max(0.0, 400.0 * height_error - 40.0 * data.qvel[dof + 2]) if height_error > 0.04 else 0.0
            data.xfrc_applied[self.base] = [0, 0, min(lift_force, 0.6 * self.base_mass * 9.81), *torque]

    def update(self, world):
        if np.linalg.norm(world.data.xpos[self.base][:2] - self.goal_xy) < self.GOAL_RADIUS:
            self.success = True

    def status(self, world):
        position = world.data.xpos[self.base]
        yaw = math.atan2(world.data.xmat[self.base][3], world.data.xmat[self.base][0])
        delta = self.goal_xy - position[:2]
        local = [math.cos(-yaw) * delta[0] - math.sin(-yaw) * delta[1], math.sin(-yaw) * delta[0] + math.cos(-yaw) * delta[1]]
        distance = float(np.linalg.norm(delta))
        return {**super().status(world), 'goal': self.goal_xy.tolist(), 'distance_m': round(distance, 3),
                'goal_local': [round(v, 3) for v in local], 'yaw': round(yaw, 3),
                'gait': {'vx': self.velocity_command[0], 'wz': self.velocity_command[1]},
                'summary': f'목표까지 {distance:.2f} m · 보행 vx {self.velocity_command[0]:.2f} wz {self.velocity_command[1]:.2f}'}


# ---------------------------------------------------------------- G1: 상자 운반
class HumanoidCarryMission(Mission):
    title = '상자 창고 운반'
    goal = '팔레트 위 상자를 양손으로 들어 창고 박스(갈색 테두리) 안에 넣기'
    steps = ['base 명령(가상 대차)으로 상자 앞에 서기, height로 무릎 굽혀 자세 낮추기',
             '양손 ee_target을 상자 좌우 옆면에 붙이고 grasp enabled=true',
             'base로 창고 박스까지 이동 → 박스 위에서 grasp enabled=false',
             '하체 보행은 미구현: 이동은 가상 대차 보조이며 상태에 표시됩니다']
    examples = [{'type': 'base', 'x': 0.2, 'y': 0.0, 'yaw': 0.0, 'height': 0.7},
                {'type': 'ee_target', 'arm': 'left', 'pos': [0.55, 0.17, 0.58]}, {'type': 'grasp', 'enabled': True}]
    quick = [
        {'label': '① 상자 앞 쪼그리기', 'commands': [{'type': 'base', 'x': 0.22, 'y': 0.0, 'yaw': 0.0, 'height': 0.56}, {'type': 'play'}]},
        {'label': '② 양손 옆면 배치', 'commands': [{'type': 'ee_target', 'arm': 'left', 'pos': [0.55, 0.17, 0.58]},
                                               {'type': 'ee_target', 'arm': 'right', 'pos': [0.55, -0.17, 0.58]}]},
        {'label': '③ 잡기', 'commands': [{'type': 'grasp', 'enabled': True}]},
        {'label': '④ 일어서기', 'commands': [{'type': 'base', 'height': 0.79}]},
        {'label': '⑤ 창고 쪽 회전', 'commands': [{'type': 'base', 'yaw': 0.793}]},
        {'label': '⑥ 창고 앞 이동', 'commands': [{'type': 'base', 'x': 1.32, 'y': 1.12}]},
        {'label': '⑦ 놓기', 'commands': [{'type': 'grasp', 'enabled': False}]},
    ]
    object_bodies = ['cargo']
    has_assist = True
    CARGO_HALF = 0.15         # 상자 폭(30cm)이 어깨 폭과 비슷해야 양손으로 옆면을 감쌀 수 있음
    PALLET_HALF = 0.14        # 팔레트를 상자보다 작게: 쪼그릴 때 무릎이 닿지 않도록
    PALLET_TOP = 0.4          # G1 팔 길이(어깨~손바닥 약 0.5m)로 쪼그려 닿는 높이
    MAX_SQUAT = 0.3
    CARGO_START = np.array([0.62, 0.0])
    STORAGE_CENTER = np.array([1.6, 1.4])
    STORAGE_HALF = 0.26
    STAND_HEIGHT = 0.793
    MAX_SPEED, MAX_TURN, MAX_HEIGHT_RATE = 0.4, 0.8, 0.15

    def __init__(self):
        super().__init__()
        self.arms = {
            side: Arm(side, [f'{side}_shoulder_pitch_joint', f'{side}_shoulder_roll_joint', f'{side}_shoulder_yaw_joint',
                             f'{side}_elbow_joint', f'{side}_wrist_roll_joint', f'{side}_wrist_pitch_joint', f'{side}_wrist_yaw_joint'],
                      f'{side}_wrist_yaw_link', palm_offset=[0.1, 0.0, 0.0])
            for side in ('left', 'right')
        }

    def build(self, spec):
        world = spec.worldbody
        pallet = world.add_body(name='pallet', pos=[*self.CARGO_START, 0])
        add_static_box(pallet, [self.PALLET_HALF, self.PALLET_HALF, self.PALLET_TOP / 2], [0, 0, self.PALLET_TOP / 2], [0.6, 0.45, 0.28, 1])
        cargo = world.add_body(name='cargo', pos=[*self.CARGO_START, self.PALLET_TOP + self.CARGO_HALF])
        cargo.add_freejoint()
        cargo.add_geom(type=GEOM_BOX, size=[self.CARGO_HALF] * 3, rgba=[0.85, 0.65, 0.4, 1], mass=0.6, friction=[1.2, 0.01, 0.001], condim=4)
        storage = world.add_body(name='storage', pos=[*self.STORAGE_CENTER, 0])
        half = self.STORAGE_HALF
        for dx, dy, sx, sy in ((half, 0, 0.015, half), (-half, 0, 0.015, half), (0, half, half, 0.015), (0, -half, half, 0.015)):
            add_static_box(storage, [sx, sy, 0.12], [dx, dy, 0.12], [0.55, 0.36, 0.2, 1])
        add_static_box(storage, [half, half, 0.002], [0, 0, 0.002], [0.3, 0.9, 0.6, 0.35], collide=False)
        self.weld = WeldAssist(spec, 'torso_link', ['cargo'], 'carry')

    def attach(self, world):
        model = world.model
        for arm in self.arms.values():
            arm.attach(model)
        self.weld.attach(model)
        self.cargo = find_id(model, mujoco.mjtObj.mjOBJ_BODY, 'cargo')
        self.pelvis = find_id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
        self.pelvis_qpos, _ = free_joint_address(model, self.pelvis)
        self.leg_joints = {side: {part: find_id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{side}_{part}_joint') for part in ('hip_pitch', 'knee', 'ankle_pitch')}
                           for side in ('left', 'right')}
        self.thigh_length, self.shin_length = self.measure_leg(world)

    def measure_leg(self, world):
        data = world.data
        mujoco.mj_forward(world.model, data)
        hip = data.xpos[find_id(world.model, mujoco.mjtObj.mjOBJ_BODY, 'left_hip_pitch_link')]
        knee = data.xpos[find_id(world.model, mujoco.mjtObj.mjOBJ_BODY, 'left_knee_link')]
        ankle = data.xpos[find_id(world.model, mujoco.mjtObj.mjOBJ_BODY, 'left_ankle_pitch_link')]
        return float(np.linalg.norm(hip - knee)), float(np.linalg.norm(knee - ankle))

    def reset(self, world):
        super().reset(world)
        self.weld.release(world)
        pose = world.base[self.pelvis_qpos:self.pelvis_qpos + 7]
        self.base_target = {'x': float(pose[0]), 'y': float(pose[1]), 'yaw': yaw_from_quat(pose[3:7]), 'height': float(pose[2])}
        self.STAND_HEIGHT = float(pose[2])

    def stop(self, world):
        pose = world.base[self.pelvis_qpos:self.pelvis_qpos + 7]
        self.base_target.update(x=float(pose[0]), y=float(pose[1]), yaw=yaw_from_quat(pose[3:7]), height=float(pose[2]))

    def squat_angles(self, drop):
        """골반을 drop(m)만큼 낮출 때 발바닥이 수평·골반 아래에 오도록 하는 고관절/무릎/발목 각도"""
        thigh, shin = self.thigh_length, self.shin_length
        reach = max(0.25, thigh + shin - drop)
        knee = math.pi - math.acos(np.clip((thigh ** 2 + shin ** 2 - reach ** 2) / (2 * thigh * shin), -1, 1))
        thigh_angle = math.asin(np.clip(shin * math.sin(knee) / reach, -1, 1))
        return -thigh_angle, knee, -(knee - thigh_angle)

    def before_substep(self, world):
        if not world.anchored:
            return
        timestep = world.model.opt.timestep
        pose = world.base[self.pelvis_qpos:self.pelvis_qpos + 7]
        # 가상 대차: 목표 위치·방향·높이로 제한 속도 이동
        delta = np.array([self.base_target['x'] - pose[0], self.base_target['y'] - pose[1]])
        distance = np.linalg.norm(delta)
        if distance > 1e-6:
            pose[0:2] += delta / distance * min(distance, self.MAX_SPEED * timestep)
        yaw = yaw_from_quat(pose[3:7])
        yaw_error = wrap_angle(self.base_target['yaw'] - yaw)
        yaw += np.clip(yaw_error, -self.MAX_TURN * timestep, self.MAX_TURN * timestep)
        pose[3:7] = quat_from_yaw(yaw)
        height_error = self.base_target['height'] - pose[2]
        # 선 자세 그대로면 다리 목표를 건드리지 않음(수동 관절 제어 유지)
        if abs(height_error) < 1e-6 and abs(self.STAND_HEIGHT - pose[2]) < 1e-6:
            return
        pose[2] += np.clip(height_error, -self.MAX_HEIGHT_RATE * timestep, self.MAX_HEIGHT_RATE * timestep)
        hip, knee, ankle = self.squat_angles(self.STAND_HEIGHT - pose[2])
        for joints in self.leg_joints.values():
            world.targets[joints['hip_pitch']] = hip
            world.targets[joints['knee']] = knee
            world.targets[joints['ankle_pitch']] = ankle

    def update(self, world):
        position = world.data.xpos[self.cargo]
        inside = np.all(np.abs(position[:2] - self.STORAGE_CENTER) < self.STORAGE_HALF - 0.03) and position[2] < 0.2
        if inside and self.weld.held_index is None:
            self.success = True

    def command(self, world, msg):
        handled, result = super().command(world, msg)
        if handled:
            return handled, result
        kind = msg.get('type')
        if kind == 'base':
            if not world.anchored:
                raise ValueError('가상 대차 이동은 몸체 고정 상태에서만 가능합니다')
            self.base_target['x'] = read_number(msg, 'x', -4, 4, self.base_target['x'])
            self.base_target['y'] = read_number(msg, 'y', -4, 4, self.base_target['y'])
            self.base_target['yaw'] = read_number(msg, 'yaw', -math.pi, math.pi, self.base_target['yaw'])
            self.base_target['height'] = read_number(msg, 'height', round(self.STAND_HEIGHT - self.MAX_SQUAT, 3), round(self.STAND_HEIGHT, 3), self.base_target['height'])
            return True, {'base_target': {k: round(v, 3) for k, v in self.base_target.items()}}
        if kind == 'ee_target':
            side = msg.get('arm')
            if side not in self.arms:
                raise ValueError('arm은 "right" 또는 "left" 입니다')
            solution, residual = solve_arm_ik(world, self.arms[side], read_position(msg))
            world.targets.update(solution)
            return True, {'arm': side, 'ik_error_m': round(residual, 4), 'reachable': residual < 0.03}
        if kind == 'grasp':
            enabled = read_bool(msg, 'enabled')
            if not enabled:
                self.weld.release(world)
                return True, {'holding': False}
            palms = {side: arm.grip_point(world.data) for side, arm in self.arms.items()}
            distances = {side: round(float(np.linalg.norm(p - world.data.xpos[self.cargo])), 3) for side, p in palms.items()}
            if self.assist and all(d < self.CARGO_HALF + 0.1 for d in distances.values()):
                self.weld.grab(world, 0)
                return True, {'holding': True, 'palm_to_cargo_m': distances}
            return True, {'holding': False, 'palm_to_cargo_m': distances,
                          'hint': f'양 손바닥을 상자 중심에서 {self.CARGO_HALF + 0.1:.2f}m 이내(좌우 옆면)에 붙이세요' if self.assist else '보조 꺼짐: 팔 힘과 마찰로만 쥡니다'}
        return False, None

    def on_assist_changed(self, world):
        if not self.assist:
            self.weld.release(world)

    def status(self, world):
        pose = world.base[self.pelvis_qpos:self.pelvis_qpos + 7] if world.anchored else world.data.qpos[self.pelvis_qpos:self.pelvis_qpos + 7]
        cargo = world.data.xpos[self.cargo]
        storage_distance = float(np.linalg.norm(cargo[:2] - self.STORAGE_CENTER))
        palms = {side: [round(float(v), 3) for v in arm.grip_point(world.data)] for side, arm in self.arms.items()}
        return {**super().status(world), 'holding': self.weld.held_index is not None,
                'base': {'x': round(float(pose[0]), 3), 'y': round(float(pose[1]), 3), 'yaw': round(yaw_from_quat(pose[3:7]), 3), 'height': round(float(pose[2]), 3)},
                'cargo': [round(float(v), 3) for v in cargo], 'storage': self.STORAGE_CENTER.tolist(), 'palms': palms,
                'virtual_cart': True,
                'summary': f"상자→창고 {storage_distance:.2f} m · {'들고 있음' if self.weld.held_index is not None else '내려놓음'}"}


MISSIONS = {
    'go2': QuadrupedGoalMission,
    'rby1': ConveyorPickMission,
    'g1': HumanoidCarryMission,
    'allegro': AllegroGraspMission,
}
