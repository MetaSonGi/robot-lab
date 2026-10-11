"""미션 검증·예제에서 공용으로 쓰는 헤드리스 실행 도우미(서버 없이 World 직접 구동)."""
import numpy as np


def run_seconds(world, seconds):
    for _ in range(int(round(seconds * 30))):
        world.step()


def send(world, **msg):
    return world.command(msg)


HAND_FIST = {'j0': 0.5, 'j1': 0.9, 'j2': 0.9, 'j3': 0.9, 'thj0': 1.0, 'thj1': 0.8, 'thj2': 0.9, 'thj3': 0.7}


def close_hand(world, ratios=HAND_FIST):
    """Allegro 관절을 범위 비율로 굽힘(웹의 '주먹 쥐기' 프리셋과 같은 값)"""
    for joint in world.joints:
        name = joint['name']
        ratio = ratios.get(name, ratios.get(name[-2:]))
        if ratio is not None:
            send(world, type='joint', joint=joint['id'], target=joint['min'] + ratio * (joint['max'] - joint['min']))


def walk_to_goal(world, timeout=30.0):
    """목표의 로봇 기준 상대 위치(goal_local)로 방향·속도를 정하는 단순 폐루프 보행 제어"""
    import math
    send(world, type='play')
    elapsed = 0.0
    while elapsed < timeout:
        status = world.mission.status(world)
        if status['success']:
            send(world, type='gait', vx=0.0, wz=0.0)
            return status, elapsed
        local_x, local_y = status['goal_local']
        heading_error = math.atan2(local_y, local_x)
        turn = max(-0.8, min(0.8, 1.5 * heading_error))
        forward = 0.45 if abs(heading_error) < 0.5 else 0.1
        send(world, type='gait', vx=forward, wz=turn)
        run_seconds(world, 0.2)
        elapsed += 0.2
    return world.mission.status(world), elapsed


def carry_cargo_to_storage(world, log=print):
    """G1: 상자 앞에 서서 자세를 낮추고, 양손을 상자 옆면에 붙여 들고, 창고 박스 위에서 놓기"""
    import math
    status = world.mission.status(world)
    cargo = np.array(status['cargo'])
    send(world, type='play')
    log('접근', send(world, type='base', x=float(cargo[0] - 0.4), y=float(cargo[1]), yaw=0.0, height=0.56))
    run_seconds(world, 2.5)
    feet = [round(float(world.data.site_xpos[world.model.site(n).id][2]), 3) for n in ('left_foot', 'right_foot')]
    log('쪼그린 자세 발 높이', feet, '골반', world.mission.status(world)['base'])
    half = world.mission.CARGO_HALF
    # 상자 옆면의 몸쪽 절반(x -0.07)을 잡음: 먼저 바깥(여유 6cm), 그다음 옆면 근처(여유 2cm)
    for gap, label in ((0.06, '바깥'), (0.02, '옆면')):
        for side, sign in (('left', 1), ('right', -1)):
            target = [float(cargo[0] - 0.07), float(cargo[1] + sign * (half + gap)), float(cargo[2] + 0.03)]
            log(side, label, send(world, type='ee_target', arm=side, pos=target))
        run_seconds(world, 1.5)
    log('파지', send(world, type='grasp', enabled=True))
    log('일어서기', send(world, type='base', height=float(world.mission.STAND_HEIGHT)))
    run_seconds(world, 2.0)
    storage = np.array(status['storage'])
    current = world.mission.status(world)['base']
    direction = storage - np.array([current['x'], current['y']])
    yaw = math.atan2(direction[1], direction[0])
    # 상자가 골반 앞 약 0.4m에 있으므로 그만큼 덜 가서 멈춤
    stop_at = storage - 0.4 * direction / np.linalg.norm(direction)
    log('회전', send(world, type='base', yaw=yaw))
    run_seconds(world, 2.5)
    log('이동', send(world, type='base', x=float(stop_at[0]), y=float(stop_at[1])))
    run_seconds(world, float(np.linalg.norm(direction)) / 0.4 + 1.5)
    log('상자 위치', world.mission.status(world)['cargo'])
    log('놓기', send(world, type='grasp', enabled=False))
    run_seconds(world, 2.0)
    return world.mission.status(world)


HANDOVER_POINT = [0.33, 0.09, 1.12]   # 몸쪽·높은 위치여야 두 팔이 벨트/서로에 걸리지 않음


def handover_left_to_right(world, log=print):
    """왼팔로 y≈+0.3 큐브를 집어 몸 앞 높은 곳으로 가져온 뒤, 오른팔이 옆에서 넘겨받음"""
    cubes = world.mission.status(world)['cubes']
    index = int(np.argmin([abs(c[1] - 0.3) for c in cubes]))
    cube = [float(v) for v in cubes[index]]
    for label, target in (('왼팔 위로 접근', [cube[0], cube[1], cube[2] + 0.15]),
                          ('왼팔 하강', [cube[0], cube[1], cube[2] + 0.012])):
        log(label, send(world, type='ee_target', arm='left', pos=target))
        run_seconds(world, 1.5)
    log('왼팔 파지', send(world, type='grasp', arm='left', enabled=True))
    log('들기', send(world, type='ee_target', arm='left', pos=[cube[0], cube[1], 1.1]))
    run_seconds(world, 1.5)
    log('전달 위치', send(world, type='ee_target', arm='left', pos=HANDOVER_POINT))
    run_seconds(world, 2.0)
    held = [float(v) for v in world.mission.status(world)['cubes'][index]]
    # 오른팔은 옆(-y)에서 대기 후 수평 접근(point_down 끔)
    for label, offset in (('오른팔 대기', -0.15), ('오른팔 접근', -0.03)):
        log(label, send(world, type='ee_target', arm='right', pos=[held[0], held[1] + offset, held[2]], point_down=False))
        run_seconds(world, 1.5 if offset < -0.1 else 2.0)
    return send(world, type='grasp', arm='right', enabled=True)


def pick_cube_on_conveyor(world, cube_index, arm='right', log=print):
    """정지된 컨베이어 위 큐브 하나를 집어 수거함에 넣는 시퀀스"""
    status = world.mission.status(world)
    cube = np.array(status['cubes'][cube_index])
    bin_x, bin_y, bin_floor = status['bin']
    plan = [
        ('위로 접근', [cube[0], cube[1], cube[2] + 0.15]),
        ('하강', [cube[0], cube[1], cube[2] + 0.012]),
    ]
    for label, target in plan:
        result = send(world, type='ee_target', arm=arm, pos=[float(v) for v in target])
        run_seconds(world, 1.5)
        now = world.mission.status(world)
        log(label, result, '실제 그립점', now['grip_points'][arm], '큐브', now['cubes'][cube_index])
    log('파지', send(world, type='grasp', arm=arm, enabled=True))
    run_seconds(world, 0.6)
    for label, target in (('들기', [cube[0], cube[1], cube[2] + 0.2]), ('수거함 위', [bin_x, bin_y, bin_floor + 0.2])):
        log(label, send(world, type='ee_target', arm=arm, pos=[float(v) for v in target]))
        run_seconds(world, 1.5)
    log('놓기', send(world, type='grasp', arm=arm, enabled=False))
    run_seconds(world, 1.5)
    return world.mission.status(world)
