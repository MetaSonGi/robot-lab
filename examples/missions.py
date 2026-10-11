"""미션 자동 풀이 예제: 서버 실행 중에 WebSocket JSON 명령만으로 4개 미션을 수행.

    .venv\\Scripts\\python.exe examples\\missions.py --robot rby1 --port 8010

웹 화면과는 별도의 시뮬레이션 세계(연결마다 독립)에서 실행된다.
"""
import argparse
import asyncio
import json
import math
import websockets

COMMAND_TIMEOUT_S = 5.0


class LabClient:
    """명령 전송 → ack/error 응답 대기, 최신 state 보관, 시뮬레이션 시간 기준 대기"""

    def __init__(self, ws):
        self.ws = ws
        self.state = None
        self.model = None
        self.replies = asyncio.Queue()
        self.model_ready = asyncio.Event()
        self.state_updated = asyncio.Event()

    async def read_forever(self):
        async for raw in self.ws:
            msg = json.loads(raw)
            kind = msg.get('type')
            if kind == 'state':
                self.state = msg
                self.state_updated.set()
            elif kind == 'model':
                self.model = msg
                self.model_ready.set()
            elif kind in ('ack', 'error'):
                await self.replies.put(msg)

    async def select(self, robot):
        self.model_ready.clear()
        await self.ws.send(json.dumps({'type': 'select', 'robot': robot}))
        await asyncio.wait_for(self.model_ready.wait(), 30)
        # select도 ack를 보내므로 소비
        await asyncio.wait_for(self.replies.get(), COMMAND_TIMEOUT_S)
        self.state = None
        await self.wait_state()

    async def call(self, log=True, **msg):
        await self.ws.send(json.dumps(msg))
        reply = await asyncio.wait_for(self.replies.get(), COMMAND_TIMEOUT_S)
        if reply['type'] == 'error':
            raise RuntimeError(f"명령 거부 {msg}: {reply['message']}")
        result = reply.get('result')
        if log:
            print(f"  → {msg['type']}: {result if result is not None else 'ok'}")
        return result

    async def wait_state(self):
        self.state_updated.clear()
        await asyncio.wait_for(self.state_updated.wait(), 10)

    async def wait_sim(self, seconds):
        """서버 연산이 느려도 동작이 같도록 벽시계가 아닌 시뮬레이션 시간으로 대기"""
        await self.wait_state()
        start = self.state['time']
        while self.state['time'] - start < seconds:
            await self.wait_state()

    @property
    def mission(self):
        return self.state['mission']


async def solve_allegro(lab):
    fist = {'j0': 0.5, 'j1': 0.9, 'j2': 0.9, 'j3': 0.9, 'thj0': 1.0, 'thj1': 0.8, 'thj2': 0.9, 'thj3': 0.7}
    await lab.call(type='play')
    await lab.wait_sim(1.5)
    for joint in lab.model['joints']:
        ratio = fist.get(joint['name'], fist.get(joint['name'][-2:]))
        if ratio is not None:
            target = joint['min'] + ratio * (joint['max'] - joint['min'])
            await lab.call(log=False, type='joint', joint=joint['id'], target=target)
    await lab.wait_sim(2.0)


async def solve_go2(lab, goal_x, goal_y):
    await lab.call(type='goal', x=goal_x, y=goal_y)
    await lab.call(type='play')
    for _ in range(150):   # 최대 약 30초
        if lab.mission['success']:
            break
        local_x, local_y = lab.mission['goal_local']
        heading_error = math.atan2(local_y, local_x)
        turn = max(-0.8, min(0.8, 1.5 * heading_error))
        forward = 0.45 if abs(heading_error) < 0.5 else 0.1
        await lab.call(log=False, type='gait', vx=forward, wz=turn)
        await lab.wait_sim(0.2)
    await lab.call(type='gait', vx=0.0, wz=0.0)


RIGHT_PICK_WINDOW_Y = (-0.35, -0.15)   # 오른팔이 수직 자세로 벨트 높이까지 닿는 구간


def cubes_in_pick_window(mission):
    low, high = RIGHT_PICK_WINDOW_Y
    return [c for c in mission['cubes'] if low < c[1] < high and c[2] > 0.75]


async def solve_rby1(lab):
    await lab.call(type='conveyor', speed=0.0)
    await lab.call(type='play')
    bin_x, bin_y, bin_floor = lab.mission['bin']
    for _ in range(3):
        # 작업 구간에 큐브가 들어올 때까지 벨트를 돌렸다가 정지
        if not cubes_in_pick_window(lab.mission):
            await lab.call(type='conveyor', speed=0.12)
            for _ in range(150):
                await lab.wait_sim(0.1)
                if cubes_in_pick_window(lab.mission):
                    break
            await lab.call(type='conveyor', speed=0.0)
            await lab.wait_sim(0.5)
        reachable = cubes_in_pick_window(lab.mission)
        if not reachable:
            print('  오른팔 작업 구간에 큐브가 없음')
            break
        cube_x, cube_y, cube_z = reachable[0]
        for target, wait in (([cube_x, cube_y, cube_z + 0.15], 1.5), ([cube_x, cube_y, cube_z + 0.012], 1.5)):
            await lab.call(type='ee_target', arm='right', pos=target)
            await lab.wait_sim(wait)
        await lab.call(type='grasp', arm='right', enabled=True)
        await lab.wait_sim(0.6)
        for target in ([cube_x, cube_y, cube_z + 0.2], [bin_x, bin_y, bin_floor + 0.2]):
            await lab.call(type='ee_target', arm='right', pos=target)
            await lab.wait_sim(1.5)
        await lab.call(type='grasp', arm='right', enabled=False)
        await lab.wait_sim(1.5)
        print(f"  수거 {lab.mission['picked']}개")
        if lab.mission['success']:
            break


async def solve_g1(lab):
    await lab.call(type='play')
    cargo_x, cargo_y, cargo_z = lab.mission['cargo']
    storage_x, storage_y = lab.mission['storage'][:2]
    half = 0.15   # 상자 반너비(missions.HumanoidCarryMission.CARGO_HALF)
    stand_height = lab.mission['base']['height']
    await lab.call(type='base', x=cargo_x - 0.4, y=cargo_y, yaw=0.0, height=0.56)
    await lab.wait_sim(2.5)
    for gap in (0.06, 0.02):   # 바깥에서 옆면으로 양손 모으기
        await lab.call(type='ee_target', arm='left', pos=[cargo_x - 0.07, cargo_y + half + gap, cargo_z + 0.03])
        await lab.call(type='ee_target', arm='right', pos=[cargo_x - 0.07, cargo_y - half - gap, cargo_z + 0.03])
        await lab.wait_sim(1.5)
    await lab.call(type='grasp', enabled=True)
    await lab.call(type='base', height=stand_height)
    await lab.wait_sim(2.0)
    base = lab.mission['base']
    dx, dy = storage_x - base['x'], storage_y - base['y']
    distance = math.hypot(dx, dy)
    await lab.call(type='base', yaw=math.atan2(dy, dx))
    await lab.wait_sim(2.5)
    # 상자가 골반 앞 약 0.4m에 있으므로 그만큼 덜 이동
    await lab.call(type='base', x=storage_x - 0.4 * dx / distance, y=storage_y - 0.4 * dy / distance)
    await lab.wait_sim(distance / 0.4 + 1.5)
    await lab.call(type='grasp', enabled=False)
    await lab.wait_sim(2.0)


async def main(robot, port, goal):
    async with websockets.connect(f'ws://127.0.0.1:{port}/ws', max_size=64 * 1024 * 1024) as ws:
        lab = LabClient(ws)
        reader = asyncio.create_task(lab.read_forever())
        try:
            await lab.select(robot)
            print(f"[{robot}] {lab.model['mission']['title']} — {lab.model['mission']['goal']}")
            if robot == 'allegro':
                await solve_allegro(lab)
            elif robot == 'go2':
                await solve_go2(lab, *goal)
            elif robot == 'rby1':
                await solve_rby1(lab)
            else:
                await solve_g1(lab)
            await lab.wait_state()
            print(f"결과: {'성공' if lab.mission['success'] else '미완료'} · {lab.mission['summary']}")
        finally:
            reader.cancel()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='미션 자동 풀이(WebSocket JSON 명령 예제)')
    parser.add_argument('--robot', choices=['go2', 'rby1', 'g1', 'allegro'], default='allegro')
    parser.add_argument('--port', type=int, default=8000, help='서버 시작 시 출력된 포트 번호')
    parser.add_argument('--goal', type=float, nargs=2, default=[2.5, 1.0], metavar=('X', 'Y'), help='Go2 목표 좌표(±4m)')
    args = parser.parse_args()
    try:
        asyncio.run(main(args.robot, args.port, args.goal))
    except OSError as error:
        raise SystemExit(f'[오류] 서버(127.0.0.1:{args.port})에 연결할 수 없습니다. 서버 실행 여부와 --port 값을 확인하세요. ({error})')
    except (RuntimeError, asyncio.TimeoutError) as error:
        raise SystemExit(f'[오류] 미션 실행 중단: {error!r}')
