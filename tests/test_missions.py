"""미션 4종을 실제 물리로 끝까지 수행해 성공 판정이 나오는지 검증."""
import json
import os
import sys
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from app import World
from mission_helpers import (run_seconds, send, close_hand, walk_to_goal, pick_cube_on_conveyor,
                             handover_left_to_right, carry_cargo_to_storage)


def quiet(*_):
    pass


@pytest.mark.parametrize('robot', ['go2', 'rby1', 'g1', 'allegro'])
def test_mission_messages_are_json(robot):
    world = World(robot)
    description, state = world.description(), world.state()
    json.dumps(description['mission'])
    json.dumps(state['mission'])
    assert description['mission']['title'] and 'summary' in state['mission']


@pytest.mark.parametrize('robot', ['go2', 'rby1', 'g1', 'allegro'])
def test_command_help_examples_are_valid(robot):
    """화면에 보여 주는 명령 예시를 그대로 보내도 오류가 나지 않아야 함"""
    help_entries = World(robot).description()['commands']
    assert help_entries and all(entry['desc'] and isinstance(entry['json'], dict) for entry in help_entries)
    for entry in help_entries:
        if entry['json']['type'] in ('select', 'record'):   # 세션(app.run_session)에서 처리하는 명령
            continue
        World(robot).command(dict(entry['json']))


def test_allegro_grasp_success():
    world = World('allegro')
    send(world, type='play')
    run_seconds(world, 1.5)
    close_hand(world)
    run_seconds(world, 2.0)
    status = world.mission.status(world)
    assert status['success'] and '엄지' in status['touching']


@pytest.mark.parametrize('assist', [True, False])
def test_go2_walks_to_goal(assist):
    world = World('go2')
    send(world, type='assist', enabled=assist)
    send(world, type='goal', x=2.0, y=1.0)
    status, elapsed = walk_to_goal(world, timeout=20)
    assert status['success'], status['summary']


def test_rby1_conveyor_pick():
    world = World('rby1')
    send(world, type='conveyor', speed=0.0)
    send(world, type='play')
    status = pick_cube_on_conveyor(world, 2, arm='right', log=quiet)
    assert status['picked'] == 1


def test_rby1_handover_left_to_right():
    world = World('rby1')
    send(world, type='conveyor', speed=0.0)
    send(world, type='play')
    result = handover_left_to_right(world, log=quiet)
    assert result.get('handover') is True, result


def test_rby1_rejects_bad_commands():
    world = World('rby1')
    with pytest.raises(ValueError):
        send(world, type='ee_target', arm='middle', pos=[0.4, 0, 0.9])
    with pytest.raises(ValueError):
        send(world, type='ee_target', arm='right', pos=[0.4, float('nan'), 0.9])
    with pytest.raises(ValueError):
        send(world, type='conveyor', speed=5)


def test_g1_carries_cargo_into_storage():
    world = World('g1')
    status = carry_cargo_to_storage(world, log=quiet)
    assert status['success'], status['summary']


def test_g1_virtual_cart_requires_anchor():
    world = World('g1')
    result = send(world, type='anchor', enabled=False)
    assert 'warning' in result   # 균형 제어기가 없어 넘어진다는 안내
    with pytest.raises(ValueError):
        send(world, type='base', x=1.0)


def test_g1_dance_plays_and_stops():
    world = World('g1')
    send(world, type='play')
    result = send(world, type='dance', routine='wave')
    assert result == {'dancing': True, 'routine': 'wave'}
    run_seconds(world, 1.5)
    status = world.mission.status(world)
    assert status['dancing'] and status['dance_routine'] == 'wave'
    # 웨이브 1.5s 키프레임(오른팔 들기)에서는 오른팔 pitch 목표가 neutral(0.2)과 달라짐
    pitch_id = world.mission.arms['right'].joint_ids[0]
    assert abs(world.targets[pitch_id] - 0.2) > 0.3
    result = send(world, type='dance_stop')
    assert result == {'dancing': False}
    assert not world.mission.status(world)['dancing']


def test_g1_dance_rejects_bad_routine_and_unanchored():
    world = World('g1')
    with pytest.raises(ValueError):
        send(world, type='dance', routine='tango')
    send(world, type='anchor', enabled=False)
    with pytest.raises(ValueError):
        send(world, type='dance', routine='wave')


def test_g1_steps_while_moving_and_stands_straight_after():
    world = World('g1')
    model, data = world.model, world.data
    feet = [model.body(f'{side}_ankle_roll_link').id for side in ('left', 'right')]
    send(world, type='play')
    send(world, type='base', x=-0.8, yaw=0.6)   # 팔레트 반대쪽 빈 공간
    lift_peaks, stepping_seen = [0.0, 0.0], False
    for _ in range(75):
        world.step()
        stepping_seen |= world.mission.status(world)['stepping']
        lift_peaks = [max(peak, float(data.xpos[foot][2])) for peak, foot in zip(lift_peaks, feet)]
    assert stepping_seen and min(lift_peaks) > 0.06   # 양발 모두 들림(서 있을 때 약 0.033m)
    run_seconds(world, 2.5)
    status = world.mission.status(world)
    assert not status['stepping'] and abs(status['base']['x'] + 0.8) < 0.01
    leg_errors = [abs(world.targets[j['id']] - data.qpos[model.jnt_qposadr[j['id']]])
                  for j in world.joints if any(part in j['name'] for part in ('hip', 'knee', 'ankle'))]
    assert max(leg_errors) < 0.02   # 멈춘 뒤 다리가 비틀린 채 남지 않음
