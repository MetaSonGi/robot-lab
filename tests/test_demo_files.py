"""웹버전 데모 녹화 파일(static/demo)이 현재 모델·재생 코드와 맞는지 검증. 모델을 바꾸면 tools/record_demos.py로 다시 녹화."""
import json
from pathlib import Path
import pytest
from app import World

DEMO_DIR = Path(__file__).resolve().parent.parent / 'static' / 'demo'


@pytest.mark.parametrize('robot', ['go2', 'rby1', 'g1', 'allegro'])
def test_demo_file_matches_model(robot):
    demo = json.loads((DEMO_DIR / f'{robot}.json').read_text(encoding='utf-8'))
    world = World(robot)
    assert demo['version'] == 1 and demo['robot'] == robot
    assert len(demo['model']['joints']) == len(world.joints)
    assert len(demo['frames']) >= 2 and demo['fps'] > 0
    rendered = {geom['id'] for geom in demo['model']['geoms']}
    assert set(demo['geom_ids']) == rendered
    for frame in demo['frames']:
        assert len(frame['j']) == len(world.joints)
        assert len(frame['p']) == 7 * len(demo['moving'])
    assert all(0 <= command['f'] <= len(demo['frames']) for command in demo['commands'])
    # 데모는 미션 성공 장면으로 끝나야 함
    assert demo['frames'][-1]['m']['success'] is True
