"""실행 데이터 누적 수집: 실행 단위 기록, 끄기, 조회 API, 토큰·경로 보호, 용량 한도."""
import csv
import io
import json
import zipfile
from fastapi.testclient import TestClient
import app as lab_server
from recorder import DataStore


def receive_until(ws, predicate, limit=400):
    for _ in range(limit):
        message = ws.receive_json()
        if predicate(message):
            return message
    raise AssertionError('기대한 메시지를 받지 못했습니다')


def run_frames(ws, frame_count):
    """play 후 실행 중 상태를 frame_count개 받을 때까지 대기, 마지막 상태 반환"""
    ws.send_json({'type': 'play'})
    seen = 0
    while seen < frame_count:
        message = receive_until(ws, lambda m: m['type'] == 'state')
        seen += message['running']
    return message


def test_running_session_is_saved_as_run(isolated_data_store):
    with TestClient(lab_server.app) as client, client.websocket_connect('/ws') as ws:
        receive_until(ws, lambda m: m['type'] == 'model')
        ws.send_json({'type': 'joint', 'joint': 1, 'target': 0.2})   # 실행 전 명령도 보관 후 기록
        state = run_frames(ws, 40)
        assert state['recording']['state'] == 'recording'
        ws.send_json({'type': 'joint', 'joint': 999, 'target': 0.1})   # 오류 명령도 기록
        receive_until(ws, lambda m: m['type'] == 'error')
        ws.send_json({'type': 'select', 'robot': 'allegro'})           # 로봇 변경 = 실행 마감
        receive_until(ws, lambda m: m['type'] == 'model')

    runs = isolated_data_store.list_runs()
    assert [run['robot'] for run in runs] == ['go2']   # 실행하지 않은 allegro는 빈 기록을 남기지 않음
    run = runs[0]
    assert run['status'] == 'finished' and run['frames'] >= 40 and run['sim_seconds'] > 1.0
    run_dir = isolated_data_store.root / 'go2' / run['run_id']
    with open(run_dir / 'states.csv', encoding='utf-8-sig', newline='') as states_file:
        rows = list(csv.DictReader(states_file))
    assert len(rows) == run['frames']
    assert {'time', 'base_z', 'FL_thigh_joint_q', 'FL_thigh_joint_target', 'mission_summary'} <= set(rows[0])
    assert float(rows[-1]['time']) > float(rows[0]['time'])
    commands = [json.loads(line) for line in (run_dir / 'commands.jsonl').read_text(encoding='utf-8').splitlines()]
    assert [c['command']['type'] for c in commands] == ['joint', 'play', 'joint']
    assert 'error' in commands[-1] and commands[0]['frame'] == 0


def test_record_command_turns_collection_off(isolated_data_store):
    with TestClient(lab_server.app) as client, client.websocket_connect('/ws') as ws:
        receive_until(ws, lambda m: m['type'] == 'model')
        ws.send_json({'type': 'record', 'enabled': False})
        ack = receive_until(ws, lambda m: m['type'] == 'ack')
        assert ack['result']['state'] == 'off' and 'warning' not in ack
        state = run_frames(ws, 20)
        assert state['recording']['state'] == 'off'
        ws.send_json({'type': 'record', 'enabled': 'yes'})
        assert 'boolean' in receive_until(ws, lambda m: m['type'] == 'error')['message']
    assert isolated_data_store.list_runs() == []


def test_reset_and_label_split_runs(isolated_data_store):
    with TestClient(lab_server.app) as client, client.websocket_connect('/ws') as ws:
        receive_until(ws, lambda m: m['type'] == 'model')
        ws.send_json({'type': 'record', 'enabled': True, 'label': '실험A'})
        run_frames(ws, 10)
        ws.send_json({'type': 'reset'})
        receive_until(ws, lambda m: m['type'] == 'ack')
        run_frames(ws, 10)
    runs = isolated_data_store.list_runs('go2')
    assert len(runs) == 2 and all(run['label'] == '실험A' for run in runs)


def test_data_api_lists_downloads_and_exports(isolated_data_store):
    with TestClient(lab_server.app) as client:
        with client.websocket_connect('/ws') as ws:
            receive_until(ws, lambda m: m['type'] == 'model')
            run_frames(ws, 10)
        summary = client.get('/api/data/summary').json()
        assert summary['robots']['go2']['runs'] == 1 and summary['robots']['g1']['runs'] == 0
        run = client.get('/api/data/runs', params={'robot': 'go2'}).json()['runs'][0]
        response = client.get(f"/api/data/runs/go2/{run['run_id']}/states.csv")
        assert response.status_code == 200 and response.text.lstrip('\ufeff').startswith('frame,time')
        archive = zipfile.ZipFile(io.BytesIO(client.get('/api/data/export').content))
        assert f"go2/{run['run_id']}/states.csv" in archive.namelist() and 'summary.json' in archive.namelist()
        # 경로 조작·없는 파일·잘못된 로봇
        assert client.get(f"/api/data/runs/go2/{run['run_id']}/..%2F..%2Fapp.py").status_code == 404
        assert client.get('/api/data/runs/go2/not-a-run/states.csv').status_code == 404
        assert client.get('/api/data/runs', params={'robot': 'r2d2'}).status_code == 400


def test_data_api_requires_token_in_public_mode(monkeypatch):
    monkeypatch.setattr(lab_server, 'ACCESS_TOKEN', 'test-token-1234567890')
    with TestClient(lab_server.app) as client:
        assert client.get('/api/data/summary').status_code == 401
        assert client.get('/api/data/summary', headers={'X-Lab-Token': 'wrong'}).status_code == 401
        assert client.get('/api/data/summary', headers={'X-Lab-Token': 'test-token-1234567890'}).status_code == 200
        preflight = client.options('/api/data/summary', headers={
            'Origin': 'https://metasongi.github.io', 'Access-Control-Request-Method': 'GET',
            'Access-Control-Request-Headers': 'x-lab-token'})
        assert preflight.headers.get('access-control-allow-origin') == 'https://metasongi.github.io'


def test_storage_limit_stops_recording(tmp_path, monkeypatch):
    tiny_store = DataStore(tmp_path / 'tiny', lab_server.MODELS, max_bytes=20_000, max_frames_per_run=54000)
    monkeypatch.setattr(lab_server, 'data_store', tiny_store)
    with TestClient(lab_server.app) as client, client.websocket_connect('/ws') as ws:
        receive_until(ws, lambda m: m['type'] == 'model')
        ws.send_json({'type': 'play'})
        stopped = receive_until(ws, lambda m: m['type'] == 'state' and m['recording']['state'] == 'stopped')
        assert '한도' in stopped['recording']['reason']
    assert tiny_store.list_runs()[0]['status'] == 'stopped'
