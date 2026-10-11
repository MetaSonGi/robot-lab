"""외부 공개 모드: 토큰·허용 출처·동시 접속 제한 검증."""
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
import app as lab_server

PAGES_ORIGIN = 'https://metasongi.github.io'


@pytest.fixture
def token_protected(monkeypatch):
    monkeypatch.setattr(lab_server, 'ACCESS_TOKEN', 'test-token-1234567890')
    return 'test-token-1234567890'


def first_message_type(ws):
    return ws.receive_json()['type']


def test_wrong_or_missing_token_is_rejected(token_protected):
    with TestClient(lab_server.app) as client:
        for query in ('', '?token=wrong', '?token=%ED%95%9C%EA%B8%80'):
            with client.websocket_connect(f'/ws{query}') as ws:
                with pytest.raises(WebSocketDisconnect) as closed:
                    ws.receive_json()
                assert closed.value.code == 4401


def test_correct_token_from_github_pages_is_accepted(token_protected):
    with TestClient(lab_server.app) as client:
        with client.websocket_connect(f'/ws?token={token_protected}', headers={'origin': PAGES_ORIGIN}) as ws:
            assert first_message_type(ws) == 'model'


def test_other_github_pages_origin_is_rejected(token_protected):
    with TestClient(lab_server.app) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect(f'/ws?token={token_protected}', headers={'origin': 'https://someone.github.io'}) as ws:
                ws.receive_json()


def test_session_limit(monkeypatch):
    monkeypatch.setattr(lab_server, 'MAX_SESSIONS', 1)
    with TestClient(lab_server.app) as client:
        with client.websocket_connect('/ws') as first:
            assert first_message_type(first) == 'model'
            with client.websocket_connect('/ws') as second:
                with pytest.raises(WebSocketDisconnect) as closed:
                    second.receive_json()
                assert closed.value.code == 4429
