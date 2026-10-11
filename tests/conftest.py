import pytest
import app as lab_server
from recorder import DataStore


@pytest.fixture(autouse=True)
def isolated_data_store(tmp_path, monkeypatch):
    """테스트가 실제 data/ 폴더에 실행 기록을 남기지 않도록 임시 폴더로 교체"""
    store = DataStore(tmp_path / 'data', lab_server.MODELS, max_bytes=200 * 1024 * 1024, max_frames_per_run=54000)
    monkeypatch.setattr(lab_server, 'data_store', store)
    return store
