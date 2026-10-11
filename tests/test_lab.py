import json
import numpy as np
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
from app import World, app

@pytest.mark.parametrize('robot,count', [('go2',12),('rby1',24),('g1',29),('allegro',16)])
def test_actual_physics_joint_response(robot,count):
    w=World(robot)
    assert len(w.joints)==count
    j=w.joints[0];initial=w.targets[j['id']]
    w.command({'type':'joint','joint':j['id'],'target':initial+.1})
    w.command({'type':'play'})
    for _ in range(60):w.step()
    q=w.state()['joints'][0]['q']
    assert q>initial+.03
    assert np.isfinite(w.data.qpos).all()
    assert w.data.time>1.9
    w.command({'type':'pause'});t=w.data.time;w.step();assert w.data.time==t
    w.command({'type':'reset'});assert w.data.time==0

@pytest.mark.parametrize('msg',[{'type':'joint','joint':999,'target':0},{'type':'joint','joint':1,'target':float('nan')},{'type':'joint','joint':1,'target':100},{'type':'anchor','enabled':'yes'},{'type':'gains','kp':float('inf'),'kd':2},{'type':'unknown'}])
def test_invalid_commands(msg):
    with pytest.raises(ValueError):World().command(msg)

def test_worlds_are_isolated():
    a,b=World(),World();a.command({'type':'joint','joint':1,'target':.2})
    assert b.targets[1]==0

def test_velocity_actuators_are_not_joint_controlled():
    # RB-Y1 바퀴는 속도 구동기라 관절 목록에서 제외되어야 함
    names={j['name'] for j in World('rby1').joints}
    assert 'left_wheel' not in names and 'right_wheel' not in names
    assert {'left_arm_0','right_arm_0','torso_0','gripper_finger_l1'}<=names

def test_model_mesh_indices():
    for robot in ('go2','rby1','g1','allegro'):
        w=World(robot);d=w.description()
        for mesh in d['meshes'].values():
            assert max(mesh['faces'])<len(mesh['vertices'])//3
        assert len(w.state()['positions'])==w.model.ngeom

def receive_type(ws,kind):
    for _ in range(50):
        msg=ws.receive_json()
        if msg['type']==kind:return msg
    raise AssertionError(kind)

def test_http_and_websocket_protocol():
    with TestClient(app) as client:
        assert client.get('/').status_code==200
        assert client.get('/lib/three.module.js').status_code==200
        assert client.get('/api/robots').json()['hardware_connected'] is False
        with client.websocket_connect('/ws') as ws:
            assert receive_type(ws,'model')['robot']=='go2'
            ws.send_text('[]');assert 'JSON' in receive_type(ws,'error')['message']
            ws.send_json({'type':'select','robot':'allegro'})
            assert receive_type(ws,'model')['robot']=='allegro'
            ws.send_json({'type':'play'});receive_type(ws,'ack')
            s=receive_type(ws,'state');assert len(s['joints'])==16
            ws.send_json({'type':'stop'});receive_type(ws,'ack')
            assert receive_type(ws,'state')['running'] is False

def test_websocket_rejects_cross_origin():
    with TestClient(app) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect('/ws',headers={'origin':'https://untrusted.example'}):pass
