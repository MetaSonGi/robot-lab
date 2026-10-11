"""Run with .venv Python while the server is running. Independent CLI world."""
import argparse
import asyncio
import json
import websockets

async def main(robot, port):
    async with websockets.connect(f'ws://127.0.0.1:{port}/ws',max_size=64*1024*1024) as ws:
        async def send(**msg):await ws.send(json.dumps(msg))
        await send(type='select',robot=robot)
        while True:
            msg=json.loads(await ws.recv())
            if msg['type']=='model' and msg['robot']==robot:break
        joint=msg['joints'][0]
        print('실제 모델 관절:',joint)
        # Pick a valid mid-range target. Lab protocol, not a manufacturer SDK.
        target=min(joint['max'], (joint['min']+joint['max'])/2+.1)
        await send(type='joint',joint=joint['id'],target=target)
        await send(type='play')
        frames=0
        while frames<90:
            msg=json.loads(await ws.recv())
            if msg['type']=='state':
                frames+=1
                if frames%15==0:
                    q=next(j for j in msg['joints'] if j['id']==joint['id'])
                    print(f"t={msg['time']:.2f} target={q['target']:.3f} actual={q['q']:.3f} effort={q['torque']:.3f}")
        await send(type='stop')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--robot',choices=['go2','rby1','g1','allegro'],default='go2')
    p.add_argument('--port',type=int,default=8000,help='서버 시작 시 출력된 포트 번호')
    args=p.parse_args()
    try:
        asyncio.run(main(args.robot,args.port))
    except OSError as e:
        # 서버 미실행 또는 포트 불일치 시 안내
        raise SystemExit(f'[오류] 서버(127.0.0.1:{args.port})에 연결할 수 없습니다. 서버 실행 여부와 --port 값을 확인하세요. ({e})')
