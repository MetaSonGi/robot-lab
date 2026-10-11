"""누적 실행 데이터 조회·분석 예제 (표준 라이브러리만 사용).

  python examples/analyze_runs.py                         # 로봇별 누적 요약 + 최근 실행의 관절 추종 오차
  python examples/analyze_runs.py --robot g1 --run <run_id>
  python examples/analyze_runs.py --download data.zip     # 전체 데이터 ZIP 저장
  python examples/analyze_runs.py --server https://xxx.trycloudflare.com --token <토큰>
"""
import argparse
import csv
import io
import json
import math
import urllib.error
import urllib.parse
import urllib.request


class DataClient:
    def __init__(self, server, token):
        self.server = server.rstrip('/')
        self.token = token

    def get(self, path, params=None):
        query = f'?{urllib.parse.urlencode(params)}' if params else ''
        request = urllib.request.Request(self.server + path + query, headers={'X-Lab-Token': self.token} if self.token else {})
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                return response.read()
        except urllib.error.HTTPError as error:
            detail = error.read().decode('utf-8', 'replace')
            raise SystemExit(f'[오류] {path} 요청 실패 (HTTP {error.code}): {detail}')
        except urllib.error.URLError as error:
            raise SystemExit(f'[오류] 서버({self.server})에 연결할 수 없습니다. 서버 실행 여부와 주소를 확인하세요. ({error.reason})')

    def get_json(self, path, params=None):
        return json.loads(self.get(path, params))


def print_summary(summary):
    print(f"{'로봇':8s} {'실행':>5s} {'누적 시간(s)':>12s} {'성공':>5s} {'명령':>6s}")
    for robot, totals in summary['robots'].items():
        print(f"{robot:8s} {totals['runs']:5d} {totals['sim_seconds']:12.1f} {totals['successes']:5d} {totals['commands']:6d}")
    print(f"저장 용량 {summary['used_bytes'] / 1048576:.1f} MB / 한도 {summary['max_bytes'] / 1048576:.0f} MB\n")


def joint_tracking_errors(states_csv_text, joint_names):
    """관절별 목표-실제 RMS 오차(rad 또는 m): 제어 게인·지연 실험 비교용"""
    rows = list(csv.DictReader(io.StringIO(states_csv_text.lstrip('\ufeff'))))
    errors = {}
    for name in joint_names:
        squared = [(float(row[f'{name}_target']) - float(row[f'{name}_q'])) ** 2 for row in rows]
        errors[name] = math.sqrt(sum(squared) / len(squared)) if squared else 0.0
    return len(rows), errors


def main():
    parser = argparse.ArgumentParser(description='실행 데이터 누적 수집 조회')
    parser.add_argument('--server', default='http://127.0.0.1:8000', help='실습 서버 주소(시작 시 출력된 포트 확인)')
    parser.add_argument('--token', default='', help='공개 모드(start_public.bat) 접속 토큰')
    parser.add_argument('--robot', choices=['go2', 'rby1', 'g1', 'allegro'])
    parser.add_argument('--run', help='분석할 run_id (생략 시 가장 최근 실행)')
    parser.add_argument('--download', metavar='ZIP_PATH', help='전체(또는 --robot) 데이터를 ZIP으로 저장')
    args = parser.parse_args()
    client = DataClient(args.server, args.token)

    print_summary(client.get_json('/api/data/summary'))
    if args.download:
        with open(args.download, 'wb') as zip_file:
            zip_file.write(client.get('/api/data/export', {'robot': args.robot} if args.robot else None))
        print(f'ZIP 저장: {args.download}')
        return

    runs = client.get_json('/api/data/runs', {'robot': args.robot, 'limit': 1000} if args.robot else {'limit': 1000})['runs']
    if args.run:
        runs = [run for run in runs if run['run_id'] == args.run]
    if not runs:
        raise SystemExit('분석할 실행 기록이 없습니다. 웹에서 ▶ 실행하거나 examples/missions.py를 돌려 데이터를 쌓으세요.')
    run = runs[0]
    print(f"[{run['robot']}] {run['run_id']} · {run['started_at']} · {run['sim_seconds']}s · "
          f"{'성공' if run['success'] else '미완료'} · {run['final_summary']}")
    states_text = client.get(f"/api/data/runs/{run['robot']}/{run['run_id']}/states.csv").decode('utf-8')
    frame_count, errors = joint_tracking_errors(states_text, run['joints'])
    print(f'상태 {frame_count}프레임 · 관절 추종 RMS 오차 상위 5개')
    for name, error in sorted(errors.items(), key=lambda item: -item[1])[:5]:
        print(f'  {name:32s} {error:.4f}')


if __name__ == '__main__':
    main()
