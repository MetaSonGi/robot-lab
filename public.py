"""외부 공개 실행: 실습 서버 + Cloudflare 임시 터널을 함께 띄우고 웹버전 접속 링크를 출력.

    .venv\\Scripts\\python.exe public.py      (또는 start_public.bat)

- 접속 토큰은 .lab_token 파일에 저장해 재사용(삭제하면 새로 발급).
- 터널 주소(*.trycloudflare.com)는 실행할 때마다 바뀌므로 매번 출력된 링크를 공유.
- Ctrl+C로 서버와 터널을 함께 종료.
"""
import os
import re
import secrets
import shutil
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).parent
TOKEN_FILE = ROOT / '.lab_token'
PAGES_URL = os.environ.get('ROBOT_LAB_PAGES_URL', 'https://metasongi.github.io/robot-lab/')
TUNNEL_URL_PATTERN = re.compile(r'https://[a-z0-9-]+\.trycloudflare\.com')
CLOUDFLARED_CANDIDATES = [
    r'C:\Program Files (x86)\cloudflared\cloudflared.exe',
    r'C:\Program Files\cloudflared\cloudflared.exe',
]
SERVER_READY_TIMEOUT_S = 60
TUNNEL_READY_TIMEOUT_S = 60


def find_cloudflared():
    found = shutil.which('cloudflared')
    if found:
        return found
    for candidate in CLOUDFLARED_CANDIDATES:
        if Path(candidate).exists():
            return candidate
    raise SystemExit('[오류] cloudflared를 찾을 수 없습니다. 설치: winget install --id Cloudflare.cloudflared')


def load_or_create_token():
    """기존 토큰 재사용. 파일이 비었거나 깨졌으면 새로 발급"""
    if TOKEN_FILE.exists():
        saved_token = TOKEN_FILE.read_text(encoding='utf-8').strip()
        if re.fullmatch(r'[A-Za-z0-9_-]{16,}', saved_token):
            return saved_token
    new_token = secrets.token_urlsafe(18)
    TOKEN_FILE.write_text(new_token, encoding='utf-8')
    return new_token


def wait_for_server(port, server_process):
    deadline = time.time() + SERVER_READY_TIMEOUT_S
    while time.time() < deadline:
        if server_process.poll() is not None:
            raise SystemExit('[오류] 실습 서버가 시작 직후 종료되었습니다. 위 로그를 확인하세요.')
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/robots', timeout=2) as response:
                if response.status == 200:
                    return
        except OSError:
            time.sleep(0.5)
    raise SystemExit(f'[오류] {SERVER_READY_TIMEOUT_S}초 안에 서버가 응답하지 않습니다.')


def watch_tunnel_output(tunnel_process, found_event, found_url):
    """cloudflared 로그(stderr)에서 공개 주소를 찾아 알림. 이후 로그는 버림"""
    for line in tunnel_process.stdout:
        match = TUNNEL_URL_PATTERN.search(line)
        if match and not found_event.is_set():
            found_url.append(match.group(0))
            found_event.set()


def stop_process(process):
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


def main():
    sys.path.insert(0, str(ROOT))
    from app import choose_server_port   # 포트 자동 선택 규칙을 서버와 공유

    cloudflared = find_cloudflared()
    token = load_or_create_token()
    port = choose_server_port('127.0.0.1')
    server_env = {**os.environ, 'ROBOT_LAB_PORT': str(port), 'ROBOT_LAB_TOKEN': token,
                  'PYTHONIOENCODING': 'utf-8', 'PYTHONUTF8': '1'}
    server_process = tunnel_process = None
    try:
        print(f'[1/2] 실습 서버 시작 (127.0.0.1:{port}, 토큰 보호)', flush=True)
        server_process = subprocess.Popen([sys.executable, str(ROOT / 'app.py')], cwd=ROOT, env=server_env)
        wait_for_server(port, server_process)

        print('[2/2] Cloudflare 터널 연결 중…', flush=True)
        tunnel_process = subprocess.Popen(
            [cloudflared, 'tunnel', '--no-autoupdate', '--url', f'http://127.0.0.1:{port}'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
        found_event, found_url = threading.Event(), []
        threading.Thread(target=watch_tunnel_output, args=(tunnel_process, found_event, found_url), daemon=True).start()
        if not found_event.wait(TUNNEL_READY_TIMEOUT_S):
            raise SystemExit('[오류] 터널 주소를 받지 못했습니다. 인터넷 연결 또는 방화벽을 확인하세요.')

        public_url = found_url[0]
        query = urllib.parse.urlencode({'server': public_url.removeprefix('https://'), 'token': token})
        print('\n' + '=' * 72)
        print(' 외부 접속 준비 완료 (이 창을 닫으면 접속이 끊깁니다)')
        print(f' 웹버전(GitHub) : {PAGES_URL}?{query}')
        print(f' 서버 직접 접속 : {public_url}/?token={token}')
        print(f' 서버 주소/토큰 : {public_url.removeprefix("https://")}  /  {token}')
        print(' 토큰을 바꾸려면 .lab_token 파일을 지우고 다시 실행하세요.')
        print('=' * 72 + '\n', flush=True)

        # 둘 중 하나라도 죽으면 함께 종료
        while server_process.poll() is None and tunnel_process.poll() is None:
            time.sleep(1)
        print('[안내] 서버 또는 터널이 종료되어 공개 실행을 마칩니다.', flush=True)
    except KeyboardInterrupt:
        print('\n[안내] 종료 중…', flush=True)
    finally:
        stop_process(tunnel_process)
        stop_process(server_process)


if __name__ == '__main__':
    main()
