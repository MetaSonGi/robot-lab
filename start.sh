#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -d .venv ]; then python3 -m venv .venv; fi
.venv/bin/python -m pip install -r requirements.txt
# 빈 포트 자동 선택(ROBOT_LAB_PORT로 직접 지정 가능)
exec .venv/bin/python app.py
