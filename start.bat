@echo off
rem NOTE: keep this file ASCII-only. cmd.exe misparses UTF-8 Korean text in .bat files.
setlocal
cd /d "%~dp0"

rem Use Python 3.13/3.12 only (MuJoCo 3.15 wheels are verified for these versions)
if exist .venv\Scripts\python.exe goto install

set "PYTHON_SELECTOR="
py -3.13 -c "import sys" >nul 2>&1 && set "PYTHON_SELECTOR=-3.13"
if not defined PYTHON_SELECTOR py -3.12 -c "import sys" >nul 2>&1 && set "PYTHON_SELECTOR=-3.12"
if not defined PYTHON_SELECTOR (
    echo [ERROR] Python 3.13 or 3.12 64-bit is required: https://www.python.org/downloads/
    pause
    exit /b 1
)

echo Creating virtual environment with Python %PYTHON_SELECTOR% ...
py %PYTHON_SELECTOR% -m venv .venv
if errorlevel 1 (
    echo [ERROR] Failed to create virtual environment.
    pause
    exit /b 1
)

:install
.venv\Scripts\python.exe -m pip install -q -r requirements.txt
if errorlevel 1 (
    echo [ERROR] Package install failed. Check your internet connection.
    pause
    exit /b 1
)

rem Force UTF-8 so Korean messages printed by Python display correctly
set "PYTHONIOENCODING=utf-8"
set "PYTHONUTF8=1"
rem "start.bat public" = server + Cloudflare tunnel (external access)
set "ENTRY_SCRIPT=app.py"
if /i "%~1"=="public" set "ENTRY_SCRIPT=public.py"
echo Press Ctrl+C to stop the server.
.venv\Scripts\python.exe %ENTRY_SCRIPT%
if errorlevel 1 (
    echo [ERROR] Server failed. See the message above.
    pause
    exit /b 1
)
