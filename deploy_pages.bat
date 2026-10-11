@echo off
rem NOTE: keep this file ASCII-only. cmd.exe misparses UTF-8 Korean text in .bat files.
rem Publish the static/ folder (committed state) to the gh-pages branch = GitHub Pages web version.
setlocal
cd /d "%~dp0"
git diff --quiet -- static
if errorlevel 1 (
    echo [ERROR] static/ has uncommitted changes. Commit them first.
    pause
    exit /b 1
)
set "PAGES_COMMIT="
for /f %%h in ('git subtree split --prefix static') do set "PAGES_COMMIT=%%h"
if not defined PAGES_COMMIT (
    echo [ERROR] git subtree split failed.
    pause
    exit /b 1
)
git push -f origin %PAGES_COMMIT%:refs/heads/gh-pages
if errorlevel 1 (
    echo [ERROR] Push failed. Check "gh auth status".
    pause
    exit /b 1
)
echo Done. https://metasongi.github.io/robot-lab/ updates in about 1 minute.
