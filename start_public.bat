@echo off
rem NOTE: keep this file ASCII-only. cmd.exe misparses UTF-8 Korean text in .bat files.
rem External access: lab server (token protected) + Cloudflare quick tunnel.
call "%~dp0start.bat" public
