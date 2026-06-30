@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_connected.ps1" %*
exit /b %ERRORLEVEL%
