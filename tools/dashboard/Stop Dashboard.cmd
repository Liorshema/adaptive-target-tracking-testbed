@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Dashboard.ps1" -Action Stop
if errorlevel 1 (
  pause
  exit /b 1
)
exit /b 0
