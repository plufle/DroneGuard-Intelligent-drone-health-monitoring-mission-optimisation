@echo off
rem Double-click to start the DroneGuard live demo (website + Raspberry Pi drone).
rem Laptop only, no Pi:  demo.bat -NoPi
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0demo.ps1" %*
pause
