@echo off
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File "%~dp0arm_debug_panel.ps1" %*
if errorlevel 1 pause
