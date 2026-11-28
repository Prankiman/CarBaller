@echo off
rem Thin wrapper so the Windows build can be started from CMD or Explorer.
rem Usage: tools\build_windows.bat [build_windows.ps1 options]
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_windows.ps1" %*
exit /b %ERRORLEVEL%
