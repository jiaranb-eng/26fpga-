@echo off
setlocal
if not defined PROCESSOR_ARCHITECTURE set PROCESSOR_ARCHITECTURE=AMD64
call D:\Efinity_2025_1\bin\setup.bat
cd /d "%~dp0"
call D:\Efinity_2025_1\bin\python3.bat video_console.py %*
exit /b %errorlevel%
