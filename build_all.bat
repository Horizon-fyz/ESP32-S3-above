@echo off
REM ---------------------------------------------------------------------------
REM Portable build entry. Delegates to idf.ps1, which auto-detects the local
REM ESP-IDF installation. No absolute path is hard-coded here, so this file
REM works on any machine.
REM
REM Usage:
REM   build_all.bat                          (same as: idf.ps1 build)
REM   build_all.bat -p COM3 flash monitor
REM   build_all.bat -p COM3 monitor
REM ---------------------------------------------------------------------------

cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0idf.ps1" %*
set RC=%ERRORLEVEL%

echo.
if %RC% NEQ 0 (
  echo === FAILED, EXIT CODE: %RC% ===
) else (
  echo === DONE ===
)
exit /b %RC%
