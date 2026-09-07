@echo off
setlocal EnableExtensions
cd /d "%~dp0"
chcp 65001 >nul

set "HOST="
set "PORT=9101"
set "FALLBACK_HOST=59.79.233.120"
if exist "%~dp0teleop_client.conf" (
  for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%~dp0teleop_client.conf") do (
    if /i "%%A"=="HOST" set "HOST=%%B"
    if /i "%%A"=="PORT" set "PORT=%%B"
    if /i "%%A"=="FALLBACK_HOST" set "FALLBACK_HOST=%%B"
  )
)
if "%HOST%"=="" set "HOST=%FALLBACK_HOST%"
if "%PORT%"=="" set "PORT=9101"
set "URL=http://%HOST%:%PORT%"

if exist "%~dp0teleop_desktop.py" (
  where python >nul 2>&1
  if %ERRORLEVEL%==0 (
    python -c "import webview" >nul 2>&1
    if %ERRORLEVEL%==0 (
      start "" pythonw "%~dp0teleop_desktop.py"
      exit /b 0
    )
  )
)

start "" "%URL%"
exit /b 0
