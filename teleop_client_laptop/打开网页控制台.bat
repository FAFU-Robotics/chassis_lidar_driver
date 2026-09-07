@echo off
setlocal EnableExtensions
cd /d "%~dp0"

rem ASCII-only file. UTF-8 Chinese rem comments break GBK cmd.exe.

set "HOST="
set "PORT=9101"
set "CANDIDATES=172.18.101.12,59.79.233.120"
set "FALLBACK_HOST=59.79.233.120"

if exist "%~dp0teleop_client.conf" (
  for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%~dp0teleop_client.conf") do (
    if /i "%%A"=="HOST" set "HOST=%%B"
    if /i "%%A"=="PORT" set "PORT=%%B"
    if /i "%%A"=="CANDIDATE_HOSTS" set "CANDIDATES=%%B"
    if /i "%%A"=="FALLBACK_HOST" set "FALLBACK_HOST=%%B"
  )
)
if "%PORT%"=="" set "PORT=9101"
if "%CANDIDATES%"=="" set "CANDIDATES=172.18.101.12,59.79.233.120"
if "%FALLBACK_HOST%"=="" set "FALLBACK_HOST=59.79.233.120"

if not exist "%~dp0teleop_desktop.py" goto probe_browser

where python >nul 2>&1
if errorlevel 1 goto try_py
python -c "import webview" >nul 2>&1
if errorlevel 1 goto probe_browser
start "" pythonw "%~dp0teleop_desktop.py"
exit /b 0

:try_py
where py >nul 2>&1
if errorlevel 1 goto probe_browser
py -3 -c "import webview" >nul 2>&1
if errorlevel 1 goto probe_browser
start "" py -3 "%~dp0teleop_desktop.py"
exit /b 0

:probe_browser
set "PICKED="
set "PROBE=%HOST%,%CANDIDATES%,%FALLBACK_HOST%"
if exist "%~dp0pick_host.ps1" (
  for /f "delims=" %%H in ('powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pick_host.ps1" -Port "%PORT%" -Hosts "%PROBE%"') do set "PICKED=%%H"
)
if "%PICKED%"=="" if not "%HOST%"=="" set "PICKED=%HOST%"
if "%PICKED%"=="" set "PICKED=172.18.101.12"

set "URL=http://%PICKED%:%PORT%"
start "" "%URL%"
exit /b 0
