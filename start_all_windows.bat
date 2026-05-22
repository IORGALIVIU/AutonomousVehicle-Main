@echo off
setlocal enabledelayedexpansion
REM ============================================================
REM   WebRTC + MQTT Windows Launcher
REM   Pornește automat toate componentele necesare
REM ============================================================

echo.
echo ============================================
echo   WebRTC + MQTT Windows Launcher
echo ============================================
echo.

REM Schimbă la directorul script-ului
cd /d "%~dp0"

REM Verifică dacă venv există
if not exist "venv\Scripts\activate.bat" (
    if not exist "..\venv\Scripts\activate.bat" (
        echo [ERROR] Virtual environment not found!
        echo Please create venv first: python -m venv venv
        pause
        exit /b 1
    )
    REM venv e un nivel mai sus
    set VENV_PATH=..\venv
) else (
    set VENV_PATH=venv
)

echo [1/6] Activating virtual environment...
call %VENV_PATH%\Scripts\activate.bat
if errorlevel 1 (
    echo [ERROR] Failed to activate venv!
    pause
    exit /b 1
)
echo [OK] Virtual environment activated
echo.

echo [1.5/6] Cleaning up old processes (ports 8080, 1883)...
powershell -NoProfile -Command ^
    "$ports = @(8080, 1883); " ^
    "foreach ($p in $ports) { " ^
    "    Get-NetTCPConnection -LocalPort $p -State Listen -EA SilentlyContinue | " ^
    "    Select-Object -ExpandProperty OwningProcess -Unique | " ^
    "    Where-Object { $_ -gt 4 } | " ^
    "    ForEach-Object { " ^
    "        Write-Host ('Killing PID {0} on port {1}' -f $_, $p); " ^
    "        Stop-Process -Id $_ -Force -EA SilentlyContinue " ^
    "    } " ^
    "}"
timeout /t 1 >nul
echo [OK] Ports 8080 and 1883 are clean
echo.

REM Verifică dacă mosquitto.conf există
if not exist "receiver\mosquitto.conf" (
    echo [2/6] Creating mosquitto.conf...
    (
        echo listener 1883 0.0.0.0
        echo allow_anonymous true
        echo log_type all
        echo log_dest stdout
    ) > receiver\mosquitto.conf
    echo [OK] mosquitto.conf created
) else (
    echo [2/6] mosquitto.conf found
)
echo.

REM Verifică dacă Mosquitto este instalat
where mosquitto >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Mosquitto not found!
    echo Please install Mosquitto: https://mosquitto.org/download/
    echo.
    echo Alternative: Run components manually without Mosquitto
    pause
    exit /b 1
)

echo [3/6] Starting Mosquitto broker...
start "MQTT Broker" cmd /k "mosquitto -c receiver\mosquitto.conf -v"

REM Asteapta activ pana la 10s
set READY=0
for /l %%i in (1,1,10) do (
    if !READY!==0 (
        netstat -ano | findstr :1883 | findstr LISTENING >nul
        if not errorlevel 1 set READY=1
        if !READY!==0 timeout /t 1 >nul
    )
)
if !READY!==0 (
    echo [ERROR] Mosquitto failed to start or bind to port 1883!
    echo Check the newly opened Mosquitto window for error messages.
    pause
    exit /b 1
)
echo [OK] Mosquitto started successfully
echo.

echo [4/6] Starting Signaling Server...
start "Signaling Server" cmd /k "cd receiver && python signaling_server.py"

REM Asteapta activ pana la 10s
set READY=0
for /l %%i in (1,1,10) do (
    if !READY!==0 (
        netstat -ano | findstr :8080 | findstr LISTENING >nul
        if not errorlevel 1 set READY=1
        if !READY!==0 timeout /t 1 >nul
    )
)
if !READY!==0 (
    echo [ERROR] Signaling server failed to start or bind to port 8080!
    echo Check the newly opened Signaling Server window for error messages.
    pause
    exit /b 1
)
echo [OK] Signaling server started successfully
echo.

echo [5/6] Starting Receiver GUI...
timeout /t 1 >nul
echo [OK] Launching GUI...
echo.

echo [6/6] All components started!
echo.
echo ============================================
echo   READY! Now start sender on Raspberry Pi
echo ============================================
echo.
echo Windows IP: 
ipconfig | findstr "IPv4" | findstr "192.168"
echo.
echo On Raspberry Pi, run:
echo   python3 sender/sender_mqtt.py --video video.mp4 --server-ip YOUR_WINDOWS_IP --mqtt-broker YOUR_WINDOWS_IP
echo.
echo Starting Receiver GUI...

REM Pornește GUI în fereastra curentă
cd receiver
python receiver_gui_mqtt.py

echo.
echo ============================================
echo   Receiver GUI closed
echo ============================================
echo.
echo Press any key to exit...
pause
