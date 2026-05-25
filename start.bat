@echo off
title NextGen Swarm Simulator - Launcher
color 0A
cls

echo.
echo  ============================================
echo   NextGen Swarm Simulator - Starting up...
echo  ============================================
echo.

:: ── Locate this script's directory (works from anywhere) ──────────────────
set ROOT=%~dp0
:: Remove trailing backslash
if "%ROOT:~-1%"=="\" set ROOT=%ROOT:~0,-1%

:: ── Check Python ──────────────────────────────────────────────────────────
python --version >nul 2>&1
if errorlevel 1 (
    echo  [ERROR] Python not found.
    echo  Please install Python 3.11+ from https://python.org/downloads
    echo  Make sure to check "Add Python to PATH" during install.
    echo.
    pause
    exit /b 1
)
echo  [OK] Python found

:: ── Check Node ────────────────────────────────────────────────────────────
node --version >nul 2>&1
if errorlevel 1 (
    echo  [ERROR] Node.js not found.
    echo  Please install Node.js 20+ from https://nodejs.org
    echo.
    pause
    exit /b 1
)
echo  [OK] Node.js found

:: ── Check uvicorn ─────────────────────────────────────────────────────────
uvicorn --version >nul 2>&1
if errorlevel 1 (
    echo.
    echo  [SETUP] Installing Python dependencies (first run only)...
    pip install -r "%ROOT%\requirements.txt"
    if errorlevel 1 (
        echo  [ERROR] pip install failed. Check your internet connection.
        pause
        exit /b 1
    )
)
echo  [OK] Python dependencies ready

:: ── Install npm packages if missing ───────────────────────────────────────
if not exist "%ROOT%\frontend\node_modules" (
    echo.
    echo  [SETUP] Installing frontend dependencies (first run only)...
    cd /d "%ROOT%\frontend"
    call npm install
    if errorlevel 1 (
        echo  [ERROR] npm install failed. Check your internet connection.
        pause
        exit /b 1
    )
    cd /d "%ROOT%"
)
echo  [OK] Frontend dependencies ready

echo.
echo  Starting backend server on http://localhost:8000 ...
start "Swarm Simulator - Backend" cmd /k ^
    "title Swarm Backend && color 0B && cd /d "%ROOT%" && set PYTHONPATH=%ROOT%\backend;%ROOT% && echo Backend starting... && uvicorn backend.src.server:app --host 0.0.0.0 --port 8000"

echo  Waiting for backend to initialise...
timeout /t 4 /nobreak >nul

echo  Starting frontend on http://localhost:5173 ...
start "Swarm Simulator - Frontend" cmd /k ^
    "title Swarm Frontend && color 0D && cd /d "%ROOT%\frontend" && npm run dev"

echo  Waiting for frontend to compile...
timeout /t 6 /nobreak >nul

echo.
echo  Opening browser...
start "" http://localhost:5173

echo.
echo  ============================================
echo   Both servers are running.
echo.
echo   Browser: http://localhost:5173
echo   API:     http://localhost:8000
echo   Docs:    http://localhost:8000/docs
echo.
echo   Close the Backend and Frontend windows
echo   to shut everything down.
echo  ============================================
echo.
pause
