@echo off
title Stopping NextGen Swarm Simulator
color 0C
echo.
echo  Stopping all simulator processes...
echo.

:: Kill by window title (set in start.bat)
taskkill /fi "WindowTitle eq Swarm Simulator - Backend*" /f >nul 2>&1
taskkill /fi "WindowTitle eq Swarm Simulator - Frontend*" /f >nul 2>&1
taskkill /fi "WindowTitle eq Swarm Backend*" /f >nul 2>&1
taskkill /fi "WindowTitle eq Swarm Frontend*" /f >nul 2>&1

:: Also free the ports in case anything is lingering
for /f "tokens=5" %%a in ('netstat -aon ^| findstr ":8000 " ^| findstr "LISTENING"') do (
    taskkill /pid %%a /f >nul 2>&1
)
for /f "tokens=5" %%a in ('netstat -aon ^| findstr ":5173 " ^| findstr "LISTENING"') do (
    taskkill /pid %%a /f >nul 2>&1
)

echo  [OK] Simulator stopped.
echo.
timeout /t 2 /nobreak >nul
