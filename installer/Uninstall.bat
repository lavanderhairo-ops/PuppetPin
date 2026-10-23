@echo off
rem Removes Puppet Pin (OFX plugin for DaVinci Resolve).
setlocal
set "DST=%CommonProgramFiles%\OFX\Plugins\PuppetPin.ofx.bundle"
set "SCRIPT=%ProgramData%\Blackmagic Design\DaVinci Resolve\Fusion\Scripts\Tool\Puppet Pin - Add Room.lua"

net session >nul 2>&1
if errorlevel 1 (
    echo Asking for administrator rights...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

tasklist /FI "IMAGENAME eq Resolve.exe" 2>nul | find /I "Resolve.exe" >nul
if not errorlevel 1 (
    echo DaVinci Resolve is running. Close it and run Uninstall.bat again.
    pause
    exit /b 1
)

if not exist "%DST%" (
    echo Puppet Pin is not installed.
    pause
    exit /b 0
)
rmdir "%DST%" 2>nul
if exist "%DST%" rmdir /s /q "%DST%"
if exist "%SCRIPT%" del /q "%SCRIPT%"
echo Puppet Pin was removed.
pause
