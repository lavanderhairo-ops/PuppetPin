@echo off
rem Installs Puppet Pin (OFX plugin for DaVinci Resolve) into the Open FX plugin folder.
setlocal
set "SRC=%~dp0PuppetPin.ofx.bundle"
set "DST=%CommonProgramFiles%\OFX\Plugins\PuppetPin.ofx.bundle"
set "SCRIPTS=%ProgramData%\Blackmagic Design\DaVinci Resolve\Fusion\Scripts\Tool"

net session >nul 2>&1
if errorlevel 1 (
    echo Asking for administrator rights...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

tasklist /FI "IMAGENAME eq Resolve.exe" 2>nul | find /I "Resolve.exe" >nul
if not errorlevel 1 (
    echo DaVinci Resolve is running. Close it and run Install.bat again.
    pause
    exit /b 1
)

if not exist "%SRC%\Contents\Win64\PuppetPin.ofx" (
    echo PuppetPin.ofx.bundle was not found next to this file. Unzip everything first.
    pause
    exit /b 1
)

rem rmdir without /s removes a junction or an empty folder; /s /q removes an old copy
if exist "%DST%" rmdir "%DST%" 2>nul
if exist "%DST%" rmdir /s /q "%DST%"
xcopy "%SRC%" "%DST%\" /e /i /y /q >nul
if errorlevel 1 (
    echo Copying failed.
    pause
    exit /b 1
)
rem Fusion page: right-click a Puppet Pin node > Scripts > Puppet Pin - Add Room
if not exist "%SCRIPTS%" mkdir "%SCRIPTS%"
copy /y "%~dp0Puppet Pin - Add Room.lua" "%SCRIPTS%\" >nul
echo.
echo Puppet Pin is installed:
echo   %DST%
echo Start DaVinci Resolve and look for Effects ^> Open FX ^> Filters ^> Puppet Pin.
pause
