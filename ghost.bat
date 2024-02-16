@echo off

IF EXIST ".pio\build\simu\program.exe" (
    start /B call ".pio\build\simu\program.exe"
) ELSE (
    echo Error: Please build the project in simulation mode before executing ghost.
    exit /b 1
)