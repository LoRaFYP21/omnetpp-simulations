@echo off
REM Run simulation with console output captured to file
REM Usage: run_with_logging.bat [config_name]

set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=EndNode1000_to_Rescue2000_DSDV_Wellington

echo Running configuration: %CONFIG%
echo Output will be saved to: results\sim_console_output.txt

cd /d "%~dp0"
..\src\LoRaMesh.exe -u Cmdenv -c %CONFIG% > results\sim_console_output.txt 2>&1

echo.
echo Simulation complete! Checking for DSDV initialization logs...
echo.
findstr /C:"PRE-DSDV" results\sim_console_output.txt
echo.
echo Full log saved to: results\sim_console_output.txt
pause
