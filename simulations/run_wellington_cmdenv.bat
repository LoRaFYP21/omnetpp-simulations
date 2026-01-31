@echo off
REM Run Wellington DSDV config in Cmdenv mode with console logging
echo Running Wellington DSDV configuration in command-line mode...
echo Output will be saved to: wellington_console_output.txt
echo.

cd /d "%~dp0"
..\src\LoRaMesh.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington > wellington_console_output.txt 2>&1

echo.
echo Simulation complete! Checking for DSDV initialization...
echo.
echo === Nodes with DSDV enabled (useDSDV=1) ===
findstr /C:"PRE-DSDV" wellington_console_output.txt | findstr "useDSDV=1"
echo.
echo === Node 1000 and 2000 specifically ===
findstr /C:"Node 1000" wellington_console_output.txt | findstr "PRE-DSDV"
findstr /C:"Node 2000" wellington_console_output.txt | findstr "PRE-DSDV"
echo.
echo Full log saved to: wellington_console_output.txt
pause
