@echo off
echo Running simulation with console output capture...
cd /d D:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations\simulations
..\src\LoRaMesh.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington > wellington_debug.txt 2>&1
echo.
echo Console output saved to wellington_debug.txt
echo Checking for PRE-DSDV logs:
echo.
findstr /C:"PRE-DSDV" wellington_debug.txt
pause