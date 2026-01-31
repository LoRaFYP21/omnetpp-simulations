@echo off
cd /d D:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations\simulations

echo Running simulation with filtered DSDV debug output...
echo This will capture: PRE-DSDV, DSDV-INIT, DSDV messages for nodes 1000 and 2000
echo.

..\src\flora.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington routing_between_2_rescue_endnodes.ini 2>&1 | findstr /C:"[PRE-DSDV]" /C:"[DSDV-INIT]" /C:"[DSDV]" /C:"Node 2000" /C:"Node 1000" > wellington_console_output.txt

echo.
echo Simulation completed. Captured debug messages in wellington_console_output.txt
echo.
echo === Checking Node 2000 Initialization ===
findstr /C:"[PRE-DSDV] Node 2000" wellington_console_output.txt
findstr /C:"[DSDV-INIT] Node 2000" wellington_console_output.txt
echo.
echo === Checking Node 2000 Packet Transmission ===
findstr /C:"[DSDV] Sent routing packet" wellington_console_output.txt | find "2000"
echo.
pause
