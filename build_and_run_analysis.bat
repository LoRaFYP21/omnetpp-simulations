@echo off
REM Build and Run 100 packets analysis
REM Step 1: Build the project
REM Step 2: Run simulations

echo ========================================
echo Step 1: Building OMNeT++ Project
echo ========================================

cd /d "%~dp0"

REM Check if OMNeT++ environment is available
if exist "C:\omnetpp-5.3\mingwenv.cmd" (
    call C:\omnetpp-5.3\mingwenv.cmd
) else if exist "D:\FYP_Simulations\omnetpp-5.3\mingwenv.cmd" (
    call D:\FYP_Simulations\omnetpp-5.3\mingwenv.cmd
) else if exist "C:\omnetpp-6.0\mingwenv.cmd" (
    call C:\omnetpp-6.0\mingwenv.cmd
) else (
    echo ERROR: OMNeT++ environment not found!
    echo Please modify this script with your OMNeT++ installation path.
    pause
    exit /b 1
)

echo Building project...
make MODE=release

if errorlevel 1 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo Step 2: Running Smart Flooding (100 packets)
echo ========================================
cd simulations
..\src\LoRaMesh.exe -u Cmdenv -c No_Duplicate_flood_End1000_to_End1001 -n .:../src --cmdenv-express-mode=true routing_between_2_endnodes.ini

echo.
echo ========================================
echo Step 3: Running Distance Vector Routing (100 packets)
echo ========================================
..\src\LoRaMesh.exe -u Cmdenv -c DV_End1000_to_End1001 -n .:../src --cmdenv-express-mode=true routing_between_2_endnodes.ini

echo.
echo ========================================
echo All simulations completed!
echo Results saved in results/ folder
echo ========================================
pause
