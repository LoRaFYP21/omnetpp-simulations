@echo off
REM Simulation Analysis Script for Windows
REM Run this after each simulation to generate comprehensive report

echo ========================================
echo LoRa Mesh Simulation Analysis
echo ========================================
echo.

REM Check if paths.csv exists
if not exist "delivered_packets\paths.csv" (
    echo ERROR: delivered_packets\paths.csv not found!
    echo Make sure you run this script from the simulations directory
    echo and that a simulation has been completed.
    pause
    exit /b 1
)

echo Analyzing simulation results...
python analyze_simulation_results.py

echo.
echo Analysis complete! Check the generated report file.
echo.
pause