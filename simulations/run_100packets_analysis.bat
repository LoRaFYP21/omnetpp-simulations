@echo off
REM Run 100 packets analysis for Smart Flooding and Distance Vector Routing
REM This batch file runs both configurations sequentially

cd /d "%~dp0"

echo ========================================
echo Running Smart Flooding (100 packets)
echo ========================================
opp_run_dbg -u Cmdenv -c No_Duplicate_flood_End1000_to_End1001 -n .:../src --cmdenv-express-mode=true -l ../out/gcc-debug/src/flora routing_between_2_endnodes.ini

echo.
echo ========================================
echo Running Distance Vector Routing (100 packets)
echo ========================================
opp_run_dbg -u Cmdenv -c DV_End1000_to_End1001 -n .:../src --cmdenv-express-mode=true -l ../out/gcc-debug/src/flora routing_between_2_endnodes.ini

echo.
echo ========================================
echo All simulations completed!
echo Results saved in results/ folder
echo ========================================
pause
