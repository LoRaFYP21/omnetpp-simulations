@echo off
REM Quick rebuild script for OMNeT++ simulation
cd /d %~dp0
call d:\FYP_Simulations\omnetpp-5.3\mingwenv.cmd
make MODE=release
