# Simulation Analysis Tool - Usage Guide

## Overview
This tool automatically analyzes your LoRa mesh network simulation results and generates comprehensive reports containing all the metrics you requested.

## What the Report Contains

### 1. **Packet Transmission Statistics**
- Total number of data packets node 0 sent
- Number of packets node 0 sent for each destination
- Verification that all expected destinations received transmissions

### 2. **Delivery Success Rate Analysis**  
- Number of nodes that successfully received data packets from node 0
- Overall delivery success rate (percentage)
- List of reachable and unreachable nodes

### 3. **Transmission Timing Analysis**
- Average, minimum, and maximum time between transmission of each data packet
- Total transmission period
- Timing statistics and warnings

### 4. **End-to-End Transit Time Analysis**
- Average, minimum, and maximum time for data packets to travel from node 0 to destinations
- Fastest and slowest deliveries with details
- Complete transit time statistics

### 5. **Summary and Recommendations**
- Overall network performance assessment
- Warnings about timing issues or network problems
- Recommendations for improvement

## How to Use

### Method 1: Automatic (Recommended)
After each simulation, simply run:
```bash
run_analysis.bat
```

### Method 2: Manual Python Script
```bash
python analyze_simulation_results.py
```

### Method 3: Custom Paths
```bash
python analyze_simulation_results.py "custom/path/paths.csv" "custom_report_name.txt"
```

## Generated Files
- Report filename format: `simulation_analysis_report_YYYYMMDD_HHMMSS.txt`
- Contains all requested metrics in structured, readable format
- Automatically timestamped for easy tracking

## Requirements
- Python 3 with pandas library
- Completed simulation with `delivered_packets/paths.csv` file
- Run from the `simulations` directory

## Example Workflow
1. Run your simulation: `./flora -c Node0ToAllNodes_SinglePacket ...`
2. After simulation completes, run: `run_analysis.bat`
3. Open the generated report file for complete analysis
4. Use the report data for performance evaluation and comparison

The tool automatically detects your simulation data and generates a comprehensive report with all the metrics you need for analysis!