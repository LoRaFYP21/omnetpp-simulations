# End Node Distance Tracking Setup

## Problem Solved
The Python script was reading old `.sca` files instead of fresh simulation results, causing constant distance readings. The new solution writes coordinates directly to a CSV file for reliable tracking.

## Solution: CSV-Based Position Tracking

### Changes Made

#### 1. C++ Code (`src/LoRaApp/LoRaEndNodeApp.cc`)
Added code to write end node positions to `simulations/endnode_positions.csv` at the end of each simulation run.

**What was added:**
- File I/O includes (`<fstream>`, `<string>`)
- CSV writing logic in the `finish()` method that records:
  - Timestamp
  - Node ID (1000, 1001, etc.)
  - Position X and Y coordinates
  - Configuration name
  - Run number

#### 2. Python Script (`simulations/endnode_distance_tracker.py`)
Updated to read coordinates from CSV first, with `.sca` files as fallback.

**New function:** `extract_end_node_coordinates_from_csv()`
- Reads the CSV file
- Gets the MOST RECENT position for each end node
- Returns coordinates with metadata (timestamp, config, run number)

### How to Rebuild the Simulation

**Option 1: Using OMNeT++ IDE (Recommended)**
1. Open OMNeT++ IDE
2. Import or open the project: `d:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations`
3. Right-click the project → Build Project
4. Or press Ctrl+B

**Option 2: Command Line (if make is available)**
```bash
cd d:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations
make MODE=release
```

**Option 3: Manual Compilation via OMNeT++ MinGW Environment**
1. Open: `d:\FYP_Simulations\omnetpp-5.3\mingwenv.cmd`
2. Navigate to project: `cd git\omnetpp-simulations`
3. Run: `make MODE=release`

### Testing Without Rebuild (Manual CSV)

If you can't rebuild immediately, you can manually create the CSV file to test:

1. Create `simulations/endnode_positions.csv` with this format:
```csv
timestamp,node_id,position_x,position_y,config_name,run_number
151.127,1000,6513.01,5163.26,routing_between_2_endnodes,0
151.127,1001,8280.99,2159.04,routing_between_2_endnodes,0
```

2. Update positions after each simulation run by checking the OMNeT++ GUI or `.sca` files

### Running the Distance Tracker

After rebuilding and running a simulation:

```powershell
cd simulations
python endnode_distance_tracker.py --just-distance --verbose
```

Expected output:
```
Reading end node positions from endnode_positions.csv...
CSV extraction: {'source': 'CSV', 'csv_file': 'endnode_positions.csv', 'rows_read': 2, ...}
End node 1000→1001 distance: 3485.84 meters
  (from simulation time 151.13s)
```

### How It Works Now

1. **During Simulation:**
   - When each end node finishes (`finish()` method), it appends its position to CSV
   - CSV grows with each run, keeping full history

2. **Python Analysis:**
   - Script reads CSV file
   - Extracts the LAST (most recent) entry for nodes 1000 and 1001
   - Calculates distance
   - Shows timestamp to confirm freshness

3. **Benefits:**
   - ✅ Always uses fresh data (most recent CSV entries)
   - ✅ Clear timestamp showing when positions were recorded
   - ✅ Includes config name and run number for tracking
   - ✅ No confusion with old `.sca` files
   - ✅ Can manually verify/edit CSV if needed

### Verification

After running a new simulation, check that:
1. `simulations/endnode_positions.csv` exists
2. New entries were added (check file size or open it)
3. Timestamp matches your simulation time
4. Distance changes with each run (if using random placement)

### Troubleshooting

**CSV file not created:**
- Ensure C++ code was rebuilt successfully
- Check that simulation runs to completion (calls `finish()` methods)
- Verify file permissions in `simulations/` directory

**Distance still constant:**
- Check if `seed-set = ${processid}` in your `.ini` file
- Verify `deploymentType = "circle"` for end nodes
- Look at CSV file - are X/Y coordinates actually changing?

**Script errors:**
- Install pandas: `pip install pandas`
- Check that you're in the `simulations` directory when running the script
- Use `--verbose` flag to see detailed extraction info

### Full Report Generation

For complete analysis with packet paths and statistics:

```powershell
cd simulations
python endnode_distance_tracker.py
```

This creates a timestamped report file like `endnode_distance_analysis_20251126_201500.txt` with:
- Fresh distance calculation from CSV
- Packet generation and delivery statistics
- Individual packet path analysis
- Transit time statistics
- Network performance summary
