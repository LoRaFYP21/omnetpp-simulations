# How to Rebuild the Simulation

The C++ code has been updated to write end node positions directly to CSV during simulation runs, completely bypassing `.sca` files to avoid position mix-ups.

## What Changed

**File:** `src/LoRaApp/LoRaEndNodeApp.cc`
- The `finish()` method now writes to `delivered_packets/endnode_positions.csv`
- Each end node appends its position when the simulation completes
- CSV is created with headers automatically if it doesn't exist

## Rebuild Steps

### Option 1: OMNeT++ IDE (Easiest)

1. **Open OMNeT++ IDE**
   - Launch from: `d:\FYP_Simulations\omnetpp-5.3\ide\omnetpp.exe`

2. **Import/Open Project**
   - If not already open: File → Import → Existing Projects into Workspace
   - Browse to: `d:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations`

3. **Clean Build (Recommended)**
   - Right-click project → Clean Project
   - Wait for clean to complete

4. **Build Project**
   - Right-click project → Build Project
   - Or press `Ctrl+B`
   - Wait for "Build Finished" in console

### Option 2: Command Line

1. **Open MinGW Environment**
   ```
   d:\FYP_Simulations\omnetpp-5.3\mingwenv.cmd
   ```

2. **Navigate and Build**
   ```bash
   cd git\omnetpp-simulations
   make clean
   make MODE=release
   ```

## After Rebuild

1. **Run Any Simulation**
   - Use your usual simulation configuration
   - Example: `routing_between_2_endnodes` in OMNeT++ IDE

2. **Check CSV Created**
   - File should appear: `simulations/delivered_packets/endnode_positions.csv`
   - Contains: timestamp, node_id, position_x, position_y, config_name, run_number

3. **Run Distance Tracker**
   ```powershell
   cd simulations
   python endnode_distance_tracker.py --just-distance
   ```

## Verification

After running a simulation, check:

```powershell
# Check CSV exists
Test-Path simulations\delivered_packets\endnode_positions.csv

# View contents
Get-Content simulations\delivered_packets\endnode_positions.csv

# Should show something like:
# timestamp,node_id,position_x,position_y,config_name,run_number
# 151.127,1000,6513.01,5163.26,routing_between_2_endnodes,0
# 151.127,1001,8280.99,2159.04,routing_between_2_endnodes,0
```

## Troubleshooting

**Build errors:**
- Make sure INET framework is properly linked
- Check that OMNeT++ 5.3 environment is set up correctly
- Try "Clean Project" before "Build Project"

**CSV not created:**
- Verify simulation ran to completion (calls `finish()`)
- Check you're running from `simulations` folder as working directory
- Ensure `delivered_packets` folder exists

**Distance still wrong:**
- Delete old CSV: `rm simulations\delivered_packets\endnode_positions.csv`
- Run simulation again
- Fresh CSV will be created with correct positions

## Why This Approach Is Better

✅ **No .sca file mix-ups** - Positions written directly during simulation
✅ **Always fresh data** - CSV updated at end of each run
✅ **Accurate timestamps** - Uses `simTime()` not file modification times
✅ **No manual extraction** - Automatic, no need to run Python first
✅ **Clear provenance** - Config name and run number recorded
