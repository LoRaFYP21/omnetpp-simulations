# Debug Guide: Capturing DSDV Logs in Cmdenv for Node 2000

## Log Levels Used in LoRaNodeApp.cc

The code uses two main logging levels for DSDV debugging:

### 1. **EV_WARN** (Warning Level) - Critical Debug Messages
These are marked with `>>>>>>>` and are ALWAYS visible in cmdenv:

**DSDV Initialization (lines 442-534):**
- `[PRE-DSDV] Node X routingProtocol='...' useDSDV=...` - Shows if DSDV is enabled
- `[DSDV-INIT] Node X ENTERING DSDV initialization block` - Confirms entering DSDV setup
- `[DSDV-INIT] Node X about to add self-route` - Before creating self-route
- `[DSDV-INIT] Node X added self-route to table, table size now: X` - After self-route added
- `[DSDV-INIT] Node X about to log initial snapshot, table size: X` - Before CSV logging
- `[DSDV-INIT] Node X completed logRoutingSnapshot` - After CSV logging

**DSDV Packet Sending (line 3094, 3130):**
- `[DSDV] No routes to advertise` - When routesToAdvertise is empty
- `[DSDV] Multi-chunk transmission not fully implemented` - Chunking warning

**Data Forwarding (lines 2652, 2834):**
- `[DSDV] Relay node: No route to destination X` - When forwarding fails

### 2. **EV_INFO** (Info Level) - Detailed Debug Messages
These require **module-log-level** configuration to be visible:

**DSDV Initialization Details (lines 482-528):**
- `[DSDV] Added self-route: dest=X` 
- `[DSDV] Added self to changedSet for immediate advertisement`
- `[DSDV] Computed dsdvNeighborTimeout=...`
- `[DSDV] Initialized DSDV routing protocol for node X`
- `[DSDV]   Incremental period: ...`
- `[DSDV]   Full update period: ...`
- `[DSDV]   Triggered min interval: ...`
- `[DSDV]   Route lifetime: ...`
- `[DSDV]   Neighbor timeout: ...`
- `[DSDV]   Chunking enabled: yes/no`
- `[DSDV]   Max entries per packet: ...`

**Timer Events (lines 1082, 1089, 1096, 1108):**
- `[DSDV] Incremental update timer fired at X` - Every ~15 seconds
- `[DSDV] Scheduled incremental update with X changed routes`
- `[DSDV] No changes to advertise, skipping incremental update`
- `[DSDV] Full update timer fired at X` - Every ~120 seconds

**Packet Transmission (lines 3063, 3076, 3128, 3171):**
- `[DSDV] Preparing full dump with X routes`
- `[DSDV] Preparing incremental update with X changed routes`
- `[DSDV] Sending chunk X/Y (... entries, dumpId=...)`
- `[DSDV] Sent routing packet with X routes` - **KEY MESSAGE FOR NODE 2000**

**Packet Reception (lines 1443, 1469, 1486, 1524, 1536):**
- `[DSDV] Received routing packet from node X`
- `[DSDV] Ignoring unreachable advertisement for X`
- `[DSDV] Installed new route to X`
- `[DSDV] Marked route to X as unreachable`
- `[DSDV] Updated route to X`

---

## How to Capture Logs in Cmdenv

### Method 1: Filter Only DSDV Messages (FASTEST - Recommended)

Use the batch script I created:

```batch
.\run_wellington_filtered.bat
```

This captures only lines containing:
- "Debugging"
- "PRE-DSDV" 
- "Node 1000"
- "Node 2000"

**Pros:** Fast execution, small log file
**Cons:** May miss context around issues

### Method 2: Capture ALL Console Output (COMPLETE - Use for Deep Debugging)

```batch
..\src\flora.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington routing_between_2_rescue_endnodes.ini > wellington_full_debug.txt 2>&1
```

**Pros:** Complete log with all initialization messages
**Cons:** Very slow, huge file

### Method 3: Capture Only INFO and WARN for Specific Modules (BALANCED)

Add to your INI file:

```ini
# Enable detailed logging for LoRaNodeApp only
**.loRaRescueNodes[*].LoRaNodeApp.cmdenv-log-level = info
**.loRaEndNodes[*].LoRaNodeApp.cmdenv-log-level = info

# Or enable for all LoRaNodeApp modules
**.LoRaNodeApp.cmdenv-log-level = info

# Global log level (applies to all other modules)
cmdenv-log-level = warn
```

Then run with output redirection:

```batch
..\src\flora.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington routing_between_2_rescue_endnodes.ini > wellington_debug.txt 2>&1
```

---

## Specific Debug Patterns for Node 2000 Issue

### To Check if Node 2000 Initializes DSDV:

**Search for these patterns:**
```powershell
# Must see this - confirms DSDV enabled
Select-String -Path wellington_debug.txt -Pattern "\[PRE-DSDV\] Node 2000"

# Must see this - confirms entering initialization
Select-String -Path wellington_debug.txt -Pattern "\[DSDV-INIT\] Node 2000 ENTERING"

# Must see this - confirms self-route added
Select-String -Path wellington_debug.txt -Pattern "\[DSDV-INIT\] Node 2000 added self-route"

# Must see this - confirms CSV logging worked
Select-String -Path wellington_debug.txt -Pattern "\[DSDV-INIT\] Node 2000 completed logRoutingSnapshot"
```

**Expected SUCCESS output:**
```
[PRE-DSDV] Node 2000 routingProtocol='dsdv' useDSDV=1
[DSDV-INIT] Node 2000 ENTERING DSDV initialization block
[DSDV-INIT] Node 2000 about to add self-route
[DSDV-INIT] Node 2000 added self-route to table, table size now: 1
[DSDV-INIT] Node 2000 about to log initial snapshot, table size: 1
[DSDV-INIT] Node 2000 completed logRoutingSnapshot
```

**If MISSING, node 2000 never initialized DSDV!**

### To Check if Node 2000 Sends DSDV Packets:

**Search for packet transmission:**
```powershell
# Check if timer fired (requires EV_INFO level)
Select-String -Path wellington_debug.txt -Pattern "Node 2000.*timer fired"

# Check if preparing to send (requires EV_INFO level)
Select-String -Path wellington_debug.txt -Pattern "\[DSDV\] Preparing.*update" | Select-String "Node 2000" -Context 0,3

# KEY: Check if actually sent packet (requires EV_INFO level)
Select-String -Path wellington_debug.txt -Pattern "\[DSDV\] Sent routing packet" | Select-String "Node 2000" -Context 1,0
```

**Expected pattern every ~15 seconds:**
```
[DSDV] Incremental update timer fired at 15.2
[DSDV] Preparing incremental update with 1 changed routes
[DSDV] Sent routing packet with 1 routes
```

### To Check Why Node 2000 Doesn't Send:

**Look for these error messages:**
```powershell
# Check if routes list is empty
Select-String -Path wellington_debug.txt -Pattern "No routes to advertise"

# Check if node failed
Select-String -Path wellington_debug.txt -Pattern "Node 2000.*failed"

# Check if changedSet is empty
Select-String -Path wellington_debug.txt -Pattern "No changes to advertise"
```

---

## Quick Diagnostic Commands

After running simulation, check these:

```powershell
# 1. Did node 2000 initialize DSDV?
Select-String -Path node_2000_routing.csv -Pattern "dsdv_initialized"
# Expected: simTime=0,event=dsdv_initialized,...

# 2. Did node 2000 send any packets?
Select-String -Path node_2000_routing.csv -Pattern "dsdv_packet_sent"
# Expected: Multiple entries with increasing simTime

# 3. How many routes does node 2000 have?
(Get-Content node_2000_routing.csv | Measure-Object -Line).Lines
# Expected: > 20 lines (header + routing events)

# 4. Check initialization logs
Select-String -Path ../wellington_debug.txt -Pattern "\[DSDV-INIT\] Node 2000"

# 5. Check packet transmission logs  
Select-String -Path ../wellington_debug.txt -Pattern "\[DSDV\] Sent routing packet.*Node 2000" -Context 2,0
```

---

## Recommended Debugging Workflow

**Step 1:** Run with filtered output (fast check)
```batch
.\run_wellington_filtered.bat
```

**Step 2:** Check if node 2000 appears in ANY log messages
```powershell
Select-String -Path wellington_console_output.txt -Pattern "2000"
```

**Step 3:** If no messages, node 2000 probably not created or initialized. Add module-level logging:

**Edit INI file** - Add these lines to `[Config EndNode1000_to_Rescue2000_DSDV_Wellington]`:
```ini
**.loRaRescueNodes[*].LoRaNodeApp.cmdenv-log-level = info
cmdenv-log-level = warn
cmdenv-express-mode = false
```

**Step 4:** Run full simulation with INFO logging:
```batch
..\src\flora.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington routing_between_2_rescue_endnodes.ini > wellington_full.txt 2>&1
```

**Step 5:** Search for specific issues:
```powershell
# Check initialization sequence
Select-String -Path wellington_full.txt -Pattern "\[PRE-DSDV\] Node 2000" -Context 0,20

# Check timer events
Select-String -Path wellington_full.txt -Pattern "timer fired" | Select-String "2000"

# Check packet sending
Select-String -Path wellington_full.txt -Pattern "Sent routing packet" -Context 5,0 | Select-String "2000"
```

---

## Key Files to Check

1. **Console Log:** `wellington_console_output.txt` or `wellington_debug.txt`
   - Contains EV_WARN and EV_INFO messages
   - Shows initialization sequence
   - Shows timer firings and packet transmissions

2. **Routing CSV:** `routing_tables/node_2000_routing.csv`
   - Contains `dsdv_initialized` event at simTime=0
   - Contains `dsdv_packet_sent` events when transmitting
   - Contains `dsdv_packet_processed` events when receiving

3. **Configuration:** `routing_between_2_rescue_endnodes.ini`
   - Check log-level settings
   - Check DSDV parameter values
   - Check cmdenv settings

---

## Common Issues and Their Log Signatures

### Issue: Node 2000 Never Initializes DSDV
**Log signature:** Missing `[PRE-DSDV] Node 2000` or `useDSDV=0`
**Cause:** Parameter propagation issue (NED file)
**Fix:** Check endRescueNode.ned has DSDV parameters

### Issue: Node 2000 Initializes But Never Sends
**Log signature:** Has `[DSDV-INIT]` but no `[DSDV] Sent routing packet`
**Cause:** Timer not firing or changedSet empty
**Fix:** Check timer scheduling, check if self-route added to changedSet

### Issue: Node 2000 Sends But Other Nodes Don't Receive
**Log signature:** Node 2000 has `Sent routing packet` but other nodes missing `Received routing packet from node 2000`
**Cause:** Physical layer issue (distance, SF, etc.)
**Fix:** Check neighbor cache, LoRa parameters, node positions

