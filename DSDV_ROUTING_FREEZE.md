# DSDV Routing Table Freeze Feature

## Overview
This implementation adds automatic routing table freezing for DSDV protocol. Once each node learns routes to all other nodes in the simulation, its routing table freezes (stops accepting updates). When all nodes' tables are frozen, DSDV routing packet transmission stops globally.

## How It Works

### 1. **Expected Destinations Calculation**
During initialization, each DSDV node calculates how many unique routing table entries it should have:
```
expectedUniqueDestinations = numberOfNodes + numberOfEndNodes + numberOfRescueNodes - 1
```
(Total nodes minus self)

**You can override this** by setting the `dsdvFreezeUniqueCount` parameter explicitly in your INI file.

For example, with:
- 20 relay nodes (loRaNodes[0..19])
- 1 end node (loRaEndNodes[0], ID 1000)
- 1 rescue node (loRaRescueNodes[0], ID 2000)

Each node expects **21 unique destinations** (22 total - 1 self) by default.

### 2. **Routing Table Freeze**
When a node's routing table reaches the expected number of unique destinations:
- The `routingFrozen` flag is set to `true`
- The `routingFrozenTime` timestamp is recorded
- All subsequent route updates are rejected with a log message
- The node announces local convergence

Log example:
```
>>>>>> Node 7: ROUTING TABLE FROZEN at t=45.3 with 21/21 unique destinations <<<<<<
```

### 3. **Global Convergence Detection**
Each node tracks:
- `locallyConverged`: Whether this node has frozen
- `globalNodesConverged`: Static counter tracking how many nodes have frozen
- `globalNodesExpectingConvergence`: Total nodes expected to converge (relay nodes only, excludes end nodes)

When `globalNodesConverged == globalNodesExpectingConvergence`:
- `globalConvergedFired` flag is set globally
- All nodes set `routingPacketsDue = false`
- All nodes set `dsdvPacketDue = false`
- DSDV routing packet transmission stops network-wide

Log example:
```
>>>>>> GLOBAL ROUTING CONVERGENCE REACHED at t=62.8 - All 20 nodes have frozen routing tables <<<<<<
>>>>>> Node 7: DSDV routing packets STOPPED globally <<<<<<
```

### 4. **Self-Message Handling**
When `globalConvergedFired` is true, routing self-messages are immediately deleted:
```cpp
if (globalConvergedFired) {
    EV_INFO << "Node " << nodeId << ": global convergence reached, deleting routing self-message" << endl;
    routingPacketsDue = false;
    if (useDSDV) {
        dsdvPacketDue = false;
    }
    delete msg;
    return;
}
```

## Code Changes

### Modified Files
1. **LoRaNodeApp.h**: Added `expectedUniqueDestinations` member variable
2. **LoRaNodeApp.cc**: Modified 6 sections

### Key Functions Modified

#### `initialize()`
- Uncommented `numberOfNodes` and `numberOfEndNodes` parameter reads
- Calculate `expectedUniqueDestinations` after DSDV self-route is added
- Log the freeze threshold

#### `addOrReplaceBestSingleRoute()`
- Enhanced freeze check to log which destination was rejected
- Added DSDV-specific convergence check:
  - Count unique destinations after each route insertion
  - If `useDSDV && uniqueCount >= expectedUniqueDestinations`: freeze table
  - Log DSDV_FREEZE event to CSV
  - Call `announceLocalConvergenceIfNeeded()` and `tryStopRoutingGlobally()`
- Kept legacy threshold check for backward compatibility

#### `announceLocalConvergenceIfNeeded()`
- Added logging for rejected end nodes
- Added logging for already-converged nodes

#### `tryStopRoutingGlobally()`
- Added progress logging: "Global convergence progress: X/Y nodes converged"
- Added DSDV-specific stop: sets `dsdvPacketDue = false`
- Enhanced logging with warning-level messages

#### `handleSelfMessage()`
- Enhanced global convergence check to delete self-messages and stop DSDV

## Configuration Requirements

### INI Parameters
Your INI file must define:
```ini
**.numberOfNodes = 20              # Number of relay nodes
**.numberOfEndNodes = 1            # Number of end nodes  
**.numberOfRescueNodes = 1         # Number of rescue nodes (optional, defaults to 1)
**.routingProtocol = "dsdv"        # Must use DSDV
**.stopRoutingWhenAllConverged = true  # Enable global stop feature

# Optional: Override auto-calculated freeze threshold
# If not set or -1, auto-calculates as: numberOfNodes + numberOfEndNodes + numberOfRescueNodes - 1
**.loRaNodes[*].LoRaNodeApp.dsdvFreezeUniqueCount = 21     # Explicit threshold
# Or leave commented to use auto-calculation
```

### Example Configuration
From your `routing_between_2_rescue_endnodes.ini`:
```ini
[Config EndNode1000_to_Rescue2000_DSDV_Wellington_Grid2km]
extends = EndNode1000_to_Rescue2000_DSDV_Wellington
description = "Grid topology, 20 relay nodes in 5×4 grid, 2500m spacing"

**.numberOfNodes = 20
**.numberOfEndNodes = 1
**.numberOfRescueNodes = 1

# Auto-calculated freeze threshold: 20 + 1 + 1 - 1 = 21 unique destinations
# To override, uncomment:
# **.loRaNodes[*].LoRaNodeApp.dsdvFreezeUniqueCount = 21

# Node positions...
```

## Expected Behavior

### Phase 1: Route Discovery (0 - ~60s)
- Nodes exchange DSDV routing packets
- Routing tables gradually fill with routes to all destinations
- Each node logs when it reaches threshold and freezes

### Phase 2: Individual Freezes (~30-70s)
- Nodes freeze asynchronously as they learn all routes
- Console shows FROZEN messages for each node
- `delivered_packets/routing_convergence.csv` logs DSDV_FREEZE events

### Phase 3: Global Stop (~60-80s)
- Last node reaches threshold and announces convergence
- Global counter reaches total expected nodes
- All nodes stop sending DSDV packets
- Console shows GLOBAL ROUTING CONVERGENCE REACHED message

### Phase 4: Data Transmission Only (>80s)
- No more DSDV routing packets
- Data packets continue using frozen routing tables
- TTL decrements normally per hop

## Verification

### Check Console Output
Look for these messages:
```
>>>>>> Node X: DSDV freeze will trigger when routing table has 21 unique destinations (total=22 - self) <<<<<<
>>>>>> Node X: ROUTING TABLE FROZEN at t=Y with 21/21 unique destinations <<<<<<
Global convergence progress: 15/20 nodes converged
>>>>>> GLOBAL ROUTING CONVERGENCE REACHED at t=Z - All 20 nodes have frozen routing tables <<<<<<
```

### Check CSV Logs
**delivered_packets/routing_convergence.csv:**
```csv
simTime,event,nodeId,threshold,uniqueCount
45.23,DSDV_FREEZE,0,21,21
46.78,DSDV_FREEZE,1,21,21
...
62.84,DSDV_FREEZE,19,21,21
```

### Check Routing Table Snapshots
**routing_tables/node_X_routing.csv** should show:
- 21 unique destination IDs (all nodes except self)
- No updates after freeze time

## Benefits

1. **Reduced Network Traffic**: DSDV routing packets stop after convergence
2. **Stable Routes**: No route flapping after freeze
3. **Predictable Behavior**: Clear transition from routing phase to data phase
4. **Better Analysis**: Convergence time clearly logged
5. **Resource Efficiency**: Nodes don't waste energy on unnecessary routing updates

## Troubleshooting

### Nodes Not Freezing
- Check `**.numberOfNodes`, `**.numberOfEndNodes`, `**.numberOfRescueNodes` are correct
- Verify all nodes are reachable (check TX power, positions)
- Look for "No route" drops in logs

### Premature Global Stop
- Check `globalNodesExpectingConvergence` matches actual relay node count
- End nodes shouldn't contribute to convergence count (check `isEndNodeHost()`)

### Routes Missing After Freeze
- Frozen routes should remain valid indefinitely
- Check `routingFrozen` flag wasn't set prematurely
- Verify DSDV timers were running before freeze

## Rebuild Instructions

After code changes, rebuild the project:
```bash
cd d:\FYP_Simulations\omnetpp-5.3\git\omnetpp-simulations
# Open OMNeT++ MinGW shell or ensure make is in PATH
make clean
make -j4
```

Or use the rebuild script:
```bash
.\rebuild.bat
```

## Testing

Run your simulation:
```bash
cd simulations
..\flora.exe -u Cmdenv -c EndNode1000_to_Rescue2000_DSDV_Wellington_Grid2km routing_between_2_rescue_endnodes.ini
```

Monitor output for freeze and global convergence messages.
