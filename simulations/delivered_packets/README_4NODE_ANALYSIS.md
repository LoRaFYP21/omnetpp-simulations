# 4-Node Legacy Simulation Analysis

Python script to analyze simulation results for 4 end nodes communicating via legacy routing/flooding.

## Features

The script `analyze_4node_legacy.py` generates comprehensive analysis reports with:

### Energy Analysis
- **Total network energy consumption** (sum of all nodes)
- **Per-node energy breakdown** (relay nodes and end nodes)

### Packet Delivery Analysis
- **Packets sent per source end node**
- **Packets delivered per source (PDR per source)**
- **Average PDR** across all 4 source nodes
- **Per-source PDR** statistics

### Transmission Efficiency
- **Average transmissions per delivered packet**
- **Network overhead ratio** (forwarding/original transmissions)
- **TX_SRC vs TX_FWD event counts**

## Usage

### Basic Usage
```bash
cd simulations/delivered_packets
python analyze_4node_legacy.py
```

The script will:
1. Look for `paths.csv` in current directory
2. Auto-detect the most recent `.sca` file in `../results/`
3. Generate report: `LEGACY_<Routing/Flooding>_<timestamp>.txt`

### Specify Files
```bash
python analyze_4node_legacy.py paths.csv ../results/Four_EndNodes_to_End1004_DV-*.sca
```

### From Analysis Reports Directory
```bash
cd simulations/analysis\ reports
python ../delivered_packets/analyze_4node_legacy.py ../delivered_packets/paths.csv ../results/Four_EndNodes*.sca
```

## Report Format

Generated reports are named: `LEGACY_<Type>_<Timestamp>.txt`

Examples:
- `LEGACY_Routing_20260223_153045.txt` — Distance Vector routing
- `LEGACY_Flooding_20260223_154120.txt` — Smart Flooding

### Report Sections

1. **Energy Consumption**
   - Total network energy (Joules)
   - Per-node breakdown (End nodes + relays)

2. **Packet Delivery Statistics**
   - Per-source: sent, delivered, PDR
   - Overall: total sent, total delivered, average PDR

3. **Transmission Efficiency**
   - Total TX events (SRC + FWD)
   - Average transmissions per delivered packet
   - Forwarding overhead ratio

4. **Summary**
   - Key metrics at a glance

## Requirements

- Python 3.6+
- No external dependencies (uses standard library only)

## Input Files

### paths.csv
Expected columns:
- `simTime`, `event`, `packetSeq`, `src`, `dst`, `currentNode`, `ttlAfterDecr`, `chosenVia`, `nextHopType`

Key events:
- `TX_SRC` — Original transmission from source
- `TX_FWD_DATA` — Forwarding transmission
- `DELIVERED` — Packet delivered to destination

### .sca File
OMNeT++ scalar results file containing:
```
scalar LoRaMesh.loRaNodes[X].LoRaNic.radio.energyConsumer totalEnergyConsumed <value>
scalar LoRaMesh.loRaEndNodes[X].LoRaNic.radio.energyConsumer totalEnergyConsumed <value>
```

## Example Output

```
======================================================================
  4-NODE LEGACY SIMULATION ANALYSIS REPORT
======================================================================

Report Generated: 2026-02-23 15:30:45
Routing Type: Routing

======================================================================
  ENERGY CONSUMPTION
======================================================================

Total Network Energy Consumed: 85.234567 J

Per-Node Energy Consumption:
--------------------------------------------------
  End1000        :   2.345678 J
  End1001        :   2.123456 J
  ...

======================================================================
  PACKET DELIVERY STATISTICS
======================================================================

Per-Source Statistics:
--------------------------------------------------
  End Node 1000:
    Packets Sent:          100
    Packets Delivered:      98
    PDR:                 98.00%

Overall Network Statistics:
--------------------------------------------------
  Total Packets Sent:          400
  Total Packets Delivered:     385
  Average PDR:               96.25%

======================================================================
  TRANSMISSION EFFICIENCY
======================================================================

Total TX_SRC Events:              400
Total TX_FWD Events:             1245
Total TX Events:                 1645
Successfully Delivered:           385

Average Transmissions per
  Delivered Packet:              4.27x

Forwarding Overhead Ratio:       3.11x
```

## Troubleshooting

### "No .sca file found"
Specify the .sca file path manually:
```bash
python analyze_4node_legacy.py paths.csv /path/to/file.sca
```

### "paths.csv not found"
Run from the `delivered_packets` directory or specify full path:
```bash
python analyze_4node_legacy.py /full/path/to/paths.csv /full/path/to/file.sca
```

### Wrong routing type detected
The script auto-detects routing vs flooding by counting UNICAST vs BCAST events. To override, edit the script's `detect_routing_type()` function.

## Integration with Other Scripts

This script works alongside:
- `analyze_both_simulations.py` — Compare flooding vs routing
- `generate_graphs.py` — Visualize comparison results
- `quick_view.py` — Real-time monitoring during simulation
