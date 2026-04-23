"""
analysis_1VIC_20grid_DV_SF.py

Single-victim (1VIC) 20-grid analysis for DV or Smart Flooding runs.
Reads simulations/delivered_packets/paths.csv and the most recent .sca file
in simulations/results/, then writes one TXT report into this folder.

Output filename format:
    <DV or SF>_<distance_m>_<YYYYMMDD_HHMMSS>_1VIC_<NodeFail|NoFail>_20grid.txt

Routing label is detected from paths.csv:
    - If any nextHopType == UNICAST  -> "DV"
    - Else                           -> "SF"

Metrics written:
- PDR (packets delivered / packets sent)
- Average / median / stddev / min / max DATA latency
- Total network energy consumed (from latest .sca)
- Node positions & Euclidean distance between two end nodes
- Avg Time On Air per Delivered Packet
- Avg Transmissions per Delivered Packet
"""

import csv
import math
import statistics
from pathlib import Path
from datetime import datetime
from typing import Optional

# Paths

def find_simulations_root(start_dir: Path) -> Path:
    """Find the `simulations/` folder regardless of where this script lives."""
    for candidate in (start_dir, *start_dir.parents):
        if (candidate / "delivered_packets").exists():
            return candidate
    return start_dir


SCRIPT_DIR = Path(__file__).resolve().parent
SIM_ROOT = find_simulations_root(SCRIPT_DIR)              # simulations/
RESULTS_DIR = SIM_ROOT / "results"                        # simulations/results
PATHS_CSV = SIM_ROOT / "delivered_packets" / "paths.csv"


# ---------------------------------------------------------------------------
# Helpers for .sca parsing and geometry
# ---------------------------------------------------------------------------

def latest_sca(results_dir: Path):
    """Return the most recently modified .sca file in results_dir, or None."""
    sca_files = sorted(results_dir.glob("*.sca"), key=lambda p: p.stat().st_mtime, reverse=True)
    return sca_files[0] if sca_files else None


def parse_sca(sca_path: Path):
    """Parse an OMNeT++ .sca file into a dict[(module, metric)] -> float."""
    scalars = {}
    with sca_path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip()
            if line.startswith("scalar "):
                # format: scalar <module> <metric> <value>
                parts = line.split()
                if len(parts) >= 4:
                    module = parts[1]
                    metric = " ".join(parts[2:-1])
                    try:
                        value = float(parts[-1])
                    except ValueError:
                        continue
                    scalars[(module, metric)] = value
    return scalars


def parse_sca_params(sca_path: Path):
    """Parse an OMNeT++ .sca file's `param` lines into dict[paramName] -> rawValueString."""
    params = {}
    with sca_path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if line.startswith("param "):
                # format: param <name> <value>
                # Note: <name> itself can include spaces? In OMNeT++ it usually doesn't.
                # We'll split into 3 parts only to preserve raw values.
                parts = line.split(maxsplit=2)
                if len(parts) == 3:
                    _, name, raw_value = parts
                    params[name] = raw_value.strip()
    return params


def _parse_time_like_seconds(raw_value: str):
    """Best-effort parse '5s'/'0s'/'-1s' into float seconds; return None on failure."""
    v = raw_value.strip().strip('"')
    if v.endswith("s"):
        try:
            return float(v[:-1])
        except ValueError:
            return None
    return None


def detect_failure_status_from_sca(sca_path: Optional[Path]) -> str:
    """Return 'NodeFail' or 'NoFail' based on failure-related params in the .sca."""
    if sca_path is None or not sca_path.exists():
        return "UnknownFail"

    # Fast path: check iteration variables/metadata first (covers cases where failure
    # settings are expressed via iteration vars but not repeated as params).
    try:
        with sca_path.open(encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("itervar relayFailCount "):
                    try:
                        if int(line.split()[-1]) > 0:
                            return "NodeFail"
                    except ValueError:
                        pass
                if line.startswith("attr iterationvars ") or line.startswith("attr measurement "):
                    if "relayFailCount=" in line or "timeToFailure" in line:
                        return "NodeFail"
                # Stop scanning early once we're past the header/params section.
                if line.startswith("scalar "):
                    break
    except Exception:
        pass

    params = parse_sca_params(sca_path)

    # 1) Coordinated subset failure mode
    subset_raw = params.get("**.loRaNodes[*].LoRaNodeApp.globalFailureSubsetCount")
    if subset_raw is not None:
        try:
            subset_count = int(str(subset_raw).strip().strip('"'))
        except ValueError:
            subset_count = None
        if subset_count is not None and subset_count > 0:
            return "NodeFail"

    # 2) Per-node failure mode (timeToFailure)
    ttf_raw = params.get("**.loRaNodes[*].LoRaNodeApp.timeToFailure")
    if ttf_raw is not None:
        ttf = ttf_raw.strip().strip('"')

        # Expressions like uniform(0s,5000s) indicate failures are enabled.
        if "(" in ttf and ")" in ttf:
            if ttf.startswith("uniform") or ttf.startswith("exponential") or ttf.startswith("normal"):
                return "NodeFail"

        # Plain time value like 2500s
        seconds = _parse_time_like_seconds(ttf)
        if seconds is not None:
            if seconds >= 0:
                return "NodeFail"
            return "NoFail"  # -1s

    # If neither mode is present, assume failures are not enabled.
    return "NoFail"


def euclidean(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def fmt_f(v, dp=4):
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "nan"
    return f"{v:.{dp}f}"


# ---------------------------------------------------------------------------
# Detect DV vs SF from paths.csv
# ---------------------------------------------------------------------------

def detect_routing_type_from_paths(paths_csv: Path) -> str:
    """Detect routing type (DV or SF) from paths.csv based on nextHopType.

    If any UNICAST nextHopType is present, treat as Distance Vector (DV).
    If only BCAST is present (no UNICAST), treat as Smart Flooding (SF).
    """
    try:
        with paths_csv.open("r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get("nextHopType") == "UNICAST":
                    return "DV"
    except Exception:
        pass
    return "SF"


# ---------------------------------------------------------------------------
# Core analysis
# ---------------------------------------------------------------------------

def analyze_1vic_20grid():
    if not PATHS_CSV.exists():
        print(f"ERROR: {PATHS_CSV} not found")
        return

    # First pass: parse events and build per-packet stats
    # We treat any node that originates TX_SRC as a data source end node.
    data_sources = set()

    # Per (srcNodeId, pktId):
    packet_first_tx_time = {}   # first TX_SRC time
    packet_delivery_time = {}   # first DELIVERED time
    packet_last_seen_time = {}  # last time we see this packet for that src
    packet_transmissions = {}   # list of TX_SRC / TX_FWD_* events (data-plane only)

    sent_packets = set()        # set of (src, pktId) that were TX_SRC at least once

    with PATHS_CSV.open("r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            try:
                event_type = row["event"]
                src_node = int(row["src"]) if row.get("src") else None
                dst_node = int(row["dst"]) if row.get("dst") else None
                pkt_id   = int(row["packetSeq"]) if row.get("packetSeq") else None
                sim_time = float(row["simTime"]) if row.get("simTime") else None
            except (ValueError, KeyError):
                continue

            if src_node is None or pkt_id is None or sim_time is None:
                continue

            # Mark data sources based on TX_SRC events
            if event_type == "TX_SRC":
                data_sources.add(src_node)

            # For convenience
            key = (src_node, pkt_id)

            # First TX time (only for data sources)
            if event_type == "TX_SRC" and src_node in data_sources:
                if key not in packet_first_tx_time:
                    packet_first_tx_time[key] = sim_time
                sent_packets.add(key)

            # First delivery time (DELIVERED at destination)
            if event_type == "DELIVERED" and src_node in data_sources:
                if key not in packet_delivery_time:
                    packet_delivery_time[key] = sim_time

            # Count data-plane transmissions (TX_SRC, TX_FWD_DATA, TX_FWD_BCAST) from data sources
            if event_type in ["TX_SRC", "TX_FWD_DATA", "TX_FWD_BCAST"] and src_node in data_sources:
                if key not in packet_transmissions:
                    packet_transmissions[key] = []
                packet_transmissions[key].append({
                    "eventType": event_type,
                    "simTime": sim_time,
                })

            # Last-seen time for this packet from data sources (any event with this src & packetSeq)
            if src_node in data_sources:
                packet_last_seen_time[key] = sim_time

    delivered_keys = list(packet_delivery_time.keys())
    num_sent = len(sent_packets)
    num_delivered = len(delivered_keys)

    # PDR
    pdr = (float(num_delivered) / num_sent * 100.0) if num_sent > 0 else float("nan")

    # Latency and time-on-air per delivered packet
    latencies = []
    time_on_air_values = []
    total_tx_events_for_delivered = 0

    for key in delivered_keys:
        start_t = packet_first_tx_time.get(key)
        end_t = packet_delivery_time.get(key)
        last_t = packet_last_seen_time.get(key)

        if start_t is not None and end_t is not None and end_t >= start_t:
            latencies.append(end_t - start_t)
        if start_t is not None and last_t is not None and last_t >= start_t:
            time_on_air_values.append(last_t - start_t)

        tx_events = packet_transmissions.get(key, [])
        total_tx_events_for_delivered += len(tx_events)

    # Latency stats
    if latencies:
        avg_latency   = statistics.mean(latencies)
        median_latency = statistics.median(latencies)
        stdev_latency = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
        min_latency   = min(latencies)
        max_latency   = max(latencies)
    else:
        avg_latency = median_latency = stdev_latency = min_latency = max_latency = float("nan")

    # Time on air and avg transmissions
    overall_avg_time_on_air = (sum(time_on_air_values) / len(time_on_air_values)) if time_on_air_values else float("nan")
    overall_avg_tx_per_delivered = (float(total_tx_events_for_delivered) / num_delivered) if num_delivered > 0 else float("nan")

    # ------------------------------------------------------------------
    # Energy and node positions from latest .sca
    # ------------------------------------------------------------------
    sca_path = latest_sca(RESULTS_DIR)
    if sca_path is None:
        total_energy = float("nan")
        sca_name = "<none>"
        src_x = src_y = dst_x = dst_y = float("nan")
        distance_m = float("nan")
    else:
        scalars = parse_sca(sca_path)
        sca_name = sca_path.name

        # Total energy: sum all totalEnergyConsumed scalars (filtered for sane values)
        total_energy = 0.0
        for (mod, met), v in scalars.items():
            if met == "totalEnergyConsumed" and 0.0 <= v <= 1e9:
                total_energy += v

        # End-node positions: assume two end nodes at indices 0 and 1.
        # In this model, coordinates are exported as CordiX/CordiY scalars.
        # Fall back to positionX/positionY if present for other scenarios.
        src_mod = "LoRaMesh.loRaEndNodes[0].LoRaNodeApp"
        dst_mod = "LoRaMesh.loRaEndNodes[1].LoRaNodeApp"

        src_x = scalars.get((src_mod, "CordiX"), float("nan"))
        src_y = scalars.get((src_mod, "CordiY"), float("nan"))
        dst_x = scalars.get((dst_mod, "CordiX"), float("nan"))
        dst_y = scalars.get((dst_mod, "CordiY"), float("nan"))

        # Fallback to positionX/positionY if Cordi* are missing
        if math.isnan(src_x):
            src_x = scalars.get((src_mod, "positionX"), float("nan"))
        if math.isnan(src_y):
            src_y = scalars.get((src_mod, "positionY"), float("nan"))
        if math.isnan(dst_x):
            dst_x = scalars.get((dst_mod, "positionX"), float("nan"))
        if math.isnan(dst_y):
            dst_y = scalars.get((dst_mod, "positionY"), float("nan"))

        if all(not math.isnan(v) for v in [src_x, src_y, dst_x, dst_y]):
            distance_m = euclidean(src_x, src_y, dst_x, dst_y)
        else:
            distance_m = float("nan")

    # ------------------------------------------------------------------
    # Build report filename and contents
    # ------------------------------------------------------------------
    routing_label = detect_routing_type_from_paths(PATHS_CSV)  # "DV" or "SF"

    failure_label = detect_failure_status_from_sca(sca_path)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dist_label = f"{round(distance_m)}" if not math.isnan(distance_m) else "unknown"
    report_name = f"{routing_label}_{dist_label}_{timestamp}_1VIC_{failure_label}_20grid.txt"
    report_path = SCRIPT_DIR / report_name

    lines = []
    lines.append("=" * 65)
    lines.append("  1VIC 20-GRID DV/SF ANALYSIS REPORT")
    lines.append(f"  Generated : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"  SCA file  : {sca_name}")
    lines.append("=" * 65)
    lines.append("")

    lines.append("PDR")
    lines.append(f"  Packets sent by source              : {num_sent}")
    lines.append(f"  Unique packets received at dest.    : {num_delivered}")
    lines.append(f"  Packet Delivery Ratio (PDR)         : {fmt_f(pdr, 2)} %")
    lines.append("")

    lines.append("LATENCY (DATA)")
    lines.append("  Average DATA latency                : " + fmt_f(avg_latency))
    lines.append("  Median DATA latency                 : " + fmt_f(median_latency))
    lines.append("  Std deviation                       : " + fmt_f(stdev_latency))
    lines.append("  Min DATA latency                    : " + fmt_f(min_latency))
    lines.append("  Max DATA latency                    : " + fmt_f(max_latency))
    lines.append("")

    lines.append("Total network energy consumed         : " + fmt_f(total_energy, 6) + " J")
    lines.append("")

    lines.append("NODE POSITIONS & DISTANCE")
    lines.append(f"  Source  (loRaEndNodes[0]) position  : ({fmt_f(src_x, 2)} m, {fmt_f(src_y, 2)} m)")
    lines.append(f"  Dest.   (loRaEndNodes[1]) position  : ({fmt_f(dst_x, 2)} m, {fmt_f(dst_y, 2)} m)")
    lines.append(f"  Euclidean distance                  : {fmt_f(distance_m, 2)} m")
    lines.append("")

    lines.append("Avg Time On Air per Delivered Packet  : " + fmt_f(overall_avg_time_on_air))
    lines.append("Avg Transmissions per Delivered Packet: " + fmt_f(overall_avg_tx_per_delivered))
    lines.append("")
    lines.append("=" * 65)

    report_text = "\n".join(lines)

    with report_path.open("w", encoding="utf-8") as fh:
        fh.write(report_text)

    print(f"Report written -> {report_path}")
    print(report_text)


if __name__ == "__main__":
    print("1VIC 20-grid DV/SF Analysis Script")
    print(f"  paths.csv : {PATHS_CSV}")
    print(f"  Output dir: {SCRIPT_DIR}\n")
    analyze_1vic_20grid()
