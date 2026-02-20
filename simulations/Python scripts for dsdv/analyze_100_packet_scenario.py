import csv
import sys
import re
from collections import defaultdict
from pathlib import Path
from datetime import datetime


def parse_initial_distance_from_ini(ini_path: Path) -> str:
    """Extract initial distance between end node and rescue node from INI file.
    
    Note: If end nodes use random deployment (circle/uniform), the actual distance
    varies per simulation run. This function looks for:
    1. Comments documenting the initial distance
    2. Distance parameters in the config
    
    If not found, pass distance as command-line argument.
    """
    try:
        text = ini_path.read_text(encoding='utf-8', errors='ignore')
        # Look for documented initial distance in comments or descriptions
        patterns = [
            r'[#\s]Initial distance[:\s]+([\d.]+)\s*m',
            r'[#\s]Distance[:\s]+([\d.]+)\s*meters',
            r'initialDistance[^=]*=\s*([\d.]+)',
            r'distance.*?=\s*([\d.]+)m',
        ]
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return match.group(1)
    except Exception as e:
        print(f"Warning: Could not parse initial distance from INI: {e}")
    return "unknown"


def parse_mobile_speed_from_sca(sca_path: Path) -> str:
    """Extract rescue node mobile speed from .sca file.
    
    The .sca file contains the actual parameters used in the simulation run,
    so it's more reliable than parsing the INI file which may have multiple configs.
    """
    try:
        text = sca_path.read_text(encoding='utf-8', errors='ignore')
        # Look for turtle script parameter with speed in filename
        # Format: param **.loRaRescueNodes[*].mobility.turtleScript "xmldoc(\"turtle_patrol_7mps_2km.xml\")"
        match = re.search(r'param\s+\*\*\.loRaRescueNodes.*turtleScript\s+.*?(\d+)mps', text, re.IGNORECASE)
        if match:
            return match.group(1)
        # Look for direct speed parameter
        match = re.search(r'param\s+\*\*\.loRaRescueNodes.*\.speed\s+([\d.]+)', text, re.IGNORECASE)
        if match:
            # Extract numeric part (e.g., "5mps" -> "5")
            speed_val = match.group(1)
            if 'mps' in speed_val:
                return speed_val.replace('mps', '')
            return speed_val
    except Exception as e:
        print(f"Warning: Could not parse mobile speed from .sca file: {e}")
    return "unknown"


def parse_energy_from_sca(sca_path: Path) -> tuple[str, int]:
    """Extract total energy consumption from OMNeT++ scalar results file.
    
    Returns:
        tuple: (total_energy_string, count_of_nodes)
    """
    try:
        text = sca_path.read_text(encoding='utf-8', errors='ignore')
        total_energy = 0.0
        count = 0
        # Parse scalar entries for energy consumption
        # Format: scalar <module> <name> <value>
        for line in text.splitlines():
            if 'energyConsumption' in line or 'totalEnergyConsumed' in line:
                parts = line.split()
                if len(parts) >= 4 and parts[0] == 'scalar':
                    try:
                        value = float(parts[3])
                        total_energy += value
                        count += 1
                    except (ValueError, IndexError):
                        continue
        if total_energy > 0:
            return f"{total_energy:.4f}", count
    except Exception as e:
        print(f"Warning: Could not parse energy from .sca file: {e}")
    return "N/A", 0


def analyze_paths_csv(
    paths_path: Path, 
    src_id: int = 1000, 
    dst_id: int = 2000, 
    expected_packets: int = 100,
    initial_distance: str = "unknown",
    mobile_speed: str = "unknown",
    total_energy: str = "N/A"
) -> None:
    """Analyze a 100-packet scenario from paths.csv.

    - Auto-detects routing mode (DSDV vs FLOODING) from nextHopType field
    - Counts how many DATA packets were sent from src_id to dst_id (TX_SRC events)
    - Counts, for each packetSeq in [0, expected_packets-1], how many DELIVERED copies were observed
    - Computes how many of the expected_packets had at least one delivery
    - Computes PDR = delivered_unique / expected_packets * 100 (ignoring multiple copies)
    - Calculates average latency from TX_SRC to first DELIVERED
    - Counts unique nodes that processed packets
    """

    sent_packet_seqs = set()              # packetSeqs actually sent from src->dst
    delivered_counts = defaultdict(int)   # packetSeq -> number of DELIVERED events
    unicast_count = 0
    broadcast_count = 0
    tx_times = {}                         # packetSeq -> TX_SRC simTime
    first_delivery_times = {}             # packetSeq -> first DELIVERED simTime
    unique_nodes = set()                  # All unique nodes that processed packets

    with paths_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                event = row["event"].strip()
                seq = int(row["packetSeq"])
                src = int(row["src"])
                dst = int(row["dst"])
                next_hop_type = row.get("nextHopType", "").strip()
                sim_time = float(row["simTime"])
                current_node = int(row["currentNode"])
            except (KeyError, ValueError):
                # Skip malformed lines
                continue

            # Restrict analysis to the flow src_id -> dst_id
            if src != src_id or dst != dst_id:
                continue

            # Track unique nodes that processed this packet
            unique_nodes.add(current_node)

            if event == "TX_SRC":
                sent_packet_seqs.add(seq)
                tx_times[seq] = sim_time

            if event == "DELIVERED":
                delivered_counts[seq] += 1
                # Record first delivery time only
                if seq not in first_delivery_times:
                    first_delivery_times[seq] = sim_time
            
            # Count unicast vs broadcast to detect routing mode
            if event in ["TX_SRC", "TX_FWD_DATA"]:
                if next_hop_type == "UNICAST":
                    unicast_count += 1
                elif next_hop_type == "BCAST":
                    broadcast_count += 1

    # Auto-detect routing mode
    if unicast_count > broadcast_count * 2:
        routing_mode = "DSDV"
    else:
        routing_mode = "FLOODING"

    # Stats over the expected 0..expected_packets-1 range
    expected_range = list(range(expected_packets))

    total_sent = len(sent_packet_seqs)
    delivered_at_least_once = sum(1 for seq in expected_range if delivered_counts.get(seq, 0) > 0)

    # Calculate average latency (from TX_SRC to first DELIVERED)
    latencies = []
    for seq in expected_range:
        if seq in tx_times and seq in first_delivery_times:
            latency = first_delivery_times[seq] - tx_times[seq]
            latencies.append(latency)
    
    avg_latency = sum(latencies) / len(latencies) if latencies else 0.0
    
    # Count unique nodes
    num_unique_nodes = len(unique_nodes)

    # PDR based only on unique sequences in expected range
    if expected_packets > 0:
        pdr = 100.0 * delivered_at_least_once / expected_packets
    else:
        pdr = 0.0

    # Per-sequence copy counts (for reporting)
    print("=== 100-packet scenario analysis ===")
    print(f"Routing Mode (auto-detected): {routing_mode}")
    print(f"Mobile Speed: {mobile_speed}")
    print(f"Initial Distance: {initial_distance}m")
    print(f"Input file: {paths_path}")
    print(f"Flow analysed: src={src_id} -> dst={dst_id}")
    print()

    print(f"Total distinct packetSeq values SENT from {src_id} to {dst_id}: {total_sent}")
    print(f"(Across all sequences, not limited to 0..{expected_packets-1})")
    print()

    print(f"Expected packets (0..{expected_packets-1}): {expected_packets}")
    print(f"Packets with at least one delivery: {delivered_at_least_once}")
    print(f"Packet Delivery Rate (unique, ignoring copies): {pdr:.2f}%")
    print(f"Average Latency: {avg_latency:.4f}s")
    print(f"Unique Nodes Processing Packets: {num_unique_nodes}")
    print()

    print("Per-packet delivery copy counts (for expected range):")
    print("seq,delivered_copies")
    for seq in expected_range:
        print(f"{seq},{delivered_counts.get(seq, 0)}")

    # Write summary report to text file in analysis reports folder
    output_dir = paths_path.parent.parent  # Go from delivered_packets to simulations
    reports_dir = output_dir / "analysis reports"
    reports_dir.mkdir(exist_ok=True)
    
    # Generate filename: ROUTING_speed_distance_timestamp.txt
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_filename = f"{routing_mode}_{mobile_speed}_{initial_distance}_{timestamp}"
    
    summary_file = reports_dir / f"{base_filename}.txt"
    details_csv = reports_dir / f"{base_filename}_details.csv"

    with summary_file.open("w") as f:
        f.write("=== 100-packet scenario analysis ===\n")
        f.write(f"Routing Mode: {routing_mode}\n")
        f.write(f"Mobile Speed: {mobile_speed}\n")
        f.write(f"Initial Distance: {initial_distance}m\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"\nInput file: {paths_path}\n")
        f.write(f"Flow analysed: src={src_id} -> dst={dst_id}\n")
        f.write("\n")
        f.write(f"Total distinct packetSeq values SENT from {src_id} to {dst_id}: {total_sent}\n")
        f.write(f"(Across all sequences, not limited to 0..{expected_packets-1})\n")
        f.write("\n")
        f.write(f"Expected packets (0..{expected_packets-1}): {expected_packets}\n")
        f.write(f"Packets with at least one delivery: {delivered_at_least_once}\n")
        f.write(f"Packet Delivery Rate (unique, ignoring copies): {pdr:.2f}%\n")
        f.write(f"Average Latency: {avg_latency:.4f}s\n")
        f.write(f"Unique Nodes Processing Packets: {num_unique_nodes}\n")
        f.write("\n")
        f.write("Summary:\n")
        f.write(f"- Sent: {total_sent} distinct packet sequences\n")
        f.write(f"- Delivered (at least once): {delivered_at_least_once} / {expected_packets}\n")
        f.write(f"- PDR: {pdr:.2f}%\n")
        f.write(f"- Avg Latency: {avg_latency:.4f}s\n")
        f.write(f"- Unique Nodes: {num_unique_nodes}\n")
        f.write(f"\nDetailed per-packet delivery counts saved to: {details_csv.name}\n")

    # Write per-packet details to CSV
    with details_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["packetSeq", "delivered_copies", "delivered_at_least_once"])
        for seq in expected_range:
            copies = delivered_counts.get(seq, 0)
            delivered = "YES" if copies > 0 else "NO"
            writer.writerow([seq, copies, delivered])

    print()
    print(f"Summary report saved to: {summary_file}")
    print(f"Detailed CSV saved to: {details_csv}")

    # Append row to summary CSV in analysis reports folder
    summary_csv_path = reports_dir / "simulation_results_summary.csv"
    csv_exists = summary_csv_path.exists()
    
    with summary_csv_path.open("a", newline="") as f:
        fieldnames = [
            "timestamp",
            "routing_mode",
            "mobile_speed_mps",
            "initial_distance_m",
            "packets_sent",
            "packets_received_unique",
            "avg_latency_s",
            "total_energy_j",
            "unique_nodes_processed",
            "pdr_percent"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        
        # Write header only if file is new
        if not csv_exists:
            writer.writeheader()
        
        # Write data row
        writer.writerow({
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "routing_mode": routing_mode,
            "mobile_speed_mps": mobile_speed,
            "initial_distance_m": initial_distance,
            "packets_sent": total_sent,
            "packets_received_unique": delivered_at_least_once,
            "avg_latency_s": f"{avg_latency:.4f}",
            "total_energy_j": total_energy,
            "unique_nodes_processed": num_unique_nodes,
            "pdr_percent": f"{pdr:.2f}"
        })
    
    print(f"Summary row appended to: {summary_csv_path}")


if __name__ == "__main__":
    # Default to the paths.csv in the delivered_packets folder next to this script
    base = Path(__file__).resolve().parent
    default_paths = base / "delivered_packets" / "paths.csv"

    # Try to find INI file (in same directory as script)
    ini_path = base / "routing_between_2_rescue_endnodes.ini"
    
    # Look for .sca files in results directory - use the most recent one
    results_dir = base / "results"
    if results_dir.exists():
        sca_files = sorted(results_dir.glob("*.sca"), key=lambda p: p.stat().st_mtime, reverse=True)
        sca_path = sca_files[0] if sca_files else None
    else:
        sca_path = None
    
    # Parse values from files if available
    initial_distance = "unknown"
    mobile_speed = "unknown"
    total_energy = "N/A"
    
    if ini_path.exists():
        initial_distance = parse_initial_distance_from_ini(ini_path)
        if initial_distance != "unknown":
            print(f"Parsed from INI: distance={initial_distance}m")
    
    if sca_path and sca_path.exists():
        print(f"Reading from .sca: {sca_path.name}")
        mobile_speed = parse_mobile_speed_from_sca(sca_path)
        total_energy, energy_count = parse_energy_from_sca(sca_path)
        print(f"Mobile speed: {mobile_speed}m/s")
        print(f"Total energy: {total_energy}J (from {energy_count} nodes)")
    
    # Command-line arguments can override (optional)
    # Usage: python analyze_100_packet_scenario.py [initial_distance_m] [mobile_speed_mps]
    if len(sys.argv) > 1:
        initial_distance = sys.argv[1]
    if len(sys.argv) > 2:
        mobile_speed = sys.argv[2]

    if not default_paths.exists():
        print(f"paths.csv not found at {default_paths}")
    else:
        analyze_paths_csv(
            default_paths, 
            initial_distance=initial_distance,
            mobile_speed=mobile_speed,
            total_energy=total_energy
        )
