#!/usr/bin/env python3
"""
End Node Distance and Path Tracker
Analyzes LoRa mesh simulation results with focus on:
- Real-time end node distance calculation from fresh .sca files
- Packet generation and delivery statistics
- Individual packet path tracking
- Comprehensive transit time analysis
"""

import pandas as pd
import os
import glob
import math
import argparse
import csv
from datetime import datetime
from pathlib import Path
import re

def find_latest_results_directory(base_dir="./"):
    """
    Find the most recent results directory containing .sca files.
    Handles timestamped result folders and ensures fresh data.
    """
    possible_dirs = [
        os.path.join(base_dir, "results"),
        os.path.join(base_dir, "simulations/results"),
        os.path.join(base_dir, "../results"),
    ]
    
    # Also search for timestamped result directories
    for pattern in ["**/results*", "**/General-*"]:
        matches = glob.glob(os.path.join(base_dir, pattern), recursive=True)
        possible_dirs.extend(matches)
    
    valid_dirs = []
    for results_dir in possible_dirs:
        if os.path.isdir(results_dir):
            sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
            if sca_files:
                # Get most recent modification time
                latest_time = max(os.path.getmtime(f) for f in sca_files)
                valid_dirs.append((results_dir, latest_time, len(sca_files)))
    
    if not valid_dirs:
        return None
    
    # Return directory with most recent files
    latest_dir = max(valid_dirs, key=lambda x: x[1])
    print(f"Using results directory: {latest_dir[0]} ({latest_dir[2]} .sca files, modified: {datetime.fromtimestamp(latest_dir[1])})")
    return latest_dir[0]

def save_coordinates_to_csv(coordinates, csv_path="delivered_packets/endnode_positions.csv"):
    """
    Save extracted coordinates to CSV file for future reference.
    Appends new data without duplicating if same run already exists.
    """
    try:
        import os
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        
        # Check if file exists and read existing data
        file_exists = os.path.exists(csv_path)
        existing_data = []
        
        if file_exists:
            try:
                with open(csv_path, 'r') as f:
                    existing_data = f.readlines()
            except:
                pass
        
        # Write/append data
        with open(csv_path, 'w') as f:
            # Write header
            f.write("timestamp,node_id,position_x,position_y,source_file,extraction_time\n")
            
            # Write existing data (skip header)
            if len(existing_data) > 1:
                for line in existing_data[1:]:
                    f.write(line)
            
            # Write new coordinates
            extraction_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            for node_id, coords in sorted(coordinates.items()):
                if 'x' in coords and 'y' in coords:
                    timestamp = coords.get('timestamp', 0.0)
                    source_file = coords.get('source_file', 'unknown')
                    f.write(f"{timestamp},{node_id},{coords['x']},{coords['y']},{source_file},{extraction_time}\n")
        
        return True
    except Exception as e:
        print(f"Warning: Could not save to CSV: {e}")
        return False

def extract_end_node_coordinates_from_csv(csv_path="endnode_positions.csv"):
    """
    Extract end node coordinates from CSV file written by simulation.
    Returns the MOST RECENT coordinates for nodes 1000 and 1001.
    """
    coordinates = {}
    extraction_info = {
        'source': 'CSV',
        'csv_file': csv_path,
        'rows_read': 0,
        'coordinates_extracted': 0
    }
    
    if not os.path.exists(csv_path):
        extraction_info['error'] = f"CSV file not found: {csv_path}"
        return coordinates, extraction_info
    
    try:
        # Read CSV and get the most recent entry for each end node
        df = pd.read_csv(csv_path)
        extraction_info['rows_read'] = len(df)
        
        # Filter for end nodes 1000 and 1001
        end_nodes = df[df['node_id'].isin([1000, 1001])]
        
        # Get the most recent position for each node (by timestamp)
        for node_id in [1000, 1001]:
            node_data = end_nodes[end_nodes['node_id'] == node_id]
            if not node_data.empty:
                # Get the last (most recent) entry
                latest = node_data.iloc[-1]
                coordinates[node_id] = {
                    'x': latest['position_x'],
                    'y': latest['position_y'],
                    'timestamp': latest['timestamp'],
                    'config': latest.get('config_name', 'unknown'),
                    'run': latest.get('run_number', 'unknown')
                }
                extraction_info['coordinates_extracted'] += 2
        
        if coordinates:
            extraction_info['latest_timestamp'] = max(c['timestamp'] for c in coordinates.values())
            
    except Exception as e:
        extraction_info['error'] = f"Error reading CSV: {e}"
    
    return coordinates, extraction_info

def extract_end_node_coordinates(results_dir):
    """
    Extract end node coordinates from the most recent .sca files.
    Returns coordinates dict and metadata about the extraction.
    """
    coordinates = {}
    extraction_info = {
        'files_scanned': 0,
        'scalars_found': 0,
        'coordinates_extracted': 0,
        'source_file': None,
        'results_dir': results_dir
    }
    
    if not results_dir or not os.path.isdir(results_dir):
        return coordinates, extraction_info
    
    # Get all .sca files, sorted by modification time (newest first)
    sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
    sca_files.sort(key=os.path.getmtime, reverse=True)
    
    # Read ONLY the most recent file to get current run's coordinates
    for sca_file in sca_files:
        extraction_info['files_scanned'] += 1
        found_coords = False
        
        try:
            with open(sca_file, 'r') as f:
                for line in f:
                    if not line.startswith('scalar '):
                        continue
                    
                    parts = line.strip().split()
                    if len(parts) < 4:
                        continue
                    
                    module_path = parts[1]
                    scalar_name = parts[2]
                    value_str = parts[3]
                    
                    extraction_info['scalars_found'] += 1
                    
                    # Look for coordinate scalars
                    if scalar_name not in ('CordiX', 'CordiY', 'positionX', 'positionY'):
                        continue
                    
                    # Extract node ID for end nodes
                    node_id = None
                    if 'loRaEndNodes[' in module_path:
                        start = module_path.find('loRaEndNodes[') + len('loRaEndNodes[')
                        end = module_path.find(']', start)
                        if end != -1:
                            try:
                                idx = int(module_path[start:end])
                                node_id = 1000 + idx  # End node ID offset
                            except ValueError:
                                continue
                    
                    if node_id not in (1000, 1001):
                        continue
                    
                    try:
                        coord_val = float(value_str)
                    except ValueError:
                        continue
                    
                    if node_id not in coordinates:
                        coordinates[node_id] = {'x': None, 'y': None}
                    
                    if scalar_name in ('CordiX', 'positionX'):
                        coordinates[node_id]['x'] = coord_val
                        coordinates[node_id]['source_file'] = os.path.basename(sca_file)
                        coordinates[node_id]['timestamp'] = 0.0  # Will be updated if we find it in .sca
                        extraction_info['coordinates_extracted'] += 1
                        found_coords = True
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
                    elif scalar_name in ('CordiY', 'positionY'):
                        coordinates[node_id]['y'] = coord_val
                        coordinates[node_id]['source_file'] = os.path.basename(sca_file)
                        coordinates[node_id]['timestamp'] = 0.0
                        extraction_info['coordinates_extracted'] += 1
                        found_coords = True
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
                    
        except Exception as e:
            print(f"Warning: Error reading {sca_file}: {e}")
            continue
        
        # Stop after finding coordinates in the first (newest) file
        if found_coords and len(coordinates) == 2:
            break
    
    # Filter out incomplete coordinates
    complete_coords = {}
    for node_id, coords in coordinates.items():
        if coords['x'] is not None and coords['y'] is not None:
            complete_coords[node_id] = coords
    
    return complete_coords, extraction_info

def calculate_distance(coord1, coord2):
    """Calculate Euclidean distance between two coordinate dictionaries."""
    if not coord1 or not coord2:
        return None
    
    dx = coord1['x'] - coord2['x']
    dy = coord1['y'] - coord2['y']
    return math.sqrt(dx*dx + dy*dy)

def append_to_summary_csv(coordinates, extraction_info, df, packet_paths, report_file, distance, routing_method):
    """
    Append one line summarizing this run to simulation_summary.csv.
    Always writes to the simulations folder regardless of cwd or report path.
    """
    simulations_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(simulations_dir, "simulation_summary.csv")
    
    # Calculate statistics
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    delivery_rate = (total_delivered / total_generated * 100) if total_generated > 0 else 0.0
    
    delivered_packets = [p for p in packet_paths.values() if p['delivered']]
    # Use all transit times from all delivered copies (to destination)
    transit_times = []
    for p in delivered_packets:
        if p.get('transit_times'):
            transit_times.extend([t for t in p['transit_times'] if t is not None])
        elif p.get('transit_time') is not None:
            transit_times.append(p['transit_time'])
    
    avg_transit_time = sum(transit_times) / len(transit_times) if transit_times else None
    min_transit_time = min(transit_times) if transit_times else None
    max_transit_time = max(transit_times) if transit_times else None
    
    # Average hop count across all delivered copies using TTL decrement
    hop_counts = []
    for p in delivered_packets:
        if p.get('deliver_hop_counts'):
            hop_counts.extend(p['deliver_hop_counts'])
        elif p.get('hop_count') is not None:
            hop_counts.append(p['hop_count'])
    avg_hop_count = sum(hop_counts) / len(hop_counts) if hop_counts else None
    
    throughput = len(transit_times) / max(transit_times) if transit_times and max(transit_times) > 0 else None
    
    # Extract coordinates
    node_1000_x = coordinates.get(1000, {}).get('x', None)
    node_1000_y = coordinates.get(1000, {}).get('y', None)
    node_1001_x = coordinates.get(1001, {}).get('x', None)
    node_1001_y = coordinates.get(1001, {}).get('y', None)
    
    # Prepare row
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    row = {
        'timestamp': timestamp,
        'routing_method': routing_method,
        'results_dir': extraction_info.get('results_dir', ''),
        'report_file': report_file,
        'distance_m': f"{distance:.2f}" if distance is not None else '',
        'endnode_1000_x': f"{node_1000_x:.2f}" if node_1000_x is not None else '',
        'endnode_1000_y': f"{node_1000_y:.2f}" if node_1000_y is not None else '',
        'endnode_1001_x': f"{node_1001_x:.2f}" if node_1001_x is not None else '',
        'endnode_1001_y': f"{node_1001_y:.2f}" if node_1001_y is not None else '',
        'packets_generated': total_generated,
        'packets_delivered': total_delivered,
        'delivery_rate': f"{delivery_rate:.2f}" if delivery_rate is not None else '',
        'avg_transit_time': f"{avg_transit_time:.3f}" if avg_transit_time is not None else '',
        'min_transit_time': f"{min_transit_time:.3f}" if min_transit_time is not None else '',
        'max_transit_time': f"{max_transit_time:.3f}" if max_transit_time is not None else '',
        'avg_hop_count': f"{avg_hop_count:.2f}" if avg_hop_count is not None else '',
        'throughput_packets_per_sec': f"{throughput:.2f}" if throughput is not None else '',
        # New: number of unique nodes that processed the packet (max over delivered packets)
        'nodes_processed': max((p.get('unique_nodes_processed', 0) for p in delivered_packets), default=0)
    }
    
    # Check if file exists to determine if we need to write header
    file_exists = os.path.exists(csv_file)
    
    try:
        os.makedirs(os.path.dirname(csv_file), exist_ok=True)
        with open(csv_file, 'a', newline='', encoding='utf-8') as f:
            fieldnames = ['timestamp', 'routing_method', 'results_dir', 'report_file', 'distance_m', 
                         'endnode_1000_x', 'endnode_1000_y', 'endnode_1001_x', 'endnode_1001_y',
                         'packets_generated', 'packets_delivered', 'delivery_rate',
                         'avg_transit_time', 'min_transit_time', 'max_transit_time',
                         'avg_hop_count', 'throughput_packets_per_sec', 'nodes_processed']
            
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            
            if not file_exists:
                writer.writeheader()
            
            writer.writerow(row)
            f.flush()
        
        print(f"✓ Summary appended to: {csv_file}")
    except Exception as e:
        print(f"Warning: Could not append to summary CSV: {e}")

def detect_routing_method(df):
    """
    Detect the routing method used based on path events.
    Returns 'flooding', 'routing', or 'mixed' based on analysis of the data.
    """
    if df.empty:
        return 'unknown'
    
    # Count different types of transmission events
    tx_src_events = len(df[df['event'] == 'TX_SRC'])
    enqueue_fwd_events = len(df[df['event'] == 'ENQUEUE_FWD'])
    tx_fwd_events = len(df[df['event'].isin(['TX_FWD_DATA', 'TX_FWD_ACK'])])
    
    # Check if we have broadcast addresses (16777215 = BROADCAST_ADDRESS)
    broadcast_events = len(df[df['chosenVia'] == 16777215])
    unicast_events = len(df[(df['chosenVia'] != 16777215) & (df['chosenVia'].notna())])
    
    # Analyze routing table lookups vs broadcast behavior
    if broadcast_events > 0 and unicast_events == 0:
        # All transmissions are broadcast - likely flooding
        if enqueue_fwd_events > tx_fwd_events * 2:
            return 'flooding'  # Many nodes enqueueing suggests flooding behavior
        else:
            return 'broadcast'
    elif unicast_events > broadcast_events:
        # More unicast than broadcast - likely routing table based
        return 'routing'
    elif broadcast_events > 0 and unicast_events > 0:
        # Mix of both - treat as routing per user request
        return 'routing'
    else:
        # Unable to determine
        return 'unknown'

def analyze_packet_paths(df):
    """
    Analyze individual packet paths from source to destination.
    Returns detailed path information for each packet.
    """
    packet_paths = {}
    
    # Group events by packet sequence
    for packet_seq in df['packetSeq'].unique():
        packet_events = df[df['packetSeq'] == packet_seq].sort_values('simTime')
        
        path_info = {
            'packet_seq': packet_seq,
            'source': None,
            'destination': None,
            'generated_time': None,
            'delivered_time': None,  # last delivery time (legacy)
            'path_nodes': [],
            'hop_count': 0,
            'transit_time': None,    # last transit time (legacy)
            'delivered': False,
            'path_events': [],
            'deliver_times': [],     # all delivery timestamps
            'transit_times': [],      # all transit times to destination
            # TTL-based hop count tracking
            'initial_ttl': None,
            'deliver_hop_counts': [],
            # First-arrival metrics (final metrics as per requirement)
            'first_transit_time': None,
            'first_hop_count': None
        }
        # Track unique nodes that processed this packet (any event with currentNode)
        nodes_seen = set()
        
        for _, event in packet_events.iterrows():
            event_data = {
                'time': event['simTime'],
                'event_type': event['event'],
                'node': event['currentNode'],
                'next_hop': event.get('chosenVia', None),
                'hop_type': event.get('nextHopType', None)
            }
            path_info['path_events'].append(event_data)
            # Count node participation
            try:
                if not pd.isna(event['currentNode']):
                    nodes_seen.add(int(event['currentNode']))
            except Exception:
                pass
            
            # Extract key information
            if event['event'] == 'TX_SRC':
                path_info['source'] = event['src']
                path_info['destination'] = event['dst']
                path_info['generated_time'] = event['simTime']
                path_info['path_nodes'].append(event['currentNode'])
                # Record initial TTL after source TX (used for TTL-based hop count)
                try:
                    path_info['initial_ttl'] = int(event['ttlAfterDecr']) if not pd.isna(event['ttlAfterDecr']) else None
                except Exception:
                    path_info['initial_ttl'] = None
            
            elif event['event'] in ['TX_FWD_DATA', 'TX_FWD_ACK']:
                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])
                    path_info['hop_count'] += 1
            
            elif event['event'] == 'DELIVERED':
                # Record every delivery occurrence
                t = event['simTime']
                path_info['deliver_times'].append(t)

                # Only count as delivered-to-destination if current node equals destination
                # If destination is not yet known (rare), we still mark delivered, but transit list
                # is appended only when destination matches.
                path_info['delivered'] = True
                path_info['delivered_time'] = t  # keep legacy "last" delivery

                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])

                if path_info['generated_time'] is not None:
                    # If destination known and matches, record transit time for this copy
                    if path_info['destination'] is None or event['currentNode'] == path_info['destination']:
                        transit = t - path_info['generated_time']
                        path_info['transit_time'] = transit  # legacy "last" transit time
                        path_info['transit_times'].append(transit)

                # TTL-based hop count for this delivered copy: initial_ttl - ttlAtDelivery
                try:
                    ttl_at_delivery = int(event['ttlAfterDecr']) if not pd.isna(event['ttlAfterDecr']) else None
                except Exception:
                    ttl_at_delivery = None
                if path_info.get('initial_ttl') is not None and ttl_at_delivery is not None:
                    hops = path_info['initial_ttl'] - ttl_at_delivery
                    if hops >= 0:
                        path_info['deliver_hop_counts'].append(hops)
                        # Update legacy hop_count to this TTL-based value (last delivery)
                        path_info['hop_count'] = hops
                        # Set first-arrival metrics if not already set and this is the first destination delivery
                        if (path_info['first_transit_time'] is None) and (path_info['destination'] is None or event['currentNode'] == path_info['destination']):
                            # first delivery copy to destination
                            if path_info['generated_time'] is not None:
                                path_info['first_transit_time'] = t - path_info['generated_time']
                            path_info['first_hop_count'] = hops
        # Attach unique nodes processed info
        path_info['nodes_processed'] = sorted(list(nodes_seen))
        path_info['unique_nodes_processed'] = len(nodes_seen)
        
        packet_paths[packet_seq] = path_info
    
    return packet_paths

def generate_detailed_report(coordinates, extraction_info, df, packet_paths, output_file=None):
    """Generate comprehensive analysis report with all requested metrics."""
    
    # Calculate distance
    distance = None
    if 1000 in coordinates and 1001 in coordinates:
        distance = calculate_distance(coordinates[1000], coordinates[1001])
    
    # Detect routing method
    routing_method = detect_routing_method(df)
    
    # Count packet statistics
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    delivered_packets = [p for p in packet_paths.values() if p['delivered']]
    
    # Transit time statistics across ALL delivery copies (to destination when known)
    transit_times = []
    for p in delivered_packets:
        # Prefer the list of transit_times if populated; fallback to legacy single value
        if p.get('transit_times'):
            transit_times.extend([t for t in p['transit_times'] if t is not None])
        elif p.get('transit_time') is not None:
            transit_times.append(p['transit_time'])
    
    # Prepare analysis reports directory inside the simulations folder
    simulations_dir = os.path.dirname(os.path.abspath(__file__))
    reports_dir = os.path.join(simulations_dir, "analysis reports")
    os.makedirs(reports_dir, exist_ok=True)

    # Generate output filename if not provided
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if distance is not None:
            output_file = f"{routing_method}_{distance:.2f}m_{timestamp}.txt"
        else:
            output_file = f"{routing_method}_no_distance_{timestamp}.txt"

    # If output_file is not an absolute path, place it inside the reports directory
    if not os.path.isabs(output_file):
        output_file = os.path.join(reports_dir, output_file)
    
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("END NODE DISTANCE AND PATH ANALYSIS REPORT")
    report_lines.append("=" * 80)
    report_lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"Data Source: paths.csv ({len(df)} events)")
    report_lines.append("")
    
    # SECTION 1: COORDINATE EXTRACTION STATUS
    report_lines.append("1. COORDINATE EXTRACTION STATUS")
    report_lines.append("-" * 50)
    report_lines.append(f"Routing method detected: {routing_method.upper()}")
    report_lines.append(f"Results directory: {extraction_info.get('results_dir', 'Not found')}")
    if extraction_info.get('source_file'):
        report_lines.append(f"Most recent scalar file: {extraction_info.get('source_file')}")
    report_lines.append(f"Files scanned: {extraction_info['files_scanned']}")
    report_lines.append(f"Scalar entries found: {extraction_info['scalars_found']}")
    report_lines.append(f"Coordinates extracted: {extraction_info['coordinates_extracted']}")
    
    # Add routing method explanation
    report_lines.append("")
    report_lines.append("ROUTING METHOD ANALYSIS:")
    if routing_method == 'flooding':
        report_lines.append("  Detected FLOODING: All transmissions use broadcast addresses")
        report_lines.append("  - Packets are forwarded by all receiving nodes")
        report_lines.append("  - No routing table lookups required")
        report_lines.append("  - High redundancy, good for reliability")
    elif routing_method == 'routing':
        report_lines.append("  Detected ROUTING: Uses unicast next-hop addressing")
        report_lines.append("  - Packets follow calculated routes via routing tables")
        report_lines.append("  - Efficient bandwidth usage")
        report_lines.append("  - Lower redundancy than flooding")
    elif routing_method == 'mixed':
        report_lines.append("  Detected MIXED: Combination of flooding and routing")
        report_lines.append("  - Some packets broadcast, others use routing tables")
        report_lines.append("  - Hybrid approach balancing efficiency and reliability")
    else:
        report_lines.append("  UNKNOWN: Unable to determine routing method from data")
        report_lines.append("  - May indicate insufficient path data or unusual configuration")
    
    # Check if coordinates vary across runs
    if 'coordinate_variation' in extraction_info and not extraction_info['coordinate_variation']:
        report_lines.append("")
        report_lines.append("⚠️  WARNING: COORDINATES DO NOT VARY BETWEEN RUNS")
        report_lines.append("  The same coordinates appear in multiple .sca files.")
        report_lines.append("  This suggests:")
        report_lines.append("  - Simulation is using fixed seed instead of random")
        report_lines.append("  - Same configuration run multiple times overwrites results") 
        report_lines.append("  - End node placement may not be truly randomized")
        report_lines.append("  Check simulation INI file for seed-set configuration")
    
    if distance is not None:
        report_lines.append("")
        report_lines.append("✓ FRESH COORDINATES FOUND:")
        report_lines.append(f"  End Node 1000 position: ({coordinates[1000]['x']:.2f}, {coordinates[1000]['y']:.2f})")
        report_lines.append(f"  End Node 1001 position: ({coordinates[1001]['x']:.2f}, {coordinates[1001]['y']:.2f})")
        report_lines.append(f"  Distance: {distance:.2f} meters")
    else:
        report_lines.append("")
        report_lines.append("❌ NO FRESH COORDINATES AVAILABLE")
        report_lines.append("  Run simulation first to generate .sca files with position data")
    
    report_lines.append("")
    
    # SECTION 2a: NETWORK ENERGY SUMMARY (aggregated from .sca scalars)
    # We sum per-node scalars if available: energyTx, energyRx, energyReceiverBusy, energyIdle, totalEnergyConsumed
    # If per-mode scalars are missing, we still report totalEnergyConsumed.
    def _aggregate_energy_scalars_from_latest_sca(results_dir):
        energy_totals = {
            'energyTx': 0.0,
            'energyRx': 0.0,
            'energyReceiverBusy': 0.0,
            'energyIdle': 0.0,
            'totalEnergyConsumed': 0.0,
        }
        found_any = False
        try:
            sca_files = glob.glob(os.path.join(results_dir or '.', "*.sca"))
            sca_files.sort(key=os.path.getmtime, reverse=True)
            if not sca_files:
                return energy_totals, found_any
            latest = sca_files[0]
            # Patterns:
            # scalar <module_path> <name> <value>
            line_pat = re.compile(r"^\s*scalar\s+(\S+)\s+(\S+)\s+([0-9eE+\-\.]+)\s*$")
            with open(latest, 'r', encoding='utf-8') as f:
                for line in f:
                    m = line_pat.match(line)
                    if not m:
                        continue
                    name = m.group(2)
                    val = float(m.group(3))
                    if name in energy_totals:
                        energy_totals[name] += val
                        found_any = True
        except Exception:
            pass
        return energy_totals, found_any

    energy_totals, energy_found = _aggregate_energy_scalars_from_latest_sca(extraction_info.get('results_dir'))

    report_lines.append("NETWORK ENERGY SUMMARY")
    report_lines.append("-" * 50)
    if energy_found or energy_totals.get('totalEnergyConsumed', 0.0) > 0.0:
        # Always report total; per-mode only if non-zero or present
        tx = energy_totals.get('energyTx')
        rx = energy_totals.get('energyRx')
        busy = energy_totals.get('energyReceiverBusy')
        idle = energy_totals.get('energyIdle')
        total = energy_totals.get('totalEnergyConsumed')

        # Per-state if available (non-zero)
        if tx:
            report_lines.append(f"  Total transmission energy consumed by network: {tx:.6f} J")
        if rx:
            report_lines.append(f"  Total receiving energy consumed by network: {rx:.6f} J")
        if busy:
            report_lines.append(f"  Total receiver-busy energy consumed by network: {busy:.6f} J")
        if idle:
            report_lines.append(f"  Total idle energy consumed by network: {idle:.6f} J")
        report_lines.append(f"  Total energy consumed by network: {total:.6f} J")
    else:
        report_lines.append("  No energy scalars found in the latest .sca file.")
        report_lines.append("  Tip: modify LoRaEnergyConsumer to record per-mode scalars (energyTx, energyRx, energyReceiverBusy, energyIdle)")

    report_lines.append("")

    # SECTION 2: PACKET GENERATION AND DELIVERY
    report_lines.append("2. PACKET GENERATION AND DELIVERY STATISTICS")
    report_lines.append("-" * 50)
    report_lines.append(f"Total data packets generated: {total_generated}")
    report_lines.append(f"Total data packets delivered: {total_delivered}")
    
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        report_lines.append(f"Delivery success rate: {delivery_rate:.1f}% ({total_delivered}/{total_generated})")
    
    report_lines.append("")
    
    # SECTION 3: INDIVIDUAL PACKET PATHS
    report_lines.append("3. INDIVIDUAL PACKET PATH ANALYSIS")
    report_lines.append("-" * 50)
    
    for packet_seq, path in packet_paths.items():
        report_lines.append(f"Packet {packet_seq}:")
        report_lines.append(f"  Source: {path['source']} → Destination: {path['destination']}")
        report_lines.append(f"  Generated at: {path['generated_time']:.3f}s")
        
        if path['delivered']:
            # Print all delivery timestamps and all transit times (if available)
            if path.get('deliver_times'):
                times_str = ', '.join(f"{t:.3f}s" for t in path['deliver_times'])
                report_lines.append(f"  ✓ Delivered copies at: {times_str}")
            else:
                report_lines.append(f"  ✓ Delivered at: {path['delivered_time']:.3f}s")

            if path.get('transit_times'):
                tt_str = ', '.join(f"{t:.3f}s" for t in path['transit_times'])
                report_lines.append(f"  Transit times (all copies): {tt_str}")
            elif path.get('transit_time') is not None:
                report_lines.append(f"  Transit time: {path['transit_time']:.3f}s")
            # Final metrics = first arrival copy
            if path.get('first_transit_time') is not None:
                report_lines.append(f"  Final transit time: {path['first_transit_time']:.3f}s")
            if path.get('first_hop_count') is not None:
                # Adjust final hop count by +1 as requested
                report_lines.append(f"  Final hop count: {path['first_hop_count'] + 1}")
        else:
            report_lines.append(f"  ❌ Not delivered")
        
        # Hop count: prefer TTL-based per delivery
        if path.get('deliver_hop_counts'):
            hops_str = ', '.join(str(h) for h in path['deliver_hop_counts'])
            report_lines.append(f"  Hop count (TTL decrement, per delivery): {hops_str}")
        else:
            report_lines.append(f"  Hop count: {path['hop_count']}")
        # New: unique nodes processed
        report_lines.append(f"  Unique nodes processed: {path.get('unique_nodes_processed', 0)}")
        nodes_list = path.get('nodes_processed') or []
        report_lines.append(f"  Nodes processed: {','.join(str(n) for n in nodes_list)}")
        # (Path and event trace omitted by user request)
        report_lines.append("")
    
    # SECTION 4: TRANSIT TIME STATISTICS
    report_lines.append("4. TRANSIT TIME STATISTICS")
    report_lines.append("-" * 50)
    
    if transit_times:
        report_lines.append(f"Successfully delivered copies: {len(transit_times)}")
        report_lines.append("")
        report_lines.append("Transit time statistics:")
        report_lines.append(f"  Average transit time: {sum(transit_times)/len(transit_times):.3f} seconds")
        report_lines.append(f"  Minimum transit time: {min(transit_times):.3f} seconds")
        report_lines.append(f"  Maximum transit time: {max(transit_times):.3f} seconds")
        
        if len(transit_times) > 1:
            import statistics
            report_lines.append(f"  Standard deviation: {statistics.stdev(transit_times):.3f} seconds")
            report_lines.append(f"  Median transit time: {statistics.median(transit_times):.3f} seconds")
        
        # Ranking
        # Ranking by fastest first copy per packet
        def first_transit(p):
            if p.get('transit_times'):
                return min(p['transit_times'])
            return p.get('transit_time') if p.get('transit_time') is not None else float('inf')
        sorted_packets = sorted(delivered_packets, key=lambda x: first_transit(x))
        report_lines.append("")
        report_lines.append("Delivery ranking (fastest to slowest):")
        for i, packet in enumerate(sorted_packets[:5], 1):
            ft = first_transit(packet)
            report_lines.append(f"  {i}. Packet {packet['packet_seq']}: {ft:.3f}s (fastest copy)")
    else:
        report_lines.append("No packets were successfully delivered - cannot calculate transit times")
    
    report_lines.append("")
    
    # SECTION 5: NETWORK PERFORMANCE SUMMARY
    report_lines.append("5. NETWORK PERFORMANCE SUMMARY")
    report_lines.append("-" * 50)
    
    if distance is not None:
        report_lines.append(f"Network span: {distance:.1f} meters between end nodes")
    
    if total_generated > 0 and total_delivered > 0:
        # Report average hop count can remain, but we also prefer final metrics per packet above.
        all_hops = []
        for p in delivered_packets:
            if p.get('first_hop_count') is not None:
                all_hops.append(p['first_hop_count'])
            elif p.get('deliver_hop_counts'):
                all_hops.append(min(p['deliver_hop_counts']))
            elif p.get('hop_count') is not None:
                all_hops.append(p['hop_count'])
        if all_hops:
            avg_hops = sum(all_hops) / len(all_hops)
            report_lines.append(f"Average hop count for delivered packets: {avg_hops:.1f}")
        
        if transit_times:
            throughput = len(transit_times) / max(transit_times) if transit_times else 0
            report_lines.append(f"Effective throughput: {throughput:.2f} packets/second")
    
    # Performance assessment
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        if delivery_rate >= 90:
            report_lines.append("✓ EXCELLENT: Very high delivery success rate")
        elif delivery_rate >= 75:
            report_lines.append("✓ GOOD: High delivery success rate")
        elif delivery_rate >= 50:
            report_lines.append("⚠ FAIR: Moderate delivery success rate")
        else:
            report_lines.append("❌ POOR: Low delivery success rate")
    
    if transit_times:
        avg_transit = sum(transit_times) / len(transit_times)
        if avg_transit < 2.0:
            report_lines.append("✓ FAST: Quick network response times")
        elif avg_transit < 5.0:
            report_lines.append("✓ GOOD: Reasonable network response times")
        else:
            report_lines.append("⚠ SLOW: High network latency")
    
    report_lines.append("")
    report_lines.append("=" * 80)
    report_lines.append("END OF REPORT")
    report_lines.append("=" * 80)
    
    # Write to file
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('\n'.join(report_lines))

        print(f"✓ Analysis complete! Report saved to: {output_file}")

        # Append summary to master CSV
        append_to_summary_csv(coordinates, extraction_info, df, packet_paths, output_file, distance, routing_method)

        return output_file
    except Exception as e:
        print(f"ERROR writing report: {e}")
        return None

def main():
    """Main function with command-line interface."""
    parser = argparse.ArgumentParser(description='End Node Distance and Path Tracker')
    parser.add_argument('--paths', default='delivered_packets/paths.csv',
                       help='Path to paths.csv file (default: delivered_packets/paths.csv)')
    parser.add_argument('--results-dir', 
                       help='Specific results directory to scan for .sca files')
    parser.add_argument('--output', 
                       help='Output report filename (auto-generated if not specified)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    parser.add_argument('--just-distance', action='store_true',
                       help='Print only end-node distance and exit')
    
    args = parser.parse_args()
    
    # Check if paths.csv exists
    if not os.path.exists(args.paths):
        print(f"ERROR: {args.paths} not found!")
        print("Make sure you're running from the correct directory and that simulation has generated path data.")
        return 1
    
    # Load paths data
    try:
        df = pd.read_csv(args.paths)
        print(f"Loaded {len(df)} events from {args.paths}")
    except Exception as e:
        print(f"ERROR loading CSV: {e}")
        return 1
    
    # Extract coordinates from .sca files
    results_dir = args.results_dir or find_latest_results_directory()
    
    if not results_dir:
        print("ERROR: No results directory found with .sca files!")
        print("Run a simulation first to generate result files.")
        return 1
    
    print(f"Extracting end node positions from .sca files...")
    coordinates, extraction_info = extract_end_node_coordinates(results_dir)
    
    if args.verbose:
        print(f"Extraction info: {extraction_info}")
    
    if not coordinates:
        print("ERROR: No coordinates found in CSV file.")
        print("Run a simulation first to populate position data.")
        return 1
    
    # If only distance is requested, still print distance but continue to generate report and summary
    if args.just_distance:
        if 1000 in coordinates and 1001 in coordinates:
            distance = calculate_distance(coordinates[1000], coordinates[1001])
            print(f"End node 1000→1001 distance: {distance:.2f} meters")
            if 'timestamp' in coordinates[1000]:
                print(f"  (from simulation time {coordinates[1000]['timestamp']:.2f}s)")
        else:
            print("Distance not available (missing coordinates)")
            # Continue so report indicates missing coordinates and appends summary accordingly

    # Analyze packet paths
    print("Analyzing packet paths...")
    packet_paths = analyze_packet_paths(df)
    
    # Generate report (also appends a summary CSV inside the function)
    report_file = generate_detailed_report(coordinates, extraction_info, df, packet_paths, args.output)
    
    # Console summary
    print("\n" + "="*50)
    print("QUICK SUMMARY:")
    
    if 1000 in coordinates and 1001 in coordinates:
        distance = calculate_distance(coordinates[1000], coordinates[1001])
        print(f"• End node distance: {distance:.1f} meters")
        print(f"  - Node 1000: ({coordinates[1000]['x']:.1f}, {coordinates[1000]['y']:.1f})")
        print(f"  - Node 1001: ({coordinates[1001]['x']:.1f}, {coordinates[1001]['y']:.1f})")
    else:
        print("• End node distance: Not available (no fresh .sca files)")
    
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    print(f"• Packets generated: {total_generated}")
    print(f"• Packets delivered: {total_delivered}")
    
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        print(f"• Delivery rate: {delivery_rate:.1f}%")
    
    delivered_packets = [p for p in packet_paths.values() if p['delivered']]
    transit_times = [p['transit_time'] for p in delivered_packets if p['transit_time'] is not None]
    
    if transit_times:
        print(f"• Average transit time: {sum(transit_times)/len(transit_times):.3f}s")
        print(f"• Transit time range: {min(transit_times):.3f}s - {max(transit_times):.3f}s")
    
    if report_file:
        print(f"\n✓ Open '{report_file}' for detailed analysis!")
    
    return 0

if __name__ == "__main__":
    exit(main())