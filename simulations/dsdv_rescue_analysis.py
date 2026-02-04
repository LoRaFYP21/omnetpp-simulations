#!/usr/bin/env python3
"""
DSDV Rescue Node Mobility Analysis
Analyzes LoRa mesh DSDV simulations with mobile rescue nodes:
- Rescue node speed detection from configuration
- Distance calculation between end node (1000) and rescue node (2000)
- DSDV-specific routing metrics
- Path analysis with unicast forwarding verification
- Report naming: "DSDV_mobile_{speed}mps_{distance}m_{timestamp}.txt"
"""

import pandas as pd
import os
import glob
import math
import argparse
import csv
import re
from datetime import datetime
from pathlib import Path

def find_latest_results_directory(base_dir="./"):
    """Find the most recent results directory containing .sca files."""
    possible_dirs = [
        os.path.join(base_dir, "results"),
        os.path.join(base_dir, "simulations/results"),
        os.path.join(base_dir, "../results"),
    ]
    
    for pattern in ["**/results*", "**/General-*"]:
        matches = glob.glob(os.path.join(base_dir, pattern), recursive=True)
        possible_dirs.extend(matches)
    
    valid_dirs = []
    for results_dir in possible_dirs:
        if os.path.isdir(results_dir):
            sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
            if sca_files:
                latest_time = max(os.path.getmtime(f) for f in sca_files)
                valid_dirs.append((results_dir, latest_time, len(sca_files)))
    
    if not valid_dirs:
        return None
    
    latest_dir = max(valid_dirs, key=lambda x: x[1])
    print(f"Using results directory: {latest_dir[0]} ({latest_dir[2]} .sca files)")
    return latest_dir[0]

def extract_rescue_node_speed(results_dir=None, config_file=None):
    """
    Extract rescue node speed from INI configuration or .sca files.
    Priority: 1) config_file parameter, 2) .sca scalar, 3) search INI files
    """
    speed = None
    speed_source = "unknown"
    
    # Try to find speed in .sca file first (if recorded as scalar)
    if results_dir and os.path.isdir(results_dir):
        sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
        sca_files.sort(key=os.path.getmtime, reverse=True)
        
        for sca_file in sca_files[:1]:  # Check only most recent
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
                        
                        # Look for rescue node speed scalar
                        if 'loRaRescueNodes' in module_path and scalar_name in ('speed', 'mobilitySpeed'):
                            try:
                                speed = float(value_str)
                                speed_source = f"sca:{os.path.basename(sca_file)}"
                                return speed, speed_source
                            except ValueError:
                                pass
            except Exception as e:
                continue
    
    # Try to extract from INI configuration files
    if config_file and os.path.exists(config_file):
        ini_files = [config_file]
    else:
        # Search for INI files in simulations directory
        simulations_dir = os.path.dirname(os.path.abspath(__file__))
        ini_files = glob.glob(os.path.join(simulations_dir, "*.ini"))
    
    # Pattern to match rescue node speed configuration
    # Examples: **.loRaRescueNodes[*].mobility.speed = 15mps
    speed_pattern = re.compile(r'\*\*\.loRaRescueNodes\[\*\]\.mobility\.speed\s*=\s*([0-9.]+)\s*mps', re.IGNORECASE)
    
    for ini_file in ini_files:
        try:
            with open(ini_file, 'r') as f:
                for line in f:
                    match = speed_pattern.search(line)
                    if match:
                        speed = float(match.group(1))
                        speed_source = f"ini:{os.path.basename(ini_file)}"
                        return speed, speed_source
        except Exception:
            continue
    
    return speed, speed_source

def extract_node_coordinates(results_dir, node_ids=[1000, 2000]):
    """
    Extract specified node coordinates from .sca files.
    node_ids: list of node IDs to extract (default: [1000, 2000])
    Returns coordinates dict and extraction info.
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
    
    sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
    sca_files.sort(key=os.path.getmtime, reverse=True)
    
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
                    
                    if scalar_name not in ('CordiX', 'CordiY', 'positionX', 'positionY'):
                        continue
                    
                    # Extract node ID
                    node_id = None
                    
                    # End nodes: loRaEndNodes[0] -> 1000
                    if 'loRaEndNodes[' in module_path:
                        start = module_path.find('loRaEndNodes[') + len('loRaEndNodes[')
                        end = module_path.find(']', start)
                        if end != -1:
                            try:
                                idx = int(module_path[start:end])
                                node_id = 1000 + idx
                            except ValueError:
                                continue
                    
                    # Rescue nodes: loRaRescueNodes[0] -> 2000
                    elif 'loRaRescueNodes[' in module_path:
                        start = module_path.find('loRaRescueNodes[') + len('loRaRescueNodes[')
                        end = module_path.find(']', start)
                        if end != -1:
                            try:
                                idx = int(module_path[start:end])
                                node_id = 2000 + idx
                            except ValueError:
                                continue
                    
                    if node_id not in node_ids:
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
                        extraction_info['coordinates_extracted'] += 1
                        found_coords = True
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
                    elif scalar_name in ('CordiY', 'positionY'):
                        coordinates[node_id]['y'] = coord_val
                        coordinates[node_id]['source_file'] = os.path.basename(sca_file)
                        extraction_info['coordinates_extracted'] += 1
                        found_coords = True
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
        except Exception as e:
            print(f"Warning: Error reading {sca_file}: {e}")
            continue
        
        # Stop after finding coordinates in the first file
        if found_coords and all(nid in coordinates for nid in node_ids):
            if all(coordinates[nid]['x'] is not None and coordinates[nid]['y'] is not None for nid in node_ids):
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

def extract_total_energy(results_dir):
    """
    Extract total energy consumption from .sca files.
    Sums residualEnergyCapacity or totalEnergyConsumed for all nodes.
    """
    total_energy = 0.0
    energy_readings = []
    energy_source = "unknown"
    
    if not results_dir or not os.path.isdir(results_dir):
        return None, energy_readings
    
    sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
    sca_files.sort(key=os.path.getmtime, reverse=True)
    
    for sca_file in sca_files[:1]:  # Check most recent file
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
                    
                    # Look for energy scalars
                    if scalar_name in ('totalEnergyConsumed', 'residualEnergyCapacity', 'energyConsumed'):
                        try:
                            energy_val = float(value_str)
                            energy_readings.append({
                                'module': module_path,
                                'metric': scalar_name,
                                'value': energy_val
                            })
                            total_energy += abs(energy_val)  # Use absolute value
                            energy_source = os.path.basename(sca_file)
                        except ValueError:
                            pass
        except Exception as e:
            print(f"Warning: Error reading energy from {sca_file}: {e}")
            continue
    
    if total_energy > 0:
        return total_energy, energy_readings
    return None, energy_readings

def detect_routing_protocol(results_dir, config_file=None):
    """
    Detect routing protocol from .sca or .ini files.
    Returns protocol name: 'dsdv', 'flooding', 'smart_flooding', or 'unknown'
    """
    protocol = 'unknown'
    
    # Try .sca file first
    if results_dir and os.path.isdir(results_dir):
        sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
        sca_files.sort(key=os.path.getmtime, reverse=True)
        
        for sca_file in sca_files[:1]:
            try:
                with open(sca_file, 'r') as f:
                    for line in f:
                        # Check for routing protocol parameter
                        if 'param' in line and 'routingProtocol' in line:
                            if 'dsdv' in line.lower():
                                return 'dsdv'
                        
                        # Check for routing metric (indicates flooding vs DSDV)
                        if 'param' in line and 'routingMetric' in line:
                            if 'loRaNodes[' in line or '**.loRaNodes[*]' in line:
                                # Extract metric value
                                if '"2"' in line or ' 2' in line:
                                    protocol = 'smart_flooding'
                                elif '"1"' in line or ' 1' in line:
                                    protocol = 'flooding'
                                elif '"3"' in line or ' 3' in line:
                                    # Could be hop-count routing, check if DSDV
                                    pass
            except Exception:
                continue
    
    # Fallback to INI file
    if protocol == 'unknown':
        if config_file and os.path.exists(config_file):
            ini_files = [config_file]
        else:
            simulations_dir = os.path.dirname(os.path.abspath(__file__))
            ini_files = glob.glob(os.path.join(simulations_dir, "*.ini"))
        
        for ini_file in ini_files:
            try:
                with open(ini_file, 'r') as f:
                    content = f.read()
                    if 'routingProtocol' in content and '"dsdv"' in content:
                        return 'dsdv'
            except Exception:
                continue
    
    return protocol if protocol != 'unknown' else 'dsdv'  # Default to DSDV

def extract_dsdv_timers(results_dir, config_file=None):
    """
    Extract DSDV timer configurations from .sca or .ini files.
    Returns dict with timer settings for relay, end, and rescue nodes.
    """
    timers = {
        'relay_incremental': None,
        'relay_full': None,
        'endnode_incremental': None,
        'endnode_full': None,
        'rescue_incremental': None,
        'rescue_full': None
    }
    
    # Try .sca file first (has actual parameter values)
    if results_dir and os.path.isdir(results_dir):
        sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
        sca_files.sort(key=os.path.getmtime, reverse=True)
        
        for sca_file in sca_files[:1]:
            try:
                with open(sca_file, 'r') as f:
                    for line in f:
                        if not line.startswith('param '):
                            continue
                        
                        parts = line.strip().split()
                        if len(parts) < 3:
                            continue
                        
                        param_path = parts[1]
                        param_value = ' '.join(parts[2:])
                        
                        # Extract numeric value from param_value (may have units like "15s")
                        value_match = re.search(r'([0-9.]+)s?', param_value)
                        if not value_match:
                            continue
                        
                        value = float(value_match.group(1))
                        
                        # Relay nodes (loRaNodes)
                        if 'loRaNodes[' in param_path or '**.loRaNodes[*]' in param_path:
                            if 'dsdvIncrementalPeriod' in param_path:
                                timers['relay_incremental'] = value
                            elif 'dsdvFullUpdatePeriod' in param_path:
                                timers['relay_full'] = value
                        
                        # End nodes
                        elif 'loRaEndNodes[' in param_path or '**.loRaEndNodes[*]' in param_path:
                            if 'dsdvIncrementalPeriod' in param_path:
                                timers['endnode_incremental'] = value
                            elif 'dsdvFullUpdatePeriod' in param_path:
                                timers['endnode_full'] = value
                        
                        # Rescue nodes
                        elif 'loRaRescueNodes[' in param_path or '**.loRaRescueNodes[*]' in param_path:
                            if 'dsdvIncrementalPeriod' in param_path:
                                timers['rescue_incremental'] = value
                            elif 'dsdvFullUpdatePeriod' in param_path:
                                timers['rescue_full'] = value
            except Exception as e:
                continue
    
    # Fallback to INI file if needed
    if any(v is None for v in timers.values()):
        if config_file and os.path.exists(config_file):
            ini_files = [config_file]
        else:
            simulations_dir = os.path.dirname(os.path.abspath(__file__))
            ini_files = glob.glob(os.path.join(simulations_dir, "*.ini"))
        
        for ini_file in ini_files:
            try:
                with open(ini_file, 'r') as f:
                    for line in f:
                        if 'dsdvIncrementalPeriod' in line or 'dsdvFullUpdatePeriod' in line:
                            value_match = re.search(r'=\s*([0-9.]+)s', line)
                            if not value_match:
                                continue
                            value = float(value_match.group(1))
                            
                            if 'loRaNodes[*]' in line and timers['relay_incremental'] is None:
                                if 'dsdvIncrementalPeriod' in line:
                                    timers['relay_incremental'] = value
                                elif 'dsdvFullUpdatePeriod' in line:
                                    timers['relay_full'] = value
                            
                            elif 'loRaEndNodes[*]' in line and timers['endnode_incremental'] is None:
                                if 'dsdvIncrementalPeriod' in line:
                                    timers['endnode_incremental'] = value
                                elif 'dsdvFullUpdatePeriod' in line:
                                    timers['endnode_full'] = value
                            
                            elif 'loRaRescueNodes[*]' in line and timers['rescue_incremental'] is None:
                                if 'dsdvIncrementalPeriod' in line:
                                    timers['rescue_incremental'] = value
                                elif 'dsdvFullUpdatePeriod' in line:
                                    timers['rescue_full'] = value
            except Exception:
                continue
    
    return timers

def analyze_dsdv_packet_paths(df):
    """
    Analyze DSDV packet paths with focus on:
    - Unicast forwarding verification
    - Route lookup success
    - Hop count accuracy
    - Duplicate deliveries (copies received at destination)
    - Unique nodes that processed packets
    """
    packet_paths = {}
    
    for packet_seq in df['packetSeq'].unique():
        packet_events = df[df['packetSeq'] == packet_seq].sort_values('simTime')
        
        path_info = {
            'packet_seq': packet_seq,
            'source': None,
            'destination': None,
            'generated_time': None,
            'delivered_time': None,
            'path_nodes': [],
            'hop_count': 0,
            'transit_time': None,
            'delivered': False,
            'path_events': [],
            'unicast_forwards': 0,
            'broadcast_forwards': 0,
            'initial_ttl': None,
            'deliver_hop_counts': [],
            'first_transit_time': None,
            'first_hop_count': None,
            'copies_at_destination': 0,  # Count duplicate deliveries
            'unique_nodes_processed': set()  # Track all nodes that touched this packet
        }
        
        for _, event in packet_events.iterrows():
            event_data = {
                'time': event['simTime'],
                'event_type': event['event'],
                'node': event['currentNode'],
                'next_hop': event.get('chosenVia', None),
                'hop_type': event.get('nextHopType', None)
            }
            path_info['path_events'].append(event_data)
            
            # Track all unique nodes that processed this packet
            try:
                if not pd.isna(event['currentNode']):
                    node_id = int(event['currentNode'])
                    path_info['unique_nodes_processed'].add(node_id)
            except Exception:
                pass
            
            if event['event'] == 'TX_SRC':
                path_info['source'] = event['src']
                path_info['destination'] = event['dst']
                path_info['generated_time'] = event['simTime']
                path_info['path_nodes'].append(event['currentNode'])
                try:
                    path_info['initial_ttl'] = int(event['ttlAfterDecr']) if not pd.isna(event['ttlAfterDecr']) else None
                except Exception:
                    path_info['initial_ttl'] = None
            
            elif event['event'] in ['TX_FWD_DATA', 'TX_FWD_ACK']:
                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])
                    path_info['hop_count'] += 1
                
                # Count unicast vs broadcast forwards
                try:
                    via = int(event['chosenVia']) if not pd.isna(event['chosenVia']) else None
                    if via == 16777215:  # BROADCAST_ADDRESS
                        path_info['broadcast_forwards'] += 1
                    elif via is not None:
                        path_info['unicast_forwards'] += 1
                except Exception:
                    pass
            
            elif event['event'] == 'DELIVERED':
                path_info['delivered'] = True
                path_info['copies_at_destination'] += 1  # Count each delivery
                
                # Only set time for first delivery
                if path_info['delivered_time'] is None:
                    path_info['delivered_time'] = event['simTime']
                
                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])
                
                if path_info['generated_time'] is not None:
                    transit = event['simTime'] - path_info['generated_time']
                    
                    if path_info['first_transit_time'] is None:
                        path_info['first_transit_time'] = transit
                        path_info['transit_time'] = transit
                
                # TTL-based hop count
                try:
                    ttl_at_delivery = int(event['ttlAfterDecr']) if not pd.isna(event['ttlAfterDecr']) else None
                except Exception:
                    ttl_at_delivery = None
                
                if path_info.get('initial_ttl') is not None and ttl_at_delivery is not None:
                    hops = path_info['initial_ttl'] - ttl_at_delivery
                    if hops >= 0:
                        path_info['deliver_hop_counts'].append(hops)
                        path_info['hop_count'] = hops
                        if path_info['first_hop_count'] is None:
                            path_info['first_hop_count'] = hops
        
        # Convert set to sorted list and count
        path_info['nodes_processed'] = sorted(list(path_info['unique_nodes_processed']))
        path_info['unique_nodes_count'] = len(path_info['unique_nodes_processed'])
        packet_paths[packet_seq] = path_info
    
    return packet_paths

def generate_dsdv_report(coordinates, extraction_info, df, packet_paths, rescue_speed, distance, total_energy, dsdv_timers, routing_protocol='dsdv', output_file=None):
    """Generate comprehensive analysis report with energy and timer data."""
    
    # Prepare analysis reports directory
    simulations_dir = os.path.dirname(os.path.abspath(__file__))
    reports_dir = os.path.join(simulations_dir, "analysis reports")
    os.makedirs(reports_dir, exist_ok=True)
    
    # Generate output filename with speed and distance
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Determine protocol prefix for filename
        if routing_protocol == 'smart_flooding':
            protocol_prefix = "SmartFlooding"
        elif routing_protocol == 'flooding':
            protocol_prefix = "Flooding"
        else:
            protocol_prefix = "DSDV"
        
        if rescue_speed is not None and distance is not None:
            output_file = f"{protocol_prefix}_mobile_{rescue_speed:.0f}mps_{distance:.0f}m_{timestamp}.txt"
        elif rescue_speed is not None:
            output_file = f"{protocol_prefix}_mobile_{rescue_speed:.0f}mps_no_distance_{timestamp}.txt"
        elif distance is not None:
            output_file = f"{protocol_prefix}_mobile_unknown_speed_{distance:.0f}m_{timestamp}.txt"
        else:
            output_file = f"{protocol_prefix}_mobile_analysis_{timestamp}.txt"
    
    if not os.path.isabs(output_file):
        output_file = os.path.join(reports_dir, output_file)
    
    # Calculate statistics
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    delivered_packets = [p for p in packet_paths.values() if p['delivered']]
    
    transit_times = []
    for p in delivered_packets:
        if p.get('transit_time') is not None:
            transit_times.append(p['transit_time'])
    
    report_lines = []
    report_lines.append("=" * 80)
    
    # Determine report title based on protocol
    if routing_protocol == 'smart_flooding':
        report_lines.append("SMART FLOODING RESCUE NODE MOBILITY ANALYSIS REPORT")
    elif routing_protocol == 'flooding':
        report_lines.append("FLOODING RESCUE NODE MOBILITY ANALYSIS REPORT")
    else:
        report_lines.append("DSDV RESCUE NODE MOBILITY ANALYSIS REPORT")
    
    report_lines.append("=" * 80)
    report_lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"Data Source: paths.csv ({len(df)} events)")
    report_lines.append("")
    
    # SECTION 1: CONFIGURATION
    report_lines.append("1. SIMULATION CONFIGURATION")
    report_lines.append("-" * 50)
    
    # Display protocol name based on detection
    if routing_protocol == 'smart_flooding':
        report_lines.append(f"Routing Protocol: Smart Flooding (SMART_BROADCAST_SINGLE_SF)")
    elif routing_protocol == 'flooding':
        report_lines.append(f"Routing Protocol: Flooding (Broadcast)")
    else:
        report_lines.append(f"Routing Protocol: DSDV (Destination-Sequenced Distance-Vector)")
    report_lines.append(f"Mobility Model: Mobile rescue node")
    
    if rescue_speed is not None:
        speed_source = extraction_info.get('speed_source', 'unknown')
        report_lines.append(f"Rescue Node Speed: {rescue_speed:.1f} m/s (source: {speed_source})")
    else:
        report_lines.append(f"Rescue Node Speed: Unknown (not found in configuration)")
    
    report_lines.append("")
    report_lines.append("Node Configuration:")
    report_lines.append(f"  • End Node (1000): Stationary data source")
    report_lines.append(f"  • Rescue Node (2000): Mobile destination")
    report_lines.append("")
    
    # DSDV Timer Configuration (only for DSDV protocol)
    if routing_protocol == 'dsdv' and dsdv_timers:
        report_lines.append("DSDV Timer Settings:")
        if dsdv_timers['relay_incremental'] is not None:
            report_lines.append(f"  • Relay Nodes (loRaNodes):")
            report_lines.append(f"    - Incremental update: {dsdv_timers['relay_incremental']:.0f}s")
            if dsdv_timers['relay_full'] is not None:
                report_lines.append(f"    - Full dump: {dsdv_timers['relay_full']:.0f}s")
        
        if dsdv_timers['endnode_incremental'] is not None:
            report_lines.append(f"  • End Nodes (loRaEndNodes):")
            report_lines.append(f"    - Incremental update: {dsdv_timers['endnode_incremental']:.0f}s")
            if dsdv_timers['endnode_full'] is not None:
                report_lines.append(f"    - Full dump: {dsdv_timers['endnode_full']:.0f}s")
        
        if dsdv_timers['rescue_incremental'] is not None:
            report_lines.append(f"  • Rescue Nodes (loRaRescueNodes):")
            report_lines.append(f"    - Incremental update: {dsdv_timers['rescue_incremental']:.0f}s")
            if dsdv_timers['rescue_full'] is not None:
                report_lines.append(f"    - Full dump: {dsdv_timers['rescue_full']:.0f}s")
    report_lines.append("")
    
    # SECTION 2: INITIAL POSITIONS AND DISTANCE
    report_lines.append("2. INITIAL POSITIONS AND DISTANCE")
    report_lines.append("-" * 50)
    
    if 1000 in coordinates and 2000 in coordinates:
        report_lines.append(f"End Node (1000) position: ({coordinates[1000]['x']:.2f}, {coordinates[1000]['y']:.2f}) m")
        report_lines.append(f"Rescue Node (2000) position: ({coordinates[2000]['x']:.2f}, {coordinates[2000]['y']:.2f}) m")
        if distance is not None:
            report_lines.append(f"Initial distance: {distance:.2f} meters")
    else:
        report_lines.append("❌ Coordinate extraction failed")
        if 1000 not in coordinates:
            report_lines.append("  • End node (1000) coordinates not found")
        if 2000 not in coordinates:
            report_lines.append("  • Rescue node (2000) coordinates not found")
    
    report_lines.append("")
    
    # SECTION 3: PACKET STATISTICS
    report_lines.append("3. PACKET GENERATION AND DELIVERY")
    report_lines.append("-" * 50)
    report_lines.append(f"Packets generated: {total_generated}")
    report_lines.append(f"Packets delivered: {total_delivered}")
    
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        report_lines.append(f"Delivery rate: {delivery_rate:.1f}% ({total_delivered}/{total_generated})")
    
    # Copies received at destination
    total_copies = sum(p.get('copies_at_destination', 0) for p in packet_paths.values())
    report_lines.append(f"Total copies received at destination: {total_copies}")
    if total_delivered > 0:
        avg_copies = total_copies / total_delivered
        report_lines.append(f"Average copies per delivered packet: {avg_copies:.2f}")
    
    # Total unique nodes that processed packets
    all_nodes_processed = set()
    for p in packet_paths.values():
        all_nodes_processed.update(p.get('nodes_processed', []))
    report_lines.append(f"Total unique nodes that processed packets: {len(all_nodes_processed)}")
    
    # Energy consumption
    if total_energy is not None and total_energy > 0:
        report_lines.append(f"")
        report_lines.append(f"Total energy consumption: {total_energy:.6f} J (Joules)")
        if total_generated > 0:
            energy_per_packet = total_energy / total_generated
            report_lines.append(f"Energy per generated packet: {energy_per_packet:.6f} J")
        if total_delivered > 0:
            energy_per_delivered = total_energy / total_delivered
            report_lines.append(f"Energy per delivered packet: {energy_per_delivered:.6f} J")
    
    report_lines.append("")
    
    # SECTION 4: ROUTING ANALYSIS
    if routing_protocol == 'smart_flooding':
        report_lines.append("4. SMART FLOODING BEHAVIOR")
    elif routing_protocol == 'flooding':
        report_lines.append("4. FLOODING BEHAVIOR")
    else:
        report_lines.append("4. DSDV ROUTING BEHAVIOR")
    report_lines.append("-" * 50)
    
    total_unicast = sum(p['unicast_forwards'] for p in packet_paths.values())
    total_broadcast = sum(p['broadcast_forwards'] for p in packet_paths.values())
    
    report_lines.append(f"Unicast forwards (routing table): {total_unicast}")
    report_lines.append(f"Broadcast forwards: {total_broadcast}")
    
    if total_unicast > 0 or total_broadcast > 0:
        unicast_pct = (total_unicast / (total_unicast + total_broadcast)) * 100
        report_lines.append(f"Unicast ratio: {unicast_pct:.1f}%")
        
        if routing_protocol == 'dsdv':
            if unicast_pct > 80:
                report_lines.append("✓ EXCELLENT: High unicast ratio indicates effective DSDV routing")
            elif unicast_pct > 50:
                report_lines.append("✓ GOOD: Moderate unicast usage with DSDV routes")
            else:
                report_lines.append("⚠ WARNING: Low unicast ratio may indicate routing issues")
        elif routing_protocol == 'smart_flooding':
            if unicast_pct < 20:
                report_lines.append("✓ EXPECTED: Low unicast ratio indicates broadcast-based flooding")
            elif unicast_pct < 50:
                report_lines.append("✓ GOOD: Smart flooding with some unicast optimization")
            else:
                report_lines.append("⚠ NOTE: High unicast ratio unusual for flooding protocol")
        else:  # flooding
            if total_broadcast > total_unicast:
                report_lines.append("✓ EXPECTED: Broadcast-dominant forwarding for flooding protocol")
            else:
                report_lines.append("⚠ NOTE: Unexpected unicast dominance in flooding protocol")
    
    report_lines.append("")
    
    # SECTION 5: INDIVIDUAL PACKET PATHS
    report_lines.append("5. PACKET PATH DETAILS")
    report_lines.append("-" * 50)
    
    for packet_seq, path in packet_paths.items():
        report_lines.append(f"\nPacket {packet_seq}:")
        report_lines.append(f"  Source: {path['source']} → Destination: {path['destination']}")
        report_lines.append(f"  Generated: {path['generated_time']:.3f}s")
        
        if path['delivered']:
            report_lines.append(f"  ✓ Delivered: {path['delivered_time']:.3f}s")
            if path['transit_time'] is not None:
                report_lines.append(f"  Transit time: {path['transit_time']:.3f}s")
            if path['first_hop_count'] is not None:
                report_lines.append(f"  Hop count: {path['first_hop_count'] + 1}")
            report_lines.append(f"  Copies received at destination: {path.get('copies_at_destination', 0)}")
        else:
            report_lines.append(f"  ❌ Not delivered")
        
        report_lines.append(f"  Unicast forwards: {path['unicast_forwards']}")
        report_lines.append(f"  Broadcast forwards: {path['broadcast_forwards']}")
        report_lines.append(f"  Unique nodes processed: {path.get('unique_nodes_count', 0)}")
        if path.get('nodes_processed'):
            report_lines.append(f"  Node IDs: {path['nodes_processed']}")
    
    report_lines.append("")
    
    # SECTION 6: TRANSIT TIME STATISTICS
    report_lines.append("6. TRANSIT TIME ANALYSIS")
    report_lines.append("-" * 50)
    
    if transit_times:
        report_lines.append(f"Delivered packets: {len(transit_times)}")
        report_lines.append(f"Average transit time: {sum(transit_times)/len(transit_times):.3f}s")
        report_lines.append(f"Minimum transit time: {min(transit_times):.3f}s")
        report_lines.append(f"Maximum transit time: {max(transit_times):.3f}s")
        
        if len(transit_times) > 1:
            import statistics
            report_lines.append(f"Std deviation: {statistics.stdev(transit_times):.3f}s")
            report_lines.append(f"Median: {statistics.median(transit_times):.3f}s")
    else:
        report_lines.append("No packets delivered - cannot calculate transit times")
    
    report_lines.append("")
    
    # SECTION 7: PERFORMANCE SUMMARY
    report_lines.append("7. PERFORMANCE ASSESSMENT")
    report_lines.append("-" * 50)
    
    if distance is not None:
        report_lines.append(f"Network span: {distance:.1f}m (end node to rescue node)")
    
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        if delivery_rate >= 90:
            report_lines.append("✓ EXCELLENT: Very high delivery rate")
        elif delivery_rate >= 75:
            report_lines.append("✓ GOOD: High delivery rate")
        elif delivery_rate >= 50:
            report_lines.append("⚠ FAIR: Moderate delivery rate")
        else:
            report_lines.append("❌ POOR: Low delivery rate")
    
    if transit_times:
        avg_transit = sum(transit_times) / len(transit_times)
        if avg_transit < 2.0:
            report_lines.append("✓ FAST: Quick network response")
        elif avg_transit < 5.0:
            report_lines.append("✓ GOOD: Reasonable response times")
        else:
            report_lines.append("⚠ SLOW: High latency")
    
    # Protocol-specific assessment
    if routing_protocol == 'dsdv' and total_unicast > 0:
        report_lines.append("✓ DSDV routing tables actively used for forwarding")
    elif routing_protocol in ['flooding', 'smart_flooding'] and total_broadcast > 0:
        report_lines.append(f"✓ {routing_protocol.replace('_', ' ').title()} protocol functioning as expected")
    
    report_lines.append("")
    report_lines.append("=" * 80)
    report_lines.append("END OF REPORT")
    report_lines.append("=" * 80)
    
    # Write report
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('\n'.join(report_lines))
        
        protocol_name = routing_protocol.replace('_', ' ').title()
        print(f"✓ {protocol_name} analysis complete! Report: {output_file}")
        return output_file
    except Exception as e:
        print(f"ERROR writing report: {e}")
        return None

def main():
    """Main function with command-line interface."""
    parser = argparse.ArgumentParser(description='DSDV Rescue Node Mobility Analysis')
    parser.add_argument('--paths', default='delivered_packets/paths.csv',
                       help='Path to paths.csv file')
    parser.add_argument('--results-dir',
                       help='Results directory with .sca files')
    parser.add_argument('--config',
                       help='INI configuration file path')
    parser.add_argument('--output',
                       help='Output report filename')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Check paths.csv
    if not os.path.exists(args.paths):
        print(f"ERROR: {args.paths} not found!")
        return 1
    
    # Load paths data
    try:
        df = pd.read_csv(args.paths)
        print(f"Loaded {len(df)} events from {args.paths}")
    except Exception as e:
        print(f"ERROR loading CSV: {e}")
        return 1
    
    # Find results directory
    results_dir = args.results_dir or find_latest_results_directory()
    if not results_dir:
        print("WARNING: No results directory found")
    
    # Extract rescue node speed
    print("Extracting rescue node speed...")
    rescue_speed, speed_source = extract_rescue_node_speed(results_dir, args.config)
    
    if rescue_speed is not None:
        print(f"  Rescue node speed: {rescue_speed:.1f} m/s (from {speed_source})")
    else:
        print(f"  WARNING: Could not determine rescue node speed")
    
    # Extract coordinates
    print("Extracting node coordinates...")
    coordinates, extraction_info = extract_node_coordinates(results_dir, node_ids=[1000, 2000])
    extraction_info['speed_source'] = speed_source
    
    if args.verbose:
        print(f"Extraction info: {extraction_info}")
    
    # Calculate distance
    distance = None
    if 1000 in coordinates and 2000 in coordinates:
        distance = calculate_distance(coordinates[1000], coordinates[2000])
        print(f"  End node (1000): ({coordinates[1000]['x']:.1f}, {coordinates[1000]['y']:.1f})")
        print(f"  Rescue node (2000): ({coordinates[2000]['x']:.1f}, {coordinates[2000]['y']:.1f})")
        print(f"  Initial distance: {distance:.1f} meters")
    else:
        print("  WARNING: Could not extract coordinates")
    
    # Detect routing protocol
    print("Detecting routing protocol...")
    routing_protocol = detect_routing_protocol(results_dir, args.config)
    print(f"  Routing protocol: {routing_protocol.replace('_', ' ').title()}")
    
    # Extract energy consumption
    print("Extracting energy consumption...")
    total_energy, energy_readings = extract_total_energy(results_dir)
    if total_energy is not None:
        print(f"  Total energy: {total_energy:.6f} J ({len(energy_readings)} readings)")
    else:
        print("  WARNING: Could not extract energy data")
    
    # Extract DSDV timers (only for DSDV protocol)
    dsdv_timers = {}
    if routing_protocol == 'dsdv':
        print("Extracting DSDV timer configurations...")
        dsdv_timers = extract_dsdv_timers(results_dir, args.config)
        if any(v is not None for v in dsdv_timers.values()):
            print("  Timer settings extracted")
        else:
            print("  WARNING: Could not extract DSDV timer settings")
    
    # Analyze packet paths
    print("Analyzing DSDV packet paths...")
    packet_paths = analyze_dsdv_packet_paths(df)
    
    # Generate report
    report_file = generate_dsdv_report(
        coordinates, extraction_info, df, packet_paths, 
        rescue_speed, distance, total_energy, dsdv_timers, routing_protocol, args.output
    )
    
    # Console summary
    print("\n" + "="*50)
    print("QUICK SUMMARY:")
    if rescue_speed is not None:
        print(f"• Rescue node speed: {rescue_speed:.1f} m/s")
    if distance is not None:
        print(f"• Initial distance: {distance:.1f} m")
    
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    print(f"• Packets: {total_delivered}/{total_generated} delivered")
    
    if total_generated > 0:
        delivery_rate = (total_delivered / total_generated) * 100
        print(f"• Delivery rate: {delivery_rate:.1f}%")
    
    delivered = [p for p in packet_paths.values() if p['delivered']]
    transit_times = [p['transit_time'] for p in delivered if p['transit_time'] is not None]
    
    if transit_times:
        print(f"• Avg transit time: {sum(transit_times)/len(transit_times):.3f}s")
    
    total_unicast = sum(p['unicast_forwards'] for p in packet_paths.values())
    print(f"• Unicast forwards: {total_unicast}")
    
    total_copies = sum(p.get('copies_at_destination', 0) for p in packet_paths.values())
    print(f"• Copies at destination: {total_copies}")
    
    all_nodes = set()
    for p in packet_paths.values():
        all_nodes.update(p.get('nodes_processed', []))
    print(f"• Unique nodes processed: {len(all_nodes)}")
    
    if total_energy is not None:
        print(f"• Total energy: {total_energy:.6f} J")
    
    if report_file:
        print(f"\n✓ Full report: {report_file}")
    
    return 0

if __name__ == "__main__":
    exit(main())
