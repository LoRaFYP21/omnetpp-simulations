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
from datetime import datetime
from pathlib import Path

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
        'source_file': None
    }
    
    if not results_dir or not os.path.isdir(results_dir):
        return coordinates, extraction_info
    
    # Get all .sca files, sorted by modification time (newest first)
    sca_files = glob.glob(os.path.join(results_dir, "*.sca"))
    sca_files.sort(key=os.path.getmtime, reverse=True)
    
    for sca_file in sca_files:
        extraction_info['files_scanned'] += 1
        
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
                        extraction_info['coordinates_extracted'] += 1
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
                    elif scalar_name in ('CordiY', 'positionY'):
                        coordinates[node_id]['y'] = coord_val
                        extraction_info['coordinates_extracted'] += 1
                        if not extraction_info['source_file']:
                            extraction_info['source_file'] = os.path.basename(sca_file)
                    
        except Exception as e:
            print(f"Warning: Error reading {sca_file}: {e}")
            continue
    
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
            'delivered_time': None,
            'path_nodes': [],
            'hop_count': 0,
            'transit_time': None,
            'delivered': False,
            'path_events': []
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
            
            # Extract key information
            if event['event'] == 'TX_SRC':
                path_info['source'] = event['src']
                path_info['destination'] = event['dst']
                path_info['generated_time'] = event['simTime']
                path_info['path_nodes'].append(event['currentNode'])
            
            elif event['event'] in ['TX_FWD_DATA', 'TX_FWD_ACK']:
                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])
                    path_info['hop_count'] += 1
            
            elif event['event'] == 'DELIVERED':
                path_info['delivered'] = True
                path_info['delivered_time'] = event['simTime']
                if event['currentNode'] not in path_info['path_nodes']:
                    path_info['path_nodes'].append(event['currentNode'])
                
                if path_info['generated_time']:
                    path_info['transit_time'] = path_info['delivered_time'] - path_info['generated_time']
        
        packet_paths[packet_seq] = path_info
    
    return packet_paths

def generate_detailed_report(coordinates, extraction_info, df, packet_paths, output_file=None):
    """Generate comprehensive analysis report with all requested metrics."""
    
    # Calculate distance
    distance = None
    if 1000 in coordinates and 1001 in coordinates:
        distance = calculate_distance(coordinates[1000], coordinates[1001])
    
    # Count packet statistics
    total_generated = len(df[df['event'] == 'TX_SRC'])
    total_delivered = len(df[df['event'] == 'DELIVERED'])
    delivered_packets = [p for p in packet_paths.values() if p['delivered']]
    
    # Transit time statistics
    transit_times = [p['transit_time'] for p in delivered_packets if p['transit_time'] is not None]
    
    # Generate output filename if not provided
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"endnode_distance_analysis_{timestamp}.txt"
    
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
    report_lines.append(f"Results directory: {extraction_info.get('source_file', 'Not found')}")
    report_lines.append(f"Files scanned: {extraction_info['files_scanned']}")
    report_lines.append(f"Scalar entries found: {extraction_info['scalars_found']}")
    report_lines.append(f"Coordinates extracted: {extraction_info['coordinates_extracted']}")
    
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
            report_lines.append(f"  ✓ Delivered at: {path['delivered_time']:.3f}s")
            report_lines.append(f"  Transit time: {path['transit_time']:.3f}s")
        else:
            report_lines.append(f"  ❌ Not delivered")
        
        report_lines.append(f"  Hop count: {path['hop_count']}")
        report_lines.append(f"  Path: {' → '.join(map(str, path['path_nodes']))}")
        
        # Detailed event trace
        report_lines.append("  Event sequence:")
        for event in path['path_events'][:10]:  # Limit to first 10 events
            hop_info = f" → {event['next_hop']}" if event['next_hop'] else ""
            report_lines.append(f"    {event['time']:.3f}s: {event['event_type']} at node {event['node']}{hop_info}")
        
        if len(path['path_events']) > 10:
            report_lines.append(f"    ... ({len(path['path_events'])-10} more events)")
        
        report_lines.append("")
    
    # SECTION 4: TRANSIT TIME STATISTICS
    report_lines.append("4. TRANSIT TIME STATISTICS")
    report_lines.append("-" * 50)
    
    if transit_times:
        report_lines.append(f"Successfully delivered packets: {len(transit_times)}")
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
        sorted_packets = sorted(delivered_packets, key=lambda x: x['transit_time'] or float('inf'))
        report_lines.append("")
        report_lines.append("Delivery ranking (fastest to slowest):")
        for i, packet in enumerate(sorted_packets[:5], 1):
            report_lines.append(f"  {i}. Packet {packet['packet_seq']}: {packet['transit_time']:.3f}s")
    else:
        report_lines.append("No packets were successfully delivered - cannot calculate transit times")
    
    report_lines.append("")
    
    # SECTION 5: NETWORK PERFORMANCE SUMMARY
    report_lines.append("5. NETWORK PERFORMANCE SUMMARY")
    report_lines.append("-" * 50)
    
    if distance is not None:
        report_lines.append(f"Network span: {distance:.1f} meters between end nodes")
    
    if total_generated > 0 and total_delivered > 0:
        avg_hops = sum(p['hop_count'] for p in delivered_packets) / len(delivered_packets)
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
    
    # Find results directory
    if args.results_dir:
        results_dir = args.results_dir
    else:
        results_dir = find_latest_results_directory()
    
    if not results_dir:
        print("WARNING: No results directory with .sca files found!")
        print("Distance calculation will not be available.")
        coordinates = {}
        extraction_info = {'files_scanned': 0, 'scalars_found': 0, 'coordinates_extracted': 0}
    else:
        # Extract coordinates
        coordinates, extraction_info = extract_end_node_coordinates(results_dir)
        
        if args.verbose:
            print(f"Coordinate extraction: {extraction_info}")
    
    # Analyze packet paths
    print("Analyzing packet paths...")
    packet_paths = analyze_packet_paths(df)
    
    # Generate report
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