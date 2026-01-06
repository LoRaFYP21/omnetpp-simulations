#!/usr/bin/env python3
"""
Comprehensive Simulation Analysis Script
Generates detailed report for LoRa mesh network simulation results
"""

import pandas as pd
import os
from datetime import datetime
import sys
import math

def get_end_node_coordinates(simulations_dir="./"):
    """Extract end node coordinates from .sca result files.
    Returns dict: {1000: (x,y), 1001: (x,y)} when available.
    Root cause fix: previous version mis-parsed module path (used parts[0] instead of parts[1]) so never matched end node patterns.
    """
    coordinates: dict[int, list] = {}

    results_dir = os.path.join(simulations_dir, "results")
    if not os.path.isdir(results_dir):
        return coordinates

    try:
        # Collect all .sca files (process all; coordinate scalars may appear in any)
        sca_files = [os.path.join(results_dir, f) for f in os.listdir(results_dir) if f.endswith('.sca')]
        if not sca_files:
            return coordinates

        for sca_file in sca_files:
            try:
                with open(sca_file, 'r') as f:
                    for line in f:
                        if not line.startswith('scalar '):
                            continue
                        parts = line.strip().split()
                        # Expected format: scalar <modulePath> <scalarName> <value>
                        if len(parts) < 4:
                            continue
                        module_path = parts[1]
                        scalar_name = parts[2]
                        value_str = parts[3]
                        if scalar_name not in ('CordiX', 'CordiY', 'positionX', 'positionY'):
                            continue

                        # Determine node id (end nodes indexed with +1000 offset)
                        node_id = None
                        if 'loRaEndNodes[' in module_path:
                            # Extract index between brackets
                            start = module_path.find('loRaEndNodes[') + len('loRaEndNodes[')
                            end = module_path.find(']', start)
                            if end != -1:
                                try:
                                    idx = int(module_path[start:end])
                                    node_id = 1000 + idx
                                except ValueError:
                                    pass
                        else:
                            # Not an end node
                            continue

                        if node_id not in (1000, 1001):
                            continue

                        try:
                            coord_val = float(value_str)
                        except ValueError:
                            continue

                        if node_id not in coordinates:
                            coordinates[node_id] = [None, None]
                        if scalar_name in ('CordiX', 'positionX'):
                            coordinates[node_id][0] = coord_val
                        elif scalar_name in ('CordiY', 'positionY'):
                            coordinates[node_id][1] = coord_val
            except Exception as inner_e:
                print(f"Warning: Failed reading {sca_file}: {inner_e}")

    except Exception as e:
        print(f"Warning: Could not scan results directory for .sca files: {e}")

    # Notify if incomplete
    for nid, (x, y) in list(coordinates.items()):
        if x is None or y is None:
            print(f"Info: Partial coordinates for node {nid} (x={x}, y={y})")

    if not coordinates:
        print("Note: End node coordinates not found in scalars (random circle deployment)")
        print("      Coordinates will be shown as estimated based on configuration")

    return coordinates
def calculate_distance(coord1, coord2):
    """Calculate Euclidean distance between two coordinates"""
    if coord1[0] is None or coord1[1] is None or coord2[0] is None or coord2[1] is None:
        return None
    
    dx = coord1[0] - coord2[0]
    dy = coord1[1] - coord2[1]
    return math.sqrt(dx * dx + dy * dy)

def analyze_simulation_results(paths_csv_file="delivered_packets/paths.csv", output_file=None):
    """
    Analyze simulation results and generate comprehensive report
    
    Args:
        paths_csv_file: Path to the paths.csv file
        output_file: Output filename (auto-generated if None)
    """
    
    # Check if paths.csv exists
    if not os.path.exists(paths_csv_file):
        print(f"ERROR: {paths_csv_file} not found!")
        return
    
    # Load data
    try:
        df = pd.read_csv(paths_csv_file)
        print(f"Loaded {len(df)} events from {paths_csv_file}")
    except Exception as e:
        print(f"ERROR loading CSV: {e}")
        return
    
    # Generate output filename if not provided
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"simulation_analysis_report_{timestamp}.txt"
    
    # Start analysis
    print("Analyzing simulation results...")
    
    # Extract end node coordinates
    simulations_dir = os.path.dirname(os.path.dirname(paths_csv_file)) if "delivered_packets" in paths_csv_file else "./"
    coordinates = get_end_node_coordinates(simulations_dir)
    
    # Calculate distance between end nodes
    end_node_distance = None
    if 1000 in coordinates and 1001 in coordinates:
        coord_1000 = coordinates[1000]
        coord_1001 = coordinates[1001]
        end_node_distance = calculate_distance(coord_1000, coord_1001)
    
    # Filter for end node 1000 packets (source node after ID offset)
    node1000_packets = df[df['src'] == 1000]
    
    # Get transmission and delivery events for end node 1000 → 1001 communication
    tx_events = node1000_packets[node1000_packets['event'] == 'TX_SRC']
    delivery_events = df[(df['event'] == 'DELIVERED') & (df['src'] == 1000) & (df['dst'] == 1001)]
    
    # Basic statistics
    total_tx = len(tx_events)
    total_delivered = len(delivery_events)
    # Initialize variables used later to avoid UnboundLocalError when sections are skipped
    transit_times = []
    transit_details = []
    time_intervals = None
    
    # Prepare report content
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("LORA MESH NETWORK SIMULATION ANALYSIS REPORT")
    report_lines.append("=" * 80)
    report_lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"Data Source: {paths_csv_file}")
    report_lines.append(f"Total Events Analyzed: {len(df)}")
    
    # Add end node distance information
    if end_node_distance is not None:
        report_lines.append(f"Distance between End Nodes 1000 and 1001: {end_node_distance:.2f} meters")
        if 1000 in coordinates and 1001 in coordinates:
            coord_1000 = coordinates[1000]
            coord_1001 = coordinates[1001]
            report_lines.append(f"End Node 1000 position: ({coord_1000[0]:.2f}, {coord_1000[1]:.2f})")
            report_lines.append(f"End Node 1001 position: ({coord_1001[0]:.2f}, {coord_1001[1]:.2f})")
    else:
        report_lines.append("Distance between End Nodes: Not available")
        report_lines.append("Note: End nodes use random circle deployment - coordinates not recorded in scalars")
        report_lines.append("      Distance varies between simulation runs due to random placement")
    
    report_lines.append("")
    
    # 1. PACKET TRANSMISSION STATISTICS
    report_lines.append("1. PACKET TRANSMISSION STATISTICS FROM END NODE 1000")
    report_lines.append("-" * 50)
    report_lines.append(f"Total data packets transmitted: {total_tx}")
    
    if total_tx > 0:
        # Packets per destination - expecting destination 1001 for end-to-end communication
        dest_counts = tx_events['dst'].value_counts().sort_index()
        unique_destinations = len(dest_counts)
        
        report_lines.append(f"Number of unique destinations: {unique_destinations}")
        report_lines.append(f"Expected destination for end-to-end test: 1001")
        
        if 1001 in dest_counts.index:
            report_lines.append("SUCCESS: Transmitted to expected destination (1001)")
        else:
            report_lines.append(f"WARNING: Expected destination 1001 not found in transmission list")
        
        report_lines.append("")
        report_lines.append("Packets sent per destination:")
        
        # Group destinations for readable output
        for i in range(0, len(dest_counts), 10):
            chunk = dest_counts.iloc[i:i+10]
            line_parts = []
            for dest, count in chunk.items():
                line_parts.append(f"Node {dest:2d}:{count}")
            report_lines.append("  " + "  ".join(line_parts))
    
    report_lines.append("")
    
    # 2. DELIVERY SUCCESS ANALYSIS
    report_lines.append("2. DELIVERY SUCCESS RATE ANALYSIS")
    report_lines.append("-" * 50)
    report_lines.append(f"Total packets delivered: {total_delivered}")
    
    if total_tx > 0:
        success_rate = (total_delivered / total_tx) * 100
        report_lines.append(f"Overall delivery success rate: {success_rate:.2f}% ({total_delivered}/{total_tx})")
    else:
        report_lines.append("No packets transmitted - cannot calculate success rate")
    
    if total_delivered > 0:
        delivered_destinations = set(delivery_events['dst'].unique())
        reachable_count = len(delivered_destinations)
        reachable_nodes = sorted(list(delivered_destinations))
        
        report_lines.append(f"Destinations that received packets: {reachable_count}")
        if 1001 in delivered_destinations:
            report_lines.append("SUCCESS: End node 1001 successfully received packets")
        else:
            report_lines.append("WARNING: End node 1001 did not receive packets")
        report_lines.append("")
        
        report_lines.append("REACHABLE NODES:")
        # Print reachable nodes in rows of 10
        for i in range(0, len(reachable_nodes), 10):
            chunk = reachable_nodes[i:i+10]
            report_lines.append(f"  {chunk}")
        
        # Unreachable nodes
        if total_tx > 0:
            transmitted_destinations = set(tx_events['dst'].unique())
            unreachable_nodes = sorted(list(transmitted_destinations - delivered_destinations))
            
            if unreachable_nodes:
                report_lines.append("")
                report_lines.append("UNREACHABLE NODES:")
                for i in range(0, len(unreachable_nodes), 10):
                    chunk = unreachable_nodes[i:i+10]
                    report_lines.append(f"  {chunk}")
    
    report_lines.append("")
    
    # 3. TRANSMISSION TIMING ANALYSIS
    report_lines.append("3. TRANSMISSION TIMING ANALYSIS")
    report_lines.append("-" * 50)
    
    if len(tx_events) > 1:
        tx_sorted = tx_events.sort_values('simTime')
        time_intervals = tx_sorted['simTime'].diff().dropna()
        
        first_tx = tx_sorted.iloc[0]['simTime']
        last_tx = tx_sorted.iloc[-1]['simTime']
        total_duration = last_tx - first_tx
        
        report_lines.append(f"First transmission time: {first_tx:.3f} seconds")
        report_lines.append(f"Last transmission time: {last_tx:.3f} seconds")
        report_lines.append(f"Total transmission period: {total_duration:.3f} seconds")
        report_lines.append("")
        
        report_lines.append("Time intervals between consecutive transmissions:")
        report_lines.append(f"  Average interval: {time_intervals.mean():.3f} seconds")
        report_lines.append(f"  Minimum interval: {time_intervals.min():.3f} seconds")
        report_lines.append(f"  Maximum interval: {time_intervals.max():.3f} seconds")
        report_lines.append(f"  Standard deviation: {time_intervals.std():.3f} seconds")
    else:
        report_lines.append("Insufficient transmission data for timing analysis")
    
    report_lines.append("")
    
    # 4. END-TO-END TRANSIT TIME ANALYSIS
    report_lines.append("4. END-TO-END TRANSIT TIME ANALYSIS")
    report_lines.append("-" * 50)
    
    if total_delivered > 0 and total_tx > 0:
        
        # Calculate transit time for each delivered packet
        for _, delivery in delivery_events.iterrows():
            packet_seq = delivery['packetSeq']
            dst_node = delivery['dst']
            delivery_time = delivery['simTime']
            
            # Find corresponding transmission
            tx_match = tx_events[tx_events['packetSeq'] == packet_seq]
            
            if len(tx_match) > 0:
                tx_time = tx_match.iloc[0]['simTime']
                transit_time = delivery_time - tx_time
                transit_times.append(transit_time)
                transit_details.append({
                    'packet_seq': packet_seq,
                    'destination': dst_node,
                    'tx_time': tx_time,
                    'delivery_time': delivery_time,
                    'transit_time': transit_time
                })
        
        if transit_times:
            report_lines.append(f"Successfully matched {len(transit_times)} packet journeys")
            report_lines.append("")
            report_lines.append("Transit time statistics:")
            report_lines.append(f"  Average transit time: {sum(transit_times)/len(transit_times):.3f} seconds")
            report_lines.append(f"  Minimum transit time: {min(transit_times):.3f} seconds")
            report_lines.append(f"  Maximum transit time: {max(transit_times):.3f} seconds")
            
            # Find fastest and slowest deliveries
            transit_details.sort(key=lambda x: x['transit_time'])
            
            fastest = transit_details[0]
            slowest = transit_details[-1]
            
            report_lines.append("")
            report_lines.append(f"FASTEST DELIVERY:")
            report_lines.append(f"  Packet {fastest['packet_seq']} to Node {fastest['destination']}: {fastest['transit_time']:.3f}s")
            
            report_lines.append(f"SLOWEST DELIVERY:")
            report_lines.append(f"  Packet {slowest['packet_seq']} to Node {slowest['destination']}: {slowest['transit_time']:.3f}s")
            
            # Show top 5 fastest and slowest
            report_lines.append("")
            report_lines.append("Top 5 fastest deliveries:")
            report_lines.append("  Seq  Dest  Transit Time")
            for detail in transit_details[:5]:
                report_lines.append(f"  {detail['packet_seq']:3d}  {detail['destination']:4d}  {detail['transit_time']:8.3f}s")
            
            report_lines.append("")
            report_lines.append("Top 5 slowest deliveries:")
            report_lines.append("  Seq  Dest  Transit Time")
            for detail in transit_details[-5:]:
                report_lines.append(f"  {detail['packet_seq']:3d}  {detail['destination']:4d}  {detail['transit_time']:8.3f}s")
        else:
            report_lines.append("Could not match transmission and delivery events for transit time calculation")
    else:
        report_lines.append("No delivered packets - cannot calculate transit times")
    
    report_lines.append("")
    
    # 5. SUMMARY AND RECOMMENDATIONS
    report_lines.append("5. SUMMARY AND RECOMMENDATIONS")
    report_lines.append("-" * 50)
    
    if total_tx > 0:
        if success_rate >= 80:
            report_lines.append("EXCELLENT: High delivery success rate")
        elif success_rate >= 50:
            report_lines.append("GOOD: Moderate delivery success rate")
        elif success_rate >= 20:
            report_lines.append("FAIR: Low delivery success rate - investigate network issues")
        else:
            report_lines.append("POOR: Very low delivery success rate - major network problems")
        
        if len(tx_events) > 1 and time_intervals is not None:
            avg_interval = time_intervals.mean()
            if 8 <= avg_interval <= 12:
                report_lines.append("TIMING: Transmission intervals within expected range (8-10s)")
            elif avg_interval < 2:
                report_lines.append("TIMING WARNING: Very fast transmission intervals - may cause congestion")
            else:
                report_lines.append(f"TIMING INFO: Transmission intervals: {avg_interval:.1f}s average")
        
        if transit_times and len(transit_times) > 0:
            max_transit = max(transit_times)
            if max_transit < 10:
                report_lines.append("LATENCY: Fast network response times")
            elif max_transit < 60:
                report_lines.append("LATENCY: Reasonable network response times")
            else:
                report_lines.append("LATENCY WARNING: High network latency detected")
    
    report_lines.append("")
    report_lines.append("=" * 80)
    report_lines.append("END OF REPORT")
    report_lines.append("=" * 80)
    
    # Write report to file
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('\n'.join(report_lines))
        
        print(f"✓ Analysis complete! Report saved to: {output_file}")
        print(f"Report contains {len(report_lines)} lines")
        
        # Print key summary to console
        print("\nKEY FINDINGS:")
        if end_node_distance is not None:
            print(f"• Distance between end nodes 1000 and 1001: {end_node_distance:.2f}m")
        if total_tx > 0:
            print(f"• Packets transmitted from end node 1000: {total_tx}")
            print(f"• Delivery success rate: {(total_delivered/total_tx)*100:.1f}%")
            if total_delivered > 0:
                print(f"• End node 1001 successfully received {total_delivered} packet(s)")
            else:
                print(f"• End node 1001 did NOT receive any packets")
            if transit_times:
                print(f"• Max transit time: {max(transit_times):.3f}s")
        
        return output_file
        
    except Exception as e:
        print(f"ERROR writing report: {e}")
        return None

def main():
    """Main function for command line usage"""
    
    # Default paths
    paths_file = "delivered_packets/paths.csv"
    output_file = None
    
    # Parse command line arguments if provided
    if len(sys.argv) > 1:
        paths_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    # Run analysis
    result = analyze_simulation_results(paths_file, output_file)
    
    if result:
        print(f"\n✓ Success! Open '{result}' to view the complete analysis report.")
    else:
        print("\n❌ Analysis failed. Check error messages above.")

if __name__ == "__main__":
    main()