#!/usr/bin/env python3
"""
Comprehensive Simulation Analysis Script
Generates detailed report for LoRa mesh network simulation results
"""

import pandas as pd
import os
from datetime import datetime
import sys

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
    
    # Filter for node 0 packets
    node0_packets = df[df['src'] == 0]
    
    # Get transmission and delivery events
    tx_events = node0_packets[node0_packets['event'] == 'TX_SRC']
    delivery_events = node0_packets[node0_packets['event'] == 'DELIVERED']
    
    # Basic statistics
    total_tx = len(tx_events)
    total_delivered = len(delivery_events)
    
    # Prepare report content
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("LORA MESH NETWORK SIMULATION ANALYSIS REPORT")
    report_lines.append("=" * 80)
    report_lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"Data Source: {paths_csv_file}")
    report_lines.append(f"Total Events Analyzed: {len(df)}")
    report_lines.append("")
    
    # 1. PACKET TRANSMISSION STATISTICS
    report_lines.append("1. PACKET TRANSMISSION STATISTICS FROM NODE 0")
    report_lines.append("-" * 50)
    report_lines.append(f"Total data packets transmitted: {total_tx}")
    
    if total_tx > 0:
        # Packets per destination
        dest_counts = tx_events['dst'].value_counts().sort_index()
        unique_destinations = len(dest_counts)
        
        report_lines.append(f"Number of unique destinations: {unique_destinations}")
        report_lines.append(f"Expected destinations (1-49): 49 nodes")
        
        if unique_destinations == 49:
            report_lines.append("SUCCESS: Transmitted to all expected destinations")
        else:
            missing_dests = set(range(1, 50)) - set(dest_counts.index)
            report_lines.append(f"WARNING: Missing destinations: {sorted(list(missing_dests))}")
        
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
        
        report_lines.append(f"Number of nodes that received packets: {reachable_count}/49")
        report_lines.append(f"Node reachability: {(reachable_count/49)*100:.1f}%")
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
        transit_times = []
        transit_details = []
        
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
        
        if len(tx_events) > 1:
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
        if total_tx > 0:
            print(f"• Packets transmitted: {total_tx}")
            print(f"• Delivery success rate: {(total_delivered/total_tx)*100:.1f}%")
            if total_delivered > 0:
                reachable_count = len(set(delivery_events['dst'].unique()))
                print(f"• Reachable nodes: {reachable_count}/49 ({(reachable_count/49)*100:.1f}%)")
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