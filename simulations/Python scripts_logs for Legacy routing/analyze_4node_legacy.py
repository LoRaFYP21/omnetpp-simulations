#!/usr/bin/env python3
"""
Analyze 4-node legacy routing simulation results
Generates comprehensive analysis report with energy and PDR metrics
"""

import csv
import re
import glob
from collections import defaultdict
from datetime import datetime
import os


def parse_sca_energy(sca_file):
    """Extract total energy consumption from .sca file"""
    total_energy = 0.0
    node_energies = {}
    
    try:
        with open(sca_file, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                # Match: scalar LoRaMesh.loRaNodes[X].LoRaNic.radio.energyConsumer totalEnergyConsumed VALUE
                # or:    scalar LoRaMesh.loRaEndNodes[X].LoRaNic.radio.energyConsumer totalEnergyConsumed VALUE
                match = re.search(r'scalar\s+LoRaMesh\.(loRa(?:End)?Nodes)\[(\d+)\]\.LoRaNic\.radio\.energyConsumer\s+totalEnergyConsumed\s+([\d.]+)', line)
                if match:
                    node_type = match.group(1)
                    node_id = int(match.group(2))
                    energy = float(match.group(3))
                    
                    # Store node energy
                    if node_type == 'loRaEndNodes':
                        node_key = f"End{1000 + node_id}"
                    else:
                        node_key = f"Relay{node_id}"
                    
                    node_energies[node_key] = energy
                    total_energy += energy
                    
    except FileNotFoundError:
        print(f"Warning: .sca file not found: {sca_file}")
        return 0.0, {}
    
    return total_energy, node_energies


def analyze_paths_csv(paths_file):
    """Analyze paths.csv for transmission and delivery statistics"""
    
    # Track sent packets per source
    sent_packets = defaultdict(set)  # src -> set of (seq, dst)
    
    # Track delivered packets
    delivered_packets = defaultdict(lambda: defaultdict(set))  # src -> dst -> set of seq
    
    # Track all transmission events (for overhead calculation)
    total_tx_events = 0
    tx_src_events = 0
    tx_fwd_events = 0
    
    # Track transmissions per packet
    packet_transmissions = defaultdict(int)  # (src, dst, seq) -> count
    
    try:
        with open(paths_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                event = row['event']
                src = int(row['src'])
                dst = int(row['dst'])
                seq = int(row['packetSeq'])
                
                # Count TX_SRC events (original transmissions)
                if event == 'TX_SRC':
                    sent_packets[src].add((seq, dst))
                    tx_src_events += 1
                    packet_transmissions[(src, dst, seq)] += 1
                
                # Count forwarding transmissions
                elif event in ['TX_FWD_DATA', 'TX_FWD_ACK']:
                    tx_fwd_events += 1
                    packet_transmissions[(src, dst, seq)] += 1
                
                # Count all TX events
                if 'TX_' in event:
                    total_tx_events += 1
                
                # Track delivered packets
                elif event == 'DELIVERED':
                    delivered_packets[src][dst].add(seq)
    
    except FileNotFoundError:
        print(f"Error: paths.csv not found: {paths_file}")
        return None
    
    return {
        'sent_packets': sent_packets,
        'delivered_packets': delivered_packets,
        'total_tx_events': total_tx_events,
        'tx_src_events': tx_src_events,
        'tx_fwd_events': tx_fwd_events,
        'packet_transmissions': packet_transmissions
    }


def calculate_metrics(analysis_data):
    """Calculate PDR and transmission efficiency metrics"""
    
    sent = analysis_data['sent_packets']
    delivered = analysis_data['delivered_packets']
    packet_tx = analysis_data['packet_transmissions']
    
    # Per-source statistics
    source_stats = {}
    total_sent = 0
    total_delivered = 0
    
    for src in sorted(sent.keys()):
        src_sent = len(sent[src])
        src_delivered = 0
        
        # Count delivered packets from this source (to any destination)
        for dst in delivered[src]:
            src_delivered += len(delivered[src][dst])
        
        pdr = (src_delivered / src_sent * 100) if src_sent > 0 else 0.0
        
        source_stats[src] = {
            'sent': src_sent,
            'delivered': src_delivered,
            'pdr': pdr
        }
        
        total_sent += src_sent
        total_delivered += src_delivered
    
    # Calculate average PDR
    avg_pdr = (total_delivered / total_sent * 100) if total_sent > 0 else 0.0
    
    # Calculate average transmissions per delivered packet
    delivered_packet_keys = set()
    for src in delivered:
        for dst in delivered[src]:
            for seq in delivered[src][dst]:
                delivered_packet_keys.add((src, dst, seq))
    
    total_transmissions_for_delivered = 0
    for pkt_key in delivered_packet_keys:
        total_transmissions_for_delivered += packet_tx.get(pkt_key, 0)
    
    avg_tx_per_delivered = (total_transmissions_for_delivered / len(delivered_packet_keys)) if delivered_packet_keys else 0.0
    
    return {
        'source_stats': source_stats,
        'total_sent': total_sent,
        'total_delivered': total_delivered,
        'avg_pdr': avg_pdr,
        'avg_tx_per_delivered': avg_tx_per_delivered,
        'num_delivered_packets': len(delivered_packet_keys)
    }


def detect_routing_type(paths_file):
    """Detect if simulation used routing or flooding based on paths.csv
    
    If any UNICAST transmissions exist → Routing
    If only BCAST transmissions exist → Flooding
    """
    unicast_found = False
    
    try:
        with open(paths_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['nextHopType'] == 'UNICAST':
                    # Found at least one unicast transmission → this is routing
                    unicast_found = True
                    break
    except:
        pass
    
    if unicast_found:
        return "Routing"
    else:
        return "Flooding"


def generate_report(paths_file, sca_file, output_dir=".", routing_type=None):
    """Generate comprehensive analysis report"""
    
    print("\n" + "="*70)
    print(" 4-NODE LEGACY SIMULATION ANALYSIS")
    print("="*70)
    
    # Get timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Detect routing type if not provided
    if routing_type is None:
        routing_type = detect_routing_type(paths_file)
    
    # Parse energy data
    print("\n[1/4] Parsing energy consumption data...")
    total_energy, node_energies = parse_sca_energy(sca_file)
    
    # Parse paths.csv
    print("[2/4] Analyzing paths.csv...")
    analysis_data = analyze_paths_csv(paths_file)
    
    if analysis_data is None:
        print("Error: Could not analyze paths.csv")
        return None
    
    # Calculate metrics
    print("[3/4] Calculating metrics...")
    metrics = calculate_metrics(analysis_data)
    
    # Generate report file
    print("[4/4] Generating report...")
    report_filename = f"LEGACY_{routing_type}_{timestamp}.txt"
    report_path = os.path.join(output_dir, report_filename)
    
    with open(report_path, 'w', encoding='utf-8') as f:
        # Header
        f.write("="*70 + "\n")
        f.write("  4-NODE LEGACY SIMULATION ANALYSIS REPORT\n")
        f.write("="*70 + "\n\n")
        
        f.write(f"Report Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Routing Type: {routing_type}\n")
        f.write(f"Paths File: {os.path.basename(paths_file)}\n")
        f.write(f"Energy File: {os.path.basename(sca_file)}\n")
        f.write("\n")
        
        # Energy Consumption
        f.write("="*70 + "\n")
        f.write("  ENERGY CONSUMPTION\n")
        f.write("="*70 + "\n\n")
        
        f.write(f"Total Network Energy Consumed: {total_energy:.6f} J\n\n")
        
        f.write("Per-Node Energy Consumption:\n")
        f.write("-" * 50 + "\n")
        
        # Sort nodes: End nodes first, then relays
        end_nodes = sorted([k for k in node_energies.keys() if k.startswith('End')])
        relay_nodes = sorted([k for k in node_energies.keys() if k.startswith('Relay')],
                            key=lambda x: int(x.replace('Relay', '')))
        
        for node in end_nodes + relay_nodes:
            energy = node_energies[node]
            f.write(f"  {node:15s}: {energy:>10.6f} J\n")
        
        f.write("\n")
        
        # Packet Delivery Statistics
        f.write("="*70 + "\n")
        f.write("  PACKET DELIVERY STATISTICS\n")
        f.write("="*70 + "\n\n")
        
        f.write("Per-Source Statistics:\n")
        f.write("-" * 50 + "\n")
        
        for src in sorted(metrics['source_stats'].keys()):
            stats = metrics['source_stats'][src]
            f.write(f"  End Node {src}:\n")
            f.write(f"    Packets Sent:      {stats['sent']:>6d}\n")
            f.write(f"    Packets Delivered: {stats['delivered']:>6d}\n")
            f.write(f"    PDR:               {stats['pdr']:>6.2f}%\n")
            f.write("\n")
        
        # Overall Statistics
        f.write("Overall Network Statistics:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Total Packets Sent:        {metrics['total_sent']:>6d}\n")
        f.write(f"  Total Packets Delivered:   {metrics['total_delivered']:>6d}\n")
        f.write(f"  Average PDR:               {metrics['avg_pdr']:>6.2f}%\n")
        f.write("\n")
        
        # Transmission Efficiency
        f.write("="*70 + "\n")
        f.write("  TRANSMISSION EFFICIENCY\n")
        f.write("="*70 + "\n\n")
        
        f.write(f"Total TX_SRC Events:           {analysis_data['tx_src_events']:>8d}\n")
        f.write(f"Total TX_FWD Events:           {analysis_data['tx_fwd_events']:>8d}\n")
        f.write(f"Total TX Events:               {analysis_data['total_tx_events']:>8d}\n")
        f.write(f"Successfully Delivered:        {metrics['num_delivered_packets']:>8d}\n")
        f.write(f"\nAverage Transmissions per\n")
        f.write(f"  Delivered Packet:            {metrics['avg_tx_per_delivered']:>8.2f}x\n")
        f.write("\n")
        
        # Network Overhead
        overhead_ratio = (analysis_data['tx_fwd_events'] / analysis_data['tx_src_events']) if analysis_data['tx_src_events'] > 0 else 0
        f.write(f"Forwarding Overhead Ratio:     {overhead_ratio:>8.2f}x\n")
        f.write(f"  (FWD transmissions per original transmission)\n")
        f.write("\n")
        
        # Summary
        f.write("="*70 + "\n")
        f.write("  SUMMARY\n")
        f.write("="*70 + "\n\n")
        
        f.write(f"✓ Total Energy: {total_energy:.3f} J\n")
        f.write(f"✓ Average PDR: {metrics['avg_pdr']:.2f}%\n")
        f.write(f"✓ Avg TX per Delivered Packet: {metrics['avg_tx_per_delivered']:.2f}x\n")
        f.write(f"✓ Network Overhead: {overhead_ratio:.2f}x\n")
        f.write("\n")
        f.write("="*70 + "\n")
    
    print(f"\n✅ Report generated: {report_path}")
    
    # Also print summary to console
    print("\n" + "="*70)
    print("  SUMMARY")
    print("="*70)
    print(f"Total Energy Consumed:         {total_energy:.3f} J")
    print(f"Average PDR:                   {metrics['avg_pdr']:.2f}%")
    print(f"Avg TX per Delivered Packet:   {metrics['avg_tx_per_delivered']:.2f}x")
    print(f"Network Overhead Ratio:        {overhead_ratio:.2f}x")
    print("="*70 + "\n")
    
    return report_path


def main():
    """Main entry point"""
    import sys
    
    # Default paths (relative to script location in Python scripts_logs for Legacy routing/)
    default_paths = "../delivered_packets/paths.csv"
    default_sca = None
    
    # Find .sca file automatically in results directory
    results_dir = "../results"
    if os.path.exists(results_dir):
        sca_files = glob.glob(os.path.join(results_dir, "*Four_EndNodes*.sca"))
        if sca_files:
            # Use most recent .sca file
            default_sca = max(sca_files, key=os.path.getmtime)
    
    # Check command line arguments
    if len(sys.argv) > 1:
        paths_file = sys.argv[1]
    else:
        paths_file = default_paths
    
    if len(sys.argv) > 2:
        sca_file = sys.argv[2]
    else:
        if default_sca:
            sca_file = default_sca
        else:
            print("Error: No .sca file found. Please specify as second argument.")
            print(f"Usage: {sys.argv[0]} [paths.csv] [file.sca]")
            return 1
    
    # Check files exist
    if not os.path.exists(paths_file):
        print(f"Error: paths.csv not found: {paths_file}")
        return 1
    
    if not os.path.exists(sca_file):
        print(f"Error: .sca file not found: {sca_file}")
        return 1
    
    print(f"\nInput files:")
    print(f"  Paths: {paths_file}")
    print(f"  Energy: {sca_file}")
    
    # Generate report
    report_path = generate_report(paths_file, sca_file)
    
    if report_path:
        print(f"\n✅ Analysis complete!")
        return 0
    else:
        print(f"\n❌ Analysis failed!")
        return 1


if __name__ == '__main__':
    exit(main())
