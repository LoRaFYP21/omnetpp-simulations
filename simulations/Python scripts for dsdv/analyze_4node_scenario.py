#!/usr/bin/env python
"""
Analyzes paths.csv for 4 end nodes sending data packets to a rescue node.
Calculates per-node transmission counts, delivery rates, PDR, and network overhead.

Expected scenario:
- 4 end nodes (IDs: 1000, 1001, 1002, 1003) each send 20 data packets
- 1 rescue node (ID: 2000) receives the packets
- Total expected: 80 packets across all end nodes
"""

import csv
import os
import re
import glob
from datetime import datetime
from collections import defaultdict

# Configuration
PATHS_CSV = os.path.join("delivered_packets", "paths.csv")
OUTPUT_DIR = "analysis reports"
END_NODE_IDS = [1000, 1001, 1002, 1003]
RESCUE_NODE_ID = 2000
EXPECTED_PACKETS_PER_NODE = 20

def parse_mobile_speed_from_sca(sca_path):
    """Extract mobile speed from turtle script parameter in .sca file."""
    try:
        with open(sca_path, 'r') as f:
            for line in f:
                # Look for turtle script parameter: param **.loRaRescueNodes[*].mobility.turtleScript "xmldoc(\"turtle_patrol_7mps_2km.xml\")"
                if 'turtleScript' in line and 'turtle_patrol_' in line:
                    match = re.search(r'turtle_patrol_(\d+)mps', line)
                    if match:
                        return match.group(1) + "mps"
        return "unknown"
    except:
        return "unknown"

def parse_routing_protocol_from_sca(sca_path):
    """Extract routing protocol from .sca file parameter."""
    try:
        with open(sca_path, 'r') as f:
            for line in f:
                # Look for: param **.routingProtocol "\"dsdv\"" or "\"legacy\""
                if 'param **.routingProtocol' in line or 'param **.loRaNodes[*].LoRaNodeApp.routingProtocol' in line:
                    if 'dsdv' in line.lower():
                        return "DSDV"
                    elif 'legacy' in line.lower():
                        return "FLOODING"
        return "unknown"
    except:
        return "unknown"

def parse_total_energy_from_sca(sca_path):
    """Extract and sum total energy consumed by all nodes from .sca file."""
    try:
        total_energy = 0.0
        with open(sca_path, 'r') as f:
            for line in f:
                # Look for: scalar LoRaMesh.loRaNodes[*].LoRaNic.radio.energyConsumer totalEnergyConsumed <value>
                # Also check for endNodes and rescueNodes
                if 'totalEnergyConsumed' in line and 'scalar' in line:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        try:
                            # Last element should be the energy value
                            energy_value = float(parts[-1])
                            total_energy += energy_value
                        except ValueError:
                            continue
        return total_energy
    except:
        return 0.0

def analyze_4node_paths():
    """Main analysis function for 4-node multi-source scenario."""
    
    if not os.path.exists(PATHS_CSV):
        print("ERROR: {} not found!".format(PATHS_CSV))
        return
    
    # Data structures for tracking
    # Per end node: TX_SRC events
    tx_src_count = {node_id: 0 for node_id in END_NODE_IDS}
    tx_src_packets = {node_id: set() for node_id in END_NODE_IDS}  # Track unique packet IDs
    
    # Per end node: DELIVERED events at rescue node
    delivered_packets = {node_id: set() for node_id in END_NODE_IDS}  # Unique packet IDs delivered
    
    # Track all transmission events per packet (for overhead calculation)
    # Key: (srcNodeId, pktId), Value: list of transmission events
    packet_transmissions = defaultdict(list)
    
    # Parse CSV
    print("Reading {}...".format(PATHS_CSV))
    with open(PATHS_CSV, 'r') as csv_file:
        reader = csv.DictReader(csv_file)
        
        for row in reader:
            event_type = row['event']
            src_node = int(row['src'])
            pkt_id = int(row['packetSeq'])
            current_node = int(row['currentNode'])
            sim_time = float(row['simTime'])
            
            # Track TX_SRC events (original transmissions from end nodes)
            if event_type == 'TX_SRC' and src_node in END_NODE_IDS:
                tx_src_count[src_node] += 1
                tx_src_packets[src_node].add(pkt_id)
            
            # Track DELIVERED events at rescue node
            if event_type == 'DELIVERED' and current_node == RESCUE_NODE_ID and src_node in END_NODE_IDS:
                delivered_packets[src_node].add(pkt_id)
            
            # Track ALL transmission events for overhead calculation
            # Count TX_SRC, TX_FWD_DATA (unicast forwards), TX_FWD_BCAST (broadcast forwards)
            if event_type in ['TX_SRC', 'TX_FWD_DATA', 'TX_FWD_BCAST'] and src_node in END_NODE_IDS:
                packet_transmissions[(src_node, pkt_id)].append({
                    'eventType': event_type,
                    'currentNode': current_node,
                    'simTime': sim_time
                })
    
    # Calculate metrics per end node
    print("\n" + "="*80)
    print("ANALYSIS RESULTS: 4 End Nodes to Rescue Node Communication")
    print("="*80)
    
    results = []
    total_tx_src = 0
    total_delivered = 0
    total_transmissions_all = 0
    total_packets_delivered_at_least_once = 0
    
    for node_id in END_NODE_IDS:
        tx_count = tx_src_count[node_id]
        unique_tx_packets = len(tx_src_packets[node_id])
        delivered_count = len(delivered_packets[node_id])
        
        # Calculate PDR for this node
        pdr = (float(delivered_count) / EXPECTED_PACKETS_PER_NODE * 100) if EXPECTED_PACKETS_PER_NODE > 0 else 0
        
        # Calculate total transmissions for delivered packets from this node
        transmissions_for_delivered = 0
        for pkt_id in delivered_packets[node_id]:
            tx_events = packet_transmissions[(node_id, pkt_id)]
            transmissions_for_delivered += len(tx_events)
        
        # Average transmissions per delivered packet for this node
        avg_tx_per_delivered = (float(transmissions_for_delivered) / delivered_count) if delivered_count > 0 else 0
        
        results.append({
            'node_id': node_id,
            'tx_src_count': tx_count,
            'unique_tx_packets': unique_tx_packets,
            'delivered_count': delivered_count,
            'pdr': pdr,
            'transmissions_for_delivered': transmissions_for_delivered,
            'avg_tx_per_delivered': avg_tx_per_delivered
        })
        
        total_tx_src += tx_count
        total_delivered += delivered_count
        total_transmissions_all += transmissions_for_delivered
        if delivered_count > 0:
            total_packets_delivered_at_least_once += delivered_count
    
    # Calculate overall metrics
    overall_pdr = (float(total_delivered) / (len(END_NODE_IDS) * EXPECTED_PACKETS_PER_NODE) * 100)
    overall_avg_tx_per_delivered = (float(total_transmissions_all) / total_delivered) if total_delivered > 0 else 0
    avg_delivered_per_node = float(total_delivered) / len(END_NODE_IDS)
    
    # Find and parse .sca file for routing protocol and mobile speed
    sca_files = glob.glob(os.path.join("results", "*.sca"))
    if sca_files:
        # Sort by modification time, get most recent
        sca_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
        most_recent_sca = sca_files[0]
        
        routing_mode = parse_routing_protocol_from_sca(most_recent_sca)
        mobile_speed = parse_mobile_speed_from_sca(most_recent_sca)
        total_energy = parse_total_energy_from_sca(most_recent_sca)
        
        print("\nDetected routing mode: {} (from {})".format(routing_mode, os.path.basename(most_recent_sca)))
        print("Detected mobile speed: {} (from {})".format(mobile_speed, os.path.basename(most_recent_sca)))
        print("Total network energy consumed: {:.2f} J (from {})".format(total_energy, os.path.basename(most_recent_sca)))
    else:
        routing_mode = "unknown"
        mobile_speed = "unknown"
        total_energy = 0.0
        print("Warning: No .sca files found, routing mode and mobile speed unknown")
    
    # Generate report with new naming format: <routing>_<speed>_<timestamp>_4node_analysis.txt
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_filename = "{}_{}_{}_{}.txt".format(routing_mode, mobile_speed, timestamp, "4node_analysis")
    report_path = os.path.join(OUTPUT_DIR, report_filename)
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    with open(report_path, 'w') as f:
        # Write header
        f.write("="*80 + "\n")
        f.write("4 END NODES TO RESCUE NODE - COMMUNICATION ANALYSIS\n")
        f.write("="*80 + "\n")
        f.write("Analysis Timestamp: {}\n".format(datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        f.write("Source File: {}\n".format(PATHS_CSV))
        f.write("Routing Mode: {}\n".format(routing_mode))
        f.write("Mobile Node Speed: {}\n".format(mobile_speed))
        f.write("Total Network Energy Consumed: {:.2f} J\n".format(total_energy))
        f.write("Expected: {} end nodes x {} packets = {} total packets\n".format(
            len(END_NODE_IDS), EXPECTED_PACKETS_PER_NODE, len(END_NODE_IDS) * EXPECTED_PACKETS_PER_NODE))
        f.write("Rescue Node ID: {}\n".format(RESCUE_NODE_ID))
        f.write("\n")
        
        # Per-node results
        f.write("-"*80 + "\n")
        f.write("PER END NODE METRICS\n")
        f.write("-"*80 + "\n\n")
        
        for result in results:
            f.write("End Node {}:\n".format(result['node_id']))
            f.write("  Data Packets Transmitted (TX_SRC):        {}\n".format(result['tx_src_count']))
            f.write("  Unique Packet IDs Transmitted:            {}\n".format(result['unique_tx_packets']))
            f.write("  Packets Delivered to Rescue Node:         {} / {}\n".format(
                result['delivered_count'], EXPECTED_PACKETS_PER_NODE))
            f.write("  Packet Delivery Ratio (PDR):              {:.2f}%\n".format(result['pdr']))
            f.write("  Total Transmissions (all nodes for delivered): {}\n".format(result['transmissions_for_delivered']))
            f.write("  Avg Transmissions per Delivered Packet:   {:.2f}\n".format(result['avg_tx_per_delivered']))
            f.write("\n")
        
        # Overall summary
        f.write("-"*80 + "\n")
        f.write("OVERALL SUMMARY\n")
        f.write("-"*80 + "\n\n")
        
        f.write("Total TX_SRC Events (all end nodes):             {}\n".format(total_tx_src))
        f.write("Total Packets Delivered to Rescue Node:          {} / {}\n".format(
            total_delivered, len(END_NODE_IDS) * EXPECTED_PACKETS_PER_NODE))
        f.write("Overall PDR (all end nodes combined):            {:.2f}%\n".format(overall_pdr))
        f.write("Average Packets Delivered per End Node:          {:.2f} / {}\n".format(
            avg_delivered_per_node, EXPECTED_PACKETS_PER_NODE))
        f.write("Total Network Energy Consumed:                   {:.2f} J\n".format(total_energy))
        f.write("\n")
        f.write("NETWORK OVERHEAD ANALYSIS:\n")
        f.write("  Total Network Transmissions (for delivered):   {}\n".format(total_transmissions_all))
        f.write("  Total Packets Successfully Delivered:          {}\n".format(total_delivered))
        f.write("  Average Transmissions per Delivered Packet:    {:.2f}\n".format(overall_avg_tx_per_delivered))
        f.write("    (This includes the original TX_SRC + all forwarding hops)\n")
        f.write("\n")
        
        # Rescue node perspective
        f.write("-"*80 + "\n")
        f.write("RESCUE NODE PERSPECTIVE\n")
        f.write("-"*80 + "\n\n")
        f.write("Rescue Node ID: {}\n".format(RESCUE_NODE_ID))
        f.write("Total Unique Packets Received: {}\n".format(total_delivered))
        f.write("Expected Total Packets: {}\n".format(len(END_NODE_IDS) * EXPECTED_PACKETS_PER_NODE))
        f.write("Reception Success Rate: {:.2f}%\n".format(overall_pdr))
        f.write("\nBreakdown by Source End Node:\n")
        for result in results:
            f.write("  From Node {}: {}/{} packets ({:.2f}%)\n".format(
                result['node_id'], result['delivered_count'], EXPECTED_PACKETS_PER_NODE, result['pdr']))
        
        f.write("\n" + "="*80 + "\n")
    
    # Print summary to console
    print("\n" + "="*80)
    print("ROUTING MODE: {} | MOBILE SPEED: {}".format(routing_mode, mobile_speed))
    print("="*80)
    print("\nPER-NODE TRANSMISSION SUMMARY:")
    print("-" * 80)
    for result in results:
        print("Node {}: TX_SRC={}, Delivered={}/{}, PDR={:.2f}%, Avg TX/Delivered={:.2f}".format(
            result['node_id'], result['tx_src_count'], result['delivered_count'], 
            EXPECTED_PACKETS_PER_NODE, result['pdr'], result['avg_tx_per_delivered']))
    
    print("\nOVERALL SUMMARY:")
    print("-" * 80)
    print("Total Packets Sent (TX_SRC): {}".format(total_tx_src))
    print("Total Packets Delivered: {}/{}".format(total_delivered, len(END_NODE_IDS) * EXPECTED_PACKETS_PER_NODE))
    print("Overall PDR: {:.2f}%".format(overall_pdr))
    print("Average Delivered per Node: {:.2f}/{}".format(avg_delivered_per_node, EXPECTED_PACKETS_PER_NODE))
    print("Total Network Energy: {:.2f} J".format(total_energy))
    print("\nNetwork Overhead: {:.2f} transmissions per delivered packet".format(overall_avg_tx_per_delivered))
    
    print("\n" + "="*80)
    print("Full report saved to: {}".format(report_path))
    print("="*80)

if __name__ == "__main__":
    print("4-Node to Rescue Node Analysis Script")
    print("="*80)
    analyze_4node_paths()
