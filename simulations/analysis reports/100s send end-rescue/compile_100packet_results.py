#!/usr/bin/env python
"""
Compile 100-packet scenario results from multiple analysis reports into a single CSV file.
Scans all .txt files in the current directory and extracts key metrics.
"""

import os
import re
import csv
from datetime import datetime

def parse_100packet_report(filepath):
    """
    Parse a 100-packet analysis report and extract key metrics.
    
    Returns:
        dict with keys: routing_mode, mobile_speed, initial_distance, timestamp,
                       total_sent, total_delivered, pdr_pct, avg_latency_s, unique_nodes
    """
    data = {
        'routing_mode': None,
        'mobile_speed': None,
        'initial_distance': None,
        'timestamp': None,
        'total_sent': None,
        'total_delivered': None,
        'pdr_pct': None,
        'avg_latency_s': None,
        'unique_nodes': None
    }
    
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        
        # Extract routing mode
        match = re.search(r'Routing Mode:\s*(\w+)', content)
        if match:
            data['routing_mode'] = match.group(1)
        
        # Extract mobile speed
        match = re.search(r'Mobile Speed:\s*(\d+)', content)
        if match:
            data['mobile_speed'] = match.group(1) + 'mps'
        
        # Extract initial distance
        match = re.search(r'Initial Distance:\s*(\S+)', content)
        if match:
            data['initial_distance'] = match.group(1)
        
        # Extract timestamp
        match = re.search(r'Generated:\s*(.+)', content)
        if match:
            data['timestamp'] = match.group(1).strip()
        
        # Extract total sent
        match = re.search(r'Total distinct packetSeq values SENT.*:\s*(\d+)', content)
        if match:
            data['total_sent'] = int(match.group(1))
        
        # Extract total delivered
        match = re.search(r'Packets with at least one delivery:\s*(\d+)', content)
        if match:
            data['total_delivered'] = int(match.group(1))
        
        # Extract PDR
        match = re.search(r'Packet Delivery Rate.*:\s*([\d.]+)%', content)
        if match:
            data['pdr_pct'] = float(match.group(1))
        
        # Extract average latency
        match = re.search(r'Average Latency:\s*([\d.]+)s', content)
        if match:
            data['avg_latency_s'] = float(match.group(1))
        
        # Extract unique nodes
        match = re.search(r'Unique Nodes Processing Packets:\s*(\d+)', content)
        if match:
            data['unique_nodes'] = int(match.group(1))
        
        return data
        
    except Exception as e:
        print("Error parsing {}: {}".format(filepath, str(e)))
        return None


def main():
    # Get current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Find all .txt analysis files (excluding those ending with _details.csv)
    txt_files = []
    for filename in os.listdir(current_dir):
        if filename.endswith('.txt') and not filename.endswith('_details.csv'):
            txt_files.append(os.path.join(current_dir, filename))
    
    if not txt_files:
        print("No .txt analysis files found in current directory.")
        return
    
    print("Found {} analysis report(s)".format(len(txt_files)))
    
    # Parse all reports
    results = []
    for filepath in txt_files:
        print("Parsing: {}".format(os.path.basename(filepath)))
        data = parse_100packet_report(filepath)
        if data and data['routing_mode']:
            results.append(data)
        else:
            print("  WARNING: Failed to parse or missing routing_mode")
    
    if not results:
        print("No valid results extracted.")
        return
    
    # Sort results: FLOODING first (by speed), then DSDV/other (by speed)
    def sort_key(item):
        mode = item['routing_mode']
        speed_str = item['mobile_speed'] or '0mps'
        speed = int(re.search(r'(\d+)', speed_str).group(1)) if re.search(r'(\d+)', speed_str) else 0
        
        # FLOODING = 0, DSDV = 1, others = 2
        if mode == 'FLOODING':
            mode_order = 0
        elif mode == 'DSDV':
            mode_order = 1
        else:
            mode_order = 2
        
        return (mode_order, speed)
    
    results.sort(key=sort_key)
    
    # Write to CSV
    output_csv = os.path.join(current_dir, '100packet_comparison.csv')
    
    fieldnames = [
        'routing_mode',
        'mobile_speed',
        'initial_distance',
        'timestamp',
        'total_sent',
        'total_delivered',
        'pdr_pct',
        'avg_latency_s',
        'unique_nodes'
    ]
    
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    
    print("\n" + "="*60)
    print("SUCCESS: Compiled {} results into CSV".format(len(results)))
    print("Output file: {}".format(output_csv))
    print("="*60)
    
    # Print summary
    print("\nSummary by routing mode:")
    by_mode = {}
    for result in results:
        mode = result['routing_mode']
        if mode not in by_mode:
            by_mode[mode] = []
        by_mode[mode].append(result)
    
    for mode in sorted(by_mode.keys()):
        runs = by_mode[mode]
        avg_pdr = sum(r['pdr_pct'] for r in runs if r['pdr_pct']) / len(runs)
        avg_latency = sum(r['avg_latency_s'] for r in runs if r['avg_latency_s']) / len([r for r in runs if r['avg_latency_s']])
        print("  {}: {} run(s), Avg PDR = {:.2f}%, Avg Latency = {:.4f}s".format(
            mode, len(runs), avg_pdr, avg_latency))


if __name__ == '__main__':
    main()
