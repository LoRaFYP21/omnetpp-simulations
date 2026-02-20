#!/usr/bin/env python
"""
Compiles all 4-node analysis reports into a single comparison CSV file.
Groups by routing mode (FLOODING first, then DSDV) and sorts by mobile speed within each group.
"""

import os
import re
import csv
import glob

# Configuration
REPORTS_DIR = "analysis reports"
OUTPUT_CSV = os.path.join(REPORTS_DIR, "4node_comparison.csv")

def parse_report_file(filepath):
    """Extract key metrics from a single analysis report file."""
    data = {
        'routing_mode': '',
        'mobile_speed': '',
        'total_tx_src': 0,
        'total_delivered': 0,
        'overall_pdr_pct': 0.0,
        'avg_delivered_per_node': 0.0,
        'avg_tx_per_delivered': 0.0,
        'total_network_energy_J': 0.0,
        'filename': os.path.basename(filepath)
    }
    
    try:
        with open(filepath, 'r') as f:
            content = f.read()
            
            # Extract routing mode
            match = re.search(r'Routing Mode:\s*(\w+)', content)
            if match:
                data['routing_mode'] = match.group(1)
            
            # Extract mobile speed
            match = re.search(r'Mobile Node Speed:\s*(\d+mps)', content)
            if match:
                data['mobile_speed'] = match.group(1)
            
            # Extract total TX_SRC
            match = re.search(r'Total TX_SRC Events \(all end nodes\):\s*(\d+)', content)
            if match:
                data['total_tx_src'] = int(match.group(1))
            
            # Extract total delivered (from "Total Packets Delivered to Rescue Node: X / Y")
            match = re.search(r'Total Packets Delivered to Rescue Node:\s*(\d+)\s*/\s*\d+', content)
            if match:
                data['total_delivered'] = int(match.group(1))
            
            # Extract overall PDR
            match = re.search(r'Overall PDR \(all end nodes combined\):\s*([\d.]+)%', content)
            if match:
                data['overall_pdr_pct'] = float(match.group(1))
            
            # Extract average delivered per node (from "Average Packets Delivered per End Node: X / Y")
            match = re.search(r'Average Packets Delivered per End Node:\s*([\d.]+)\s*/\s*\d+', content)
            if match:
                data['avg_delivered_per_node'] = float(match.group(1))
            
            # Extract average transmissions per delivered packet
            match = re.search(r'Average Transmissions per Delivered Packet:\s*([\d.]+)', content)
            if match:
                data['avg_tx_per_delivered'] = float(match.group(1))
            
            # Extract total network energy
            match = re.search(r'Total Network Energy Consumed:\s*([\d.]+)\s*J', content)
            if match:
                data['total_network_energy_J'] = float(match.group(1))
    
    except Exception as e:
        print("Error parsing {}: {}".format(filepath, str(e)))
    
    return data

def extract_speed_value(speed_str):
    """Extract numeric value from speed string (e.g., '7mps' -> 7)."""
    match = re.search(r'(\d+)', speed_str)
    return int(match.group(1)) if match else 0

def compile_reports():
    """Main function to compile all reports into a comparison CSV."""
    
    # Find all analysis report files
    pattern = os.path.join(REPORTS_DIR, "*_4node_analysis.txt")
    report_files = glob.glob(pattern)
    
    if not report_files:
        print("ERROR: No analysis report files found in '{}'".format(REPORTS_DIR))
        return
    
    print("Found {} analysis report files".format(len(report_files)))
    
    # Parse all reports
    all_data = []
    for filepath in report_files:
        print("  Parsing: {}".format(os.path.basename(filepath)))
        data = parse_report_file(filepath)
        if data['routing_mode']:  # Only include if routing mode was detected
            all_data.append(data)
    
    if not all_data:
        print("ERROR: No valid data extracted from reports")
        return
    
    # Sort data: FLOODING first (sorted by speed), then DSDV (sorted by speed)
    flooding_data = [d for d in all_data if d['routing_mode'] == 'FLOODING']
    dsdv_data = [d for d in all_data if d['routing_mode'] == 'DSDV']
    
    # Sort each group by mobile speed
    flooding_data.sort(key=lambda x: extract_speed_value(x['mobile_speed']))
    dsdv_data.sort(key=lambda x: extract_speed_value(x['mobile_speed']))
    
    # Combine: FLOODING first, then DSDV
    sorted_data = flooding_data + dsdv_data
    
    print("\nSorted order ({} FLOODING, {} DSDV):".format(len(flooding_data), len(dsdv_data)))
    for d in sorted_data:
        print("  {} - {}".format(d['routing_mode'], d['mobile_speed']))
    
    # Write to CSV
    csv_columns = [
        'routing_mode',
        'mobile_speed',
        'total_tx_src',
        'total_delivered',
        'overall_pdr_pct',
        'avg_delivered_per_node',
        'avg_tx_per_delivered',
        'total_network_energy_J'
    ]
    
    with open(OUTPUT_CSV, 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(sorted_data)
    
    print("\n" + "="*80)
    print("SUCCESS: Comparison CSV created")
    print("="*80)
    print("Output file: {}".format(OUTPUT_CSV))
    print("Total rows: {}".format(len(sorted_data)))
    print("  - FLOODING: {} rows".format(len(flooding_data)))
    print("  - DSDV: {} rows".format(len(dsdv_data)))
    print("="*80)

if __name__ == "__main__":
    print("="*80)
    print("4-Node Analysis Reports Compiler")
    print("="*80)
    compile_reports()
