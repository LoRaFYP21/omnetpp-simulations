#!/usr/bin/env python
"""
Consolidates all 4-node analysis reports into a single comparison CSV.
Extracts key metrics from text reports and organizes by routing mode and speed.
"""

import os
import re
import csv
from collections import defaultdict

# Configuration
REPORTS_DIR = "analysis reports"
OUTPUT_CSV = os.path.join(REPORTS_DIR, "4node_comparison.csv")

def parse_report_file(filepath):
    """Extract key metrics from a 4-node analysis report."""
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        
        data = {}
        
        # Extract routing mode
        match = re.search(r'Routing Mode:\s*(\w+)', content)
        data['routing_mode'] = match.group(1) if match else 'unknown'
        
        # Extract mobile speed
        match = re.search(r'Mobile Node Speed:\s*(\S+)', content)
        data['mobile_speed'] = match.group(1) if match else 'unknown'
        
        # Extract total TX_SRC events
        match = re.search(r'Total TX_SRC Events.*?:\s*(\d+)', content)
        data['total_tx_src'] = int(match.group(1)) if match else 0
        
        # Extract total packets delivered (e.g., "59 / 80")
        match = re.search(r'Total Packets Delivered to Rescue Node:\s*(\d+)\s*/\s*(\d+)', content)
        data['total_delivered'] = int(match.group(1)) if match else 0
        
        # Extract overall PDR
        match = re.search(r'Overall PDR.*?:\s*([\d.]+)%', content)
        data['overall_pdr_pct'] = float(match.group(1)) if match else 0.0
        
        # Extract average delivered per node
        match = re.search(r'Average Packets Delivered per End Node:\s*([\d.]+)\s*/\s*(\d+)', content)
        data['avg_delivered_per_node'] = float(match.group(1)) if match else 0.0
        
        # Extract average transmissions per delivered packet
        match = re.search(r'Average Transmissions per Delivered Packet:\s*([\d.]+)', content)
        data['avg_tx_per_delivered'] = float(match.group(1)) if match else 0.0
        
        # Extract total network energy
        match = re.search(r'Total Network Energy Consumed:\s*([\d.]+)\s*J', content)
        data['total_network_energy_J'] = float(match.group(1)) if match else 0.0
        
        return data
        
    except Exception as e:
        print("Error parsing {}: {}".format(filepath, str(e)))
        return None

def extract_speed_number(speed_str):
    """Extract numeric speed from string like '7mps' -> 7"""
    match = re.search(r'(\d+)', speed_str)
    return int(match.group(1)) if match else 999

def main():
    """Main function to consolidate all reports into CSV."""
    
    if not os.path.exists(REPORTS_DIR):
        print("ERROR: {} directory not found!".format(REPORTS_DIR))
        return
    
    # Find all 4node analysis report files
    report_files = []
    for filename in os.listdir(REPORTS_DIR):
        if filename.endswith('_4node_analysis.txt'):
            report_files.append(os.path.join(REPORTS_DIR, filename))
    
    if not report_files:
        print("No 4-node analysis reports found in {}".format(REPORTS_DIR))
        return
    
    print("Found {} report files".format(len(report_files)))
    
    # Parse all reports
    parsed_data = []
    for filepath in report_files:
        print("Parsing: {}".format(os.path.basename(filepath)))
        data = parse_report_file(filepath)
        if data:
            parsed_data.append(data)
    
    if not parsed_data:
        print("ERROR: No data could be parsed from reports")
        return
    
    # Group by routing mode and sort by speed
    flooding_data = []
    dsdv_data = []
    
    for data in parsed_data:
        if data['routing_mode'] == 'FLOODING':
            flooding_data.append(data)
        elif data['routing_mode'] == 'DSDV':
            dsdv_data.append(data)
    
    # Sort each group by speed (numerically)
    flooding_data.sort(key=lambda x: extract_speed_number(x['mobile_speed']))
    dsdv_data.sort(key=lambda x: extract_speed_number(x['mobile_speed']))
    
    # Combine: FLOODING first, then DSDV
    sorted_data = flooding_data + dsdv_data
    
    print("\nData organization:")
    print("  FLOODING entries: {} (sorted by speed)".format(len(flooding_data)))
    print("  DSDV entries: {} (sorted by speed)".format(len(dsdv_data)))
    
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
        writer = csv.DictWriter(csvfile, fieldnames=csv_columns, lineterminator='\n')
        writer.writeheader()
        
        for data in sorted_data:
            # Write only the requested columns
            row = {col: data.get(col, '') for col in csv_columns}
            writer.writerow(row)
    
    print("\n" + "="*80)
    print("CSV file created: {}".format(OUTPUT_CSV))
    print("Total rows: {} (excluding header)".format(len(sorted_data)))
    print("="*80)
    
    # Print preview
    print("\nPreview of CSV contents:")
    print("-"*80)
    print(",".join(csv_columns))
    for i, data in enumerate(sorted_data[:5]):  # Show first 5 rows
        row_values = [str(data.get(col, '')) for col in csv_columns]
        print(",".join(row_values))
    if len(sorted_data) > 5:
        print("... ({} more rows)".format(len(sorted_data) - 5))
    print("-"*80)

if __name__ == "__main__":
    print("4-Node Analysis Report Consolidation Script")
    print("="*80)
    main()
