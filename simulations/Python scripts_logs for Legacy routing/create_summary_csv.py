#!/usr/bin/env python3
"""
Create summary CSV from legacy analysis report text files
Extracts key metrics from each LEGACY_*.txt report and consolidates them
"""

import os
import re
import csv
from datetime import datetime


def extract_metrics_from_report(file_path):
    """Extract key metrics from a legacy analysis report text file"""
    metrics = {
        'filename': os.path.basename(file_path),
        'routing_type': None,
        'timestamp': None,
        'total_energy': None,
        'avg_pdr': None,
        'avg_tx_per_delivered': None,
        'forwarding_overhead': None
    }
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Extract routing type from filename
            if 'Routing' in metrics['filename']:
                metrics['routing_type'] = 'Routing'
            elif 'Flooding' in metrics['filename']:
                metrics['routing_type'] = 'Flooding'
            
            # Extract timestamp from filename (format: LEGACY_Type_YYYYMMDD_HHMMSS.txt)
            timestamp_match = re.search(r'_(\d{8}_\d{6})\.txt$', metrics['filename'])
            if timestamp_match:
                metrics['timestamp'] = timestamp_match.group(1)
            
            # Extract Total Network Energy Consumed
            energy_match = re.search(r'Total Network Energy Consumed:\s+([\d.]+)\s*J', content)
            if energy_match:
                metrics['total_energy'] = float(energy_match.group(1))
            
            # Extract Average PDR
            pdr_match = re.search(r'Average PDR:\s+([\d.]+)%', content)
            if pdr_match:
                metrics['avg_pdr'] = float(pdr_match.group(1))
            
            # Extract Average Transmissions per Delivered Packet
            tx_match = re.search(r'Average Transmissions per\s+Delivered Packet:\s+([\d.]+)x', content)
            if tx_match:
                metrics['avg_tx_per_delivered'] = float(tx_match.group(1))
            
            # Extract Forwarding Overhead Ratio
            overhead_match = re.search(r'Forwarding Overhead Ratio:\s+([\d.]+)x', content)
            if overhead_match:
                metrics['forwarding_overhead'] = float(overhead_match.group(1))
                
    except Exception as e:
        print(f"Warning: Could not parse {file_path}: {e}")
    
    return metrics


def create_summary_csv(input_dir='.', output_file='summary.csv'):
    """Create summary CSV from all LEGACY_*.txt files in directory"""
    
    print("\n" + "="*70)
    print(" LEGACY REPORTS SUMMARY GENERATOR")
    print("="*70)
    
    # Find all LEGACY_*.txt files
    txt_files = [f for f in os.listdir(input_dir) 
                 if f.startswith('LEGACY_') and f.endswith('.txt')]
    
    if not txt_files:
        print(f"\nNo LEGACY_*.txt files found in {input_dir}")
        return
    
    print(f"\nFound {len(txt_files)} report file(s):")
    for f in txt_files:
        print(f"  - {f}")
    
    # Extract metrics from each file
    print("\nExtracting metrics...")
    all_metrics = []
    for txt_file in sorted(txt_files):
        file_path = os.path.join(input_dir, txt_file)
        metrics = extract_metrics_from_report(file_path)
        all_metrics.append(metrics)
        print(f"  ✓ {txt_file}")
    
    # Write to CSV
    print(f"\nWriting summary to {output_file}...")
    csv_path = os.path.join(input_dir, output_file)
    
    with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
        fieldnames = [
            'Filename',
            'Routing Type',
            'Timestamp',
            'Total Network Energy (J)',
            'Average PDR (%)',
            'Avg TX per Delivered Packet',
            'Forwarding Overhead Ratio'
        ]
        
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for metrics in all_metrics:
            writer.writerow({
                'Filename': metrics['filename'],
                'Routing Type': metrics['routing_type'] or 'Unknown',
                'Timestamp': metrics['timestamp'] or 'Unknown',
                'Total Network Energy (J)': f"{metrics['total_energy']:.3f}" if metrics['total_energy'] is not None else 'N/A',
                'Average PDR (%)': f"{metrics['avg_pdr']:.2f}" if metrics['avg_pdr'] is not None else 'N/A',
                'Avg TX per Delivered Packet': f"{metrics['avg_tx_per_delivered']:.2f}" if metrics['avg_tx_per_delivered'] is not None else 'N/A',
                'Forwarding Overhead Ratio': f"{metrics['forwarding_overhead']:.2f}" if metrics['forwarding_overhead'] is not None else 'N/A'
            })
    
    print(f"✅ Summary created: {csv_path}")
    
    # Print summary stats
    print("\n" + "="*70)
    print(" SUMMARY STATISTICS")
    print("="*70)
    print(f"Total reports processed: {len(all_metrics)}")
    
    # Count by routing type
    routing_counts = {}
    for m in all_metrics:
        rt = m['routing_type'] or 'Unknown'
        routing_counts[rt] = routing_counts.get(rt, 0) + 1
    
    print("\nReports by routing type:")
    for rt, count in routing_counts.items():
        print(f"  {rt}: {count}")
    
    print("\n" + "="*70)


def main():
    """Main entry point"""
    import sys
    
    # Get current directory (script location)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Check command line arguments
    if len(sys.argv) > 1:
        output_file = sys.argv[1]
    else:
        output_file = 'summary.csv'
    
    create_summary_csv(input_dir=script_dir, output_file=output_file)
    
    print(f"\n✅ Done!")
    return 0


if __name__ == '__main__':
    exit(main())
