#!/usr/bin/env python3
"""
Scan analysis report .txt files in this folder and produce a consolidated CSV.
Creates protocol-specific CSV files: 'dsdv_routing_mobile.csv' or 'flooding_mobile.csv'

Fields produced:
- report_file
- rescue_node_speed_mps
- relay_incr_update_s, relay_full_dump_s
- endnode_incr_update_s, endnode_full_dump_s
- rescue_incr_update_s, rescue_full_dump_s
- initial_distance_m
- delivery_rate (1 if delivered, 0 if not)
- total_copies_at_destination
- unique_nodes_processed
- total_energy_j
- transit_time_s
- hop_count

Usage:
 python generate_reports_summary.py [--reports-dir path]

"""
import os
import re
import csv
import argparse
from pathlib import Path

REPORT_PATTERN = "*.txt"

def parse_report_file(path):
    """Return a dict with parsed fields (empty strings when not found)."""
    text = Path(path).read_text(encoding='utf-8', errors='ignore')
    lines = text.splitlines()

    def find_first(regex, flags=0):
        m = re.search(regex, text, flags)
        return m.group(1).strip() if m else ''

    data = {}
    data['report_file'] = os.path.basename(path)

    # Detect protocol type
    protocol = ''
    if 'DSDV' in text or 'Destination-Sequenced Distance-Vector' in text:
        protocol = 'DSDV'
    elif 'FLOODING' in text or 'Smart Flooding' in text:
        protocol = 'FLOODING'
    data['protocol'] = protocol

    # Rescue node speed
    speed = find_first(r"Rescue Node Speed:\s*([\d.]+)\s*m/s")
    data['rescue_node_speed_mps'] = speed

    # DSDV Timer Settings - Relay Nodes
    relay_section = re.search(r"Relay Nodes \(loRaNodes\):\s*\n\s*-\s*Incremental update:\s*([\d.]+)s\s*\n\s*-\s*Full dump:\s*([\d.]+)s", text)
    if relay_section:
        data['relay_incr_update_s'] = relay_section.group(1)
        data['relay_full_dump_s'] = relay_section.group(2)
    else:
        data['relay_incr_update_s'] = ''
        data['relay_full_dump_s'] = ''

    # DSDV Timer Settings - End Nodes
    endnode_section = re.search(r"End Nodes \(loRaEndNodes\):\s*\n\s*-\s*Incremental update:\s*([\d.]+)s\s*\n\s*-\s*Full dump:\s*([\d.]+)s", text)
    if endnode_section:
        data['endnode_incr_update_s'] = endnode_section.group(1)
        data['endnode_full_dump_s'] = endnode_section.group(2)
    else:
        data['endnode_incr_update_s'] = ''
        data['endnode_full_dump_s'] = ''

    # DSDV Timer Settings - Rescue Nodes
    rescue_section = re.search(r"Rescue Nodes \(loRaRescueNodes\):\s*\n\s*-\s*Incremental update:\s*([\d.]+)s\s*\n\s*-\s*Full dump:\s*([\d.]+)s", text)
    if rescue_section:
        data['rescue_incr_update_s'] = rescue_section.group(1)
        data['rescue_full_dump_s'] = rescue_section.group(2)
    else:
        data['rescue_incr_update_s'] = ''
        data['rescue_full_dump_s'] = ''

    # Initial distance
    dist = find_first(r"Initial distance:\s*([\d.]+)\s*meters")
    data['initial_distance_m'] = dist

    # Delivery rate (binary: 1 if delivered, 0 if not)
    delivered = find_first(r"Packets delivered:\s*(\d+)")
    if not delivered:
        delivered = find_first(r"Total data packets delivered:\s*(\d+)")
    try:
        delivery_rate = '1' if int(delivered or 0) > 0 else '0'
    except ValueError:
        delivery_rate = '0'
    data['delivery_rate'] = delivery_rate

    # Total copies received at destination
    copies = find_first(r"Total copies received at destination:\s*(\d+)")
    data['total_copies_at_destination'] = copies

    # Unique nodes processed
    unique_nodes = find_first(r"Total unique nodes that processed packets:\s*(\d+)")
    data['unique_nodes_processed'] = unique_nodes

    # Total energy consumption
    energy = find_first(r"Total energy consumption:\s*([\d.]+)\s*J")
    data['total_energy_j'] = energy

    # Transit time (first packet)
    transit = find_first(r"Transit time:\s*([\d.]+)s")
    if not transit:
        transit = find_first(r"Average transit time:\s*([\d.]+)s")
    data['transit_time_s'] = transit

    # Hop count (from packet details)
    hop_count = find_first(r"Hop count:\s*(\d+)")
    data['hop_count'] = hop_count

    return data


def main():
    parser = argparse.ArgumentParser(description='Generate CSV summary from analysis report .txt files')
    parser.add_argument('--reports-dir', default='.', help='Directory containing analysis report .txt files (default: current dir)')
    args = parser.parse_args()

    reports_dir = Path(args.reports_dir)
    if not reports_dir.exists():
        print(f"Reports directory not found: {reports_dir}")
        return 1

    report_files = sorted(reports_dir.glob(REPORT_PATTERN))
    if not report_files:
        print(f"No report .txt files found in {reports_dir}")
        return 0

    # Define fieldnames for CSV
    fieldnames = [
        'report_file',
        'rescue_node_speed_mps',
        'relay_incr_update_s', 'relay_full_dump_s',
        'endnode_incr_update_s', 'endnode_full_dump_s',
        'rescue_incr_update_s', 'rescue_full_dump_s',
        'initial_distance_m',
        'delivery_rate',
        'total_copies_at_destination',
        'unique_nodes_processed',
        'total_energy_j',
        'transit_time_s',
        'hop_count'
    ]

    # Parse all reports and group by protocol
    dsdv_rows = []
    flooding_rows = []
    
    for rpt in report_files:
        try:
            d = parse_report_file(rpt)
            if d.get('protocol') == 'DSDV':
                dsdv_rows.append(d)
            elif d.get('protocol') == 'FLOODING':
                flooding_rows.append(d)
            else:
                print(f"Warning: Unknown protocol in {rpt.name}")
        except Exception as e:
            print(f"Warning: failed to parse {rpt}: {e}")

    # Write DSDV CSV if we have DSDV reports
    if dsdv_rows:
        out_path = reports_dir / 'dsdv_routing_mobile.csv'
        try:
            with open(out_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for r in dsdv_rows:
                    row = {k: r.get(k, '') for k in fieldnames}
                    writer.writerow(row)
            print(f"✓ Wrote DSDV summary CSV with {len(dsdv_rows)} rows to: {out_path}")
        except Exception as e:
            print(f"ERROR writing DSDV CSV: {e}")
            return 1

    # Write Flooding CSV if we have Flooding reports
    if flooding_rows:
        out_path = reports_dir / 'flooding_mobile.csv'
        try:
            with open(out_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for r in flooding_rows:
                    row = {k: r.get(k, '') for k in fieldnames}
                    writer.writerow(row)
            print(f"✓ Wrote Flooding summary CSV with {len(flooding_rows)} rows to: {out_path}")
        except Exception as e:
            print(f"ERROR writing Flooding CSV: {e}")
            return 1

    if not dsdv_rows and not flooding_rows:
        print("No valid reports found with recognized protocol (DSDV or FLOODING)")
        return 0

    return 0

if __name__ == '__main__':
    raise SystemExit(main())
