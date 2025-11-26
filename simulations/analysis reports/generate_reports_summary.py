#!/usr/bin/env python3
"""
Scan analysis report .txt files in this folder and produce a consolidated CSV.
Writes `analysis_reports_summary.csv` next to the reports.

Fields produced:
- report_file
- results_dir
- endnode_1000_x, endnode_1000_y
- endnode_1001_x, endnode_1001_y
- distance_m
- total_generated
- total_delivered
- delivery_rate_pct
- avg_transit_time_s
- min_transit_time_s
- max_transit_time_s
- hop_counts (semicolon-separated for each delivered packet)
- throughput_packets_per_s

Usage:
 python generate_reports_summary.py [--reports-dir path] [--out-file filename]

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

    # results_dir
    data['results_dir'] = find_first(r"Results directory:\s*(.+)")

    # end node positions
    m = re.search(r"End Node 1000 position:\s*\(([-\d.]+),\s*([-\d.]+)\)", text)
    if m:
        data['endnode_1000_x'] = m.group(1)
        data['endnode_1000_y'] = m.group(2)
    else:
        data['endnode_1000_x'] = ''
        data['endnode_1000_y'] = ''

    m = re.search(r"End Node 1001 position:\s*\(([-\d.]+),\s*([-\d.]+)\)", text)
    if m:
        data['endnode_1001_x'] = m.group(1)
        data['endnode_1001_y'] = m.group(2)
    else:
        data['endnode_1001_x'] = ''
        data['endnode_1001_y'] = ''

    # distance
    dist = find_first(r"Distance:\s*([-\d.]+)\s*meters")
    if not dist:
        dist = find_first(r"Network span:\s*([-\d.]+)\s*meters")
    data['distance_m'] = dist

    # totals
    data['total_generated'] = find_first(r"Total data packets generated:\s*(\d+)")
    data['total_delivered'] = find_first(r"Total data packets delivered:\s*(\d+)")

    # delivery rate
    dr = find_first(r"Delivery success rate:\s*([\d.]+)%")
    if not dr:
        # fallback: look for line like "Delivery success rate: 12.3% (2/10)"
        dr = find_first(r"Delivery success rate:\s*([\d.]+)%")
    data['delivery_rate_pct'] = dr

    # transit times
    data['avg_transit_time_s'] = find_first(r"Average transit time:\s*([\d.]+)\s*seconds")
    data['min_transit_time_s'] = find_first(r"Minimum transit time:\s*([\d.]+)\s*seconds")
    if not data['min_transit_time_s']:
        data['min_transit_time_s'] = find_first(r"Minimum transit time:\s*([\d.]+)\s*seconds")
    data['max_transit_time_s'] = find_first(r"Maximum transit time:\s*([\d.]+)\s*seconds")

    # throughput (packets/second)
    th = find_first(r"throughput:\s*([\d.]+)\s*packets/?second", re.IGNORECASE)
    if not th:
        th = find_first(r"Effective throughput:\s*([\d.]+)\s*packets/?second", re.IGNORECASE)
    data['throughput_packets_per_s'] = th

    # Hop counts per delivered packet: parse per-packet section
    hop_counts = []
    current_packet = None
    current_delivered = False

    for i, line in enumerate(lines):
        line = line.strip()
        # Packet start
        m = re.match(r"Packet\s+(\S+):", line)
        if m:
            current_packet = m.group(1)
            current_delivered = False
            continue
        if current_packet is None:
            continue
        # Delivered marker
        if 'Delivered at' in line or '✓ Delivered' in line:
            current_delivered = True
            continue
        # Hop count line
        m = re.match(r"Hop count:\s*(\d+)", line)
        if m:
            hop = int(m.group(1))
            if current_delivered:
                hop_counts.append(hop)
            # reset packet context after reading hop count to avoid duplicate association
            current_packet = None
            current_delivered = False

    # Save hop counts as semicolon-separated string (or empty)
    data['hop_counts'] = ';'.join(str(x) for x in hop_counts) if hop_counts else ''

    # Flag indicating whether at least one copy was delivered
    try:
        delivered_count = int(data.get('total_delivered') or 0)
    except ValueError:
        delivered_count = 0
    data['delivered_any'] = 'yes' if delivered_count > 0 else 'no'

    return data


def main():
    parser = argparse.ArgumentParser(description='Generate CSV summary from analysis report .txt files')
    parser.add_argument('--reports-dir', default='.', help='Directory containing analysis report .txt files (default: current dir)')
    parser.add_argument('--out-file', default='analysis_reports_summary.csv', help='Output CSV filename (placed in reports dir)')
    args = parser.parse_args()

    reports_dir = Path(args.reports_dir)
    if not reports_dir.exists():
        print(f"Reports directory not found: {reports_dir}")
        return 1

    report_files = sorted(reports_dir.glob(REPORT_PATTERN))
    if not report_files:
        print(f"No report .txt files found in {reports_dir}")
        return 0

    out_path = reports_dir / args.out_file
    fieldnames = [
        'report_file', 'results_dir',
        'endnode_1000_x','endnode_1000_y','endnode_1001_x','endnode_1001_y',
        'distance_m','total_generated','total_delivered','delivery_rate_pct',
        'delivered_any',
        'avg_transit_time_s','min_transit_time_s','max_transit_time_s',
        'hop_counts','throughput_packets_per_s'
    ]

    rows = []
    for rpt in report_files:
        try:
            d = parse_report_file(rpt)
            rows.append(d)
        except Exception as e:
            print(f"Warning: failed to parse {rpt}: {e}")

    # Write CSV
    try:
        with open(out_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in rows:
                # Ensure all fields present
                row = {k: r.get(k, '') for k in fieldnames}
                writer.writerow(row)
        print(f"Wrote summary CSV with {len(rows)} rows to: {out_path}")
    except Exception as e:
        print(f"ERROR writing CSV: {e}")
        return 1

    return 0

if __name__ == '__main__':
    raise SystemExit(main())
