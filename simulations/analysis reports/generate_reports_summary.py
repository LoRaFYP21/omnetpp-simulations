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

    # Delivery rate is intentionally not exported in the summary CSV

    # final transit time (first-arrival) — parse "Final transit time:" (may end with 's' or ' seconds')
    final_tt = find_first(r"Final transit time:\s*([\d.]+)(?:s|\s*seconds)?")
    if not final_tt:
        # fallback: use the first numeric in the "Transit times (all copies):" list
        final_tt = find_first(r"Transit times \(all copies\):\s*([\d.]+)")
    data['transit_time_s'] = final_tt

    # throughput (packets/second)
    th = find_first(r"throughput:\s*([\d.]+)\s*packets/?second", re.IGNORECASE)
    if not th:
        th = find_first(r"Effective throughput:\s*([\d.]+)\s*packets/?second", re.IGNORECASE)
    data['throughput_packets_per_s'] = th

    # Final hop count: parse "Final hop count:" from report
    final_hop = find_first(r"Final hop count:\s*(\d+)")
    # Save as single value in hop_counts column (keeping column name same)
    data['hop_counts'] = final_hop or ''

    # Unique nodes processed: parse either per-packet lines and take max, or a summary if present
    # We look for lines like: "Unique nodes processed: N"
    # We'll take the maximum N found across packets as the summary value
    nodes_processed_matches = re.findall(r"Unique nodes processed:\s*(\d+)", text)
    if nodes_processed_matches:
        try:
            data['nodes_processed'] = str(max(int(n) for n in nodes_processed_matches))
        except Exception:
            data['nodes_processed'] = ''
    else:
        data['nodes_processed'] = ''

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
        'distance_m','total_generated','total_delivered',
        'delivered_any',
        'transit_time_s',
        'hop_counts','throughput_packets_per_s','nodes_processed'
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
