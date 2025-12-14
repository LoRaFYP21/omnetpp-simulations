import argparse
import csv
import os
import re
from collections import defaultdict
from typing import Dict, List, Optional, Tuple


def parse_meta(filename: str) -> Tuple[str, Optional[int]]:
    name = os.path.basename(filename).lower()
    scenario = "routing" if "routing" in name else ("flooding" if "flooding" in name else "unknown")
    m = re.search(r"(two|four|six|eight|ten)\s*[_\- ]*pair", name)
    pairs_map = {"two": 2, "four": 4, "six": 6, "eight": 8, "ten": 10}
    pairs = pairs_map.get(m.group(1)) if m else None
    return scenario, pairs


def parse_int(val: str) -> Optional[int]:
    try:
        return int(val)
    except Exception:
        return None


def parse_float(val: str) -> Optional[float]:
    try:
        return float(val)
    except Exception:
        return None


def load_csv_rows(path: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    with open(path, newline='', encoding='utf-8', errors='ignore') as fh:
        reader = csv.DictReader(fh)
        # Skip non-standard trailing summary rows by requiring src/dst
        for entry in reader:
            if 'src' not in entry or 'dst' not in entry:
                continue
            if not entry['src'] or not entry['dst']:
                continue
            rows.append(entry)
    return rows


def compute_success_per_run(rows: List[Dict[str, str]]) -> Dict[int, float]:
    """
    For a given file's rows, compute success rate per run_index:
    success_rate(run) = delivered_pairs_count / unique_pairs_in_run.
    """
    by_run: Dict[int, List[Tuple[int, int, int]]] = defaultdict(list)
    delivered_by_run: Dict[int, int] = defaultdict(int)

    for r in rows:
        run_index = parse_int(r.get('run_index', ''))
        src = parse_int(r.get('src', ''))
        dst = parse_int(r.get('dst', ''))
        delivered = parse_int(r.get('delivered', ''))
        if run_index is None or src is None or dst is None:
            continue
        # Track unique pairs per run
        by_run[run_index].append((src, dst, delivered or 0))

    success_rate_per_run: Dict[int, float] = {}
    for run, triples in by_run.items():
        # Unique pairs for the run
        unique_pairs = {(src, dst) for (src, dst, _del) in triples}
        if not unique_pairs:
            continue
        # Count delivered pairs (any row for that src,dst with delivered==1)
        delivered_pairs = 0
        for (src, dst) in unique_pairs:
            # find any corresponding rows flagged delivered
            if any((s == src and d == dst and delv == 1) for (s, d, delv) in triples):
                delivered_pairs += 1
        success_rate_per_run[run] = delivered_pairs / float(len(unique_pairs))

    return success_rate_per_run


def write_csv(path: str, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fieldnames = list(rows[0].keys())
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> None:
    parser = argparse.ArgumentParser(description='Compute per-run success rates and averages for routing vs flooding by pair count')
    parser.add_argument('--input-dir', default=os.path.join('.', '13th december congestion'), help='Folder containing *pair_report.csv files')
    parser.add_argument('--pattern', default='*pair_report.csv', help='Glob pattern for input files')
    parser.add_argument('--per-run-output', default='success_per_run.csv', help='Output CSV for per-run success rates')
    parser.add_argument('--summary-output', default='success_summary.csv', help='Output CSV for average success rates by scenario/pairs')
    args = parser.parse_args()

    import glob
    files = glob.glob(os.path.join(args.input_dir, args.pattern))
    if not files:
        print(f"No files matched in {args.input_dir} with pattern {args.pattern}")
        return

    per_run_rows: List[Dict[str, object]] = []
    # Aggregation: (scenario, pairs) -> list of success rates
    bucket: Dict[Tuple[str, Optional[int]], List[float]] = defaultdict(list)

    for f in files:
        scenario, pairs = parse_meta(f)
        rows = load_csv_rows(f)
        per_run = compute_success_per_run(rows)
        for run_idx, rate in sorted(per_run.items()):
            per_run_rows.append({
                'file': os.path.basename(f),
                'scenario': scenario,
                'pairs': pairs,
                'run_index': run_idx,
                'success_rate': round(rate, 6),
            })
            bucket[(scenario, pairs)].append(rate)

    # Write per-run CSV
    write_csv(args.per_run_output, per_run_rows)

    # Build summary averages
    summary_rows: List[Dict[str, object]] = []
    for (scenario, pairs), rates in sorted(bucket.items(), key=lambda x: (str(x[0][0]), x[0][1] or -1)):
        if not rates:
            continue
        avg_rate = sum(rates) / float(len(rates))
        summary_rows.append({
            'scenario': scenario,
            'pairs': pairs,
            'runs': len(rates),
            'avg_success_rate': round(avg_rate, 6),
        })

    write_csv(args.summary_output, summary_rows)

    print(f"Wrote per-run success rates -> {args.per_run_output} ({len(per_run_rows)} rows)")
    print(f"Wrote summary averages -> {args.summary_output} ({len(summary_rows)} rows)")


if __name__ == '__main__':
    main()
