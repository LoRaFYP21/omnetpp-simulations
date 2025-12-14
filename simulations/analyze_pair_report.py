import csv
import os
import sys
from typing import Dict, List, Optional


def parse_float(v: str) -> Optional[float]:
    try:
        v = v.strip()
        if v == "" or v.lower() == "nan":
            return None
        return float(v)
    except Exception:
        return None


def parse_int(v: str) -> Optional[int]:
    try:
        v = v.strip()
        if v == "" or v.lower() == "nan":
            return None
        return int(float(v))
    except Exception:
        return None


def analyze_csv(path: str):
    runs: Dict[int, Dict[str, List[float]]] = {}

    with open(path, "r", newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            run_idx = parse_int(row.get("run_index", ""))
            src = row.get("src", "").strip()
            dst = row.get("dst", "").strip()
            # skip footer or empty rows
            if run_idx is None or src == "" or dst == "":
                continue

            delivered = parse_int(row.get("delivered", ""))
            energy = parse_float(row.get("total_energy_j", ""))
            transit = parse_float(row.get("transit_time_s", ""))
            uniq_nodes = parse_int(row.get("unique_nodes_processed_first_packet", ""))

            d = runs.setdefault(run_idx, {
                "delivered": [],
                "energy": [],
                "transit_times": [],
                "unique_nodes": [],
                "pairs": 0,
            })

            d["pairs"] += 1
            if delivered is not None:
                d["delivered"].append(delivered)
            if energy is not None:
                d["energy"].append(energy)
            if transit is not None and delivered == 1:
                d["transit_times"].append(transit)
            if uniq_nodes is not None:
                d["unique_nodes"].append(uniq_nodes)

    # Compute per-run metrics
    per_run = []
    per_run_success_rates = []
    per_run_energy_values = []
    per_run_min_transit = []
    per_run_max_transit = []
    per_run_min_unique = []
    per_run_max_unique = []

    for run_idx in sorted(runs.keys()):
        d = runs[run_idx]
        pairs = max(d["pairs"], 1)
        delivered_count = sum(1 for x in d["delivered"] if x == 1)
        success_rate = delivered_count / pairs

        # Energy: deduplicate per run to avoid counting the same value multiple times
        energy_val = None
        if d["energy"]:
            # Most files repeat the same total_energy per row; take the first unique
            # or average duplicates if they differ
            unique_vals = sorted(set(d["energy"]))
            if len(unique_vals) == 1:
                energy_val = unique_vals[0]
            else:
                energy_val = sum(d["energy"]) / len(d["energy"])  # fallback average

        # Transit times (only delivered rows)
        min_transit = None
        max_transit = None
        if d["transit_times"]:
            min_transit = min(d["transit_times"])  # per-run min
            max_transit = max(d["transit_times"])  # per-run max

        # Unique nodes
        min_unique = None
        max_unique = None
        if d["unique_nodes"]:
            min_unique = min(d["unique_nodes"])  # per-run min
            max_unique = max(d["unique_nodes"])  # per-run max

        per_run.append({
            "run_index": run_idx,
            "success_rate": success_rate,
            "energy": energy_val,
            "min_transit_time": min_transit,
            "max_transit_time": max_transit,
            "min_unique_nodes": min_unique,
            "max_unique_nodes": max_unique,
        })

        per_run_success_rates.append(success_rate)
        if energy_val is not None:
            per_run_energy_values.append(energy_val)
        if min_transit is not None:
            per_run_min_transit.append(min_transit)
        if max_transit is not None:
            per_run_max_transit.append(max_transit)
        if min_unique is not None:
            per_run_min_unique.append(min_unique)
        if max_unique is not None:
            per_run_max_unique.append(max_unique)

    # Overall averages (across runs)
    def avg(lst: List[float]) -> Optional[float]:
        return round(sum(lst) / len(lst), 6) if lst else None

    overall = {
        "average_success_rate_per_run": avg(per_run_success_rates),
        "average_energy": avg(per_run_energy_values),
        "average_min_transit_time": avg(per_run_min_transit),
        "average_max_transit_time": avg(per_run_max_transit),
        "average_min_unique_nodes": avg(per_run_min_unique),
        "average_max_unique_nodes": avg(per_run_max_unique),
    }

    return per_run, overall


def analyze_overall_rows(path: str):
    pairs_count = 0
    delivered_count = 0

    transit_times: List[float] = []
    unique_nodes: List[int] = []
    energy_by_run: Dict[int, float] = {}
    energy_values_set: set = set()

    with open(path, "r", newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            run_idx = parse_int(row.get("run_index", ""))
            src = row.get("src", "").strip()
            dst = row.get("dst", "").strip()
            # For overall metrics, allow missing run_index; only require src/dst
            if src == "" or dst == "":
                continue

            pairs_count += 1

            delivered = parse_int(row.get("delivered", ""))
            energy = parse_float(row.get("total_energy_j", ""))
            transit = parse_float(row.get("transit_time_s", ""))
            uniq_nodes = parse_int(row.get("unique_nodes_processed_first_packet", ""))

            if delivered == 1:
                delivered_count += 1
                if transit is not None:
                    transit_times.append(transit)
                if uniq_nodes is not None:
                    unique_nodes.append(uniq_nodes)

            # Deduplicate energy by run index to avoid counting repeated per-row totals
            if run_idx is not None and energy is not None and run_idx not in energy_by_run:
                energy_by_run[run_idx] = energy
            # When run_index is missing, fallback to unique energy value set
            if run_idx is None and energy is not None:
                energy_values_set.add(energy)

    def avg(lst: List[float]) -> Optional[float]:
        return round(sum(lst) / len(lst), 6) if lst else None

    # Prefer per-run dedup values; fallback to unique energy values if run_index missing
    energy_values = list(energy_by_run.values()) if energy_by_run else list(energy_values_set)

    overall_rows = {
        "success_rate": round(delivered_count / pairs_count, 6) if pairs_count > 0 else None,
        "average_energy": avg(energy_values),
        "transit_time_min": min(transit_times) if transit_times else None,
        "transit_time_avg": avg(transit_times),
        "transit_time_max": max(transit_times) if transit_times else None,
        "unique_nodes_min": min(unique_nodes) if unique_nodes else None,
        "unique_nodes_avg": avg([float(x) for x in unique_nodes]) if unique_nodes else None,
        "unique_nodes_max": max(unique_nodes) if unique_nodes else None,
    }

    return overall_rows


def write_per_run_csv(per_run: List[Dict], out_path: str):
    fieldnames = [
        "run_index",
        "success_rate",
        "energy",
        "min_transit_time",
        "max_transit_time",
        "min_unique_nodes",
        "max_unique_nodes",
    ]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in per_run:
            w.writerow(row)


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_pair_report.py <csv1> [<csv2> ...]")
        sys.exit(1)

    inputs = sys.argv[1:]
    for in_path in inputs:
        if not os.path.exists(in_path):
            print(f"Input file not found: {in_path}")
            continue

        print(f"\nAnalyzing: {in_path}")
        per_run, overall = analyze_csv(in_path)
        overall_rows = analyze_overall_rows(in_path)

        # Derive base name for overall summary only
        base, ext = os.path.splitext(in_path)

        # Write overall summary CSV
        overall_csv = base + ".summary_overall.csv"
        with open(overall_csv, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "success_rate",
                "average_energy",
                "transit_time_min",
                "transit_time_avg",
                "transit_time_max",
                "unique_nodes_min",
                "unique_nodes_avg",
                "unique_nodes_max",
            ]
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerow(overall_rows)

        # Print summary (overall across rows)
        print("Overall (Across All Rows):")
        for k, v in overall_rows.items():
            print(f"- {k}: {v}")

        print(f"Overall summary written to: {overall_csv}")


if __name__ == "__main__":
    main()
