import argparse
import csv
import math
import os
from collections import defaultdict, namedtuple
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

Row = namedtuple(
    'Row',
    [
        'simTime',
        'event',
        'packetSeq',
        'src',
        'dst',
        'currentNode',
        'ttlAfterDecr',
        'chosenVia',
        'nextHopType',
    ],
)


def read_paths_csv(path: str) -> List[Row]:
    rows: List[Row] = []
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        for entry in reader:
            rows.append(
                Row(
                    simTime=float(entry['simTime']),
                    event=entry['event'],
                    packetSeq=int(entry['packetSeq']),
                    src=int(entry['src']),
                    dst=int(entry['dst']),
                    currentNode=int(entry['currentNode']),
                    ttlAfterDecr=int(entry['ttlAfterDecr']),
                    chosenVia=int(entry['chosenVia']),
                    nextHopType=entry['nextHopType'],
                )
            )
    return rows


class Positions:
    """Container for end node coordinates."""

    def __init__(self) -> None:
        self._positions: Dict[int, Tuple[Optional[float], Optional[float]]] = {}

    def load_csv(self, path: str) -> None:
        with open(path, newline='') as fh:
            reader = csv.DictReader(fh)
            for entry in reader:
                node_id = int(entry['id'])
                x = float(entry['x'])
                y = float(entry['y'])
                self._positions[node_id] = (x, y)

    def load_from_scalars(self, files: Iterable[Path]) -> None:
        for sca_path in files:
            try:
                with open(sca_path, 'r', encoding='utf-8', errors='ignore') as fh:
                    for line in fh:
                        if not line.startswith('scalar '):
                            continue
                        parts = line.strip().split()
                        if len(parts) < 4:
                            continue
                        module_path = parts[1]
                        scalar_name = parts[2]
                        value = parts[3]
                        if scalar_name not in {'CordiX', 'CordiY', 'positionX', 'positionY'}:
                            continue
                        if 'loRaEndNodes[' not in module_path:
                            continue
                        try:
                            start = module_path.index('loRaEndNodes[') + len('loRaEndNodes[')
                            end = module_path.index(']', start)
                            node_index = int(module_path[start:end])
                        except (ValueError, IndexError):
                            continue
                        node_id = 1000 + node_index
                        try:
                            coord_value = float(value)
                        except ValueError:
                            continue
                        x, y = self._positions.get(node_id, (None, None))
                        if scalar_name in {'CordiX', 'positionX'}:
                            x = coord_value
                        else:
                            y = coord_value
                        self._positions[node_id] = (x, y)
            except OSError:
                continue

    def get(self, node_id: int) -> Optional[Tuple[float, float]]:
        coords = self._positions.get(node_id)
        if not coords:
            return None
        if coords[0] is None or coords[1] is None:
            return None
        return coords


def euclidean_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def compute_report(
    rows: List[Row],
    positions: Optional[Positions] = None,
    run_index: Optional[int] = None,
) -> List[Dict[str, Optional[float]]]:
    by_triplet: Dict[Tuple[int, int, int], List[Row]] = defaultdict(list)
    for row in rows:
        by_triplet[(row.src, row.dst, row.packetSeq)].append(row)

    for key in by_triplet:
        by_triplet[key].sort(key=lambda r: r.simTime)

    pairs = sorted({(src, dst) for (src, dst, _seq) in by_triplet.keys() if src >= 1000 and dst >= 1000})

    report_rows: List[Dict[str, Optional[float]]] = []
    for (src, dst) in pairs:
        tx_events = [
            r
            for (s, d, _seq), lst in by_triplet.items()
            if s == src and d == dst
            for r in lst
            if r.event == 'TX_SRC'
        ]
        if not tx_events:
            continue
        first_tx = min(tx_events, key=lambda r: r.simTime)
        seq_rows = by_triplet.get((src, dst, first_tx.packetSeq), [])
        first_delivered = next(
            (r for r in seq_rows if r.event == 'DELIVERED' and r.currentNode == dst),
            None,
        )

        transit_time = None
        hop_count = None
        copies_received = 0
        if first_delivered:
            transit_time = first_delivered.simTime - first_tx.simTime
            hop_count = first_tx.ttlAfterDecr - first_delivered.ttlAfterDecr
            copies_received = sum(1 for r in seq_rows if r.event == 'DELIVERED' and r.currentNode == dst)
        else:
            copies_received = sum(
                1
                for (s, d, _seq), lst in by_triplet.items()
                if s == src and d == dst
                for r in lst
                if r.event == 'DELIVERED' and r.currentNode == dst
            )

        distance_m = None
        if positions is not None:
            p_src = positions.get(src)
            p_dst = positions.get(dst)
            if p_src and p_dst:
                distance_m = euclidean_distance(p_src, p_dst)

        report_rows.append(
            {
                'run_index': run_index,
                'src': src,
                'dst': dst,
                'distance_m': round(distance_m, 2) if distance_m is not None else None,
                'delivered': 1 if copies_received > 0 else 0,
                'transit_time_s': round(transit_time, 6) if transit_time is not None else None,
                'first_packet_hop_count': hop_count,
                'copies_received_at_dst_for_first_packet': copies_received,
                'first_tx_time_s': round(first_tx.simTime, 6),
                'first_delivery_time_s': round(first_delivered.simTime, 6) if first_delivered else None,
            }
        )

    return report_rows


def write_rows(path: str, rows_to_write: List[Dict[str, Optional[float]]], append: bool) -> None:
    if not path:
        return
    out_dir = os.path.dirname(path) or '.'
    os.makedirs(out_dir, exist_ok=True)
    file_exists = os.path.exists(path)
    mode = 'a' if append and file_exists else 'w'
    fieldnames = [
        'run_index',
        'src',
        'dst',
        'distance_m',
        'delivered',
        'transit_time_s',
        'first_packet_hop_count',
        'copies_received_at_dst_for_first_packet',
        'first_tx_time_s',
        'first_delivery_time_s',
    ]
    try:
        with open(path, mode, newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            if mode == 'w' or not file_exists:
                writer.writeheader()
            for row in rows_to_write:
                writer.writerow(row)
    except PermissionError:
        # Fallback: write to a new file to avoid lock/permission issues
        base, ext = os.path.splitext(os.path.basename(path))
        fallback = os.path.join(out_dir, f"{base}_fallback{ext or '.csv'}")
        with open(fallback, 'a' if append else 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            if not os.path.exists(fallback) or not append:
                writer.writeheader()
            for row in rows_to_write:
                writer.writerow(row)
        print(f"Permission denied for '{path}', wrote to '{fallback}' instead.")


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate per-run pair metrics from paths.csv')
    parser.add_argument('--paths', default='delivered_packets/paths.csv', help='Path to the paths.csv file')
    parser.add_argument('--positions', default=None, help='Optional CSV containing columns id,x,y (meters)')
    parser.add_argument('--output', default='pair_report.csv', help='Aggregate CSV path (appended)')
    parser.add_argument('--sca-dir', default=None, help='Directory containing .sca files for this run')
    parser.add_argument('--run-index', type=int, default=None, help='Run index to record alongside metrics')
    parser.add_argument('--per-run-output', default=None, help='Optional CSV written for this run only (overwritten)')
    args = parser.parse_args()

    rows = read_paths_csv(args.paths)

    positions: Optional[Positions] = None
    if args.positions:
        positions = Positions()
        positions.load_csv(args.positions)
    else:
        # Try to locate scalars automatically when --sca-dir isn't provided
        candidate_dirs: List[Path] = []
        if args.sca_dir:
            candidate_dirs.append(Path(args.sca_dir))
        # common defaults relative to CWD
        candidate_dirs.extend([
            Path('results'),
            Path('.') / 'results',
            Path('..') / 'results',
        ])
        for sca_dir in candidate_dirs:
            if sca_dir.exists():
                files = sorted(sca_dir.glob('*.sca'))
                if files:
                    # Prefer the most recent .sca file to reflect current run
                    try:
                        latest_file = max(files, key=lambda p: p.stat().st_mtime)
                        files_to_load = [latest_file]
                    except Exception:
                        files_to_load = files
                    positions = Positions()
                    positions.load_from_scalars(files_to_load)
                    break

    report_rows = compute_report(rows, positions, run_index=args.run_index)

    write_rows(args.output, report_rows, append=True)
    if args.per_run_output:
        write_rows(args.per_run_output, report_rows, append=False)

    print(f"Wrote {len(report_rows)} rows to {args.output}")
    for row in report_rows:
        print(
            f"{row['src']}->{row['dst']}: transit={row['transit_time_s']}s, hops={row['first_packet_hop_count']}, copies={row['copies_received_at_dst_for_first_packet']}, dist={row['distance_m']}"
        )


if __name__ == '__main__':
    main()
