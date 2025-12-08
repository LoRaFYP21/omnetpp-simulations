import csv
import argparse
import math
from collections import defaultdict, namedtuple
from typing import Dict, List, Tuple, Optional

Row = namedtuple('Row', ['simTime','event','packetSeq','src','dst','currentNode','ttlAfterDecr','chosenVia','nextHopType'])

def read_paths_csv(path: str) -> List[Row]:
    rows: List[Row] = []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for d in r:
            rows.append(Row(
                simTime=float(d['simTime']),
                event=d['event'],
                packetSeq=int(d['packetSeq']),
                src=int(d['src']),
                dst=int(d['dst']),
                currentNode=int(d['currentNode']),
                ttlAfterDecr=int(d['ttlAfterDecr']),
                chosenVia=int(d['chosenVia']),
                nextHopType=d['nextHopType'],
            ))
    return rows

class Positions:
    def __init__(self):
        self.pos: Dict[int, Tuple[float,float]] = {}
    def load_csv(self, path: str):
        with open(path, newline='') as f:
            r = csv.DictReader(f)
            # expect columns: id,x,y (meters)
            for d in r:
                nid = int(d['id'])
                x = float(d['x'])
                y = float(d['y'])
                self.pos[nid] = (x,y)
    def get(self, nid: int) -> Optional[Tuple[float,float]]:
        return self.pos.get(nid)

def euclidean_distance(p1: Tuple[float,float], p2: Tuple[float,float]) -> float:
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    return math.hypot(dx, dy)

def compute_report(rows: List[Row], positions: Optional[Positions]=None) -> List[Dict]:
    # Group rows by (src,dst,packetSeq)
    by_triplet: Dict[Tuple[int,int,int], List[Row]] = defaultdict(list)
    for row in rows:
        by_triplet[(row.src,row.dst,row.packetSeq)].append(row)
    # Sort each list by time
    for k in by_triplet:
        by_triplet[k].sort(key=lambda r: r.simTime)

    # Identify end-node pairs present (src>=1000 and dst>=1000)
    pairs = sorted({(src,dst) for (src,dst,seq) in by_triplet.keys() if src>=1000 and dst>=1000})

    report: List[Dict] = []
    for (src,dst) in pairs:
        # find earliest TX_SRC for this (src,dst)
        tx_rows = [r for (s,d,seq), lst in by_triplet.items() if s==src and d==dst for r in lst if r.event=='TX_SRC']
        if not tx_rows:
            continue
        first_tx = min(tx_rows, key=lambda r: r.simTime)
        # match the packetSeq of the first TX to track its first delivery path
        seq = first_tx.packetSeq
        seq_rows = by_triplet.get((src,dst,seq), [])
        first_delivered = None
        for r in seq_rows:
            if r.event == 'DELIVERED' and r.currentNode == dst:
                first_delivered = r
                break
        # transit time
        transit_time = None
        hop_count = None
        copies_received = 0
        if first_delivered:
            transit_time = first_delivered.simTime - first_tx.simTime
            # derive hop count from TTL: hops = (ttl at TX_SRC after decrement?) - (ttl at delivery)
            # We assume TX_SRC ttlAfterDecr reflects initial TTL after source decrement.
            hop_count = first_tx.ttlAfterDecr - first_delivered.ttlAfterDecr
            # count total deliveries at destination for this packetSeq (including duplicates)
            copies_received = sum(1 for r in seq_rows if r.event=='DELIVERED' and r.currentNode==dst)
        else:
            # still count any deliveries for other seqs to give total copies later
            copies_received = sum(1 for (s,d,seq2), lst in by_triplet.items() if s==src and d==dst for rr in lst if rr.event=='DELIVERED' and rr.currentNode==dst)

        # distance if available
        distance_m = None
        if positions is not None:
            p_src = positions.get(src)
            p_dst = positions.get(dst)
            if p_src and p_dst:
                distance_m = euclidean_distance(p_src, p_dst)

        report.append({
            'src': src,
            'dst': dst,
            'first_tx_time_s': round(first_tx.simTime, 6),
            'first_delivery_time_s': round(first_delivered.simTime, 6) if first_delivered else None,
            'transit_time_s': round(transit_time, 6) if transit_time is not None else None,
            'first_packet_hop_count': hop_count,
            'copies_received_at_dst_for_first_packet': copies_received,
            'distance_m': round(distance_m, 2) if distance_m is not None else None,
        })
    return report

def main():
    ap = argparse.ArgumentParser(description='Generate pair-wise report from paths.csv')
    ap.add_argument('--paths', default='delivered_packets/paths.csv', help='Path to paths.csv')
    ap.add_argument('--positions', default=None, help='Optional CSV with end-node positions: columns id,x,y (meters)')
    ap.add_argument('--output', default='pair_report.csv', help='Output CSV file')
    args = ap.parse_args()

    rows = read_paths_csv(args.paths)
    positions = None
    if args.positions:
        positions = Positions()
        positions.load_csv(args.positions)

    report = compute_report(rows, positions)

    # Write CSV
    fieldnames = ['src','dst','distance_m','first_tx_time_s','first_delivery_time_s','transit_time_s','first_packet_hop_count','copies_received_at_dst_for_first_packet']
    with open(args.output, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in report:
            w.writerow(row)

    # Also print a quick summary
    print(f'Wrote {len(report)} rows to {args.output}')
    for r in report:
        print(f"{r['src']}->{r['dst']}: transit={r['transit_time_s']}s, hops={r['first_packet_hop_count']}, copies={r['copies_received_at_dst_for_first_packet']}, dist={r['distance_m']}")

if __name__ == '__main__':
    main()
