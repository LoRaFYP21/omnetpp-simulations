#!/usr/bin/env python3
"""Failure & Routing Impact Analysis

Generates:
  failures_summary.txt
  failures_curve.csv
  survival_curve.csv
  pdr_summary.csv
  partition_candidates.txt

Usage:
  python analysis/failure_analysis.py [--results results] [--out analysis] [--buckets 20]
"""
import argparse, re, glob, os, csv
from collections import defaultdict, namedtuple

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default='results')
    ap.add_argument('--out', default='analysis')
    ap.add_argument('--buckets', type=int, default=20)
    ap.add_argument('--keep-duplicates', action='store_true', help='Do not deduplicate failureTime scalars')
    return ap.parse_args()

Scalar = namedtuple('Scalar','module name value repetition')
Failure = namedtuple('Failure','node time repetition')

scalar_line_re = re.compile(r'^scalar +(\S+) +(\S+) +(\S+)')
run_line_re = re.compile(r'^run +(.+)$')
repr_re = re.compile(r'-r +(\d+)')
NODE_TAG = '.loRaNodes['
SCALARS = { 'failed','failureTime','sentDataPackets','receivedDataPacketsForMe',
            'routingTableSizeMean','lastDataPacketReceptionTime'}

def load_scalars(results_dir):
    scalars = []
    for path in glob.glob(os.path.join(results_dir,'*.sca')):
        rep = 0
        with open(path,'r') as f:
            for line in f:
                line=line.strip()
                if not line: continue
                mrun = run_line_re.match(line)
                if mrun:
                    rm = repr_re.search(line)
                    if rm: rep = int(rm.group(1))
                    continue
                m = scalar_line_re.match(line)
                if not m: continue
                module,name,value = m.groups()
                if name in SCALARS:
                    scalars.append(Scalar(module,name,value,rep))
    return scalars

def extract_failures(scalars, keep_duplicates=False):
    failures = []
    # Some simulations emit duplicate failureTime scalars for the same node
    # (e.g. multiple components reporting the same event). Deduplicate on
    # (repetition, node, rounded_time) to avoid duplicate lines in summary.
    seen = set()
    dropped = 0
    for s in scalars:
        if s.name == 'failureTime' and NODE_TAG in s.module:
            node = int(s.module.split(NODE_TAG)[1].split(']')[0])
            t = float(s.value)
            key = (s.repetition, node, round(t, 9))
            if not keep_duplicates and key in seen:
                dropped += 1
                continue
            seen.add(key)
            failures.append(Failure(node, t, s.repetition))
    failures.sort(key=lambda f:(f.repetition,f.time))
    return failures, dropped

def write_failures_summary(out_dir, failures):
    by_rep = defaultdict(list)
    for f in failures:
        by_rep[f.repetition].append(f)
    with open(os.path.join(out_dir,'failures_summary.txt'),'w') as fh:
        for rep in sorted(by_rep):
            fh.write(f'Repetition {rep}\n')
            for i,f in enumerate(by_rep[rep],1):
                fh.write(f' {i:3d} node {f.node} time {f.time:.3f}\n')
            fh.write('\n')

def build_curves(out_dir, failures, buckets):
    by_rep = defaultdict(list)
    for f in failures:
        by_rep[f.repetition].append(f)
    with open(os.path.join(out_dir,'failures_curve.csv'),'w',newline='') as fc,\
         open(os.path.join(out_dir,'survival_curve.csv'),'w',newline='') as sc:
        fcw = csv.writer(fc); scw = csv.writer(sc)
        fcw.writerow(['repetition','time','cumulativeFailures'])
        scw.writerow(['repetition','time','survivalFraction'])
        for rep, lst in sorted(by_rep.items()):
            times=[f.time for f in lst]
            if not times: continue
            max_t=max(times)
            step = max_t/buckets if max_t>0 else max_t
            for b in range(buckets+1):
                t=b*step
                count=sum(1 for x in times if x<=t)
                surv=(len(times)-count)/len(times)
                fcw.writerow([rep,f'{t:.6f}',count])
                scw.writerow([rep,f'{t:.6f}',f'{surv:.6f}'])

def extract_pdr(out_dir, scalars):
    metrics=defaultdict(lambda:{'sent':0,'recv':0,'lastRx':'','rtSize':''})
    for s in scalars:
        if NODE_TAG not in s.module: continue
        node=int(s.module.split(NODE_TAG)[1].split(']')[0])
        key=(s.repetition,node)
        if s.name=='sentDataPackets': metrics[key]['sent']=int(float(s.value))
        elif s.name=='receivedDataPacketsForMe': metrics[key]['recv']=int(float(s.value))
        elif s.name=='lastDataPacketReceptionTime': metrics[key]['lastRx']=s.value
        elif s.name=='routingTableSizeMean': metrics[key]['rtSize']=s.value
    with open(os.path.join(out_dir,'pdr_summary.csv'),'w',newline='') as fh:
        w=csv.writer(fh)
        w.writerow(['repetition','nodeId','sent','received','PDR','lastReceptionTime','routingTableSizeMean'])
        for (rep,node),vals in sorted(metrics.items()):
            sent=vals['sent']; recv=vals['recv']; pdr= (recv/sent) if sent>0 else ''
            w.writerow([rep,node,sent,recv,pdr,vals['lastRx'],vals['rtSize']])

def detect_partitions(out_dir, failures, scalars):
    last_rx={}
    failed_nodes=defaultdict(set)
    for f in failures:
        failed_nodes[f.repetition].add(f.node)
    for s in scalars:
        if s.name=='lastDataPacketReceptionTime' and NODE_TAG in s.module:
            node=int(s.module.split(NODE_TAG)[1].split(']')[0])
            last_rx[(s.repetition,node)]=float(s.value)
    max_fail_time=defaultdict(float)
    for f in failures:
        if f.time>max_fail_time[f.repetition]:
            max_fail_time[f.repetition]=f.time
    partitioned=defaultdict(list)
    for (rep,node),t in last_rx.items():
        if node not in failed_nodes[rep] and t<max_fail_time.get(rep,0):
            partitioned[rep].append(node)
    with open(os.path.join(out_dir,'partition_candidates.txt'),'w') as fh:
        for rep in sorted(partitioned):
            fh.write(f'Repetition {rep}: {sorted(partitioned[rep])}\n')

def main():
    args=parse_args()
    os.makedirs(args.out, exist_ok=True)
    scalars=load_scalars(args.results)
    failures,dropped=extract_failures(scalars, keep_duplicates=args.keep_duplicates)
    write_failures_summary(args.out, failures)
    if dropped>0 and not args.keep_duplicates:
        with open(os.path.join(args.out,'failures_summary.txt'), 'a') as fh:
            fh.write(f'Done.\n')
    build_curves(args.out, failures, args.buckets)
    extract_pdr(args.out, scalars)
    detect_partitions(args.out, failures, scalars)
    print('failure_analysis complete.')

if __name__=='__main__':
    main()
