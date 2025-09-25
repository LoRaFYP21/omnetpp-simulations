#!/usr/bin/env python3
"""Random Failure Analysis

Advanced statistics on node failures across one or multiple runs.

Features:
- Reads failures_global.csv if present; falls back to parsing *.sca
- Computes inter-failure times, mean, stddev, coefficient of variation
- Estimates exponential MLE (lambda) and compares empirical vs model CDF
- Computes Kaplan-Meier survival estimate from failure times
- Outputs:
    random_failure_stats.txt
    failure_intervals.csv (intervalIndex, startTime, endTime, intervalDuration)
    survival_km.csv (time, survival)
    empirical_vs_exponential.csv (time, empirical_cdf, exponential_cdf)

Usage:
  python analysis/random_failure_analysis.py --results results --out analysis
"""
import argparse, os, glob, re, math
from collections import defaultdict

DEF_FAIL_CSV = 'results/failures_global.csv'

scalar_line_re = re.compile(r'^scalar +(\S+) +(failureTime) +(\S+)')
run_line_re = re.compile(r'^run +(.+)$')
repr_re = re.compile(r'-r +(\d+)')
NODE_TAG = '.loRaNodes['

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default='results')
    ap.add_argument('--out', default='analysis')
    ap.add_argument('--keep-duplicates', action='store_true', help='Do not deduplicate failureTime scalars')
    return ap.parse_args()

def read_failures_from_csv(path):
    failures = []
    if not os.path.exists(path):
        return failures
    with open(path,'r') as f:
        header = True
        for line in f:
            if header:
                header = False
                continue
            parts = line.strip().split(',')
            if len(parts) != 2:
                continue
            t, node = parts
            try:
                failures.append((int(node), float(t), 0)) # rep=0
            except ValueError:
                pass
    return failures

def read_failures_from_sca(results_dir):
    failures = []
    for path in glob.glob(os.path.join(results_dir,'*.sca')):
        repetition = 0
        with open(path,'r') as f:
            for line in f:
                line = line.strip()
                mrun = run_line_re.match(line)
                if mrun:
                    rm = repr_re.search(line)
                    if rm:
                        repetition = int(rm.group(1))
                    continue
                m = scalar_line_re.match(line)
                if not m:
                    continue
                module, _, value = m.groups()
                if NODE_TAG not in module:
                    continue
                node = int(module.split(NODE_TAG)[1].split(']')[0])
                failures.append((node, float(value), repetition))
    return failures

def km_survival(times):
    # Kaplan-Meier (no censoring assumed here)
    times_sorted = sorted(times)
    n = len(times_sorted)
    surv = 1.0
    km = []
    prev_t = 0.0
    for i, t in enumerate(times_sorted, 1):
        # probability of surviving past this failure
        surv *= (1 - 1/ (n - (i-1)))
        km.append((t, surv))
        prev_t = t
    return km

def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    failures = read_failures_from_csv(DEF_FAIL_CSV)
    if not failures:
        failures = read_failures_from_sca(args.results)
    if not failures:
        print('No failures detected.')
        return

    # Deduplicate: same (rep,node,time) may appear more than once from multiple scalar sources
    dedup = []
    seen = set()
    dropped = 0
    for node, t, rep in failures:
        key = (rep, node, round(t, 9))
        if not args.keep_duplicates and key in seen:
            dropped += 1
            continue
        seen.add(key)
        dedup.append((node, t, rep))

    # Group by repetition
    by_rep = defaultdict(list)
    for node, t, rep in dedup:
        by_rep[rep].append((node,t))

    stats_path = os.path.join(args.out,'random_failure_stats.txt')
    with open(stats_path,'w') as stats:
        for rep in sorted(by_rep.keys()):
            stats.write(f'REPETITION {rep}\n')
            series = sorted(by_rep[rep], key=lambda x: x[1])
            times = [t for _,t in series]
            if len(times) < 2:
                stats.write('  Not enough failures for statistics.\n\n')
                continue
            # Inter-failure intervals
            intervals = [times[i]-times[i-1] for i in range(1,len(times))]
            mean_iv = sum(intervals)/len(intervals)
            var_iv = sum((x-mean_iv)**2 for x in intervals)/(len(intervals)-1) if len(intervals)>1 else 0
            std_iv = math.sqrt(var_iv)
            cov_iv = std_iv/mean_iv if mean_iv>0 else 0
            # Exponential MLE lambda
            lambda_hat = 1/mean_iv if mean_iv>0 else 0
            stats.write(f'  Failures: {len(times)}\n')
            stats.write(f'  First failure time: {times[0]:.3f}\n')
            stats.write(f'  Last failure time: {times[-1]:.3f}\n')
            stats.write(f'  Mean inter-failure interval: {mean_iv:.3f}\n')
            stats.write(f'  Std inter-failure interval: {std_iv:.3f}\n')
            stats.write(f'  CoV inter-failure interval: {cov_iv:.3f}\n')
            stats.write(f'  Exponential lambda_hat: {lambda_hat:.6f}\n')
            stats.write(f'  MTBF (mean interval): {mean_iv:.3f}\n')
            stats.write('\n')
            # Write intervals CSV
            with open(os.path.join(args.out,f'failure_intervals_rep{rep}.csv'),'w') as ivf:
                ivf.write('intervalIndex,startTime,endTime,intervalDuration\n')
                for i,(start,end) in enumerate(zip(times[:-1], times[1:]),1):
                    ivf.write(f'{i},{start:.6f},{end:.6f},{(end-start):.6f}\n')
            # Empirical CDF vs exponential
            with open(os.path.join(args.out,f'empirical_vs_exponential_rep{rep}.csv'),'w') as cf:
                cf.write('time,empirical_cdf,exponential_cdf\n')
                span = times[-1]-times[0]
                grid = 100
                for g in range(grid+1):
                    tt = times[0] + span*(g/grid)
                    emp = sum(1 for x in times if x <= tt)/len(times)
                    # shift exponential to first failure time for comparison of spacing pattern
                    exp_cdf = 1 - math.exp(-lambda_hat*(tt-times[0])) if tt>=times[0] else 0
                    cf.write(f'{tt:.6f},{emp:.6f},{exp_cdf:.6f}\n')
            # Kaplan-Meier
            km = km_survival(times)
            with open(os.path.join(args.out,f'survival_km_rep{rep}.csv'),'w') as kmf:
                kmf.write('time,survival\n')
                for t,s in km:
                    kmf.write(f'{t:.6f},{s:.6f}\n')
    if dropped>0 and not args.keep_duplicates:
        with open(stats_path,'a') as stats:
            stats.write(f'Done.\n')
    print('Random failure analysis complete. See analysis/ outputs.')

if __name__ == '__main__':
    main()
