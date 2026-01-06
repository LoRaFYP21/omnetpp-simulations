"""
Batch-run simulations and collect outputs.

Usage examples:
1) Sequential runs (user supplies command template with {run} placeholder):
   python run_simulations.py --cmd "path\\to\\flora.exe -u Cmdenv -c Congestion_Flooding_4Pairs -r {run}" --runs 100

2) If your INI uses environment variable for seed (e.g. seed-set = ${env(RUN_SEED)}), supply env_template and the runner will set it:
   python run_simulations.py --cmd "path\\to\\flora.exe -u Cmdenv -c Congestion_Flooding_4Pairs" --runs 100 --env-template RUN_SEED={run}

3) Run in parallel (max_workers):
   python run_simulations.py --cmd "..." --runs 100 --max-workers 8

What it does:
- Runs the provided command (formatted with `run` index if present) per repetition index.
- After each run, it copies `delivered_packets/paths.csv` into `results/run_{i}/paths.csv` (creates results folder).
- Optionally runs `paths_report.py` on each run and/or aggregates reports into `results/aggregate_pairs.csv`.

Notes:
- Provide the exact command template appropriate for your simulation environment. The script will not try to guess OMNeT++ flags.
- If your simulation creates other outputs you want preserved, extend the `collect_files` list in the script.

"""
import argparse
import subprocess
import shutil
import os
import shlex
import time
import glob
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List


def _resolve_output_path(base_dir: str, path: str) -> str:
    """Return absolute path for output artifacts."""
    if os.path.isabs(path):
        return path
    return os.path.join(base_dir, path)

DEFAULT_COLLECT = ["delivered_packets/paths.csv"]


def run_one(
    cmd_template,
    run_index,
    env_template,
    workdir,
    collect_files,
    analysis_script,
    output_dir,
    aggregate_report_path=None,
):
    cmd = cmd_template.format(run=run_index)
    env = os.environ.copy()
    if env_template:
        for kv in env_template.split(','):
            if '=' in kv:
                k, v = kv.split('=', 1)
                env[k] = v.format(run=run_index)
    print(f"[run {run_index}] exec: {cmd}")
    
    # prepare run folder early
    run_folder = os.path.join(output_dir, f"run_{run_index}")
    os.makedirs(run_folder, exist_ok=True)
    
    # Build command with OMNeT++ repetition index to vary RNG/state
    # If the command already contains a repetition flag, we don't add ours.
    # Parse command safely and run without shell so flora.exe is the child process
    formatted_cmd = cmd_template.format(run=run_index)
    repetition_flag_present = any(arg in {"-r", "--repetition"} for arg in shlex.split(formatted_cmd, posix=False))
    run_cmd = shlex.split(formatted_cmd, posix=False)
    if not repetition_flag_present:
        run_cmd += ["-r", str(run_index)]

    # capture stdout/stderr to log
    log_file = os.path.join(run_folder, "run.log")
    run_started_at = time.time()
    with open(log_file, 'w') as log:
        p = subprocess.run(run_cmd, shell=False, cwd=workdir, env=env, stdout=log, stderr=subprocess.STDOUT)
    success = (p.returncode == 0)
    
    # collect files IMMEDIATELY after sim finishes (before paths.csv is overwritten)
    for rel in collect_files:
        src = os.path.join(workdir, rel)
        if os.path.exists(src):
            dst = os.path.join(run_folder, os.path.basename(rel))
            shutil.copy2(src, dst)

    # Also collect scalar/vector files (*.sca, *.vec) produced by OMNeT++ so analysis can extract coordinates
    # Poll for up to ~8 seconds for these files (some simulators flush at exit)
    search_patterns = [
        os.path.join(workdir, "*.sca"),
        os.path.join(workdir, "*.vec"),
        os.path.join(workdir, "results", "*.sca"),
        os.path.join(workdir, "results", "*.vec"),
    ]
    wait_secs = 8.0
    poll_interval = 0.5
    attempts = max(1, int(wait_secs / poll_interval))
    collected: List[str] = []
    for _attempt in range(attempts):
        candidate: List[str] = []
        for pattern in search_patterns:
            for path in glob.glob(pattern):
                try:
                    mtime = os.path.getmtime(path)
                except OSError:
                    continue
                if mtime >= run_started_at - 1.0:
                    candidate.append(path)
        if candidate:
            collected = candidate
            break
        time.sleep(poll_interval)

    for f in collected:
        try:
            shutil.copy2(f, os.path.join(run_folder, os.path.basename(f)))
        except Exception:
            pass
    
    # optional: run per-run analysis on the COPIED paths.csv
    if analysis_script and os.path.exists(os.path.join(workdir, analysis_script)):
        # Adjust command for endnode_distance_tracker.py arguments
        # It expects: --paths <file> --output <file> --results-dir <dir>
        # We'll use the copied paths.csv and point results-dir to workdir (where .sca files are)
        analysis_output = os.path.join(run_folder, f"analysis_report_run_{run_index}.txt")
        analysis_script_path = os.path.join(workdir, analysis_script)
        paths_arg = os.path.join(run_folder, "paths.csv")
        # Skip running analysis if paths.csv is missing for this run
        if os.path.exists(paths_arg):
            cmd = [
                "python",
                analysis_script_path,
                "--paths",
                paths_arg,
                "--output",
                aggregate_report_path or analysis_output,
                "--run-index",
                str(run_index),
                "--sca-dir",
                run_folder,
                "--per-run-output",
                os.path.join(run_folder, "pair_report.csv"),
            ]
            if aggregate_report_path is None:
                # Fall back to per-run file when aggregate is not supplied
                cmd = [
                    "python",
                    analysis_script_path,
                    "--paths",
                    paths_arg,
                    "--output",
                    analysis_output,
                    "--run-index",
                    str(run_index),
                    "--sca-dir",
                    run_folder,
                    "--per-run-output",
                    os.path.join(run_folder, "pair_report.csv"),
                ]
            print(f"[run {run_index}] running analysis: {' '.join(cmd)}")
            subprocess.run(cmd, shell=False, cwd=workdir)
    return run_index, success


def main():
    ap = argparse.ArgumentParser(description='Run simulation batch and collect outputs')
    ap.add_argument('--cmd', required=True, help='Command template to run per simulation. Use {run} where you want the repetition index interpolated.')
    ap.add_argument('--runs', type=int, default=10, help='Number of runs (1..N)')
    ap.add_argument('--start', type=int, default=0, help='Start index (inclusive)')
    ap.add_argument('--workdir', default='.', help='Working directory where simulation writes outputs')
    ap.add_argument('--output-dir', default='results', help='Directory to collect per-run outputs')
    ap.add_argument('--collect', default=','.join(DEFAULT_COLLECT), help='Comma-separated paths (relative to workdir) to collect after each run')
    ap.add_argument('--max-workers', type=int, default=1, help='Parallel worker threads (set >1 to run in parallel)')
    ap.add_argument('--env-template', default=None, help='Optional comma-separated ENVVAR=template entries, use {run} in template')
    ap.add_argument('--analysis-script', default='paths_report.py', help='Optional analysis script to run per-run against collected paths.csv')
    ap.add_argument('--aggregate-report', default='pair_report.csv', help='Filename (absolute or relative to output-dir) for appended per-run metrics')
    args = ap.parse_args()

    collect_files = [p.strip() for p in args.collect.split(',') if p.strip()]
    os.makedirs(args.output_dir, exist_ok=True)

    aggregate_report_path = None
    if args.analysis_script:
        resolved = _resolve_output_path(args.output_dir, args.aggregate_report)
        os.makedirs(os.path.dirname(resolved), exist_ok=True)
        aggregate_report_path = resolved

    indexes = list(range(args.start, args.start + args.runs))

    results = []
    if args.max_workers <= 1:
        for i in indexes:
            print(f"Starting run {i}")
            res = run_one(
                args.cmd,
                i,
                args.env_template,
                args.workdir,
                collect_files,
                args.analysis_script,
                args.output_dir,
                aggregate_report_path=aggregate_report_path,
            )
            results.append(res)
            # Small stagger between runs to ensure distinct OS PIDs/time-based behaviors
            time.sleep(0.2)
    else:
        with ThreadPoolExecutor(max_workers=args.max_workers) as ex:
            futures = {
                ex.submit(
                    run_one,
                    args.cmd,
                    i,
                    args.env_template,
                    args.workdir,
                    collect_files,
                    args.analysis_script,
                    args.output_dir,
                    aggregate_report_path,
                ): i
                for i in indexes
            }
            for f in as_completed(futures):
                i, success = f.result()
                results.append((i, success))

    # summary
    succ = sum(1 for r in results if r[1])
    print(f"Completed {len(results)} runs, {succ} succeeded, {len(results)-succ} failed")

if __name__ == '__main__':
    main()
