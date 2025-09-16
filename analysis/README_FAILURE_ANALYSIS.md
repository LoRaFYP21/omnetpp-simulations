# Failure & Resilience Analysis

## Instrumentation Summary
- Scalars: `failed`, `failureTime`, routing & delivery metrics.
- Vector: `activeState` (1 while alive, 0 at failure) per node.
- Global CSV: `results/failures_global.csv` (simTime,nodeId).
- Routing table exports: `routing_tables/node<ID>_*.{csv,txt}` at finish.

## Standard Analysis Script
`failure_analysis.py` (previously provided):
```powershell
python analysis/failure_analysis.py
```
Outputs: failures summary, cumulative failures, survival fraction, PDR summary, partition candidates.

## Advanced Random Failure Statistics
Run:
```powershell
python analysis/random_failure_analysis.py
```
Outputs per repetition:
- `random_failure_stats.txt` (mean / std / CoV inter-failure intervals, λ MLE)
- `failure_intervals_repX.csv`
- `empirical_vs_exponential_repX.csv` (empirical vs exponential CDF)
- `survival_km_repX.csv` (Kaplan–Meier estimate)

Interpretation:
- CoV ≈ 1: Failures resemble exponential (memoryless) process.
- CoV < 1: Failures too regular (deterministic-like scheduling).
- CoV > 1: Failures clustered/bursty; consider mixture or heavy-tail.

## Configuring Random Failures
Deterministic (all around same time with jitter):
```
**.loRaNodes[*].LoRaNodeApp.timeToFailure = 400s
**.loRaNodes[*].LoRaNodeApp.failureJitterFrac = 0.2
```
Memoryless (recommended for random independent failures):
```
**.loRaNodes[*].LoRaNodeApp.timeToFailure = exponential(400s)
```
Different failure rates per group:
```
**.loRaNodes[0..19].LoRaNodeApp.timeToFailure = exponential(300s)
**.loRaNodes[20..99].LoRaNodeApp.timeToFailure = exponential(600s)
```
Subset only:
```
**.loRaNodes[0..9].LoRaNodeApp.timeToFailure = exponential(500s)
**.loRaNodes[10..*.LoRaNodeApp.timeToFailure = -1s   # disable others
```
(Use explicit upper index instead of * if parser requires.)

## Survival Curve Quick Plot
```python
import pandas as pd, matplotlib.pyplot as plt
surv = pd.read_csv('analysis/survival_km_rep0.csv')
plt.plot(surv.time, surv.survival); plt.xlabel('Time (s)'); plt.ylabel('KM Survival'); plt.show()
```

## Partition Diagnosis
Check `partition_candidates.txt` from standard script: surviving nodes whose `lastDataPacketReceptionTime` < maximum failure time likely became isolated before all failures completed.

## Next Enhancements (Optional)
- Periodic routing snapshots (add timer calling logRoutingSnapshot).
- Energy vs failure correlation (record remaining energy).
- Recovery events (add restart scheduling symmetric to failure).

## Troubleshooting
- Empty outputs: Ensure failure parameters are not commented and > simulation warmup.
- No failures_global.csv: No node hit its failure time (maybe simulation ended earlier).
- Irregular vector recording: Confirm `**.vector-recording = true` in `.ini`.

---
Generated as part of automated failure analysis tooling.
