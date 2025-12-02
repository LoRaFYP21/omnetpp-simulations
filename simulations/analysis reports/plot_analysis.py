import pandas as pd
import matplotlib.pyplot as plt
import os

# ============================
# CONFIG
# ============================
CSV_FILE = "analysis_reports_summary.csv"
OUTPUT_DIR = "plots"

# Create output directory if needed
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================
# LOAD DATA
# ============================
df = pd.read_csv(CSV_FILE)

# If your column names are different, change them here:
COL_DISTANCE = "distance_m"
COL_DELIVRATE = "delivery_rate_pct"
COL_AVG_HOPS = "hop_counts"  # Changed from avg_hops to match CSV
COL_TRANSIT = "avg_transit_time_s"
COL_THROUGHPUT = "throughput_packets_per_s"
COL_MIN_TRANSIT = "min_transit_time_s"
COL_MAX_TRANSIT = "max_transit_time_s"

# Helper to safely drop NaNs for a pair of columns
def clean_xy(dataframe, x_col, y_col):
    return dataframe[[x_col, y_col]].dropna()

# ============================
# 1) Distance vs Delivery Rate
# ============================
data = clean_xy(df, COL_DISTANCE, COL_DELIVRATE)
successes = data[data[COL_DELIVRATE] > 0]
failures = data[data[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_DISTANCE], successes[COL_DELIVRATE], c='blue', label='Successful')
plt.scatter(failures[COL_DISTANCE], failures[COL_DELIVRATE], c='red', label='Failed', marker='o')
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Delivery Rate (%)")
plt.title("Distance vs Delivery Rate")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_delivery_rate.png"), dpi=300)
# plt.show()

# ============================
# 2) Distance vs Average Hop Count
# ============================
data_all = df[[COL_DISTANCE, COL_AVG_HOPS, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DISTANCE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_AVG_HOPS])
failures = data_all[data_all[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_DISTANCE], successes[COL_AVG_HOPS], c='blue', label='Successful')
plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Average Hop Count")
plt.title("Distance vs Average Hop Count")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_avg_hops.png"), dpi=300)
# plt.show()

# ============================
# 3) Hop Count vs Delivery Rate
# ============================
data_all = df[[COL_AVG_HOPS, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DELIVRATE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_AVG_HOPS])
failures = data_all[data_all[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_AVG_HOPS], successes[COL_DELIVRATE], c='blue', label='Successful')
plt.scatter([0]*len(failures), failures[COL_DELIVRATE], c='red', label='Failed', marker='o')
plt.xlabel("Average Hop Count")
plt.ylabel("Delivery Rate (%)")
plt.title("Hop Count vs Delivery Rate")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "hops_vs_delivery_rate.png"), dpi=300)
# plt.show()

# ============================
# 4) Distance vs Transit Time
# ============================
data_all = df[[COL_DISTANCE, COL_TRANSIT, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DISTANCE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_TRANSIT])
failures = data_all[data_all[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_DISTANCE], successes[COL_TRANSIT], c='blue', label='Successful')
plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Average Transit Time (s)")
plt.title("Distance vs Transit Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_transit_time.png"), dpi=300)
# plt.show()

# ============================
# 5) Hop Count vs Transit Time
# ============================
data_all = df[[COL_AVG_HOPS, COL_TRANSIT, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DELIVRATE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_AVG_HOPS, COL_TRANSIT])
failures = data_all[data_all[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_AVG_HOPS], successes[COL_TRANSIT], c='blue', label='Successful')
plt.scatter([0]*len(failures), [0]*len(failures), c='red', label='Failed', marker='o')
plt.xlabel("Average Hop Count")
plt.ylabel("Average Transit Time (s)")
plt.title("Hop Count vs Transit Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "hops_vs_transit_time.png"), dpi=300)
# plt.show()

# ============================
# 6) Throughput vs Distance
# ============================
data_all = df[[COL_DISTANCE, COL_THROUGHPUT, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DISTANCE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_THROUGHPUT])
failures = data_all[data_all[COL_DELIVRATE] == 0]

plt.figure()
plt.scatter(successes[COL_DISTANCE], successes[COL_THROUGHPUT], c='blue', label='Successful')
plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Throughput (packets/s)")
plt.title("Throughput vs Distance")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "throughput_vs_distance.png"), dpi=300)
# plt.show()

# ============================
# 7) Distance vs Min Transit Time
# ============================
if COL_MIN_TRANSIT in df.columns:
    data_all = df[[COL_DISTANCE, COL_MIN_TRANSIT, COL_DELIVRATE]].copy()
    data_all = data_all.dropna(subset=[COL_DISTANCE])
    successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_MIN_TRANSIT])
    failures = data_all[data_all[COL_DELIVRATE] == 0]

    plt.figure()
    plt.scatter(successes[COL_DISTANCE], successes[COL_MIN_TRANSIT], c='blue', label='Successful')
    plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
    plt.xlabel("Distance between End Nodes (m)")
    plt.ylabel("Min Transit Time (s)")
    plt.title("Distance vs Min Transit Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_min_transit_time.png"), dpi=300)
    # plt.show()

# ============================
# 8) Distance vs Max Transit Time
# ============================
if COL_MAX_TRANSIT in df.columns:
    data_all = df[[COL_DISTANCE, COL_MAX_TRANSIT, COL_DELIVRATE]].copy()
    data_all = data_all.dropna(subset=[COL_DISTANCE])
    successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_MAX_TRANSIT])
    failures = data_all[data_all[COL_DELIVRATE] == 0]

    plt.figure()
    plt.scatter(successes[COL_DISTANCE], successes[COL_MAX_TRANSIT], c='blue', label='Successful')
    plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
    plt.xlabel("Distance between End Nodes (m)")
    plt.ylabel("Max Transit Time (s)")
    plt.title("Distance vs Max Transit Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_max_transit_time.png"), dpi=300)
    # plt.show()

print(f"Done. Plots saved in: {OUTPUT_DIR}/")
