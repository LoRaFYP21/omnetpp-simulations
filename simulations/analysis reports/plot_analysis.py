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

# Helper to safely drop NaNs for a pair of columns
def clean_xy(dataframe, x_col, y_col):
    return dataframe[[x_col, y_col]].dropna()

# ============================
# 1) Distance vs Delivery Rate
# ============================
data = clean_xy(df, COL_DISTANCE, COL_DELIVRATE)

plt.figure()
plt.scatter(data[COL_DISTANCE], data[COL_DELIVRATE])
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Delivery Rate (%)")
plt.title("Distance vs Delivery Rate")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_delivery_rate.png"), dpi=300)
# plt.show()

# ============================
# 2) Distance vs Average Hop Count
# ============================
data = clean_xy(df, COL_DISTANCE, COL_AVG_HOPS)

plt.figure()
plt.scatter(data[COL_DISTANCE], data[COL_AVG_HOPS])
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Average Hop Count")
plt.title("Distance vs Average Hop Count")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_avg_hops.png"), dpi=300)
# plt.show()

# ============================
# 3) Hop Count vs Delivery Rate
# ============================
data = clean_xy(df, COL_AVG_HOPS, COL_DELIVRATE)

plt.figure()
plt.scatter(data[COL_AVG_HOPS], data[COL_DELIVRATE])
plt.xlabel("Average Hop Count")
plt.ylabel("Delivery Rate (%)")
plt.title("Hop Count vs Delivery Rate")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "hops_vs_delivery_rate.png"), dpi=300)
# plt.show()

# ============================
# 4) Distance vs Transit Time
# ============================
data = clean_xy(df, COL_DISTANCE, COL_TRANSIT)

plt.figure()
plt.scatter(data[COL_DISTANCE], data[COL_TRANSIT])
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Average Transit Time (s)")
plt.title("Distance vs Transit Time")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_vs_transit_time.png"), dpi=300)
# plt.show()

# ============================
# 5) Hop Count vs Transit Time
# ============================
data = clean_xy(df, COL_AVG_HOPS, COL_TRANSIT)

plt.figure()
plt.scatter(data[COL_AVG_HOPS], data[COL_TRANSIT])
plt.xlabel("Average Hop Count")
plt.ylabel("Average Transit Time (s)")
plt.title("Hop Count vs Transit Time")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "hops_vs_transit_time.png"), dpi=300)
# plt.show()

# ============================
# 6) Throughput vs Distance
# ============================
data = clean_xy(df, COL_DISTANCE, COL_THROUGHPUT)

plt.figure()
plt.scatter(data[COL_DISTANCE], data[COL_THROUGHPUT])
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Throughput (packets/s)")
plt.title("Throughput vs Distance")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "throughput_vs_distance.png"), dpi=300)
# plt.show()

print(f"Done. Plots saved in: {OUTPUT_DIR}/")
