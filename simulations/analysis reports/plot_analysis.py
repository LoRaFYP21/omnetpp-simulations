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
# Calculate effective throughput as 1 / transit_time
data_all = df[[COL_DISTANCE, COL_TRANSIT, COL_DELIVRATE]].copy()
data_all = data_all.dropna(subset=[COL_DISTANCE])
successes = data_all[data_all[COL_DELIVRATE] > 0].dropna(subset=[COL_TRANSIT])
failures = data_all[data_all[COL_DELIVRATE] == 0]

# Compute effective throughput for successes
effective_throughput = 1.0 / successes[COL_TRANSIT]

plt.figure()
plt.scatter(successes[COL_DISTANCE], effective_throughput, c='blue', label='Successful')
plt.scatter(failures[COL_DISTANCE], [0]*len(failures), c='red', label='Failed', marker='o')
plt.xlabel("Distance between End Nodes (m)")
plt.ylabel("Effective Throughput (packets/s)")
plt.title("Effective Throughput vs Distance")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "throughput_vs_distance.png"), dpi=300)
# plt.show()

# ============================
# 7) End Node Positions Visualization
# ============================
# Calculate distance from grid center (10000, 10000) for each end node
grid_center_x = 10000
grid_center_y = 10000

# Extract end node positions
endnode_data = df[['endnode_1000_x', 'endnode_1000_y', 'endnode_1001_x', 'endnode_1001_y', COL_DELIVRATE]].copy()

# Calculate distances from center for both end nodes
endnode_data['node1000_dist_from_center'] = ((endnode_data['endnode_1000_x'] - grid_center_x)**2 + 
                                              (endnode_data['endnode_1000_y'] - grid_center_y)**2)**0.5
endnode_data['node1001_dist_from_center'] = ((endnode_data['endnode_1001_x'] - grid_center_x)**2 + 
                                              (endnode_data['endnode_1001_y'] - grid_center_y)**2)**0.5

# Separate successes and failures
successes = endnode_data[endnode_data[COL_DELIVRATE] > 0]
failures = endnode_data[endnode_data[COL_DELIVRATE] == 0]

plt.figure(figsize=(10, 10))

# Plot grid center
plt.plot(grid_center_x, grid_center_y, 'k+', markersize=20, markeredgewidth=3, label='Grid Center')

# Plot circle boundary (9km radius)
circle = plt.Circle((grid_center_x, grid_center_y), 9000, color='gray', fill=False, 
                     linestyle='--', linewidth=2, label='End Node Placement Circle (9km radius)')
plt.gca().add_patch(circle)

# Plot relay grid boundaries
plt.axvline(x=1000, color='orange', linestyle=':', linewidth=1.5, alpha=0.7, label='Relay Grid Boundaries')
plt.axvline(x=16000, color='orange', linestyle=':', linewidth=1.5, alpha=0.7)
plt.axhline(y=1000, color='orange', linestyle=':', linewidth=1.5, alpha=0.7)
plt.axhline(y=16000, color='orange', linestyle=':', linewidth=1.5, alpha=0.7)
plt.axhline(y=18500, color='orange', linestyle=':', linewidth=1, alpha=0.5)

# Plot successful end node pairs (connected by lines)
for _, row in successes.iterrows():
    plt.plot([row['endnode_1000_x'], row['endnode_1001_x']], 
             [row['endnode_1000_y'], row['endnode_1001_y']], 
             'b-', alpha=0.3, linewidth=0.5)
plt.scatter(successes['endnode_1000_x'], successes['endnode_1000_y'], 
            c='blue', s=50, alpha=0.6, edgecolors='darkblue', label='Successful (Node 1000)')
plt.scatter(successes['endnode_1001_x'], successes['endnode_1001_y'], 
            c='lightblue', s=50, alpha=0.6, edgecolors='blue', marker='s', label='Successful (Node 1001)')

# Plot failed end node pairs (connected by lines)
for _, row in failures.iterrows():
    plt.plot([row['endnode_1000_x'], row['endnode_1001_x']], 
             [row['endnode_1000_y'], row['endnode_1001_y']], 
             'r-', alpha=0.3, linewidth=0.5)
plt.scatter(failures['endnode_1000_x'], failures['endnode_1000_y'], 
            c='red', s=50, alpha=0.6, edgecolors='darkred', label='Failed (Node 1000)')
plt.scatter(failures['endnode_1001_x'], failures['endnode_1001_y'], 
            c='lightcoral', s=50, alpha=0.6, edgecolors='red', marker='s', label='Failed (Node 1001)')

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("End Node Placement and Delivery Success")
plt.legend(loc='upper right', fontsize=8)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.xlim(0, 20000)
plt.ylim(0, 20000)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "endnode_placement_map.png"), dpi=300)
# plt.show()

# ============================
# 8) Distance from Center vs Delivery Rate
# ============================
plt.figure(figsize=(12, 5))

# Plot for Node 1000
plt.subplot(1, 2, 1)
successes_1000 = endnode_data[endnode_data[COL_DELIVRATE] > 0]
failures_1000 = endnode_data[endnode_data[COL_DELIVRATE] == 0]

plt.scatter(successes_1000['node1000_dist_from_center'], successes_1000[COL_DELIVRATE], 
            c='blue', label='Successful', alpha=0.6)
plt.scatter(failures_1000['node1000_dist_from_center'], failures_1000[COL_DELIVRATE], 
            c='red', label='Failed', alpha=0.6)
plt.axvline(x=9000, color='gray', linestyle='--', linewidth=2, label='Circle Radius (9km)')
plt.axvline(x=6500, color='orange', linestyle=':', linewidth=1.5, label='Grid Edge (~6.5km from center)')
plt.xlabel("Node 1000 Distance from Center (m)")
plt.ylabel("Delivery Rate (%)")
plt.title("Node 1000: Distance from Center vs Delivery Rate")
plt.legend()
plt.grid(True, alpha=0.3)

# Plot for Node 1001
plt.subplot(1, 2, 2)
plt.scatter(successes_1000['node1001_dist_from_center'], successes_1000[COL_DELIVRATE], 
            c='blue', label='Successful', alpha=0.6)
plt.scatter(failures_1000['node1001_dist_from_center'], failures_1000[COL_DELIVRATE], 
            c='red', label='Failed', alpha=0.6)
plt.axvline(x=9000, color='gray', linestyle='--', linewidth=2, label='Circle Radius (9km)')
plt.axvline(x=6500, color='orange', linestyle=':', linewidth=1.5, label='Grid Edge (~6.5km from center)')
plt.xlabel("Node 1001 Distance from Center (m)")
plt.ylabel("Delivery Rate (%)")
plt.title("Node 1001: Distance from Center vs Delivery Rate")
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "distance_from_center_vs_delivery.png"), dpi=300)
# plt.show()

print(f"Done. Plots saved in: {OUTPUT_DIR}/")
