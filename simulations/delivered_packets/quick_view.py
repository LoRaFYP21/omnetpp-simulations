#!/usr/bin/env python3
"""Quick preview of live simulation results"""

import pandas as pd
import os

os.chdir(r'e:\omnetpp-simulations\simulations\delivered_packets')

print("="*60)
print("LIVE SIMULATION RESULTS - Smart Flooding")
print("="*60)

# Check packets delivered to destination
try:
    delivered = pd.read_csv('node_1001_delivered.csv')
    unique_packets = delivered['seq'].nunique()
    total_deliveries = len(delivered)
    
    print(f"\n📦 Packets delivered to node 1001:")
    print(f"   Unique packets: {unique_packets}")
    print(f"   Total deliveries (including duplicates): {total_deliveries}")
    print(f"   Duplicate ratio: {total_deliveries/unique_packets:.2f}x")
except Exception as e:
    print(f"Error reading delivered packets: {e}")

# Check forwarding activity
try:
    paths = pd.read_csv('paths.csv')
    
    # Count transmissions
    tx_src = len(paths[paths['event'] == 'TX_SRC'])
    tx_fwd = len(paths[paths['event'] == 'TX_FWD_DATA'])
    
    print(f"\n🔄 Forwarding Activity:")
    print(f"   Source transmissions: {tx_src}")
    print(f"   Relay forwards: {tx_fwd}")
    print(f"   Total hops: {len(paths)}")
    print(f"   Avg hops per packet: {tx_fwd/tx_src:.1f}x")
    
    # Show latest packets
    print(f"\n📊 Latest packet sequences transmitted:")
    latest = paths[paths['event'] == 'TX_SRC'].tail(5)
    for _, row in latest.iterrows():
        print(f"   Seq {int(row['packetSeq']):3d} @ t={row['simTime']:.2f}s")
        
except Exception as e:
    print(f"Error reading paths: {e}")

print("\n" + "="*60)
print("Simulation still running... refresh to see updates!")
print("="*60)
