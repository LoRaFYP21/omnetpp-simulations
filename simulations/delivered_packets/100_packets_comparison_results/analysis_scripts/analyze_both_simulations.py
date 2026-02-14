#!/usr/bin/env python3
"""
Comprehensive analysis of Smart Flooding vs Distance Vector routing
"""

import csv
from collections import defaultdict

def analyze_simulation(paths_file, node_delivered_file, sim_name):
    """Analyze a single simulation's results"""
    
    # Read paths.csv
    tx_src_count = 0
    delivered_count = 0
    total_events = 0
    event_types = defaultdict(int)
    
    with open(paths_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_events += 1
            event = row['event']
            event_types[event] += 1
            
            if event == 'TX_SRC':
                tx_src_count += 1
            elif event == 'DELIVERED':
                delivered_count += 1
    
    # Count unique delivered packets from node_1001_delivered.csv
    unique_delivered = 0
    delivered_seqs = set()
    
    try:
        with open(node_delivered_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    seq = int(row['seq'])
                    delivered_seqs.add(seq)
                except (ValueError, KeyError):
                    continue  # Skip header or invalid rows
        unique_delivered = len(delivered_seqs)
    except FileNotFoundError:
        unique_delivered = delivered_count  # Fallback
    
    # Calculate metrics
    pdr = (unique_delivered / tx_src_count * 100) if tx_src_count > 0 else 0
    events_per_packet = (total_events / tx_src_count) if tx_src_count > 0 else 0
    overhead = total_events - tx_src_count
    
    print(f"\n{'='*60}")
    print(f" {sim_name}")
    print(f"{'='*60}")
    print(f"  Packets Sent (TX_SRC):     {tx_src_count}")
    print(f"  Unique Packets Delivered:  {unique_delivered}")
    print(f"  Packet Delivery Ratio:     {pdr:.1f}%")
    print(f"  Total Routing Events:      {total_events}")
    print(f"  Network Overhead:          {overhead} events")
    print(f"  Events per Packet:         {events_per_packet:.2f}x")
    print(f"\n  Event Breakdown:")
    for event, count in sorted(event_types.items()):
        print(f"    {event:20s}: {count:6d}")
    
    return {
        'name': sim_name,
        'tx_src': tx_src_count,
        'delivered': unique_delivered,
        'pdr': pdr,
        'total_events': total_events,
        'overhead': overhead,
        'events_per_packet': events_per_packet
    }

def main():
    print("\n" + "="*60)
    print(" ROUTING PROTOCOL COMPARISON ANALYSIS")
    print(" 100 Packets: Node 1000 → Node 1001")
    print("="*60)
    
    # Analyze Smart Flooding (from backup)
    flooding_results = analyze_simulation(
        'smart_flooding_results/paths_flooding.csv',
        'smart_flooding_results/node_1001_flooding.csv',
        'SMART FLOODING (No Duplicate)'
    )
    
    # Analyze Distance Vector (current files)
    # Note: Current paths.csv contains DV data, but we need to separate it
    # For now, let's analyze the current paths.csv and node_1001_delivered.csv
    print("\n" + "="*60)
    print(" NOTE: Analyzing current paths.csv for Distance Vector")
    print(" (File may contain mixed data - checking...)")
    print("="*60)
    
    # Check if current files have UNICAST or BCAST
    with open('paths.csv', 'r') as f:
        content = f.read()
        unicast_count = content.count('UNICAST')
        bcast_count = content.count('BCAST')
        print(f"\n  UNICAST events: {unicast_count}")
        print(f"  BCAST events:   {bcast_count}")
    
    if unicast_count > bcast_count:
        print("\n  → File appears to be Distance Vector data")
        dv_results = analyze_simulation(
            'paths.csv',
            'node_1001_delivered.csv',
            'DISTANCE VECTOR (Hop-Count Routing)'
        )
    else:
        print("\n  → File appears to be Smart Flooding data (already analyzed)")
        print("\n  ⚠️  Distance Vector simulation may need to complete!")
        return
    
    # Comparison
    print("\n" + "="*60)
    print(" COMPARATIVE ANALYSIS")
    print("="*60)
    
    print(f"\n  Packet Delivery Ratio:")
    print(f"    Smart Flooding:  {flooding_results['pdr']:.1f}%")
    print(f"    Distance Vector: {dv_results['pdr']:.1f}%")
    improvement = dv_results['pdr'] - flooding_results['pdr']
    print(f"    Improvement:     {improvement:+.1f}%")
    
    print(f"\n  Network Efficiency (Events per Packet):")
    print(f"    Smart Flooding:  {flooding_results['events_per_packet']:.2f}x")
    print(f"    Distance Vector: {dv_results['events_per_packet']:.2f}x")
    reduction = ((flooding_results['events_per_packet'] - dv_results['events_per_packet']) / 
                 flooding_results['events_per_packet'] * 100)
    print(f"    Overhead Reduction: {reduction:.1f}%")
    
    print(f"\n  Total Network Overhead:")
    print(f"    Smart Flooding:  {flooding_results['overhead']:,} events")
    print(f"    Distance Vector: {dv_results['overhead']:,} events")
    savings = flooding_results['overhead'] - dv_results['overhead']
    print(f"    Savings:         {savings:,} events ({savings/flooding_results['overhead']*100:.1f}%)")
    
    print("\n" + "="*60)
    print(" CONCLUSION")
    print("="*60)
    
    if dv_results['pdr'] > flooding_results['pdr']:
        print(f"  ✓ Distance Vector achieves {abs(improvement):.1f}% better delivery")
    else:
        print(f"  ✗ Smart Flooding achieves {abs(improvement):.1f}% better delivery")
    
    if reduction > 0:
        print(f"  ✓ Distance Vector reduces overhead by {reduction:.1f}%")
    else:
        print(f"  ✗ Distance Vector increases overhead by {abs(reduction):.1f}%")
    
    print("\n" + "="*60 + "\n")

if __name__ == '__main__':
    main()
