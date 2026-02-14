#!/usr/bin/env python3
"""
Generate comparison graphs for Smart Flooding vs Distance Vector routing
"""

import csv
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

def load_simulation_data(paths_file, node_delivered_file):
    """Load and process simulation data"""
    
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
    
    # Count unique delivered packets
    delivered_seqs = set()
    try:
        with open(node_delivered_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    seq = int(row['seq'])
                    delivered_seqs.add(seq)
                except (ValueError, KeyError):
                    continue
    except FileNotFoundError:
        pass
    
    unique_delivered = len(delivered_seqs) if delivered_seqs else delivered_count
    
    return {
        'tx_src': tx_src_count,
        'delivered': unique_delivered,
        'pdr': (unique_delivered / tx_src_count * 100) if tx_src_count > 0 else 0,
        'total_events': total_events,
        'overhead': total_events - tx_src_count,
        'events_per_packet': (total_events / tx_src_count) if tx_src_count > 0 else 0,
        'event_types': dict(event_types)
    }

def create_graphs():
    """Generate all comparison graphs"""
    
    print("Loading Smart Flooding data...")
    flooding = load_simulation_data(
        'smart_flooding_results/paths_flooding.csv',
        'smart_flooding_results/node_1001_flooding.csv'
    )
    
    print("Loading Distance Vector data...")
    dv = load_simulation_data(
        'paths.csv',
        'node_1001_delivered.csv'
    )
    
    # Set up the style
    plt.style.use('seaborn-v0_8-darkgrid')
    colors = ['#FF6B6B', '#4ECDC4']  # Red for Flooding, Teal for DV
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # ====================
    # 1. Packet Delivery Ratio
    # ====================
    ax1 = plt.subplot(2, 3, 1)
    protocols = ['Smart\nFlooding', 'Distance\nVector']
    pdrs = [flooding['pdr'], dv['pdr']]
    bars = ax1.bar(protocols, pdrs, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax1.set_ylabel('Packet Delivery Ratio (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Packet Delivery Ratio Comparison', fontsize=14, fontweight='bold')
    ax1.set_ylim(0, 100)
    ax1.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bar, pdr in zip(bars, pdrs):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{pdr:.1f}%', ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # ====================
    # 2. Network Efficiency (Events per Packet)
    # ====================
    ax2 = plt.subplot(2, 3, 2)
    events_pp = [flooding['events_per_packet'], dv['events_per_packet']]
    bars = ax2.bar(protocols, events_pp, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax2.set_ylabel('Events per Packet', fontsize=12, fontweight='bold')
    ax2.set_title('Network Efficiency\n(Lower is Better)', fontsize=14, fontweight='bold')
    ax2.grid(axis='y', alpha=0.3)
    
    for bar, epp in zip(bars, events_pp):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{epp:.1f}x', ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # ====================
    # 3. Total Network Overhead
    # ====================
    ax3 = plt.subplot(2, 3, 3)
    overheads = [flooding['overhead'], dv['overhead']]
    bars = ax3.bar(protocols, overheads, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    ax3.set_ylabel('Total Events', fontsize=12, fontweight='bold')
    ax3.set_title('Total Network Overhead\n(Lower is Better)', fontsize=14, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3)
    
    for bar, overhead in zip(bars, overheads):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{overhead:,}', ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # ====================
    # 4. Packets Sent vs Delivered
    # ====================
    ax4 = plt.subplot(2, 3, 4)
    x = np.arange(2)
    width = 0.35
    
    sent = [flooding['tx_src'], dv['tx_src']]
    delivered = [flooding['delivered'], dv['delivered']]
    
    bars1 = ax4.bar(x - width/2, sent, width, label='Sent', color='#95E1D3', 
                    alpha=0.8, edgecolor='black', linewidth=1.5)
    bars2 = ax4.bar(x + width/2, delivered, width, label='Delivered', color='#F38181',
                    alpha=0.8, edgecolor='black', linewidth=1.5)
    
    ax4.set_ylabel('Number of Packets', fontsize=12, fontweight='bold')
    ax4.set_title('Packets Sent vs Delivered', fontsize=14, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(protocols)
    ax4.legend(fontsize=10)
    ax4.grid(axis='y', alpha=0.3)
    
    # Add value labels
    for bar in bars1:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{int(height)}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    for bar in bars2:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{int(height)}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # ====================
    # 5. Event Type Breakdown - Smart Flooding
    # ====================
    ax5 = plt.subplot(2, 3, 5)
    
    # Get top event types for flooding
    flood_events = sorted(flooding['event_types'].items(), key=lambda x: x[1], reverse=True)[:6]
    labels_f = [e[0] for e in flood_events]
    values_f = [e[1] for e in flood_events]
    
    colors_pie = plt.cm.Set3(np.linspace(0, 1, len(labels_f)))
    wedges, texts, autotexts = ax5.pie(values_f, labels=labels_f, autopct='%1.1f%%',
                                         startangle=90, colors=colors_pie)
    ax5.set_title('Smart Flooding\nEvent Distribution', fontsize=14, fontweight='bold')
    
    for autotext in autotexts:
        autotext.set_color('black')
        autotext.set_fontweight('bold')
        autotext.set_fontsize(9)
    
    # ====================
    # 6. Event Type Breakdown - Distance Vector
    # ====================
    ax6 = plt.subplot(2, 3, 6)
    
    # Get top event types for DV
    dv_events = sorted(dv['event_types'].items(), key=lambda x: x[1], reverse=True)[:6]
    labels_d = [e[0] for e in dv_events]
    values_d = [e[1] for e in dv_events]
    
    colors_pie = plt.cm.Set2(np.linspace(0, 1, len(labels_d)))
    wedges, texts, autotexts = ax6.pie(values_d, labels=labels_d, autopct='%1.1f%%',
                                         startangle=90, colors=colors_pie)
    ax6.set_title('Distance Vector\nEvent Distribution', fontsize=14, fontweight='bold')
    
    for autotext in autotexts:
        autotext.set_color('black')
        autotext.set_fontweight('bold')
        autotext.set_fontsize(9)
    
    # ====================
    # Overall title
    # ====================
    fig.suptitle('LoRa Mesh Routing Protocol Comparison\nSmart Flooding vs Distance Vector (100 Packets)',
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.tight_layout(rect=[0, 0, 1, 0.99])
    
    # Save figure
    output_file = 'routing_comparison_graphs.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\n✅ Graphs saved to: {output_file}")
    
    # Also create a summary metrics graph
    create_summary_graph(flooding, dv)
    
    plt.show()

def create_summary_graph(flooding, dv):
    """Create a summary comparison graph"""
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Calculate improvement percentages
    pdr_improvement = dv['pdr'] - flooding['pdr']
    efficiency_improvement = ((flooding['events_per_packet'] - dv['events_per_packet']) / 
                              flooding['events_per_packet'] * 100)
    overhead_reduction = ((flooding['overhead'] - dv['overhead']) / flooding['overhead'] * 100)
    
    metrics = ['PDR\nImprovement', 'Efficiency\nGain', 'Overhead\nReduction']
    values = [pdr_improvement, efficiency_improvement, overhead_reduction]
    colors_map = ['#4CAF50' if v > 0 else '#F44336' for v in values]
    
    bars = ax.barh(metrics, values, color=colors_map, alpha=0.8, edgecolor='black', linewidth=2)
    
    ax.set_xlabel('Improvement (%)', fontsize=14, fontweight='bold')
    ax.set_title('Distance Vector Performance Gains over Smart Flooding',
                 fontsize=16, fontweight='bold', pad=20)
    ax.axvline(x=0, color='black', linestyle='-', linewidth=1)
    ax.grid(axis='x', alpha=0.3)
    
    # Add value labels
    for bar, value in zip(bars, values):
        width = bar.get_width()
        label_x = width + (2 if width > 0 else -2)
        ha = 'left' if width > 0 else 'right'
        ax.text(label_x, bar.get_y() + bar.get_height()/2.,
                f'{value:+.1f}%', ha=ha, va='center',
                fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    output_file = 'performance_improvement.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ Summary graph saved to: {output_file}")

if __name__ == '__main__':
    print("\n" + "="*60)
    print(" GENERATING COMPARISON GRAPHS")
    print("="*60 + "\n")
    
    try:
        create_graphs()
        print("\n" + "="*60)
        print(" ✅ All graphs generated successfully!")
        print("="*60 + "\n")
    except Exception as e:
        print(f"\n❌ Error generating graphs: {e}")
        import traceback
        traceback.print_exc()
