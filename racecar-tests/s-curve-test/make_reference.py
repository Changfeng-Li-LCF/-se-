#!/usr/bin/env python3
"""Generate a reference artifact only; does not communicate with ROS."""
import argparse
import csv
import json
from pathlib import Path
from tracking_core import generate

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True)
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--heading-deg', type=float, default=80.0)
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    path = generate(args.radius, args.heading_deg)
    (out/'reference_local.json').write_text(json.dumps(path, indent=2), encoding='utf-8')
    with (out/'reference_local.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(path['points'][0])); w.writeheader(); w.writerows(path['points'])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    pts = path['points']
    axes[0].plot([p['x'] for p in pts], [p['y'] for p in pts], lw=2, color='#007f91', label='Reference only')
    for p in pts[::40]:
        import math
        axes[0].arrow(p['x'], p['y'], .18*math.cos(p['yaw']), .18*math.sin(p['yaw']), width=.015, color='#007f91')
    axes[0].scatter([pts[0]['x'],pts[-1]['x']], [pts[0]['y'],pts[-1]['y']], color=['#333333','#d35c29'])
    axes[0].set(xlabel='Forward x (m)', ylabel='Left y (m)', title='S reference in start frame (no vehicle run)')
    axes[0].set_aspect('equal'); axes[0].grid(alpha=.3); axes[0].legend()
    axes[1].plot([p['s'] for p in pts], [p['curvature'] for p in pts], color='#8b4fb5')
    axes[1].set(xlabel='Arc length (m)', ylabel='Curvature (1/m)', title=f'Continuous steering demand; min radius {args.radius:g} m')
    axes[1].axhline(0, color='gray', lw=.5); axes[1].grid(alpha=.3)
    fig.savefig(out/'reference_preview.png', dpi=160)
    fig.savefig(out/'reference_preview.svg')
    print(json.dumps({k:v for k,v in path.items() if k != 'points'}, indent=2))

if __name__ == '__main__':
    main()
