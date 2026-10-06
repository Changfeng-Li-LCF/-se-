"""Generate the reference files and a printable preview; never contacts the car."""
import argparse
import csv
import json
from pathlib import Path
from route import make_route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length', type=float, default=4.0)
    parser.add_argument('--amplitude', type=float, default=0.25)
    parser.add_argument('--wheelbase', type=float, default=0.248)
    parser.add_argument('--output', default=str(Path(__file__).parent))
    args = parser.parse_args()
    route = make_route(args.length, args.amplitude, args.wheelbase)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'reference.json').write_text(json.dumps(route, indent=2), encoding='utf-8')
    with (output / 'reference.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(route['poses'][0]))
        writer.writeheader()
        writer.writerows(route['poses'])
    print(json.dumps({k:v for k,v in route.items() if k != 'poses'}, indent=2))

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        from matplotlib.patches import Rectangle
    except ImportError:
        print('Reference files saved; install matplotlib to generate the picture.')
        return
    font = Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['svg.fonttype'] = 'path'
    fig = plt.figure(figsize=(10, 7), facecolor='#f5f7fa')
    grid = fig.add_gridspec(2, 2, width_ratios=[1.08, 1], hspace=0.4, wspace=0.34)
    ax = fig.add_subplot(grid[:, 0])
    ax.add_patch(Rectangle((-2, 0), 4, 5, facecolor='#eaf0f6', edgecolor='#9cabb8', linestyle='--'))
    xs = [p['x_m'] for p in route['poses']]
    ys = [p['y_m'] for p in route['poses']]
    ax.plot([-y for y in ys], xs, color='#137cbd', linewidth=3)
    ax.scatter([0, 0], [0, args.length], s=65, color=['#20864d', '#cc6234'], zorder=4)
    ax.annotate('起点 / 当前车头向前', (0, 0), xytext=(0.12, 0.18), fontsize=10)
    ax.annotate('终点 / 车头向前', (0, args.length), xytext=(0.15, args.length+0.1), fontsize=10)
    ax.annotate('', xy=(0, 0.5), xytext=(0, 0.1), arrowprops=dict(arrowstyle='->', color='#20864d', lw=2))
    ax.text(-1.6, 4.65, '你提供的可用空间\n前方 ≥ 5 m，左右各 ≥ 2 m', fontsize=10, color='#526272')
    ax.set(xlim=(-2.1, 2.1), ylim=(-0.25, 5.15), aspect='equal',
           xlabel='横向距离（m；图左侧为车左）', ylabel='向前距离（m）', title='S 弯参考路径 · 俯视图')
    ax.grid(alpha=0.2)
    bx = fig.add_subplot(grid[0, 1])
    bx.plot(xs, ys, color='#137cbd', linewidth=2)
    bx.axhline(0, color='#9cabb8', linewidth=1)
    bx.set(xlabel='向前距离（m）', ylabel='左偏为正（m）', title='横向偏移（放大显示）')
    bx.grid(alpha=0.2)
    cx = fig.add_subplot(grid[1, 1])
    cx.plot(xs, [p['model_steering_deg'] for p in route['poses']], color='#8a58ad', linewidth=2)
    cx.axhline(0, color='#9cabb8', linewidth=1)
    cx.set(xlabel='向前距离（m）', ylabel='模型转向角（°）', title='按轴距 0.248 m 估算；非实测舵机角度')
    cx.grid(alpha=0.2)
    fig.suptitle('只准备路径 · 未发车', fontsize=20, color='#18354b', y=0.98)
    fig.text(0.5, 0.025, f"向前 {args.length:.1f} m  |  左右各 {args.amplitude:.2f} m  |  路程 {route['length_m']:.2f} m  |  最大模型转角 {route['max_model_steering_deg']:.1f}°",
             ha='center', fontsize=11, color='#18354b')
    fig.subplots_adjust(top=0.88, bottom=0.13)
    fig.savefig(output/'s_path_preview.png', dpi=170, facecolor=fig.get_facecolor())
    fig.savefig(output/'s_path_preview.svg', facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    main()
