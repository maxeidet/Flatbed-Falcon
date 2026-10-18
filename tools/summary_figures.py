"""Still images (1920x1080 PNG) summarising the controller and Kalman filter runs, in the same
dark style as the videos, for a presentation:

  results_overview.png   one card per run: landed or not, why, touchdown and tracking error
  tracking_error.png     distribution of the drone's distance to the true bed while tracking
  along_track.png        where along the road each drone sits relative to the bed
  height.png             the drone's height above the bed with and without the Kalman filter
  error_chain.png        how big each step of the error chain is: link -> aim point -> drone

All statistics use the TRACK state at the 4 m tracking height, from 20 s after TRACK starts
(the same selection as the tables in CONTROLLERS.md).

    .venv/bin/python tools/summary_figures.py --out videos/figures
"""
import argparse
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overlay_video import load_rows, ok  # noqa: E402

RUNS = [  # label, CSV, colour, short note
    ('PID', 'logs/run_20261018_105148.csv', '#50aaff', 'kp 1.20  kd 2.20  ki 0.30'),
    ('LQR', 'logs/run_20261018_120635.csv', '#ff963c', 'kp 1.07  kd 1.58'),
    ('LQI', 'logs/run_20261018_120347.csv', '#5ad26e', 'kp 1.88  kd 2.01  ki 0.58'),
    ('LQI, no Kalman', 'logs/run_20261018_124456.csv', '#e65ac8', 'kp 1.88  kd 2.01  ki 0.58'),
]
BG, PANEL, TEXT, MUTED, GATE = '#161a21', '#1f242d', '#e6ebf5', '#9aa0aa', '#46c85a'


class Run:
    def __init__(self, label, path, color, note):
        self.label, self.color, self.note = label, color, note
        rows = load_rows(path)
        start = next(r['t'] for r in rows if r['state'] == 'TRACK')
        landed = next((r for r in rows if r['state'] == 'LANDED'), None)
        self.landed = (landed['t'] - start, landed['err_drone']) if landed else None
        self.track_time = (landed['t'] if landed else rows[-1]['t']) - start
        self.sel = [r for r in rows if r['state'] == 'TRACK' and r['t'] - start > 20 and ok(r['h']) and r['h'] < 4.5
                    and ok(r['drone_x'])]
        for r in self.sel:
            c, s = math.cos(r['truck_yaw']), math.sin(r['truck_yaw'])
            r['along'] = (r['drone_y'] - r['bed_e']) * c + (r['drone_x'] - r['bed_n']) * s

    def vals(self, key):
        return [r[key] for r in self.sel if ok(r[key])]

    def med(self, key):
        return statistics.median(self.vals(key))


def style(plt):
    plt.rcParams.update({'figure.facecolor': BG, 'axes.facecolor': PANEL, 'axes.edgecolor': '#3a404c',
                         'axes.labelcolor': TEXT, 'text.color': TEXT, 'xtick.color': MUTED, 'ytick.color': MUTED,
                         'grid.color': '#3a404c', 'font.size': 20, 'axes.titlesize': 26, 'axes.titleweight': 'bold',
                         'axes.titlepad': 18, 'font.family': 'DejaVu Sans'})


def figure(plt, title, subtitle):
    fig = plt.figure(figsize=(16, 9), dpi=120)
    fig.text(0.035, 0.94, title, fontsize=34, fontweight='bold', color=TEXT)
    fig.text(0.035, 0.895, subtitle, fontsize=18, color=MUTED)
    return fig


def overview(plt, runs, out):
    fig = figure(plt, 'Results', 'same truck and radio link for every run (0.1 m noise, 0.1 s latency, 5 % dropouts)')
    reasons = {'LQR': 'stays ~0.8 m behind\nthe bed: no integral\nterm against drag',
               'LQI, no Kalman': 'hovers 0.8 m too low:\nnoisy bed height'}
    gap, x0 = 0.022, 0.035
    w = (1 - 2 * x0 - gap * (len(runs) - 1)) / len(runs)
    for n, run in enumerate(runs):
        ax = fig.add_axes([x0 + n * (w + gap), 0.08, w, 0.76])
        ax.set_xlim(0, 1), ax.set_ylim(0, 1), ax.axis('off')
        ax.add_patch(plt.Rectangle((0, 0), 1, 1, color=PANEL, transform=ax.transAxes))
        ax.add_patch(plt.Rectangle((0, 0.965), 1, 0.035, color=run.color, transform=ax.transAxes))
        ax.text(0.07, 0.88, run.label, fontsize=26 if len(run.label) < 6 else 22, fontweight='bold', va='center')
        ax.text(0.07, 0.81, run.note, fontsize=15, color=MUTED, va='center')
        if run.landed:
            ax.text(0.07, 0.68, 'LANDED', fontsize=30, fontweight='bold', color=GATE, va='center')
            ax.text(0.07, 0.6, f'{run.landed[0]:.0f} s after tracking\nstarted', fontsize=15, color=MUTED,
                    va='center', linespacing=1.3)
            ax.text(0.07, 0.45, f'{run.landed[1]:.2f} m', fontsize=36, fontweight='bold', va='center')
            ax.text(0.07, 0.36, 'from the bed centre\nat touchdown', fontsize=15, color=MUTED, va='center',
                    linespacing=1.3)
        else:
            ax.text(0.07, 0.68, 'NO LANDING', fontsize=26, fontweight='bold', color='#dc3c3c', va='center')
            ax.text(0.07, 0.6, f'{run.track_time:.0f} s of tracking', fontsize=15, color=MUTED, va='center')
            ax.text(0.07, 0.42, reasons.get(run.label, ''), fontsize=17, va='center', linespacing=1.4)
        if run.label != 'LQI, no Kalman':  # its truck never slowed for a landing: not comparable
            ax.text(0.07, 0.2, f'{run.med("err_drone"):.2f} m', fontsize=28, fontweight='bold', va='center')
            ax.text(0.07, 0.11, 'typical distance to the\nbed while tracking', fontsize=15, color=MUTED,
                    va='center', linespacing=1.3)
        else:
            ax.text(0.07, 0.16, 'horizontally it tracked\nfine; only the height\nfailed the gate', fontsize=15,
                    color=MUTED, va='center', linespacing=1.3)
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def pick(runs, label):
    return next(r for r in runs if r.label == label)


def controllers(runs):
    """The controller runs, all with the Kalman filter."""
    return [r for r in runs if r.label != 'LQI, no Kalman']


def tracking_error(plt, runs, out):
    runs = controllers(runs)
    fig = figure(plt, 'How close each drone stays above the bed',
                 'horizontal distance drone → true bed centre while tracking at 4 m, Kalman filter on')
    ax = fig.add_axes([0.07, 0.12, 0.9, 0.68])
    data = [run.vals('err_drone') for run in runs]
    parts = ax.violinplot(data, showextrema=False, widths=0.8)
    for body, run in zip(parts['bodies'], runs):
        body.set_facecolor(run.color), body.set_alpha(0.55), body.set_edgecolor(run.color)
    for n, (vals, run) in enumerate(zip(data, runs), start=1):
        m = statistics.median(vals)
        ax.plot([n - 0.3, n + 0.3], [m, m], color=TEXT, lw=4)
        ax.text(n + 0.33, m, f'{m:.2f} m', va='center', fontsize=20, fontweight='bold')
    ax.axhspan(0, 0.3, color=GATE, alpha=0.18)
    ax.text(0.55, 0.15, 'descent gate < 0.3 m', color=GATE, fontsize=17, va='center')
    ax.axhline(1.0, color=MUTED, ls='--', lw=1.5)
    ax.text(0.55, 1.04, 'bed edge (1 m to the side)', color=MUTED, fontsize=15)
    ax.set_xticks(range(1, len(runs) + 1), [r.label for r in runs], fontsize=22)
    ax.set_ylim(0, 2.0), ax.set_ylabel('distance [m]'), ax.grid(axis='y', alpha=0.4)
    ax.set_xlim(0.4, len(runs) + 0.8)
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def along_track(plt, runs, out):
    fig = figure(plt, 'Where along the road each drone sits',
                 'drone position along the driving direction relative to the bed centre, while tracking')
    ax = fig.add_axes([0.07, 0.12, 0.9, 0.68])
    import numpy as np
    bins = np.linspace(-2.5, 2.0, 91)
    for run in controllers(runs):
        vals = run.vals('along')
        ax.hist(vals, bins=bins, histtype='stepfilled', alpha=0.35, color=run.color, density=True)
        ax.hist(vals, bins=bins, histtype='step', lw=2.5, color=run.color, density=True,
                label=f'{run.label}   median {statistics.median(vals):+.2f} m')
    ax.axvspan(-1.25, 1.25, color=GATE, alpha=0.08)
    ax.axvline(0, color=TEXT, lw=1.5)
    ax.text(-1.22, ax.get_ylim()[1] * 0.92, 'rear edge\nof the bed', color=MUTED, fontsize=15)
    ax.text(1.22, ax.get_ylim()[1] * 0.92, 'front edge\n(cab side)', color=MUTED, fontsize=15, ha='right')
    ax.set_xlabel('along the road [m]   (−: behind the bed centre, +: ahead)'), ax.set_yticks([])
    ax.legend(loc='upper right', frameon=False, fontsize=19, bbox_to_anchor=(1.0, 0.8))
    top = ax.get_ylim()[1]
    ax.annotate('LQR lags behind:\nno integral term to hold\nthe rotor drag', xy=(-1.05, top * 0.5),
                xytext=(-2.45, top * 0.62),
                color='#ff963c', fontsize=18, arrowprops=dict(color='#ff963c', arrowstyle='->', lw=2))
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def height(plt, runs, out):
    fig = figure(plt, 'The Kalman filter keeps the height steady',
                 'drone height above the bed while tracking, same controller (LQI), same radio link')
    ax = fig.add_axes([0.07, 0.12, 0.9, 0.68])
    import numpy as np
    bins = np.linspace(2.6, 4.6, 81)
    for run in (pick(runs, 'LQI'), pick(runs, 'LQI, no Kalman')):
        vals = run.vals('h')
        label = 'with Kalman filter' if run.label == 'LQI' else 'without (raw measurements)'
        ax.hist(vals, bins=bins, histtype='stepfilled', alpha=0.4, color=run.color, density=True)
        ax.hist(vals, bins=bins, histtype='step', lw=2.5, color=run.color, density=True,
                label=f'{label}   median {statistics.median(vals):.2f} m')
    ax.axvspan(3.7, 4.3, color=GATE, alpha=0.18)
    ax.text(4.33, ax.get_ylim()[1] * 0.5, 'descent gate\n4 ± 0.3 m', color=GATE, fontsize=19, ha='left',
            fontweight='bold')
    ax.set_xlabel('height above the bed [m]'), ax.set_yticks([])
    ax.legend(loc='upper left', frameon=False, fontsize=19)
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def error_chain(plt, runs, out):
    fig = figure(plt, 'What the Kalman filter does with the radio data',
                 'median horizontal error while tracking: what the radio delivers, and what the drone aims at')
    ax = fig.add_axes([0.07, 0.12, 0.9, 0.68])
    import numpy as np
    steps = [('err_raw', 'radio measurement\n→ true bed'), ('err_filt', 'aim point after processing\n→ true bed')]
    pair = (pick(runs, 'LQI'), pick(runs, 'LQI, no Kalman'))
    x = np.arange(len(steps))
    for n, run in enumerate(pair):
        label = 'LQI with Kalman filter' if n == 0 else 'LQI without (extrapolation)'
        vals = [run.med(k) for k, _ in steps]
        bars = ax.bar(x + (n - 0.5) * 0.38, vals, width=0.36, color=run.color, alpha=0.85, label=label)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.02, f'{v:.2f} m', ha='center', fontsize=20,
                    fontweight='bold')
    ax.set_xticks(x, [s for _, s in steps], fontsize=21)
    ax.set_ylabel('distance [m]'), ax.set_ylim(0, 0.9), ax.grid(axis='y', alpha=0.4)
    ax.legend(loc='upper right', frameon=False, fontsize=19)
    ax.text(0, 0.8, 'mostly latency:\n0.1 s × ~6 m/s', ha='center', color=MUTED, fontsize=16)
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default='videos/figures', help='output folder')
    ap.add_argument('--skip', action='append', default=[], help='leave out the run with this label, e.g. PID')
    args = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    style(plt)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    runs = [Run(label, os.path.join(root, path), color, note) for label, path, color, note in RUNS
            if label not in args.skip]
    os.makedirs(args.out, exist_ok=True)
    for name, fn in (('results_overview', overview), ('tracking_error', tracking_error),
                     ('along_track', along_track), ('height', height), ('error_chain', error_chain)):
        path = os.path.join(args.out, f'{name}.png')
        fn(plt, runs, path)
        print(f'Wrote {path}')


if __name__ == '__main__':
    main()
