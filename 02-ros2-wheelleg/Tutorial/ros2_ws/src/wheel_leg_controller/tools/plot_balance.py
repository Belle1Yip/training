#!/usr/bin/env python3
"""Plot pitch from a CSV produced by evaluate_balance.py."""

import argparse
import csv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', default='balance_pitch.png')
    parser.add_argument('--title', default='Wheel-leg balance pitch')
    args = parser.parse_args()

    with open(args.input, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    times = [float(row['t']) for row in rows]
    pitch = [float(row['pitch_deg']) for row in rows]

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, pitch, linewidth=1.2)
    ax.axhline(0.0, color='black', linewidth=0.8)
    ax.set(xlabel='Simulation time (s)', ylabel='Pitch (deg)', title=args.title)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)
    print(args.output)


if __name__ == '__main__':
    main()
