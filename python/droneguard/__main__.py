"""Command line: the Python twin of droneguard_run / droneguard_compare.

    python -m droneguard              fly with the health-aware RTH rule
    python -m droneguard fixed        fly with the naive fixed 25% rule
    python -m droneguard compare      both, one after the other
    add --plot to see the graphs (needs matplotlib)
"""

from __future__ import annotations

import argparse
from dataclasses import replace

from .params import SimParams
from .sim import simulate
from .summary import mission_end, summarize


def _plot(results):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
    for name, r in results:
        e = mission_end(r.mission_state, r.soc) + 1
        t = r.t[:e]
        axes[0].plot(t, r.dist[:e], label=name)
        axes[1].plot(t, 100 * r.soc[:e], label=f"{name}: charge")
        axes[1].plot(t, 100 * r.rth_threshold[:e], "--", label=f"{name}: needed to get home")
        axes[2].plot(t, r.I_motor[:e], label=name)
        axes[3].plot(t, r.overall_h[:e], label=name)
    axes[0].set_ylabel("Distance [m]")
    axes[1].set_ylabel("Charge [%]")
    axes[2].set_ylabel("Motor current [A]")
    axes[3].set_ylabel("Overall health")
    axes[3].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(True)
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(prog="python -m droneguard")
    ap.add_argument("mode", nargs="?", default="dynamic",
                    choices=["dynamic", "fixed", "compare"])
    ap.add_argument("--plot", action="store_true", help="show graphs")
    args = ap.parse_args()

    modes = ["dynamic", "fixed"] if args.mode == "compare" else [args.mode]
    results = []
    for mode in modes:
        r = simulate(replace(SimParams(), Use_Dynamic_RTH=float(mode == "dynamic")))
        print(f"\n[{mode} rule]")
        print(summarize(r).text())
        results.append((mode, r))
    if args.plot:
        _plot(results)


if __name__ == "__main__":
    main()
