"""Compare the Python simulation with a mission exported from Simulink, sample by sample.

    In MATLAB:   startup; droneguard_export                 (or droneguard_export(f, 'fixed'))
    Then here:   python tools/compare_with_matlab.py tests/matlab_dynamic.csv [fixed]

Prints the largest difference in every signal. The fault ramps and solver are
identical, so the differences should be tiny (rounding only).
"""

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from droneguard import COLUMNS, SimParams, simulate  # noqa: E402


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    csv = Path(sys.argv[1])
    mode = sys.argv[2] if len(sys.argv) > 2 else "dynamic"

    m = np.genfromtxt(csv, delimiter=",", names=True)
    r = simulate(replace(SimParams(), Use_Dynamic_RTH=float(mode == "dynamic")))

    n = min(len(m), len(r.t))
    print(f"{csv.name}: {len(m)} MATLAB rows, {len(r.t)} Python rows, comparing {n}\n")
    print(f"{'signal':<15}{'max |diff|':>14}{'at t [s]':>10}")
    worst = 0.0
    for name in COLUMNS:
        diff = np.abs(m[name][:n] - getattr(r, name)[:n])
        i = int(np.argmax(diff))
        scale = max(np.abs(m[name][:n]).max(), 1e-12)
        worst = max(worst, diff[i] / scale)
        print(f"{name:<15}{diff[i]:>14.3e}{m['t'][i]:>10.1f}")
    print(f"\nworst relative difference: {worst:.2e}")


if __name__ == "__main__":
    main()
