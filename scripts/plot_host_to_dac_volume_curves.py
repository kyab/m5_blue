#!/usr/bin/env python3
"""Redraw docs/images/host-to-dac-volume-mapping-curves.png from the firmware mapping.

The curves mirror ``map_host_volume_to_dac()`` and ``DacVolumeCurve`` in
``src/main.cpp``. Keep both in sync when a curve is added or removed.

The plotted value is the continuous 0-100 volume handed to
``ES8388::setDACVolume()``, before it is rounded to an integer.

Usage:
    python3 -m venv .venv
    .venv/bin/pip install -r scripts/requirements-plots.txt
    .venv/bin/python scripts/plot_host_to_dac_volume_curves.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

HOST_VOLUME_MAX = 127
DAC_VOLUME_MAX = 100

OUTPUT_PATH = Path(__file__).resolve().parents[1] / "docs" / "images" / "host-to-dac-volume-mapping-curves.png"

# Okabe-Ito colors plus explicit dash patterns and markers so the series stay
# distinguishable in grayscale and for colorblind readers.
CURVES = [
    ("Linear", lambda x: x, "#000000", "solid", "o"),
    ("Gamma 0.5 (sqrt)", lambda x: np.sqrt(x), "#0072B2", (0, (5, 2)), "s"),
    ("Gamma 0.33 (default)", lambda x: np.power(x, 1.0 / 3.0), "#D55E00", (0, (1, 1)), "^"),
    ("Gamma 0.25", lambda x: np.power(x, 0.25), "#009E73", (0, (3, 1, 1, 1)), "v"),
    ("Gamma 1.6", lambda x: np.power(x, 1.6), "#CC79A7", (0, (7, 2, 1, 2)), "D"),
    ("Exp k=3", lambda x: (np.exp(3.0 * x) - 1.0) / (math.exp(3.0) - 1.0), "#56B4E9", (0, (4, 1, 4, 1, 1, 1)), "P"),
]

DEFAULT_CURVE_LABEL = "Gamma 0.33 (default)"


def mapped_dac_volume(host: np.ndarray, curve) -> np.ndarray:
    """Continuous 0-100 DAC volume, matching map_host_volume_to_dac() before rounding."""
    x = host / float(HOST_VOLUME_MAX)
    return DAC_VOLUME_MAX * np.clip(curve(x), 0.0, 1.0)


def main() -> None:
    host = np.linspace(0.0, HOST_VOLUME_MAX, 1024)

    fig, ax = plt.subplots(figsize=(10.0, 5.8))

    for label, curve, color, dashes, marker in CURVES:
        is_default = label == DEFAULT_CURVE_LABEL
        ax.plot(
            host,
            mapped_dac_volume(host, curve),
            label=label,
            color=color,
            linestyle=dashes,
            linewidth=2.6 if is_default else 1.8,
            marker=marker,
            markevery=128,
            markersize=6,
            zorder=3 if is_default else 2,
        )

    ax.set_title("Host to DAC volume mapping — continuous curves (main.cpp)")
    ax.set_xlabel("Host volume (0-127)")
    ax.set_ylabel("Mapped value before rounding (0-100)")
    ax.set_xlim(0, HOST_VOLUME_MAX)
    ax.set_ylim(0, DAC_VOLUME_MAX)
    ax.grid(True, which="both", color="#CCCCCC", linewidth=0.6)
    ax.legend(loc="lower right", framealpha=0.95)

    fig.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=140)
    print(f"wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
