#!/usr/bin/env python3
"""Redraw docs/images/host-to-dac-volume-mapping-curves.png from the firmware mapping.

The curves mirror ``map_host_volume_to_dac()`` and ``DacVolumeCurve`` in
``src/main.cpp``. Keep both in sync when a curve is added or removed.

Panel 1 plots the continuous 0-100 value handed to ``ES8388::setDACVolume()``.
Panel 2 plots the analog output level that actually reaches LOUT1/ROUT1 after
``setDACVolume()`` quantizes 0-100 into the 0x00-0x21 register range, which the
ES8388 defines as -45 dB to +4.5 dB in 1.5 dB steps (0x1E = 0 dB).

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

# ES8388 LOUT1VOL/ROUT1VOL (registers 0x2E/0x2F).
REG_STEP_MAX = 0x21
REG_STEP_DB = 1.5
REG_DB_MIN = -45.0

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


def register_steps(dac_volume: np.ndarray) -> np.ndarray:
    """ES8388::setDACVolume() integer mapping: 0-100 -> 0x00-0x21."""
    volume = np.clip(np.rint(dac_volume).astype(int), 0, DAC_VOLUME_MAX)
    steps = (volume * 33 + 50) // 100
    return np.minimum(steps, REG_STEP_MAX)


def output_db(dac_volume: np.ndarray) -> np.ndarray:
    return REG_DB_MIN + REG_STEP_DB * register_steps(dac_volume)


def main() -> None:
    host_continuous = np.linspace(0.0, HOST_VOLUME_MAX, 1024)
    host_integer = np.arange(0, HOST_VOLUME_MAX + 1)

    fig, (ax_mapped, ax_db) = plt.subplots(2, 1, figsize=(10.0, 9.5), sharex=True)

    for label, curve, color, dashes, marker in CURVES:
        width = 2.6 if label == DEFAULT_CURVE_LABEL else 1.8
        zorder = 3 if label == DEFAULT_CURVE_LABEL else 2

        ax_mapped.plot(
            host_continuous,
            mapped_dac_volume(host_continuous, curve),
            label=label,
            color=color,
            linestyle=dashes,
            linewidth=width,
            marker=marker,
            markevery=128,
            markersize=6,
            zorder=zorder,
        )
        ax_db.step(
            host_integer,
            output_db(mapped_dac_volume(host_integer, curve)),
            where="post",
            label=label,
            color=color,
            linestyle=dashes,
            linewidth=width,
            marker=marker,
            markevery=16,
            markersize=6,
            zorder=zorder,
        )

    ax_mapped.set_title("Host to DAC volume mapping — continuous curves (main.cpp)")
    ax_mapped.set_ylabel("Mapped value before rounding (0-100)")
    ax_mapped.set_ylim(0, DAC_VOLUME_MAX)
    ax_mapped.legend(loc="lower right", framealpha=0.95)

    ax_db.set_title("Resulting LOUT1/ROUT1 output level after 1.5 dB register quantization")
    ax_db.set_xlabel("Host volume (0-127)")
    ax_db.set_ylabel("Output level (dB)")
    ax_db.set_ylim(REG_DB_MIN, REG_DB_MIN + REG_STEP_DB * REG_STEP_MAX + 2.0)
    ax_db.axhline(0.0, color="#666666", linewidth=1.0, linestyle=(0, (2, 3)), zorder=1)
    ax_db.annotate("0 dB (register 0x1E)", xy=(2, 1.0), fontsize=9, color="#444444")
    ax_db.legend(loc="lower right", framealpha=0.95)

    for ax in (ax_mapped, ax_db):
        ax.set_xlim(0, HOST_VOLUME_MAX)
        ax.grid(True, which="both", color="#CCCCCC", linewidth=0.6)

    fig.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=140)
    print(f"wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
