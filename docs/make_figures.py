"""Generate the README figures from the recorded data.

Run from the repository root:

    python docs/make_figures.py

The ESP32 runs are read directly from the raw serial logs; the M340 runs from
the CSV files written by m340/tools/modbus_logger.py. Sample period: 0.1 s.
"""
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

TS = 0.1  # sample period [s]

# Colours: one fixed colour per run type across all figures
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
INITIAL = "#2a78d6"   # fixed gains
DD_EFRIT = "#eb6834"  # DD-E-FRIT gains
LAG2 = "#1baf7a"      # simulated LAG2 plant
SETPOINT = "#52514e"

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": TEXT_2,
    "axes.titlecolor": TEXT,
    "axes.titlesize": 12,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.labelsize": 10,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "axes.axisbelow": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": TEXT_2,
    "ytick.color": TEXT_2,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.frameon": False,
    "legend.fontsize": 9,
    "lines.linewidth": 1.4,
    "font.family": "DejaVu Sans",
})


def read_serial_log(path):
    """DATA,<sp>,<rpm>,<pwm>,<kp>,<ki>,<kd> lines of an ESP32 serial log."""
    rows = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = re.search(r"DATA,([\d\s\.,-]+)", line)
            if m:
                parts = [p.strip() for p in m.group(1).strip().split(",")]
                if len(parts) >= 6:
                    rows.append([float(p) for p in parts[:6]])
    return pd.DataFrame(rows, columns=["SetPoint", "RPM", "PWM", "Kp", "Ki", "Kd"])


def time_axis(df):
    return np.arange(len(df)) * TS


def comparison_figure(initial, dd, title, unit_u, path):
    """Speed and control signal of a fixed-gain run vs a DD-E-FRIT run."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 2]})
    t0, t1 = time_axis(initial), time_axis(dd)

    ax1.plot(t0, initial["SetPoint"], color=SETPOINT, linestyle="--", linewidth=1.2, label="Setpoint")
    ax1.plot(t0, initial["RPM"], color=INITIAL, label="Fixed gains")
    ax1.plot(t1, dd["RPM"], color=DD_EFRIT, label="DD-E-FRIT")
    ax1.set_ylabel("Speed [RPM]")
    ax1.set_title(title)
    ax1.legend(loc="upper left", ncol=3)

    ax2.plot(t0, initial["PWM"], color=INITIAL, label="Fixed gains")
    ax2.plot(t1, dd["PWM"], color=DD_EFRIT, label="DD-E-FRIT")
    ax2.set_ylabel(f"Control signal [{unit_u}]")
    ax2.set_xlabel("Time [s]")
    ax2.set_xlim(0, max(t0[-1], t1[-1]))

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("saved", path)


def drive_vs_lag2_figure(drive, lag2, path):
    """Fixed-gain runs on the real drive and on the simulated LAG2 plant."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.8),
                                   gridspec_kw={"width_ratios": [3, 2]})
    td, tl = time_axis(drive), time_axis(lag2)

    for ax in (ax1, ax2):
        ax.plot(td, drive["SetPoint"], color=SETPOINT, linestyle="--", linewidth=1.2, label="Setpoint")
        ax.plot(td, drive["RPM"], color=INITIAL, label="Real ATV630 drive")
        ax.plot(tl, lag2["RPM"], color=LAG2, label="Simulated LAG2 plant")
        ax.set_xlabel("Time [s]")

    ax1.set_ylabel("Speed [RPM]")
    ax1.set_title("M340, fixed gains: real drive vs simulated plant")
    ax1.set_xlim(0, max(td[-1], tl[-1]))
    ax1.legend(loc="upper left")

    # Zoom on the first step: the drive follows its own acceleration ramp,
    # the LAG2 plant shows a second-order response
    ax2.set_title("First 3 s")
    ax2.set_xlim(0, 3)
    ax2.set_ylim(-10, 460)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("saved", path)


if __name__ == "__main__":
    esp_initial = read_serial_log("data/esp32/serial_logs/trial6_initial.log")
    esp_dd = read_serial_log("data/esp32/serial_logs/trial6_dd_efrit.log")
    comparison_figure(esp_initial, esp_dd, "ESP32 DC motor: fixed gains vs DD-E-FRIT",
                      "PWM 0-255", "docs/figures/esp32_comparison.png")

    m340_initial = pd.read_csv("data/m340/initial_run.csv")
    m340_dd = pd.read_csv("data/m340/dd_efrit_run.csv")
    comparison_figure(m340_initial, m340_dd, "M340 + ATV630 drive: fixed gains vs DD-E-FRIT",
                      "drive command", "docs/figures/m340_comparison.png")

    lag2 = pd.read_csv("data/m340/lag2_sim/initial_run_freq1.5.csv")
    drive_vs_lag2_figure(m340_initial, lag2, "docs/figures/m340_drive_vs_lag2.png")
