# analysis.py
# Post-flight analysis script. It loads the flight log from the SD card and replays the data through the same Kalman filter logic used on the rocket.
# It is used to tune the filter noise values (Q and R) against real data and to plot actual flight results against OpenRocket predictions.
# Everything lives in this one file until it gets too big to read comfortably.
#
# How to run it:   python analysis.py flt000.csv
# Optional:        python analysis.py flt000.csv openrocket.csv   (a CSV with columns "time" in s and "altitude" in m)

import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------------
# SETTINGS: these are the numbers you tune, then copy into filter.h
# ---------------------------------------------------------------
Q_ALT = 0.01   # how much we distrust the altitude prediction each loop
Q_VEL = 0.1    # how much we distrust the speed prediction each loop
R = 4.0        # how noisy the barometer is (variance, so noise in meters squared)

# These must match flight.h so the replay detects apogee the same way the rocket did
MIN_DEPLOY_ALT_M = 30.0
APOGEE_DROP_M = 3.0
APOGEE_HOLD_S = 0.1
BARO_SMOOTH_TAU_S = 0.1

# Names for the flight states, in the same order as the enum in flight.h
STATE_NAMES = ["PAD", "ARMED", "BOOST", "COAST", "DESCENT", "LANDED"]


def run_filter(vert_accel, baro_alt, dt, reset_rows=()):
    # This is the exact same math as filter.cpp, written with plain numbers instead of a class
    # alt and vel are what the filter thinks, and P is how unsure it is about them
    # reset_rows are the rows where main.cpp reset the filter (the moment the arm switch was flipped on the pad)
    alt, vel = 0.0, 0.0
    p00, p01, p10, p11 = 10.0, 0.0, 0.0, 10.0

    # We save the filter's answer at every step so we can plot it later
    out_alt = np.zeros(len(dt))
    out_vel = np.zeros(len(dt))

    for k in range(len(dt)):
        d = dt[k]
        a = vert_accel[k]

        # PREDICT: guess the new altitude and speed from physics
        alt = alt + vel * d + 0.5 * a * d * d
        vel = vel + a * d

        # Our confidence gets worse because we only guessed
        p00 = p00 + d * (p10 + p01) + d * d * p11 + Q_ALT
        p01 = p01 + d * p11
        p10 = p10 + d * p11
        p11 = p11 + Q_VEL

        # UPDATE: compare the guess to the barometer and nudge toward it
        error = baro_alt[k] - alt
        total_doubt = p00 + R
        trust_alt = p00 / total_doubt
        trust_vel = p10 / total_doubt
        alt = alt + trust_alt * error
        vel = vel + trust_vel * error

        # We just got real data, so our confidence gets better
        old_p00, old_p01 = p00, p01
        p00 = old_p00 - trust_alt * old_p00
        p01 = old_p01 - trust_alt * old_p01
        p10 = p10 - trust_vel * old_p00
        p11 = p11 - trust_vel * old_p01

        # main.cpp resets the filter right after the update in the loop where the arm switch goes on
        if k in reset_rows:
            alt, vel = 0.0, 0.0
            p00, p01, p10, p11 = 10.0, 0.0, 0.0, 10.0

        out_alt[k] = alt
        out_vel[k] = vel

    return out_alt, out_vel


def baro_apogee(t, baro_alt, launch_idx):
    # Replays the baro apogee detector from flight.cpp: smooth the altitude, track its highest point,
    # and call it apogee once it has dropped APOGEE_DROP_M below that and stayed there
    smooth = baro_alt[launch_idx]
    highest = smooth
    drop_since = None

    for k in range(launch_idx + 1, len(t)):
        d = min(max(t[k] - t[k - 1], 0.0), 0.1)
        smooth = smooth + (baro_alt[k] - smooth) * (d / (BARO_SMOOTH_TAU_S + d))
        highest = max(highest, smooth)

        # Only count it once we are high enough and at least 3 s after launch (the lockout in flight.h)
        if t[k] - t[launch_idx] < 3.0 or highest < MIN_DEPLOY_ALT_M:
            continue

        if smooth <= highest - APOGEE_DROP_M:
            if drop_since is None:
                drop_since = t[k]
            if t[k] - drop_since >= APOGEE_HOLD_S:
                return k
        else:
            drop_since = None

    return None


def main():
    if len(sys.argv) < 2:
        print("Usage: python analysis.py <logfile.csv> [openrocket.csv]")
        sys.exit(1)

    # Load the log: pandas reads the header line and gives every column a name
    df = pd.read_csv(sys.argv[1])
    t = df["t_ms"].to_numpy() / 1000.0

    # Time between rows (the first row has no previous row, so we reuse the second gap)
    dt = np.diff(t, prepend=t[0] - 0.01)
    dt = np.clip(dt, 0.001, 0.05)

    # Rows where the baro failed carry the last good value forward, like main.cpp does
    baro = df["baro_alt"].where(df["baro_ok"] == 1).ffill().fillna(0.0).to_numpy()
    vert_accel = df["vert_accel"].to_numpy()

    state = df["state"].to_numpy()
    events = df["events"].to_numpy().astype(int)
    arm = df["arm"].to_numpy()

    # The rocket resets its filter when the arm switch flips on while the state was still PAD
    reset_rows = {k for k in range(1, len(t)) if arm[k] == 1 and arm[k - 1] == 0 and state[k - 1] == 0}

    # Replay the filter with the settings at the top of this file
    replay_alt, replay_vel = run_filter(vert_accel, baro, dt, reset_rows)

    # ---- Find the important moments ----

    launched = np.where(state >= 2)[0]
    if len(launched) == 0:
        print("This log has no launch in it (the state never left PAD or ARMED).")
        launch_idx = None
    else:
        launch_idx = launched[0]
        print(f"Launch detected at t = {t[launch_idx]:.2f} s")

    fired = np.where(state >= 4)[0]
    if len(fired) > 0:
        i = fired[0]
        source = "baro apogee detector" if events[i] & 4 else "BACKUP TIMER" if events[i] & 8 else "unknown"
        print(f"Chute fired at t = {t[i]:.2f} s ({t[i] - t[launch_idx]:.2f} s after launch) at {baro[i]:.1f} m by the {source}")
    else:
        print("The chute never fired in this log.")

    print(f"Highest baro altitude: {baro.max():.1f} m")

    if launch_idx is not None:
        # Compare the three apogee detectors against each other
        i_baro = baro_apogee(t, baro, launch_idx)
        i_kal = None
        for k in range(launch_idx, len(t)):
            if t[k] - t[launch_idx] > 3.0 and replay_vel[k] <= 0 and replay_alt[k] > MIN_DEPLOY_ALT_M:
                i_kal = k
                break
        if i_baro is not None:
            print(f"Replayed baro detector says apogee at t = {t[i_baro]:.2f} s")
        if i_kal is not None:
            print(f"Replayed Kalman filter says apogee at t = {t[i_kal]:.2f} s")

    # ---- Tuning help: how noisy was the baro while sitting still on the pad? ----
    # Before launch the rocket is not moving, so any wiggle in the baro is pure noise
    # (the variance of that noise is exactly what R should be)
    pad_end = launch_idx if launch_idx is not None else len(t)
    pad_start = (max(reset_rows) + 100) if reset_rows else 10
    if pad_end - pad_start > 100:
        x = t[pad_start:pad_end]
        y = baro[pad_start:pad_end]

        # Remove any slow drift (weather) with a straight-line fit, so only the fast noise is left
        wiggle = y - np.polyval(np.polyfit(x, y, 1), x)
        print(f"Baro noise on the pad: std = {wiggle.std():.3f} m, so R should be about {wiggle.var():.3f} (currently {R})")

    # ---- Plots ----
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)

    axs[0].plot(t, baro, label="baro (raw)", alpha=0.5)
    axs[0].plot(t, df["filt_alt"], label="filter on the rocket")
    axs[0].plot(t, replay_alt, "--", label="filter replay (current Q and R)")

    # Optional: draw the OpenRocket prediction, lined up so its time 0 is our launch
    if len(sys.argv) > 2 and launch_idx is not None:
        orc = pd.read_csv(sys.argv[2])
        axs[0].plot(orc["time"] + t[launch_idx], orc["altitude"], label="OpenRocket", linewidth=2)

    axs[0].set_ylabel("altitude (m)")
    axs[0].legend()

    axs[1].plot(t, df["filt_vel"], label="filter on the rocket")
    axs[1].plot(t, replay_vel, "--", label="filter replay")
    axs[1].set_ylabel("vertical speed (m/s)")
    axs[1].legend()

    axs[2].plot(t, df["az"], label="IMU up axis")
    axs[2].plot(t, df["hz"], label="high-g z", alpha=0.6)
    axs[2].set_ylabel("acceleration (m/s^2)")
    axs[2].legend()

    axs[3].step(t, state, where="post")
    axs[3].set_yticks(range(len(STATE_NAMES)))
    axs[3].set_yticklabels(STATE_NAMES)
    axs[3].set_xlabel("time (s)")

    plt.tight_layout()
    out = sys.argv[1].rsplit(".", 1)[0] + "_plot.png"
    plt.savefig(out, dpi=120)
    print(f"Saved plot to {out}")
    plt.show()


main()
