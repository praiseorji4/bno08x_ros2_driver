#!/usr/bin/env python3
"""
E2 static noise analysis for the BNO085 (Allan variance + data health).

Reads the rosbag2 MCAP recorded with bno085_i2c_allan.yaml and writes plots, a
summary.md and a summary.json to an output folder. It does not need ROS: the
messages are decoded straight from the MCAP file, so it runs on a laptop or WSL.

    python3 -m pip install mcap numpy matplotlib zstandard lz4
    python3 e2_allan_analysis.py ~/imu_data/20260927_2200_e2_allan -o ~/imu_data/e2_report

Useful options:
    --skip-start 1800    drop the first 30 min (warm-up) from the Allan analysis
    --gyro-lsb 0.001064  raw gyro scale [rad/s per count], if known from a motion test
    --max-hours 2        only read the first 2 h (quick look while the run continues)

What it computes:
  * Data health per topic: message count, rate, time-step jitter, dropped reports
    (from the 8-bit SH-2 sequence numbers), accuracy status, host latency and the
    sensor-vs-host clock drift.
  * Overlapping Allan deviation for the gyro (uncalibrated, rad/s; and raw counts),
    the accelerometer (raw counts, scaled by gravity) and the magnetometer (raw counts).
  * Noise terms read from the Allan curves: white noise (ARW / VRW), bias
    instability and rate random walk, with the usual unit conversions.
  * Bias and temperature over time, and the firmware's own gyro-bias estimate.
  * Suggested diagonal covariances for /imu in the EKF.
"""
import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np

G = 9.80665
RAD2DEG = 180.0 / math.pi

TOPIC_GYRO_RAW = "/bno08x/raw/gyroscope"
TOPIC_ACCEL_RAW = "/bno08x/raw/accelerometer"
TOPIC_MAG_RAW = "/bno08x/raw/magnetometer"
TOPIC_GYRO_UNCAL = "/bno08x/gyroscope_uncalibrated"
TOPIC_REPORT_INFO = "/bno08x/report_info"
TOPICS = [TOPIC_GYRO_RAW, TOPIC_ACCEL_RAW, TOPIC_MAG_RAW, TOPIC_GYRO_UNCAL, TOPIC_REPORT_INFO]

SENSOR_NAMES = {
    0x01: "accelerometer", 0x02: "gyroscope", 0x03: "magnetic field",
    0x05: "rotation vector", 0x07: "gyro uncalibrated", 0x08: "game rotation vector",
    0x0F: "mag uncalibrated", 0x13: "stability", 0x14: "raw accel", 0x15: "raw gyro",
    0x16: "raw mag", 0x2A: "gyro-integrated RV",
}


# --------------------------------------------------------------------------------------
# Fast CDR decoding
#
# Every message on these topics has the same frame_id, so every message of a type has
# the same byte length and layout. That lets a whole batch be parsed at once with a
# numpy structured dtype instead of decoding millions of messages one by one in Python.
# The layout follows ROS 2 CDR (little endian, 4-byte encapsulation header, fields
# aligned to their own size relative to the end of that header).
# --------------------------------------------------------------------------------------

def _align(o, n):
    return (o + n - 1) // n * n


def _layout(kind, frame_id_len):
    """Return (fields, total_size) for a message whose frame_id string has frame_id_len
    bytes including the terminating NUL. Offsets are into the full serialized buffer."""
    f = []
    o = 0
    f.append(("sec", "<i4", o)); f.append(("nanosec", "<u4", o + 4))
    o = 8 + 4 + frame_id_len                      # string length prefix + bytes
    f.append(("sensor_id", "u1", o)); f.append(("sequence", "u1", o + 1)); f.append(("accuracy", "u1", o + 2))
    o = _align(o + 3, 4); f.append(("delay_us", "<u4", o)); o += 4
    o = _align(o, 8); f.append(("sample_time_us", "<u8", o)); o += 8
    f.append(("receive_time_us", "<u8", o)); o += 8
    if kind == "RawSensor":
        o = _align(o, 2)
        for i, n in enumerate(("x", "y", "z", "temperature")):
            f.append((n, "<i2", o + 2 * i))
        o += 8
        o = _align(o, 4); f.append(("sensor_timestamp_us", "<u4", o)); o += 4
    elif kind == "GyroUncalibrated":
        o = _align(o, 8)
        for i, n in enumerate(("wx", "wy", "wz", "bx", "by", "bz")):
            f.append((n, "<f8", o + 8 * i))
        o += 48
    elif kind == "Report":
        pass
    else:
        raise ValueError(kind)
    return [(n, t, off + 4) for n, t, off in f], o + 4


def _dtype(kind, frame_id_len, itemsize):
    fields, size = _layout(kind, frame_id_len)
    if size > itemsize:
        raise ValueError(f"{kind}: message is {itemsize} bytes, layout needs {size}")
    return np.dtype({"names": [n for n, _, _ in fields], "formats": [t for _, t, _ in fields],
                     "offsets": [o for _, _, o in fields], "itemsize": itemsize})


def _frame_id_len(buf):
    return struct.unpack_from("<I", buf, 12)[0]


def _check_against_generic_decoder(kind, schema, payload, rec):
    """Compare the fast decode of one message with mcap_ros2's generic decoder, if installed."""
    try:
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        return
    dec = DecoderFactory().decoder_for("cdr", schema)
    if dec is None:
        return
    m = dec(payload)
    checks = [("sequence", m.info.sequence), ("sample_time_us", m.info.sample_time_us),
              ("receive_time_us", m.info.receive_time_us), ("sec", m.header.stamp.sec)]
    if kind == "RawSensor":
        checks += [("x", m.x), ("z", m.z), ("temperature", m.temperature),
                   ("sensor_timestamp_us", m.sensor_timestamp_us)]
    elif kind == "GyroUncalibrated":
        checks += [("wx", m.angular_velocity.x), ("bz", m.bias.z)]
    for name, want in checks:
        got = rec[name].item()
        if not (got == want or (isinstance(want, float) and abs(got - want) < 1e-12)):
            raise RuntimeError(f"fast decoder mismatch on {kind}.{name}: {got} != {want}")


def bag_files(path):
    p = Path(path).expanduser()
    if p.is_file():
        return [p]
    files = sorted(p.glob("*.mcap"), key=lambda f: (len(f.name), f.name))
    if not files:
        sys.exit(f"No .mcap files found in {p}")
    return files


def extract(path, max_hours=None):
    """Read the bag and return {topic: structured numpy array}."""
    from mcap.reader import make_reader

    kinds = {}
    batches = {t: [] for t in TOPICS}
    out = {t: [] for t in TOPICS}
    layouts = {}
    checked = set()
    t_first = None
    t_stop = None
    n_read = 0
    t0 = time.time()

    def flush(topic):
        msgs = batches[topic]
        if not msgs:
            return
        size = len(msgs[0])
        same = [m for m in msgs if len(m) == size]
        if len(same) != len(msgs):
            print(f"  warning: {len(msgs) - len(same)} {topic} messages had an unexpected size, skipped")
        out[topic].append(np.frombuffer(b"".join(same), dtype=layouts[topic][1]))
        batches[topic] = []

    for f in bag_files(path):
        print(f"Reading {f} ({f.stat().st_size / 1e9:.2f} GB)")
        with open(f, "rb") as fh:
            reader = make_reader(fh)
            for schema, channel, message in reader.iter_messages(topics=TOPICS):
                topic = channel.topic
                if topic not in kinds:
                    kinds[topic] = schema.name.split("/")[-1]
                if t_first is None:
                    t_first = message.log_time
                    if max_hours:
                        t_stop = t_first + int(max_hours * 3600e9)
                if t_stop and message.log_time > t_stop:
                    break
                data = message.data
                if topic not in layouts or len(data) != layouts[topic][0]:
                    if topic in layouts:
                        flush(topic)
                    dt = _dtype(kinds[topic], _frame_id_len(data), len(data))
                    layouts[topic] = (len(data), dt)
                if topic not in checked:
                    _check_against_generic_decoder(kinds[topic], schema, data,
                                                   np.frombuffer(data, dtype=layouts[topic][1])[0])
                    checked.add(topic)
                batches[topic].append(data)
                if len(batches[topic]) >= 200_000:
                    flush(topic)
                n_read += 1
                if n_read % 2_000_000 == 0:
                    print(f"  {n_read / 1e6:.0f} M messages, {time.time() - t0:.0f} s")
        if t_stop and message.log_time > t_stop:
            break
    for t in TOPICS:
        if t in layouts:
            flush(t)
    print(f"Decoded {n_read} messages in {time.time() - t0:.0f} s")
    return {t: np.concatenate(v) for t, v in out.items() if v}


# --------------------------------------------------------------------------------------
# Timing and integrity
# --------------------------------------------------------------------------------------

def unwrap(x, bits):
    """Unwrap an unsigned counter that wraps at 2**bits."""
    x = x.astype(np.int64)
    d = np.diff(x)
    mod = 1 << bits
    d = np.where(d < -mod // 2, d + mod, d)
    d = np.where(d > mod // 2, d - mod, d)
    return np.concatenate([[x[0]], x[0] + np.cumsum(d)])


def stamps_s(a):
    return a["sec"].astype(np.float64) + a["nanosec"].astype(np.float64) * 1e-9


def sample_grid(a, nominal_dt):
    """Place each sample on a uniform grid, counting gaps from both the time step and the
    8-bit sequence number (sequence alone can't see gaps of 256 or more)."""
    t = stamps_s(a)
    dt = np.diff(t)
    dseq = (np.diff(a["sequence"].astype(np.int64)) % 256)
    steps_time = np.maximum(np.rint(dt / nominal_dt).astype(np.int64), 1)
    steps = np.where(steps_time > 128, steps_time, np.where(dseq == 0, 256, dseq))
    idx = np.concatenate([[0], np.cumsum(steps)])
    return idx, int(np.sum(steps - 1))


def timing_report(name, a):
    t = stamps_s(a)
    n = len(t)
    dur = t[-1] - t[0]
    dt = np.diff(t)
    rate = (n - 1) / dur if dur > 0 else float("nan")
    nominal = float(np.median(dt))
    idx, missing = sample_grid(a, nominal)
    lat = (a["receive_time_us"].astype(np.int64) - a["sample_time_us"].astype(np.int64)) / 1000.0
    acc = np.bincount(a["accuracy"], minlength=4)[:4]
    r = {
        "topic": name, "messages": int(n), "duration_h": dur / 3600, "rate_hz": rate,
        "dt_median_ms": nominal * 1e3, "dt_std_ms": float(np.std(dt) * 1e3),
        "dt_p99_ms": float(np.percentile(dt, 99) * 1e3), "dt_max_ms": float(dt.max() * 1e3),
        "dt_min_ms": float(dt.min() * 1e3), "backwards_steps": int(np.sum(dt <= 0)),
        "dropped": missing, "dropped_pct": 100.0 * missing / (missing + n),
        "longest_gap_ms": float(dt.max() * 1e3),
        "latency_ms_p50": float(np.percentile(lat, 50)), "latency_ms_p99": float(np.percentile(lat, 99)),
        "latency_ms_max": float(lat.max()),
        "accuracy_counts": {k: int(v) for k, v in zip(["unreliable", "low", "medium", "high"], acc)},
    }
    if "sensor_timestamp_us" in a.dtype.names and np.any(a["sensor_timestamp_us"]):
        st = unwrap(a["sensor_timestamp_us"], 32) * 1e-6
        ht = a["sample_time_us"].astype(np.float64) * 1e-6
        slope = np.polyfit(ht - ht[0], st - st[0], 1)[0]
        r["sensor_clock_drift_ppm"] = (slope - 1.0) * 1e6
    return r, t, idx


def fill_grid(values, idx):
    """Put samples on the uniform grid and linearly interpolate the (few) dropped ones."""
    full = np.arange(idx[-1] + 1)
    if len(full) == len(idx):
        return values.astype(np.float64)
    return np.interp(full, idx, values.astype(np.float64))


# --------------------------------------------------------------------------------------
# Allan deviation
# --------------------------------------------------------------------------------------

def allan(y, tau0, points_per_decade=12):
    """Overlapping Allan deviation of rate data y sampled every tau0 seconds.
    Returns tau, adev, and the 1-sigma fractional error of each point."""
    y = np.asarray(y, dtype=np.float64)
    y = y - y.mean()
    n = len(y)
    theta = np.concatenate([[0.0], np.cumsum(y)]) * tau0
    m_max = (n - 1) // 4
    ms = np.unique(np.logspace(0, np.log10(m_max), int(np.log10(m_max) * points_per_decade)).astype(np.int64))
    taus, adev, err = [], [], []
    for m in ms:
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        var = np.sum(d * d) / (2.0 * (m * tau0) ** 2 * len(d))
        taus.append(m * tau0)
        adev.append(math.sqrt(var))
        err.append(1.0 / math.sqrt(2.0 * max(n / m - 1.0, 1.0)))
    return np.array(taus), np.array(adev), np.array(err)


def noise_terms(tau, adev):
    """Read white noise (N), bias instability (B) and random walk (K) off an Allan curve,
    using the standard slope method (IEEE Std 952)."""
    lt, la = np.log10(tau), np.log10(adev)
    slope = np.gradient(la, lt)
    i_min = int(np.argmin(adev))
    res = {"tau_min_s": float(tau[i_min]), "adev_min": float(adev[i_min]),
           "B": float(adev[i_min] / 0.664)}
    white = np.where((slope > -0.6) & (slope < -0.4) & (np.arange(len(tau)) < i_min))[0]
    if len(white):
        res["N"] = float(np.median(adev[white] * np.sqrt(tau[white])))
        res["N_fit_tau_range_s"] = [float(tau[white[0]]), float(tau[white[-1]])]
    else:
        # No clean -1/2 region: read the value on a -1/2 line through the shortest tau.
        res["N"] = float(adev[0] * np.sqrt(tau[0]))
        res["N_note"] = "no clean -1/2 slope region; read at the shortest tau"
    rrw = np.where((slope > 0.4) & (slope < 0.6) & (np.arange(len(tau)) > i_min))[0]
    if len(rrw):
        res["K"] = float(np.median(adev[rrw] * np.sqrt(3.0 / tau[rrw])))
        res["K_fit_tau_range_s"] = [float(tau[rrw[0]]), float(tau[rrw[-1]])]
    else:
        res["K"] = None
        res["K_note"] = "no +1/2 slope region inside this recording"
    if i_min >= len(tau) - 3:
        res["B_note"] = "the curve is still falling at the longest tau: B is an upper bound, record longer"
    return res


def moving_mean(x, w):
    if len(x) < w:
        return np.array([x.mean()])
    c = np.concatenate([[0.0], np.cumsum(x)])
    return (c[w:] - c[:-w]) / w


def block_means(x, w):
    n = len(x) // w
    return x[: n * w].reshape(n, w).mean(axis=1)


# --------------------------------------------------------------------------------------
# Main analysis
# --------------------------------------------------------------------------------------

def fmt(x, nd=3):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "n/a"
    if x == 0:
        return "0"
    if abs(x) >= 1000 or abs(x) < 1e-3:
        return f"{x:.{nd}e}"
    return f"{x:.{nd}g}" if abs(x) < 1 else f"{x:.{nd + 1}g}"


def analyze(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    cache = out / "extracted.npz"
    key = np.array(f"{Path(args.bag).expanduser().resolve()}|{args.max_hours}")
    z = np.load(cache) if cache.exists() and not args.reread else None
    if z is not None and ("cache_key" not in z.files or str(z["cache_key"]) != str(key)):
        z = None  # different bag or --max-hours: decode again
    if z is not None:
        print(f"Using cached extraction {cache} (pass --reread to decode the bag again)")
        data = {t: z[k] for k, t in zip(["gyro_raw", "accel_raw", "mag_raw", "gyro_uncal", "report_info"], TOPICS)
                if k in z.files}
    else:
        data = extract(args.bag, args.max_hours)
        np.savez(cache, cache_key=key, **{k: data[t] for k, t in zip(["gyro_raw", "accel_raw", "mag_raw", "gyro_uncal", "report_info"],
                                                     TOPICS) if t in data})
    missing = [t for t in TOPICS[:4] if t not in data]
    if missing:
        print("Topics not in the bag (their analysis is skipped):", ", ".join(missing))

    summary = {"bag": str(args.bag), "skip_start_s": args.skip_start, "timing": {}, "gyro": {}, "accel": {}, "mag": {}}
    md = [f"# BNO085 E2 static analysis", "", f"Bag: `{args.bag}`", ""]
    grids = {}

    # ---- Data health ---------------------------------------------------------------
    md += ["## 1. Data health", "",
           "| Topic | Msgs | Hours | Rate Hz | dt median / std / max ms | Dropped | Latency p50 / p99 ms | Accuracy (U/L/M/H) | Clock drift ppm |",
           "|---|---|---|---|---|---|---|---|---|"]
    t_ref = None
    for topic in TOPICS[:4]:
        if topic not in data:
            continue
        r, t, idx = timing_report(topic, data[topic])
        t_ref = t[0] if t_ref is None else min(t_ref, t[0])
        grids[topic] = (t, idx, r["dt_median_ms"] / 1e3)
        summary["timing"][topic] = r
        a = r["accuracy_counts"]
        md.append(f"| `{topic}` | {r['messages']} | {r['duration_h']:.2f} | {r['rate_hz']:.2f} | "
                  f"{r['dt_median_ms']:.2f} / {r['dt_std_ms']:.2f} / {r['dt_max_ms']:.1f} | "
                  f"{r['dropped']} ({r['dropped_pct']:.3f}%) | {r['latency_ms_p50']:.2f} / {r['latency_ms_p99']:.2f} | "
                  f"{a['unreliable']}/{a['low']}/{a['medium']}/{a['high']} | "
                  f"{fmt(r.get('sensor_clock_drift_ppm'))} |")
    if TOPIC_REPORT_INFO in data:
        ri = data[TOPIC_REPORT_INFO]
        ids, counts = np.unique(ri["sensor_id"], return_counts=True)
        summary["timing"]["report_info_counts"] = {SENSOR_NAMES.get(int(i), hex(i)): int(c) for i, c in zip(ids, counts)}
        md += ["", "Reports seen on `/bno08x/report_info`: " +
               ", ".join(f"{SENSOR_NAMES.get(int(i), hex(i))} {c}" for i, c in zip(ids, counts))]
    md += ["", "Dropped = reports missing from the SH-2 sequence numbers (and from time gaps longer than 128 samples). "
               "Latency = host processing time minus the sh2 library's sample time. "
               "Clock drift = how fast the sensor's clock runs against the Pi's.", ""]

    # timing plot
    fig, axs = plt.subplots(len(grids), 1, figsize=(11, 2.4 * len(grids)), sharex=True, squeeze=False)
    for ax, (topic, (t, idx, dt0)) in zip(axs[:, 0], grids.items()):
        ax.plot((t[1:] - t_ref) / 3600, np.diff(t) * 1e3, ",", alpha=0.5)
        ax.set_ylabel("dt [ms]"); ax.set_title(topic, fontsize=9); ax.set_yscale("log")
    axs[-1, 0].set_xlabel("time [h]")
    fig.tight_layout(); fig.savefig(out / "timing.png", dpi=110); plt.close(fig)

    def series(topic, fields, scale=1.0):
        """Samples on the uniform grid, after dropping the warm-up."""
        t, idx, dt0 = grids[topic]
        a = data[topic]
        start = np.searchsorted(t, t[0] + args.skip_start)
        cols = [fill_grid(a[f][start:], idx[start:] - idx[start]) * scale for f in fields]
        return np.stack(cols, axis=1), dt0

    axes = ["x", "y", "z"]
    fits_all = {}

    def allan_block(label, y, dt0, key, units, conv, fig_name, extra_curves=None):
        """Allan deviation for 3 axes; returns the per-axis noise terms."""
        fig, ax = plt.subplots(figsize=(8, 5.5))
        fits = {}
        for i, axn in enumerate(axes):
            tau, ad, er = allan(y[:, i], dt0)
            nt = noise_terms(tau, ad)
            fits[axn] = nt
            ax.errorbar(tau, ad, yerr=ad * er, fmt="-", lw=1.2, elinewidth=0.6, label=f"{axn}")
            ax.plot(tau, nt["N"] / np.sqrt(tau), ":", color="gray", lw=0.8)
            fits_all[f"{key}_{axn}"] = (tau, ad)
        if extra_curves:
            for lbl, (tau, ad) in extra_curves.items():
                ax.plot(tau, ad, "--", lw=0.8, label=lbl)
        ax.set_xscale("log"); ax.set_yscale("log"); ax.grid(True, which="both", alpha=0.3)
        ax.set_xlabel("tau [s]"); ax.set_ylabel(f"Allan deviation [{units}]"); ax.set_title(label)
        ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(out / fig_name, dpi=120); plt.close(fig)
        return fits

    # ---- Gyroscope -------------------------------------------------------------------
    gyro_scale = args.gyro_lsb
    scale_source = "given with --gyro-lsb" if gyro_scale else None
    md += ["## 2. Gyroscope", ""]
    if TOPIC_GYRO_UNCAL in grids:
        wu, dt0 = series(TOPIC_GYRO_UNCAL, ["wx", "wy", "wz"])
        fits_u = allan_block("Gyro, uncalibrated report (rad/s)", wu, dt0, "gyro_uncal", "rad/s", None,
                             "allan_gyro_uncal.png")
        summary["gyro"]["uncalibrated"] = fits_u
        # Per-sample noise after removing slow drift (60 s moving mean): what the EKF sees each update.
        w = max(int(60 / dt0), 1)
        per_sample = [float(np.std(wu[w // 2: w // 2 + len(moving_mean(wu[:, i], w)), i] - moving_mean(wu[:, i], w)))
                      for i in range(3)]
        summary["gyro"]["per_sample_std_rad_s"] = per_sample
        md += ["Uncalibrated gyro report (SH-2 0x07, rad/s, 1/512 rad/s resolution). This is the same sensor data as "
               "the calibrated gyro on `/imu`, before the firmware removes its bias estimate.", "",
               "| Axis | ARW [deg/sqrt(h)] | White noise [rad/s/sqrt(Hz)] | Bias instability [deg/h] | at tau [s] | Rate random walk [deg/h/sqrt(h)] | Per-sample std [deg/s] |",
               "|---|---|---|---|---|---|---|"]
        for i, axn in enumerate(axes):
            f = fits_u[axn]
            k = f["K"] * RAD2DEG * 3600 * 60 if f["K"] else None
            md.append(f"| {axn} | {fmt(f['N'] * RAD2DEG * 60)} | {fmt(f['N'])} | {fmt(f['B'] * RAD2DEG * 3600)} | "
                      f"{fmt(f['tau_min_s'])} | {fmt(k)} | {fmt(per_sample[i] * RAD2DEG)} |")
        md += [""] + [f"- {axn}: {f[k]}" for axn, f in fits_u.items() for k in ("N_note", "B_note", "K_note") if k in f]
        md += ["", "![](allan_gyro_uncal.png)", ""]

        # Firmware bias estimate over time
        a = data[TOPIC_GYRO_UNCAL]
        t = stamps_s(a)
        b = np.stack([a["bx"], a["by"], a["bz"]], axis=1)
        changes = int(np.sum(np.any(np.diff(b, axis=0) != 0, axis=1)))
        summary["gyro"]["firmware_bias_changes"] = changes
        summary["gyro"]["firmware_bias_first_last_deg_s"] = [(b[0] * RAD2DEG).tolist(), (b[-1] * RAD2DEG).tolist()]
        md += [f"Firmware bias estimate: changed {changes} times; first {np.round(b[0] * RAD2DEG, 3).tolist()} deg/s, "
               f"last {np.round(b[-1] * RAD2DEG, 3).tolist()} deg/s.", ""]

    if TOPIC_GYRO_RAW in grids:
        gr, dt0 = series(TOPIC_GYRO_RAW, ["x", "y", "z"])
        extra = None
        fits_c = allan_block("Gyro, raw ADC counts", gr, dt0, "gyro_raw", "counts", None, "allan_gyro_raw_counts.png")
        summary["gyro"]["raw_counts"] = fits_c
        if not gyro_scale and TOPIC_GYRO_UNCAL in grids:
            # Scale from the Allan curves themselves: both reports carry the same sensor noise, so
            # the ratio of their Allan deviations is the count-to-rad/s scale (static data can't give
            # it by regression). Each report also adds its own rounding noise (1 count for raw,
            # 1/512 rad/s for uncalibrated), white with variance LSB^2/12 per sample, which adds
            # LSB^2/(12 m) to the Allan variance at tau = m*tau0. That is removed before the ratio.
            q_u = 1.0 / 512.0
            ratios = []
            for axn in axes:
                tu, au = fits_all[f"gyro_uncal_{axn}"]
                tc, ac = fits_all[f"gyro_raw_{axn}"]
                n_ = min(len(tu), len(tc))
                m = tu[:n_] / dt0
                vu = au[:n_] ** 2 - q_u ** 2 / (12.0 * m)
                vc = ac[:n_] ** 2 - 1.0 / (12.0 * m)
                sel = (tu[:n_] >= 1.0) & (tu[:n_] <= 100.0) & (vu > 0) & (vc > 0)
                if np.any(sel):
                    ratios.append(float(np.median(np.sqrt(vu[sel] / vc[sel]))))
            if ratios:
                gyro_scale = float(np.median(ratios))
                spread = float((max(ratios) - min(ratios)) / gyro_scale * 100)
                scale_source = (f"estimated from the Allan curves (uncalibrated / raw, tau 1-100 s, "
                                f"rounding noise removed); "
                                f"per-axis {', '.join(fmt(r) for r in ratios)}, spread {spread:.1f}%")
        if gyro_scale:
            summary["gyro"]["raw_scale_rad_s_per_count"] = gyro_scale
            summary["gyro"]["raw_scale_source"] = scale_source
            md += [f"Raw gyro scale: {fmt(gyro_scale)} rad/s per count ({fmt(gyro_scale * RAD2DEG)} deg/s, "
                   f"{fmt(1 / (gyro_scale * RAD2DEG))} counts per deg/s), {scale_source}. "
                   "Confirm it with a turn-table or hand-rotation test (E3) before using raw counts in the paper.", "",
                   "| Axis | Raw ARW [deg/sqrt(h)] | Raw bias instability [deg/h] | Raw RRW [deg/h/sqrt(h)] |", "|---|---|---|---|"]
            for axn in axes:
                f = fits_c[axn]
                k = f["K"] * gyro_scale * RAD2DEG * 3600 * 60 if f["K"] else None
                md.append(f"| {axn} | {fmt(f['N'] * gyro_scale * RAD2DEG * 60)} | "
                          f"{fmt(f['B'] * gyro_scale * RAD2DEG * 3600)} | {fmt(k)} |")
            md.append("")
        md += ["![](allan_gyro_raw_counts.png)", ""]

        # Bias vs time and temperature
        t, idx, _ = grids[TOPIC_GYRO_RAW]
        a = data[TOPIC_GYRO_RAW]
        w = max(int(60 / grids[TOPIC_GYRO_RAW][2]), 1)
        tb = block_means(t - t_ref, w) / 3600
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        for f_, lbl in zip(["x", "y", "z"], axes):
            v = block_means(a[f_].astype(np.float64), w)
            ax[0].plot(tb, (v - v[0]) * (gyro_scale * RAD2DEG if gyro_scale else 1), label=lbl)
        ax[0].set_ylabel("bias change [deg/s]" if gyro_scale else "bias change [counts]")
        ax[0].set_title("Raw gyro, 60 s means (bias drift)"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
        temp = block_means(a["temperature"].astype(np.float64), w)
        ax[1].plot(tb, temp); ax[1].set_ylabel("gyro temperature [counts]"); ax[1].set_xlabel("time [h]")
        ax[1].grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(out / "gyro_bias_temperature.png", dpi=110); plt.close(fig)
        corr = {}
        if np.std(temp) > 0:
            for f_ in axes:
                v = block_means(a[f_].astype(np.float64), w)
                corr[f_] = {"r": float(np.corrcoef(temp, v)[0, 1]),
                            "counts_per_temp_count": float(np.polyfit(temp, v, 1)[0])}
        summary["gyro"]["temperature"] = {"first": float(temp[0]), "last": float(temp[-1]),
                                          "min": float(temp.min()), "max": float(temp.max()), "bias_correlation": corr}
        md += [f"Gyro temperature (raw counts): start {temp[0]:.0f}, end {temp[-1]:.0f}, range {temp.min():.0f} to "
               f"{temp.max():.0f}."]
        if corr:
            md.append("Bias vs temperature correlation (60 s means): " +
                      ", ".join(f"{k} r={v['r']:.2f}" for k, v in corr.items()) + ".")
        md += ["", "![](gyro_bias_temperature.png)", ""]

    # ---- Accelerometer ------------------------------------------------------------------
    if TOPIC_ACCEL_RAW in grids:
        ar, dt0 = series(TOPIC_ACCEL_RAW, ["x", "y", "z"])
        mean = ar.mean(axis=0)
        acc_scale = args.accel_lsb or G / float(np.linalg.norm(mean))
        src = "given with --accel-lsb" if args.accel_lsb else "from gravity: |mean raw vector| = 1 g"
        am = ar * acc_scale
        fits_a = allan_block("Accelerometer, raw (scaled to m/s^2)", am, dt0, "accel", "m/s^2", None, "allan_accel.png")
        summary["accel"] = {"raw_scale_m_s2_per_count": acc_scale, "scale_source": src, "fits": fits_a,
                            "mean_m_s2": (mean * acc_scale).tolist()}
        w = max(int(60 / dt0), 1)
        per_sample = [float(np.std(am[w // 2: w // 2 + len(moving_mean(am[:, i], w)), i] - moving_mean(am[:, i], w)))
                      for i in range(3)]
        summary["accel"]["per_sample_std_m_s2"] = per_sample
        tilt = math.degrees(math.acos(min(1.0, abs(mean[2]) / np.linalg.norm(mean))))
        md += ["## 3. Accelerometer", "",
               f"Raw scale {fmt(acc_scale)} m/s^2 per count ({src}; {fmt(1 / acc_scale * G)} counts per g). "
               f"Mean reading {np.round(mean * acc_scale, 3).tolist()} m/s^2, so the board sat {tilt:.1f} deg from level.",
               "", "| Axis | VRW [m/s/sqrt(h)] | White noise [ug/sqrt(Hz)] | Bias instability [ug] | at tau [s] | Per-sample std [m/s^2] |",
               "|---|---|---|---|---|---|"]
        for i, axn in enumerate(axes):
            f = fits_a[axn]
            md.append(f"| {axn} | {fmt(f['N'] * 60)} | {fmt(f['N'] / G * 1e6)} | {fmt(f['B'] / G * 1e6)} | "
                      f"{fmt(f['tau_min_s'])} | {fmt(per_sample[i])} |")
        md += [""] + [f"- {axn}: {f[k]}" for axn, f in fits_a.items() for k in ("N_note", "B_note", "K_note") if k in f]
        md += ["", "![](allan_accel.png)", ""]

    # ---- Magnetometer -------------------------------------------------------------------
    if TOPIC_MAG_RAW in grids:
        mr, dt0 = series(TOPIC_MAG_RAW, ["x", "y", "z"])
        fits_m = allan_block("Magnetometer, raw ADC counts", mr, dt0, "mag", "counts", None, "allan_mag_counts.png")
        std = mr.std(axis=0)
        summary["mag"] = {"fits": fits_m, "mean_counts": mr.mean(axis=0).tolist(), "std_counts": std.tolist()}
        md += ["## 4. Magnetometer", "",
               f"Raw counts: mean {np.round(mr.mean(axis=0), 1).tolist()}, std {np.round(std, 2).tolist()}. "
               "Its heading quality is judged in E7 (near the motors), not here.", "", "![](allan_mag_counts.png)", ""]

    # ---- EKF suggestions -------------------------------------------------------------------
    md += ["## 5. Suggested /imu covariances for the EKF", ""]
    sug = {}
    if "per_sample_std_rad_s" in summary["gyro"]:
        v = [s ** 2 for s in summary["gyro"]["per_sample_std_rad_s"]]
        sug["angular_velocity_covariance_diag"] = v
        md.append(f"- angular_velocity: diagonal {[fmt(x) for x in v]} (rad/s)^2, the measured per-sample variance. "
                  "robot_localization does not estimate gyro bias, so inflate this (e.g. x4) if the bias "
                  "plot shows drift of the same size as the noise.")
    if "per_sample_std_m_s2" in summary.get("accel", {}):
        v = [s ** 2 for s in summary["accel"]["per_sample_std_m_s2"]]
        sug["linear_acceleration_covariance_diag"] = v
        md.append(f"- linear_acceleration: diagonal {[fmt(x) for x in v]} (m/s^2)^2 (only matters if accel is fused).")
    md.append("- orientation: not measured by E2; keep the placeholder until E4/E7.")
    summary["ekf_suggestion"] = sug
    md += ["", "## 6. How to read this", "",
           "- White noise (ARW/VRW) is the -1/2 slope at short tau; the dotted gray line is the fitted white-noise line.",
           "- Bias instability is the flat bottom of the curve (value / 0.664).",
           "- Rate random walk is a +1/2 slope at long tau; it only appears if the recording is long enough.",
           "- Error bars grow at long tau because there are few independent averages there.",
           f"- The first {args.skip_start:.0f} s were excluded (warm-up)." if args.skip_start else
           "- No warm-up was excluded. If the bias plot shows a settling curve at the start, rerun with --skip-start.",
           ""]

    (out / "summary.md").write_text("\n".join(md))
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    print(f"\nWrote {out / 'summary.md'}, summary.json and plots in {out}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="rosbag2 folder (or a single .mcap file)")
    p.add_argument("-o", "--out", default=None, help="output folder (default: <bag>_report)")
    p.add_argument("--skip-start", type=float, default=0.0, help="seconds to drop at the start (warm-up)")
    p.add_argument("--max-hours", type=float, default=None, help="only read this many hours")
    p.add_argument("--gyro-lsb", type=float, default=None, help="raw gyro scale [rad/s per count]")
    p.add_argument("--accel-lsb", type=float, default=None, help="raw accel scale [m/s^2 per count]")
    p.add_argument("--reread", action="store_true", help="ignore the cached extraction")
    args = p.parse_args()
    if args.out is None:
        args.out = str(Path(args.bag).expanduser().resolve()).rstrip("/") + "_report"
    analyze(args)


if __name__ == "__main__":
    main()
