"""Acoustic layer: GCC-PHAT, DOA, detector, candidates, calibration, against the simulator.

Run with `-s` to see the DOA accuracy tables."""

import math

import numpy as np
import pytest


from obslam.acoustic import (
    Detector,
    angle_diff,
    array_to_bearing,
    bearing_to_array,
    calibrate,
    candidates,
    estimate_doa,
    gcc_phat,
    locate,
)
from obslam.analysis import bearing_deg as map_bearing
from obslam.audio_source import SimAudioSource, _delayed, plane_wave_delays
from obslam.config import MEASURED_MICS_MM

# the working layout (firmware channel order: top, left, right, bottom), spacings 40–80 mm
BOARD = MEASURED_MICS_MM
SAMPLE_US = 1e6 / 48000


def run_events(bearings, snr=20.0, kind="knock", est=BOARD, true=None, yaw=0.0, elevation=0.0, reverb=True, seed=1, calibrated=True, gap_frames=None, delays=None, offsets=None):
    """Emit one source per bearing into the simulator, run the detector and the locator."""
    src = SimAudioSource(est, array_yaw_deg=yaw, true_mics_mm=true, reverb=reverb, seed=seed, channel_delays_us=delays)
    det = Detector(48000)
    for _ in range(60):  # 0.6 s of floor
        det.feed(src.next_block().samples)
    gap = gap_frames or (120 if kind == "voice" else 40)
    rows = []
    for b in bearings:
        truth = src.emit(b, elevation, snr, kind)
        dets = []
        for _ in range(gap):
            f = src.next_block()
            dets += det.feed(f.samples, f.t_us, f.t)
        loc = None
        if len(dets) == 1:
            loc = locate(dets[0].window, 48000, est, array_yaw_deg=yaw, calibrated=calibrated, snr_db=dets[0].snr_db, offsets_us=offsets)
        rows.append({"bearing": b, "n_det": len(dets), "det": dets[0] if dets else None, "loc": loc, "truth": truth})
    return rows


def errors(rows):
    return [abs(angle_diff(r["loc"]["bearing_deg"], r["bearing"])) for r in rows]


def table(title, rows):
    print(f"\n{title}")
    print("  true°   est°    err°   σ°    conf   snr dB  elev°")
    for r in rows:
        l = r["loc"]
        print(f"  {r['bearing']:5.0f}  {l['bearing_deg']:6.1f}  {angle_diff(l['bearing_deg'], r['bearing']):5.1f}  {l['bearing_sigma_deg']:4.1f}  {l['confidence']:5.2f}  {r['det'].snr_db:6.1f}  {l['elevation_deg']:5.1f}")
    e = errors(rows)
    print(f"  max {max(e):.2f}°  rms {math.sqrt(np.mean(np.square(e))):.2f}°  median σ {np.median([r['loc']['bearing_sigma_deg'] for r in rows]):.2f}°")


# ---- GCC-PHAT ----------------------------------------------------------------------------------
def test_gcc_phat_recovers_fractional_delays_at_20db():
    rng = np.random.default_rng(0)
    fs = 48000
    n = 2160  # 45 ms, the detector's window
    nfft = 8192
    f = np.fft.rfftfreq(nfft, 1 / fs)
    t = np.arange(nfft) / fs
    # a knock-like burst: broadband, decaying
    src = rng.normal(0, 1, nfft) * np.exp(-np.maximum(t - 0.005, 0) / 0.008) * (t > 0.005)
    S = np.fft.rfft(src)
    worst = 0.0
    for d_us in (-201.7, -83.3, -20.7, 3.1, 10.4, 47.9, 101.2, 228.0):
        a = _delayed(S, f, 0.001 + d_us * 1e-6, nfft)[:n]
        b = _delayed(S, f, 0.001, nfft)[:n]
        p = float(np.mean(a[240:720] ** 2))  # loudest part, as the detector's SNR
        sigma = math.sqrt(p / 10 ** (20 / 10))
        a = a + rng.normal(0, sigma, n)
        b = b + rng.normal(0, sigma, n)
        g = gcc_phat(a, b, fs, max_tau=240e-6)
        err = abs(g.tau * 1e6 - d_us)
        worst = max(worst, err)
        assert err < 3.0, (d_us, g.tau * 1e6)
        assert 0.1 < g.peak <= 1.0 and not g.edge
    print(f"\nGCC-PHAT worst error over 8 delays at 20 dB: {worst:.2f} µs ({worst / SAMPLE_US:.3f} samples)")


def test_gcc_phat_sign_and_search_limit():
    rng = np.random.default_rng(2)
    x = rng.normal(0, 1, 4096)
    late = np.r_[np.zeros(5), x[:-5]]  # a hears 5 samples later than b
    g = gcc_phat(late, x, 48000, max_tau=200e-6)
    assert g.tau * 48000 == pytest.approx(5, abs=0.05) and g.peak > 0.9
    # a search limit below the true delay cannot find it: what is left is a noise peak, and the
    # coherence says so
    g2 = gcc_phat(late, x, 48000, max_tau=50e-6)
    assert g2.peak < 0.3


# ---- conventions -------------------------------------------------------------------------------
def test_bearing_conventions_match_the_map():
    # findings bearings are clockwise from base +x: a point at (1, -1) is at 45°
    assert map_bearing(1.0, -1.0) == pytest.approx(45.0)
    # a plane wave "from bearing 45°" must come from (1, -1): the mic nearest that way hears first
    mics = np.array([[1.0, -1.0, 0.0], [-1.0, 1.0, 0.0]]) * 0.02
    d = plane_wave_delays(mics, bearing_to_array(45.0, 0.0), 0.0, 343.0)
    assert d[0] < d[1]
    # array yaw is the bearing of the array +x axis, clockwise from base +x
    assert array_to_bearing(0.0, 30.0) == pytest.approx(30.0)
    assert array_to_bearing(90.0, 30.0) == pytest.approx(300.0)  # array +y is 90° CCW of its +x
    for yaw in (0.0, 30.0, 200.0):
        for b in (0.0, 10.0, 359.0):
            assert array_to_bearing(bearing_to_array(b, yaw), yaw) == pytest.approx(b % 360.0)


# ---- DOA on the simulator -------------------------------------------------------------------------
@pytest.fixture(scope="module")
def circle():
    return run_events(range(0, 360, 15))


def test_doa_around_the_circle(circle):
    assert all(r["n_det"] == 1 for r in circle)
    table("DOA, knock, 20 dB SNR, reverberant, working board layout, every 15°", circle)
    e = errors(circle)
    assert max(e) < 3.0
    assert math.sqrt(np.mean(np.square(e))) < 1.5
    # σ is honest: errors stay within 2σ (with a little slack for the 3% of a Gaussian beyond)
    assert sum(err <= 2 * r["loc"]["bearing_sigma_deg"] for err, r in zip(e, circle)) >= 0.9 * len(e)
    assert all(r["loc"]["bearing_sigma_deg"] < 5 for r in circle)
    assert np.median([r["loc"]["confidence"] for r in circle]) > 0.8
    # tdoa_us["ij"] = t_i − t_j, against the simulator's truth
    for r in circle:
        for k, v in r["truth"]["tdoa_us"].items():
            assert r["loc"]["tdoa_us"][k] == pytest.approx(v, abs=12.0)


def test_doa_with_yaw_elevation_and_voice():
    rows = run_events(range(0, 360, 45), yaw=30.0, elevation=35.0, seed=4)
    table("DOA, knock 20 dB, array yaw 30°, source 35° above the array plane", rows)
    assert max(errors(rows)) < 3.0
    assert all(abs(r["loc"]["elevation_deg"] - 35.0) < 8.0 for r in rows)
    voice = run_events(range(0, 360, 45), kind="voice", seed=5)
    table("DOA, shout 20 dB", voice)
    assert all(r["n_det"] == 1 for r in voice)
    assert max(errors(voice)) < 5.0


def test_doa_degrades_at_low_snr(circle):
    low = run_events(range(0, 360, 30), snr=10.0, seed=7)
    low = [r for r in low if r["loc"]]
    assert len(low) >= 8
    table("DOA, knock, 10 dB SNR", low)
    hi_conf = np.mean([r["loc"]["confidence"] for r in circle])
    lo_conf = np.mean([r["loc"]["confidence"] for r in low])
    assert lo_conf < 0.5 * hi_conf
    assert np.median([r["loc"]["bearing_sigma_deg"] for r in low]) > np.median([r["loc"]["bearing_sigma_deg"] for r in circle])
    # still honest: most errors within 2σ
    e = errors(low)
    assert sum(err <= 2 * r["loc"]["bearing_sigma_deg"] + 1 for err, r in zip(e, low)) >= 0.75 * len(low)


def test_doa_with_wrong_geometry_loses_confidence():
    # the estimator believes the old 30 mm placeholder square; the board is the real one
    square = [[-15, -15, 0], [15, -15, 0], [15, 15, 0], [-15, 15, 0]]
    rows = run_events(range(0, 360, 30), est=square, true=BOARD, seed=8)
    rows = [r for r in rows if r["loc"]]
    conf = [r["loc"]["confidence"] for r in rows]
    print(f"\nwrong geometry: confidences {np.round(conf, 2).tolist()}")
    assert np.mean(conf) < 0.3
    assert any("|k|" in x or "disagree" in x or "search limit" in x for r in rows for x in r["loc"]["reasons"])


def test_mirrored_channels_are_invisible_per_event_but_caught_by_calibration():
    """Swapping ch2/ch3 on this left-right symmetric board is a mirror image: every event fits a
    plane wave perfectly, just at the mirrored bearing. Only the clap calibration catches it."""
    swapped = [BOARD[0], BOARD[2], BOARD[1], BOARD[3]]
    rows = run_events([20, 70, 160, 250, 290, 340], true=swapped, seed=9)
    assert np.mean([r["loc"]["confidence"] for r in rows]) > 0.6
    assert max(errors(rows)) > 30
    clicks = [{"bearing_deg": r["bearing"], "tdoa_us": r["loc"]["tdoa_us"]} for r in rows]
    res = calibrate(clicks, BOARD)
    assert res["order"]["confirmed"] is False and res["ok"] is False
    assert res["order"]["best_rival_channel_to_mic"] == [1, 3, 2, 4]
    assert "looks wrong" in res["order"]["verdict"]


def test_uncalibrated_caps_confidence(circle):
    r = circle[3]
    loc = locate(r["det"].window, 48000, BOARD, calibrated=False, snr_db=r["det"].snr_db)
    assert loc["bearing_deg"] == r["loc"]["bearing_deg"]
    assert loc["confidence"] <= 0.5 and r["loc"]["confidence"] > 0.5
    assert any("not calibrated" in x for x in loc["reasons"])


def test_degenerate_geometry_gives_no_bearing(circle):
    w = circle[0]["det"].window
    for mics, why in (([[0, 0, 0]] * 4, "not set"), ([[0, 0, 0], [10, 0, 0], [20, 0, 0], [30, 0, 0]], "collinear")):
        loc = locate(w, 48000, mics)
        assert loc["bearing_deg"] is None and loc["bearing_sigma_deg"] is None and loc["elevation_deg"] is None
        assert loc["confidence"] == 0.0 and why in loc["reason"]
    d = estimate_doa([0.0] * 6, np.zeros((4, 3)))
    assert d.azimuth_deg is None and d.confidence == 0.0


# ---- detector ------------------------------------------------------------------------------------
def test_detector_quiet_on_noise_and_once_per_knock():
    src = SimAudioSource(BOARD, seed=11)
    det = Detector(48000)
    hits = []
    for _ in range(1000):  # 10 s of noise only
        f = src.next_block()
        hits += det.feed(f.samples, f.t_us, f.t)
    assert hits == [] and det.triggers == 0
    noise_dbfs = 20 * np.log10(np.sqrt(det.floor) / 32767)
    assert np.all(np.abs(noise_dbfs + 70) < 1.0)  # the sim's -70 dBFS floor, as the detector sees it
    truths = []
    for b in (10, 100, 190, 280, 330):
        truths.append(src.emit(b, 0, 18))
        for _ in range(50):  # 0.5 s apart, beyond min_gap
            f = src.next_block()
            hits += det.feed(f.samples, f.t_us, f.t)
    assert len(hits) == 5
    for d, t in zip(hits, truths):
        # onset within one 2.5 ms block of the truth, window = 5 ms before to 40 ms after
        assert abs(d.t_us - t["t_us"]) <= 3000
        assert d.window.shape == (d.pre + 1920, 4) and d.pre == 240
        assert d.snr_db == pytest.approx(18, abs=3)
        assert not d.sustained


def test_detector_one_event_per_shout_and_dead_time():
    src = SimAudioSource(BOARD, seed=12)
    det = Detector(48000, min_gap_ms=300)
    for _ in range(60):
        det.feed(src.next_block().samples)
    src.emit(60, 0, 20, "voice")  # 450 ms: longer than the dead time
    hits = []
    for _ in range(120):
        f = src.next_block()
        hits += det.feed(f.samples, f.t_us, f.t)
    assert len(hits) == 1 and hits[0].sustained
    # two knocks 100 ms apart: the second falls in the dead time
    src.emit(60, 0, 20, delay_ms=20)
    src.emit(200, 0, 20, delay_ms=120)
    hits = []
    for _ in range(60):
        f = src.next_block()
        hits += det.feed(f.samples, f.t_us, f.t)
    assert len(hits) == 1


def test_detector_survives_a_gap():
    src = SimAudioSource(BOARD, seed=13)
    det = Detector(48000)
    for _ in range(60):
        det.feed(src.next_block().samples)
    det.reset_stream()  # frames were lost
    src.emit(30, 0, 20)
    hits = []
    for _ in range(40):
        f = src.next_block()
        hits += det.feed(f.samples, f.t_us, f.t)
    assert len(hits) == 1  # the floor survived the reset, so no new warm-up was needed


# ---- candidates ----------------------------------------------------------------------------------
def region(id_, bearing, dist, size=0.2):
    a = math.radians(-bearing)
    x, y = dist * math.cos(a), dist * math.sin(a)
    return {"id": id_, "bearing_deg": bearing, "distance_m": dist, "x_m": x, "y_m": y, "bbox_m": [x - size / 2, y - size / 2, x + size / 2, y + size / 2]}


def test_candidates_pick_the_right_regions_in_order():
    findings = {
        "voids": [region("V1", 47, 3.0), region("V2", 44, 1.5), region("V3", 120, 1.0)],
        "shadows": [region("S1", 49, 2.0), region("S2", 60, 2.0)],
        "depressions": [region("D1", 45, 1.0)],
        "obstacles": [region("O1", 43, 0.8), region("O2", 300, 1.0)],
    }
    c = candidates(45.0, 3.0, findings)  # ± 6°
    assert [x["id"] for x in c] == ["V2", "V1", "S1", "O1"]  # voids, shadows, obstacles; nearest first; no depressions
    assert [x["kind"] for x in c] == ["void", "void", "shadow", "obstacle"]
    assert c[0] == {"kind": "void", "id": "V2", "distance_m": 1.5, "bearing_deg": 44, "off_axis_deg": -1.0}
    assert candidates(None, 3.0, findings) == [] and candidates(45.0, 3.0, None) == []
    # a wide region straddling the bearing counts although its centroid is off-axis
    wide = {"voids": [region("V9", 75, 1.0, size=1.5)]}
    assert [x["id"] for x in candidates(45.0, 3.0, wide)] == ["V9"]


def test_candidates_wrap_around_north():
    findings = {"voids": [region("V1", 2, 1.0), region("V2", 357, 2.0), region("V3", 12, 1.0)], "shadows": [region("S1", 355, 3.0)]}
    c = candidates(358.0, 3.0, findings)
    assert [x["id"] for x in c] == ["V1", "V2", "S1"]
    assert c[0]["off_axis_deg"] == 4.0 and c[2]["off_axis_deg"] == -3.0
    c = candidates(1.0, 2.0, findings)
    assert [x["id"] for x in c] == ["V1", "V2"]


# ---- calibration ---------------------------------------------------------------------------------
def test_calibration_recovers_yaw_and_i2s_skew():
    skew = [0.0, 0.0, SAMPLE_US, SAMPLE_US]  # the I2S1 pair (ch3, ch4) one sample late
    rows = run_events(range(0, 360, 45), yaw=30.0, delays=skew, seed=14)
    # without the offsets one sample of skew costs several degrees on this 40–80 mm board
    assert max(errors(rows)) > 4
    clicks = [{"bearing_deg": r["bearing"], "tdoa_us": r["loc"]["tdoa_us"]} for r in rows]
    res = calibrate(clicks, BOARD)
    print(f"\ncalibration: yaw {res['array_yaw_deg']}°, offsets {res['channel_offsets_samples']} samples, residual {res['residual_us']} µs, {res['order']['verdict']}")
    assert res["array_yaw_deg"] == pytest.approx(30.0, abs=1.0)
    np.testing.assert_allclose(res["channel_offsets_us"], skew, atol=2.0)
    assert res["residual_us"] < 3.0 and res["ok"] and res["order"]["confirmed"]
    assert all(abs(p["error_deg"]) < 2.0 for p in res["per_click"])
    # with the fitted yaw and offsets applied, new events are right again
    after = run_events([20, 110, 200, 290], yaw=30.0, delays=skew, seed=15, offsets=res["channel_offsets_us"])
    for r in after:
        loc = locate(r["det"].window, 48000, BOARD, array_yaw_deg=res["array_yaw_deg"], offsets_us=res["channel_offsets_us"], snr_db=r["det"].snr_db)
        assert abs(angle_diff(loc["bearing_deg"], r["bearing"])) < 3.0


def test_calibration_refines_positions():
    true = [[-35, 0, 0], [-70, -24, 0], [0, -24, 0], [-35, -80, 0]]  # the side mics really 30 % down
    rows = run_events(list(range(0, 360, 30)), true=true, seed=16)
    clicks = [{"bearing_deg": r["bearing"], "tdoa_us": r["loc"]["tdoa_us"]} for r in rows]
    res = calibrate(clicks, BOARD, refine_positions=True)
    ref = np.array(res["refined"]["mics_mm"])
    assert np.max(np.linalg.norm(ref[:, :2] - np.array(true)[:, :2], axis=1)) < 1.5
    assert res["residual_us"] < res["refined"]["residual_before_us"]
    with pytest.raises(ValueError):
        calibrate(clicks[:3], BOARD)
    with pytest.raises(ValueError):
        calibrate([dict(c, bearing_deg=10 + i) for i, c in enumerate(clicks[:5])], BOARD)  # all from one side


def test_cpu_cost_per_event(circle):
    import time

    w = circle[0]["det"].window
    t0 = time.perf_counter()
    for _ in range(20):
        locate(w, 48000, BOARD, snr_db=20)
    per = (time.perf_counter() - t0) / 20
    print(f"\nlocate(): {per * 1000:.2f} ms per event")
    assert per < 0.05
