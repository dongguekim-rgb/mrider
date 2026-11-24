# Copyright 2026 Jaerock Kwon
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
The arithmetic behind the bench tools, with no ROS in it.

Every number a bench tool reports comes from a function here.

Units: seconds and DEGREES throughout. The vehicle speaks radians; the
numeric contract (docs/design/dbw.md 12) is written in degrees, and so is
everything a person reads off a record sheet.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

# --- the numeric contract: docs/design/dbw.md 12 ----------------------------
STEER_LIMIT_DEG = 22.5
SS_ERROR_MAX_DEG = 1.0
SWEEP_RMS_MAX_DEG = 1.5
STEP_RISE_90_MAX_S = 0.400
STEP_OVERSHOOT_MAX = 0.15
CMD_STALE_S = 0.500
STATUS_HZ_MIN = 50.0
IMU_HZ_MIN = 100.0
LOOP_HZ_MIN = 200.0
# safety.md row 2: the laptop calls the link lost when status is stale 250 ms.
DROPOUT_GAP_S = 0.250

# DbwStatus.mode / faults
MODE_ESTOP = 0
MODE_MANUAL_RC = 1
MODE_AUTONOMOUS = 2
FAULT_SETPOINT_STALE = 2


@dataclass
class Sample:
    """One DbwStatus as it arrived, with the command in force at that moment."""

    t: float          # receipt time on the laptop, s (monotonic)
    cmd: float        # commanded steering angle, deg
    sp: float         # DbwStatus.steering_setpoint, deg
    meas: float       # DbwStatus.steering_angle, deg
    mode: int = MODE_AUTONOMOUS
    faults: int = 0


@dataclass
class Segment:
    """One piece of a test schedule: hold, step or sweep, from t0 to t1."""

    kind: str
    t0: float
    t1: float
    target: float     # deg; for a sweep, the amplitude
    start: float = 0.0  # deg; the target of the segment before


def window(samples: Sequence[Sample], t0: float, t1: float) -> List[Sample]:
    return [s for s in samples if t0 <= s.t <= t1]


def steady_state_error(samples: Sequence[Sample], t0: float, t1: float) -> Optional[float]:
    """Mean |setpoint - measured| over [t0, t1], deg. None if no samples."""
    w = window(samples, t0, t1)
    if not w:
        return None
    return sum(abs(s.sp - s.meas) for s in w) / len(w)


def rms_error(samples: Sequence[Sample], t0: float, t1: float) -> Optional[float]:
    """RMS of (setpoint - measured) over [t0, t1], deg."""
    w = window(samples, t0, t1)
    if not w:
        return None
    return math.sqrt(sum((s.sp - s.meas) ** 2 for s in w) / len(w))


def step_response(samples: Sequence[Sample], t_step: float, start: float, target: float,
                  t_end: float) -> Dict[str, Optional[float]]:
    """
    Rise to 90 % and overshoot for a step from `start` to `target` at t_step.

    rise_90_s:  first time the measured angle covers 90 % of the step, after
                t_step. None if it never does.
    overshoot:  the largest excursion past the target, as a fraction of the
                step. 0.0 if it never passes the target.
    """
    span = target - start
    w = window(samples, t_step, t_end)
    if not w or abs(span) < 1e-9:
        return {'rise_90_s': None, 'overshoot': None}
    rise = None
    peak = 0.0
    for s in w:
        frac = (s.meas - start) / span
        if rise is None and frac >= 0.9:
            rise = s.t - t_step
        peak = max(peak, frac)
    return {'rise_90_s': rise, 'overshoot': max(0.0, peak - 1.0)}


def first_time(samples: Sequence[Sample], t0: float, t1: float, pred) -> Optional[float]:
    for s in window(samples, t0, t1):
        if pred(s):
            return s.t
    return None


def setpoint_latency(samples: Sequence[Sample], t_cmd: float, target: float,
                     t_end: float, tol: float = 0.2) -> Optional[float]:
    """Command sent -> the Teensy echoing it as its setpoint, s."""
    t = first_time(samples, t_cmd, t_end, lambda s: abs(s.sp - target) <= tol)
    return None if t is None else t - t_cmd


def motion_latency(samples: Sequence[Sample], t_cmd: float, target: float,
                   t_end: float, tol: float = 0.5) -> Optional[float]:
    """Command sent -> the measured angle moving `tol` deg towards it, s."""
    w = window(samples, t_cmd, t_end)
    if not w:
        return None
    a0 = w[0].meas
    sign = 1.0 if target >= a0 else -1.0
    t = first_time(samples, t_cmd, t_end, lambda s: sign * (s.meas - a0) >= tol)
    return None if t is None else t - t_cmd


def max_rate(samples: Sequence[Sample], t0: float, t1: float, span_s: float = 0.05) -> Optional[float]:
    """
    Largest |d(measured)/dt| over [t0, t1], deg/s.

    Differenced over at least span_s, not sample to sample: at 100 Hz one
    count of sensor quantisation would otherwise read as a lurch.
    """
    w = window(samples, t0, t1)
    best = None
    j = 0
    for i in range(len(w)):
        while j < len(w) and w[j].t - w[i].t < span_s:
            j += 1
        if j >= len(w):
            break
        rate = abs(w[j].meas - w[i].meas) / (w[j].t - w[i].t)
        best = rate if best is None else max(best, rate)
    return best


def median(values: Sequence[float]) -> Optional[float]:
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def gaps(times: Sequence[float], threshold: float) -> List[float]:
    """Every interval between consecutive arrivals longer than threshold."""
    return [b - a for a, b in zip(times, times[1:]) if b - a > threshold]


def rates_per_window(times: Sequence[float], t_start: float, t_end: float,
                     window_s: float) -> List[float]:
    """Message rate in each whole window_s from t_start, Hz."""
    out = []
    t = t_start
    i = 0
    ts = sorted(times)
    while t + window_s <= t_end + 1e-9:
        n = 0
        while i < len(ts) and ts[i] < t:
            i += 1
        k = i
        while k < len(ts) and ts[k] < t + window_s:
            n += 1
            k += 1
        out.append(n / window_s)
        t += window_s
    return out


_LOOP = re.compile(
    r'loop\s+(?P<hz>[\d.]+)\s+Hz\s+\(period\s+(?P<pmin>\d+)\.\.(?P<pmax>\d+)\s+us,'
    r'\s+exec max\s+(?P<exec>\d+)\s+us,\s+overruns\s+(?P<over>\d+)\)')


def parse_loop_event(text: str) -> Optional[Dict[str, float]]:
    """The firmware's 5-second line on /mitt/dbw/event, or None if not one."""
    m = _LOOP.search(text)
    if not m:
        return None
    return {
        'hz': float(m.group('hz')),
        'period_min_us': float(m.group('pmin')),
        'period_max_us': float(m.group('pmax')),
        'exec_max_us': float(m.group('exec')),
        'overruns': float(m.group('over')),
    }


def _r(x: Optional[float], nd: int = 3) -> Optional[float]:
    return None if x is None else round(float(x), nd)


def _pass(value: Optional[float], limit: float) -> Optional[bool]:
    return None if value is None else bool(value <= limit)


def analyze_steer(samples: Sequence[Sample], segments: Sequence[Segment],
                  limit_deg: float = STEER_LIMIT_DEG) -> Dict:
    """
    Turn one steer_test run into the record a gate asks for.

    Returns a dict ready for YAML: per-item measurements, a pass/fail against
    dbw.md 12 for each, and a `for_simulation` block for Track S.
    """
    out: Dict = {'steady_state': [], 'steps': [], 'clamp': [], 'sweep': None}

    # steady state: the last second of every reachable hold of 3 s or more
    for seg in segments:
        if seg.kind == 'hold' and abs(seg.target) <= limit_deg and seg.t1 - seg.t0 >= 2.9:
            e = steady_state_error(samples, seg.t1 - 1.0, seg.t1)
            out['steady_state'].append({'target_deg': seg.target, 'error_deg': _r(e)})
    ss = [x['error_deg'] for x in out['steady_state'] if x['error_deg'] is not None]
    ss_worst = max(ss) if ss else None

    # steps
    sp_lat, mo_lat, rises, overs = [], [], [], []
    for seg in segments:
        if seg.kind != 'step':
            continue
        r = step_response(samples, seg.t0, seg.start, seg.target, seg.t1)
        sl = setpoint_latency(samples, seg.t0, seg.target, seg.t1)
        ml = motion_latency(samples, seg.t0, seg.target, seg.t1)
        out['steps'].append({
            'from_deg': seg.start, 'to_deg': seg.target,
            'rise_90_s': _r(r['rise_90_s']), 'overshoot': _r(r['overshoot']),
            'setpoint_latency_s': _r(sl), 'motion_latency_s': _r(ml),
        })
        sp_lat.append(sl)
        mo_lat.append(ml)
        rises.append(r['rise_90_s'])
        overs.append(r['overshoot'])
    rise_worst = None if not rises or any(x is None for x in rises) else max(rises)
    over_worst = max((x for x in overs if x is not None), default=None)

    # clamp, range and rate: holds commanded past the limit
    rates = []
    for seg in segments:
        if seg.kind == 'hold' and abs(seg.target) > limit_deg:
            w = window(samples, seg.t0, seg.t1)
            sp_max = max((abs(s.sp) for s in w), default=None)
            tail = window(samples, seg.t1 - 1.0, seg.t1)
            reached = sum(s.meas for s in tail) / len(tail) if tail else None
            out['clamp'].append({
                'commanded_deg': seg.target,
                'setpoint_max_abs_deg': _r(sp_max),
                'clamped': None if sp_max is None else bool(sp_max <= limit_deg + 0.05),
                'reached_deg': _r(reached),
            })
            rates.append(max_rate(samples, seg.t0, min(seg.t1, seg.t0 + 2.0)))
    lefts = [c['reached_deg'] for c in out['clamp'] if c['reached_deg'] is not None and c['commanded_deg'] > 0]
    rights = [c['reached_deg'] for c in out['clamp'] if c['reached_deg'] is not None and c['commanded_deg'] < 0]

    for seg in segments:
        if seg.kind == 'sweep':
            rms = rms_error(samples, seg.t0 + 0.5, seg.t1)
            out['sweep'] = {'amplitude_deg': seg.target, 'rms_error_deg': _r(rms)}
    sweep_rms = out['sweep']['rms_error_deg'] if out['sweep'] else None

    clamp_ok = [c['clamped'] for c in out['clamp']]
    out['verdict'] = {
        'steady_state_error_deg': {'worst': _r(ss_worst), 'limit': SS_ERROR_MAX_DEG,
                                   'pass': _pass(ss_worst, SS_ERROR_MAX_DEG)},
        'sweep_rms_deg': {'value': sweep_rms, 'limit': SWEEP_RMS_MAX_DEG,
                          'pass': _pass(sweep_rms, SWEEP_RMS_MAX_DEG)},
        'step_rise_90_s': {'worst': _r(rise_worst), 'limit': STEP_RISE_90_MAX_S,
                           'pass': _pass(rise_worst, STEP_RISE_90_MAX_S)},
        'step_overshoot': {'worst': _r(over_worst), 'limit': STEP_OVERSHOOT_MAX,
                           'pass': _pass(over_worst, STEP_OVERSHOOT_MAX)},
        'clamp_at_limit': {'pass': (all(clamp_ok) if clamp_ok and None not in clamp_ok
                                    else None)},
    }
    rate_vals = [x for x in rates if x is not None]
    sp_med = median(sp_lat)
    mo_med = median(mo_lat)
    out['for_simulation'] = {
        'steering_range_deg': {'left': _r(max(lefts) if lefts else None),
                               'right': _r(min(rights) if rights else None)},
        'steering_rate_limit_deg_s': _r(max(rate_vals) if rate_vals else None, 1),
        'command_to_setpoint_latency_ms': _r(None if sp_med is None else 1000 * sp_med, 1),
        'command_to_motion_latency_ms': _r(None if mo_med is None else 1000 * mo_med, 1),
        'step_rise_90_s': _r(median(rises)),
    }
    return out
