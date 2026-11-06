"""Bounded-memory offline audit helpers. No ROS imports or hardware operations."""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path


class Audit:
    def __init__(self):
        self.failures = Counter()
        self.examples = {}
        self.files = {}

    def require(self, ok, reason, detail=None):
        if not ok:
            self.failures[reason] += 1
            if detail is not None and len(self.examples.setdefault(reason, [])) < 3:
                self.examples[reason].append(detail)
        return bool(ok)

    def rows(self, path):
        path = Path(path)
        digest = hashlib.sha256()
        count = size = 0
        def nonfinite(value):
            raise ValueError(f'Non-finite JSON token: {value}')
        with path.open('rb') as stream:
            for line in stream:
                digest.update(line)
                size += len(line)
                if not line.endswith(b'\n'):
                    raise ValueError(f'Truncated JSONL final line: {path.name}')
                if not line.strip():
                    raise ValueError(f'Empty JSONL line: {path.name}')
                count += 1
                row = json.loads(line, parse_constant=nonfinite)
                if not isinstance(row, dict):
                    raise ValueError(f'Object required in {path.name}')
                yield row
        identity = dict(rows=count, bytes=size, sha256=digest.hexdigest())
        if path.name in self.files:
            self.require(identity == self.files[path.name], 'artifact_changed_between_passes', path.name)
        self.files[path.name] = identity

    def report(self):
        return dict(failures=dict(self.failures), examples=self.examples, files=self.files)


class Moments:
    def __init__(self):
        self.n = 0
        self.mean = self.m2 = 0.0
        self.minimum = math.inf
        self.maximum = -math.inf

    def add(self, value):
        if not finite(value):
            raise ValueError('Finite measurement required')
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    def report(self):
        return dict(samples=self.n, mean=self.mean if self.n else None,
                    std=math.sqrt(max(0., self.m2 / self.n)) if self.n else None,
                    min=self.minimum if self.n else None, max=self.maximum if self.n else None)


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def stamp_ns(row):
    stamp = row['header']['stamp']
    if type(stamp['sec']) is not int or type(stamp['nanosec']) is not int or not 0 <= stamp['nanosec'] < 1_000_000_000:
        raise ValueError('Invalid source ROS stamp')
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def source_clock(row):
    """Use recorded, bounded local ROS/steady conversion, never callback time."""
    before, after, ros_ns = row['clock_before'], row['clock_after'], row['clock_ros_ns']
    if not finite(before) or not finite(after) or not 0 <= after - before <= .002 or type(ros_ns) is not int:
        raise ValueError('Unbounded ROS-to-steady clock observation')
    source = (before + after) / 2 + (stamp_ns(row) - ros_ns) * 1e-9
    uncertainty = (after - before) / 2
    if (not finite(row['host_time']) or source > row['host_time'] + uncertainty + 1e-6
            or abs(source - row['source_time']) > 1e-9
            or abs(uncertainty - row['source_time_uncertainty']) > 1e-9):
        raise ValueError('Invalid/future source clock mapping')
    return source, row['host_time'] - source, uncertainty


class Coverage:
    def __init__(self, lo, hi, max_gap):
        self.lo, self.hi, self.max_gap = lo, hi, max_gap
        self.before = self.after = None
        self.previous = None
        self.maximum = 0.0
        self.nonmonotonic = 0
        self.samples = 0

    def add(self, stamp):
        if not finite(stamp):
            raise ValueError('Finite source timestamp required')
        if self.previous is not None:
            if stamp <= self.previous:
                self.nonmonotonic += 1
            if stamp > self.lo and self.previous < self.hi:
                self.maximum = max(self.maximum, stamp - self.previous)
        self.previous = stamp
        if stamp <= self.lo:
            self.before = stamp
        if stamp >= self.hi and self.after is None:
            self.after = stamp
        if self.lo <= stamp <= self.hi:
            self.samples += 1

    def report(self):
        passed = self.before is not None and self.after is not None and self.maximum <= self.max_gap and not self.nonmonotonic
        return dict(passed=passed, first_bracket=self.before, last_bracket=self.after,
                    samples=self.samples, max_source_gap=self.maximum,
                    nonmonotonic=self.nonmonotonic, allowed_gap=self.max_gap)


class StepResponse:
    """Streaming overlapping step windows; at most two pending events per axis."""
    def __init__(self, steps):
        self.windows = []
        for i, step in enumerate(steps):
            for phase, lo, hi, minimum, span in (('before', -.03, -.01, 8, .014), ('after', .03, .08, 20, .038)):
                self.windows.append(dict(event=i, phase=phase, lo=step['actual_time'] + lo,
                                         hi=step['actual_time'] + hi, minimum=minimum, span=span))
        self.windows.sort(key=lambda w: w['lo'])
        self.next = {a: 0 for a in (1, 2, 3)}
        self.active = {a: [] for a in (1, 2, 3)}
        self.previous = {a: None for a in (1,2,3)}
        self.stats = {(a, i, phase): dict(velocity=Moments(), torque=Moments(), first=None, last=None)
                      for a in (1, 2, 3) for i in range(len(steps)) for phase in ('before', 'after')}
        self.steps = steps
        self.coverage = {(a,w['event'],w['phase']): Coverage(w['lo'],w['hi'],.020)
                         for a in (1,2,3) for w in self.windows}

    def add(self, axis, stamp, velocity, torque):
        current = self.next[axis]
        while current < len(self.windows) and self.windows[current]['lo'] <= stamp:
            window=self.windows[current]
            self.active[axis].append(window)
            if self.previous[axis] is not None:
                self.coverage[(axis,window['event'],window['phase'])].add(self.previous[axis])
            current += 1
        self.next[axis] = current
        for window in self.active[axis]:
            self.coverage[(axis,window['event'],window['phase'])].add(stamp)
            if stamp>window['hi']:
                continue
            result = self.stats[(axis, window['event'], window['phase'])]
            result['velocity'].add(velocity)
            result['torque'].add(torque)
            if result['first'] is None:
                result['first'] = stamp
            result['last'] = stamp
        self.active[axis] = [w for w in self.active[axis] if stamp <= w['hi']]
        self.previous[axis]=stamp

    def report(self):
        result = []
        for axis in (1, 2, 3):
            for i, step in enumerate(self.steps):
                phases = {}
                complete = True
                for phase, minimum in (('before', 8), ('after', 20)):
                    stat = self.stats[(axis, i, phase)]
                    observed = stat['last'] - stat['first'] if stat['first'] is not None else 0.
                    coverage=self.coverage[(axis,i,phase)].report()
                    phases[phase] = dict(velocity=stat['velocity'].report(), torque=stat['torque'].report(), span=observed, coverage=coverage)
                    complete &= stat['velocity'].n >= minimum and coverage['passed']
                before, after = phases['before']['velocity']['mean'], phases['after']['velocity']['mean']
                result.append(dict(axis=axis, event=i, **step, **phases, complete=bool(complete),
                                   actual_delta_velocity=after-before if before is not None and after is not None else None))
        return dict(complete=all(row['complete'] for row in result), responses=result,
                    note='Measured speed responses are continuous; no instantaneous physical velocity jump is required.')
