"""Stationarity evidence from distinct, timestamped motor feedback samples."""
from collections import deque
import math


class StationarityWindow:
    def __init__(self):
        self.samples = deque(maxlen=512)

    def add(self, received, position, velocity, count):
        if self.samples and count == self.samples[-1][3]:
            return
        self.samples.append((received, position, velocity, count))

    def summary(self, now):
        samples = list(self.samples)
        if samples and math.isfinite(samples[-1][0]):
            cutoff = samples[-1][0] - .12
            samples = [s for s in samples if not math.isfinite(s[0]) or s[0] >= cutoff]
        result = dict(quiet=False, samples=len(samples), duration=0., position_span=None,
                      velocity_rms=None, velocity_peak=None)
        if len(samples) < 20 or not math.isfinite(now) or not all(
                all(math.isfinite(x) for x in s[:3]) for s in samples):
            return result
        if any(b[0] < a[0] or b[3] <= a[3] for a, b in zip(samples, samples[1:])):
            return result
        duration = samples[-1][0] - samples[0][0]
        span = max(s[1] for s in samples) - min(s[1] for s in samples)
        rms = math.hypot(*(s[2] for s in samples)) / math.sqrt(len(samples))
        peak = max(abs(s[2]) for s in samples)
        result.update(duration=duration, position_span=span, velocity_rms=rms, velocity_peak=peak,
                      quiet=(0 <= now - samples[-1][0] < .05 and duration >= .1 - 1e-9
                             and span <= .001 and rms <= .04 and peak <= .15))
        return result
