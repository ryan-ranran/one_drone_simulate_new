"""Open-loop height estimate from commanded normalized thrust (no height sensor)."""
import math


class ThrustHeightEstimator:
    def __init__(self, hover_thrust, accel_scale=9.81):
        if not (math.isfinite(hover_thrust) and 0 < hover_thrust < 1 and
                math.isfinite(accel_scale) and accel_scale > 0):
            raise ValueError('invalid thrust height estimator parameters')
        self.hover, self.scale = hover_thrust, accel_scale
        self.reset()

    def reset(self):
        self.velocity = self.height = 0.

    def update(self, thrust, tilt, dt):
        if not all(math.isfinite(v) for v in (thrust, tilt, dt)) or dt <= 0:
            raise ValueError('invalid thrust height feedback')
        dt = min(dt, .1)
        # Hover thrust balances gravity; vertical lift shrinks with body tilt.
        accel = self.scale * (thrust / self.hover * math.cos(tilt) - 1.)
        # The ground carries the vehicle until thrust exceeds weight.
        if self.height <= 0 and self.velocity <= 0 and accel <= 0:
            self.velocity = self.height = 0.
            return self.height
        self.velocity += accel * dt
        self.height = max(0., self.height + self.velocity * dt)
        if self.height == 0.:
            self.velocity = max(0., self.velocity)
        return self.height
