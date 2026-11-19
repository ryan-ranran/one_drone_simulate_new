import math


def alignment_velocity(error_x, error_y, gain, max_speed):
    if min(gain, max_speed) <= 0 or not all(map(math.isfinite, (error_x, error_y))):
        raise ValueError('invalid visual alignment velocity inputs')
    vx, vy = gain * error_x, gain * error_y
    magnitude = math.hypot(vx, vy)
    if magnitude > max_speed:
        scale = max_speed / magnitude
        vx, vy = vx * scale, vy * scale
    return vx, vy


def alignment_ready(error_x, error_y, speed, position_tolerance, speed_tolerance):
    values = (error_x, error_y, speed, position_tolerance, speed_tolerance)
    if not all(math.isfinite(value) for value in values):
        return False
    return (math.hypot(error_x, error_y) <= position_tolerance and
            speed <= speed_tolerance)
