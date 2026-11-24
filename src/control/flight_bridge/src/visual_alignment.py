#这是一个提供给flight_bridge调用的，脱离了ros2环境的，专门搞数学计算的文件
import math

#接收距离误差，输出相应的调整的速度
def alignment_velocity(error_x, error_y, gain, max_speed):
    if min(gain, max_speed) <= 0 or not all(map(math.isfinite, (error_x, error_y))):
        raise ValueError('invalid visual alignment velocity inputs')
    vx, vy = gain * error_x, gain * error_y
    magnitude = math.hypot(vx, vy)
    if magnitude > max_speed:
        scale = max_speed / magnitude
        vx, vy = vx * scale, vy * scale
    return vx, vy

#专门检测现在的水平误差是否小于我们设置的容忍误差，速度是否够小可以降落
def alignment_ready(error_x, error_y, speed, position_tolerance, speed_tolerance):
    values = (error_x, error_y, speed, position_tolerance, speed_tolerance)
    if not all(math.isfinite(value) for value in values):
        return False
    return (math.hypot(error_x, error_y) <= position_tolerance and
            speed <= speed_tolerance)
