import math


def apply_deadzone(value, deadzone):
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def shape(value, curve):
    return math.copysign(abs(value) ** curve, value) if value else 0.0


def limit_vector(x, y, max_linear):
    magnitude = math.hypot(x, y)
    if magnitude <= max_linear or magnitude == 0.0:
        return x, y
    scale = max_linear / magnitude
    return x * scale, y * scale


def approach(current, target, max_delta):
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)
