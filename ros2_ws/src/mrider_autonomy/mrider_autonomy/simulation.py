"""Deterministic ideal bicycle model, also used by the ROS simulator."""
import math
from .core import Controller, Config


def path_points(kind):
    if kind == 'straight':
        return [(i*0.1, 0.0) for i in range(81)]
    if kind == 'curve':
        return [(4*math.sin(i*math.pi/160), 4*(1-math.cos(i*math.pi/160)))
                for i in range(81)]
    raise ValueError('unknown path: ' + kind)


def step(pose, speed, steering, dt, wheelbase):
    x, y, yaw = pose
    return (x+speed*math.cos(yaw)*dt, y+speed*math.sin(yaw)*dt,
            yaw+speed*math.tan(steering)*dt/wheelbase)


def run(kind, config=Config()):
    points = path_points(kind)
    controller = Controller(config)
    pose = (0.0, -0.2, 0.0)
    dt = 1/config.control_rate
    max_steering = 0.0
    for index in range(int(90/dt)):
        now = 1+index*dt
        controller.set_path(points, config.frame, now, now, now)
        controller.set_odometry(*pose, config.frame, config.rear_frame, now, now, now)
        command = controller.command(now, now)
        max_steering = max(max_steering, abs(command.steering))
        if command.reason == 'goal_reached':
            return {'path': kind, 'steps': index, 'seconds': round(index*dt, 4),
                    'goal_error_m': math.hypot(pose[0]-points[-1][0], pose[1]-points[-1][1]),
                    'max_abs_steering_rad': max_steering, 'final_speed_mps': command.speed}
        pose = step(pose, command.speed, command.steering, dt, config.wheelbase)
    raise RuntimeError('goal not reached: ' + kind)
