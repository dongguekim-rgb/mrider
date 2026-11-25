"""Office floor plan and deterministic pedestrian proxies (metres, seconds).

Actors use square collision envelopes, not human behaviour or learned models.
"""
import math

ROOMS = [(0., 3.9, 'Office A'), (4., 3.9, 'Meeting room'),
         (9., 3.9, 'Office B'), (0., -4.2, 'Storage'),
         (4., -4.2, 'Reception'), (9., -4.2, 'Lounge')]


def walls():
    result = []
    # 5 m corridor; three door openings per side, 1.5 / 2 / 1.5 m wide.
    for y in (-2.7, 2.5):
        result.extend((a, y, b, y + .2) for a, b in
                      [(-1.8, .5), (2., 5.), (7., 9.), (10.5, 11.8)])
    for x in (2.6, 7.7):
        result.extend([(x, 2.7, x + .2, 4.8), (x, -4.8, x + .2, -2.7)])
    return result


def furniture():
    return [(-1.3, 3.2, -.3, 4.2), (3.1, 3.4, 4.6, 4.3),
            (8.2, 3.2, 8.9, 4.4), (10.4, 3.3, 11.3, 4.3),
            (-1.3, -4.5, -.4, -3.1), (3., -4.5, 4.5, -3.5),
            (8.2, -4.5, 9., -3.5), (10.5, -4.5, 11.3, -3.1),
            (3.4, -2.3, 4.4, -1.4)]  # delivery trolley in the corridor


def box(x, y, radius=.25):
    return x-radius, y-radius, x+radius, y+radius


class Pedestrians:
    """One triggered crossing plus continuous room motion; no auto reset.

    Crossing waits at the corridor edge until the vehicle reaches x=2.5,
    walks to y=0, waits 3 s, then walks through the opposite doorway.
    Vehicle stops never pause the pedestrian clock.
    """
    def __init__(self, trigger_x=2.5, crossing_x=6., room_x=10., hold_y=0., hold_duration=3.):
        self.trigger_x=trigger_x;self.crossing_x=crossing_x;self.room_x=room_x
        self.hold_y=hold_y;self.hold_duration=hold_duration
        self.elapsed = 0.
        self.crossing_elapsed = None
        self.crossing_y = -2.1
        self.finished = False

    def step(self, dt, vehicle_x):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('nonnegative finite dt required')
        self.elapsed += dt
        if self.crossing_elapsed is None and vehicle_x >= self.trigger_x:
            self.crossing_elapsed = 0.
        if self.crossing_elapsed is not None:
            self.crossing_elapsed += dt
            t = self.crossing_elapsed
            arrival=(self.hold_y+2.1)/.6
            self.crossing_y = min(3.3, self.hold_y+max(0.,t-arrival-self.hold_duration)*.6) if t>=arrival else -2.1+.6*t
            self.finished = self.crossing_y >= 3.3
        return [box(self.crossing_x, self.crossing_y), box(self.room_x, 2.45 + .65*math.sin(self.elapsed*.6))]
