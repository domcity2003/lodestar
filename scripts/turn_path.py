"""Construct a continuous horizontal route with a lower bound on turn radius."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PathPiece:
    start: np.ndarray
    end: np.ndarray
    source: int | None = None
    center: np.ndarray | None = None
    sweep: float = 0.0

    @property
    def length(self):
        if self.center is None:
            return float(np.linalg.norm(self.end - self.start))
        return abs(self.sweep) * float(np.linalg.norm(self.start - self.center))

    def sample(self, step):
        """Coordinates at sample boundaries and unit tangents at interval midpoints."""
        distance = np.linspace(0, self.length, max(1, math.ceil(self.length / step)) + 1)
        if self.center is None:
            direction = (self.end - self.start) / self.length
            return self.start + distance[:, None] * direction, np.tile(direction, (len(distance) - 1, 1)), distance
        radius = np.linalg.norm(self.start - self.center)
        angles = math.atan2(*(self.start - self.center)[::-1]) + self.sweep * distance / self.length
        points = self.center + radius * np.column_stack((np.cos(angles), np.sin(angles)))
        points[0], points[-1] = self.start, self.end
        middle = (angles[:-1] + angles[1:]) / 2
        tangents = np.sign(self.sweep) * np.column_stack((-np.sin(middle), np.cos(middle)))
        return points, tangents, distance


def left(vector):
    return np.array([-vector[1], vector[0]])


def sweep_between(start, end, sign):
    angle = math.atan2(end[1], end[0]) - math.atan2(start[1], start[0])
    angle = sign * angle % math.tau
    return 0.0 if math.isclose(angle, math.tau, abs_tol=1e-12) else sign * angle


def turnaround(point, incoming, outgoing, radius):
    """Return to the same corner using the shortest same-point Dubins connector."""
    options = []
    for sign in (-1, 1):
        c0 = point + sign * radius * left(incoming)
        c1 = point + sign * radius * left(outgoing)
        delta = c1 - c0
        tangent = delta / np.linalg.norm(delta)
        offset = -sign * radius * left(tangent)
        a, b = c0 + offset, c1 + offset
        first = sweep_between(point - c0, offset, sign)
        second = sweep_between(offset, point - c1, sign)
        options.append([PathPiece(point, a, center=c0, sweep=first), PathPiece(a, b),
                        PathPiece(b, point, center=c1, sweep=second)])
        # The same-turn circles are at most 2R apart. Intersect the two 2R
        # circles to put an opposite-turn circle tangent to both of them.
        midpoint = (c0 + c1) / 2
        offset = math.sqrt((2 * radius)**2 - float(delta @ delta) / 4) * left(tangent)
        for middle in (midpoint + offset, midpoint - offset):
            a, b = (c0 + middle) / 2, (c1 + middle) / 2
            first = sweep_between(point - c0, a - c0, sign)
            second = sweep_between(a - middle, b - middle, -sign)
            third = sweep_between(b - c1, point - c1, sign)
            options.append([PathPiece(point, a, center=c0, sweep=first),
                            PathPiece(a, b, center=middle, sweep=second),
                            PathPiece(b, point, center=c1, sweep=third)])
    return [p for p in min(options, key=lambda path: sum(p.length for p in path)) if p.length > 1e-7]


def rounded_path(legs, radius):
    """Use a tangent fillet if both trims fit, otherwise keep the vertex and turn around.

    Bridge any gaps between source legs with unsearched straight segments. A corner is
    processed in route order, sharing the available straight with the preceding corner.
    """
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("turn radius must be finite and positive")
    lines = []
    for index, (start, end) in enumerate(legs):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        if np.linalg.norm(end - start) < 1:
            continue
        if lines and np.linalg.norm(start - lines[-1].end) > 1e-7:
            lines.append(PathPiece(lines[-1].end, start))
        lines.append(PathPiece(start, end, index))
    if not lines:
        raise ValueError("route needs at least one nontrivial leg")

    lengths = [p.length for p in lines]
    directions = [(p.end - p.start) / length for p, length in zip(lines, lengths)]
    pieces, fillets, loops = [], 0, 0
    start = lines[0].start
    consumed = 0.0
    for i, line in enumerate(lines):
        end = line.end
        connector = []
        trim = 0.0
        if i + 1 < len(lines):
            u, v = directions[i:i + 2]
            angle = math.atan2(float(left(u) @ v), float(u @ v))
            if abs(angle) > 1e-7:
                cut = radius * math.tan(abs(angle) / 2)
                if cut < min(lengths[i] - consumed, lengths[i + 1]) - 1e-6:
                    end = line.end - cut * u
                    exit_point = line.end + cut * v
                    center = end + math.copysign(radius, angle) * left(u)
                    connector = [PathPiece(end, exit_point, center=center, sweep=angle)]
                    trim = cut
                    fillets += 1
                else:
                    connector = turnaround(line.end, u, v, radius)
                    loops += 1
        pieces.append(PathPiece(start, end, line.source))
        pieces.extend(connector)
        start = connector[-1].end if connector else end
        consumed = trim
    return pieces, fillets, loops


def ground_radius(speed, wind_bound, bank, gravity):
    """Minimum ground radius for any direction of a steady wind up to wind_bound."""
    if not 0 <= wind_bound < speed:
        raise ValueError("wind bound must be nonnegative and less than airspeed")
    return (speed + wind_bound)**2 / (gravity * math.tan(bank))
