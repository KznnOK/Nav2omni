"""Collision checks of the robot outline against the occupancy map.

Works on the map published by map_server, so it is the same on the real
robot. The outline is the rectangle from robot_geometry.yaml (maximum outline
plus the LiDAR body), grown by a margin.
"""
import math

import numpy as np


class OccupancyChecker:

    def __init__(self, grid_msg, outline, margin=0.0, keepout=()):
        info = grid_msg.info
        self.resolution = info.resolution
        self.origin = (info.origin.position.x, info.origin.position.y)
        data = np.array(grid_msg.data, dtype=np.int16).reshape(
            info.height, info.width)
        rows, cols = np.nonzero(data >= 50)
        # occupied cell centers in the map frame
        self.points = np.stack([
            self.origin[0] + (cols + 0.5) * self.resolution,
            self.origin[1] + (rows + 0.5) * self.resolution,
        ], axis=1)
        # a cell is a square; treat it as a disk of half the diagonal
        self.cell_radius = 0.5 * math.sqrt(2) * self.resolution
        self.outline = [(float(x), float(y)) for x, y in outline]
        self.margin = margin
        self.keepout = [tuple(map(float, k)) for k in keepout]

    def _half_extents(self):
        xs = [p[0] for p in self.outline]
        ys = [p[1] for p in self.outline]
        return max(xs), -min(xs), max(ys), -min(ys)

    def pose_collides(self, x, y, yaw, use_keepout=False):
        """True when the outline (grown by margin) overlaps an occupied cell."""
        front, back, left, right = self._half_extents()
        grow = self.margin + self.cell_radius
        reach = math.hypot(max(front, back), max(left, right)) + grow
        near = self.points[
            (np.abs(self.points[:, 0] - x) <= reach)
            & (np.abs(self.points[:, 1] - y) <= reach)]
        if len(near):
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = near[:, 0] - x, near[:, 1] - y
            along = c * dx + s * dy
            across = -s * dx + c * dy
            inside = ((along <= front + grow) & (along >= -back - grow)
                      & (across <= left + grow) & (across >= -right - grow))
            if inside.any():
                return True
        if use_keepout:
            corners = self.corners(x, y, yaw, self.margin)
            for x0, x1, y0, y1 in self.keepout:
                if _rect_polygon_overlap((x0, x1, y0, y1), corners):
                    return True
        return False

    def corners(self, x, y, yaw, grow=0.0):
        front, back, left, right = self._half_extents()
        c, s = math.cos(yaw), math.sin(yaw)
        local = [(front + grow, left + grow), (-back - grow, left + grow),
                 (-back - grow, -right - grow), (front + grow, -right - grow)]
        return [(x + c * a - s * b, y + s * a + c * b) for a, b in local]

    def segment_clear(self, start, end, yaw, step=0.02):
        """Straight move at a fixed heading from start to end (x, y)."""
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, int(math.ceil(length / step)))
        for k in range(count + 1):
            t = k / count
            if self.pose_collides(start[0] + t * (end[0] - start[0]),
                                  start[1] + t * (end[1] - start[1]), yaw):
                return False
        return True

    def rotation_clear(self, x, y, yaw_from, yaw_to, step_deg=3.0):
        """In-place rotation; keep-out zones count as obstacles."""
        delta = math.atan2(math.sin(yaw_to - yaw_from),
                           math.cos(yaw_to - yaw_from))
        count = max(1, int(math.ceil(abs(math.degrees(delta)) / step_deg)))
        return not any(
            self.pose_collides(x, y, yaw_from + delta * k / count,
                               use_keepout=True)
            for k in range(count + 1))


def _rect_polygon_overlap(rect, polygon):
    """Separating axis test between an axis-aligned rectangle and a polygon."""
    x0, x1, y0, y1 = rect
    box = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for shape in (box, polygon):
        for i in range(len(shape)):
            ax, ay = shape[i]
            bx, by = shape[(i + 1) % len(shape)]
            nx, ny = -(by - ay), bx - ax
            pa = [nx * px + ny * py for px, py in box]
            pb = [nx * px + ny * py for px, py in polygon]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True
