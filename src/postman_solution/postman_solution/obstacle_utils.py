"""Camera-only Tier-3 obstacle mapping and collision-aware grid search.

No ROS imports and no use of organizer / Gazebo ground-truth topics.  Wall
coordinates are *measured* from magenta pixels; never read the world file.
"""
import heapq
import math

import cv2
import numpy as np

from postman_solution.nav_utils import dist

# The top surfaces of the obstacles are 0.20 m above the arena floor.
# This is their visual height, NOT a hardcoded obstacle position.
WALL_TOP_Z = 0.20
GRID_RESOLUTION = 0.03
GRID_EXTENT = 2.76

# A circumscribing circle of the robot's 0.20 m front / 0.0885 m side
# footprint is 0.219 m.  Add a small safety margin for localisation error.
ROBOT_CLEARANCE = 0.295
# Arena wall faces are -0.125 and 2.875 m; keep centre clear at any yaw.
# The organizer samples Tier-3 targets in an interior part of the arena.
# Reserve extra perimeter space for localization uncertainty and chassis yaw.
# Departure from and return to home use an explicit slow corner corridor in
# tier3_node.py, instead of asking A* to skirt the south / west walls.
SAFE_MIN = 0.30
SAFE_MAX = 2.45


def find_magenta_walls(rgb, camera):
    """Find wall-top quadrilaterals, in arena (x,y), from an RGB image.

    Each item is four measured world points. Does not assume wall centres,
    angles, or an obstacle layout; works with differently placed rectangles.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (125, 80, 50), (175, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for cnt in contours:
        if cv2.contourArea(cnt) < 35:
            continue
        rect = cv2.minAreaRect(cnt)
        (u, v), (width, height), angle = rect
        if min(width, height) < 4 or max(width, height) < 22:
            continue
        if not (0.01 <= width * camera.metres_per_pixel(WALL_TOP_Z) <= 1.0):
            continue
        corners_px = cv2.boxPoints(rect)
        corners = [camera.pix_to_world(float(p[0]), float(p[1]), WALL_TOP_Z)
                   for p in corners_px]
        if not all(-0.1 < x < 2.9 and -0.1 < y < 2.9 for x, y in corners):
            continue
        found.append(corners)
    return found


class GridPlanner:
    """2D A*, obstacle inflation, line-of-sight path simplification."""
    def __init__(self, polygons, resolution=GRID_RESOLUTION, clearance=ROBOT_CLEARANCE):
        self.res = resolution
        self.n = int(math.ceil(GRID_EXTENT / resolution)) + 1
        self.polygons = polygons
        ycells, xcells = np.mgrid[0:self.n, 0:self.n]
        xx = xcells * resolution
        yy = ycells * resolution
        border_blocked = ((xx < SAFE_MIN) | (xx > SAFE_MAX) |
                          (yy < SAFE_MIN) | (yy > SAFE_MAX))

        # Inflate *measured* wall tops by the entire robot footprint. Using
        # a disk instead of inflating only length/width also allows in-place
        # rotation near obstacle corners without scraping them.
        obstacle_mask = np.zeros((self.n, self.n), dtype=np.uint8)
        for polygon in polygons:
            pts = np.rint(np.asarray(polygon, dtype=np.float64) / resolution)
            pts = pts.astype(np.int32)
            cv2.fillConvexPoly(obstacle_mask, pts, 255)
        r = math.ceil(clearance / resolution)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r * 2 + 1, r * 2 + 1))
        inflated = cv2.dilate(obstacle_mask, kernel) > 0
        self.blocked = np.logical_or(border_blocked, inflated)
        self._cache = {}

    def _index(self, p):
        return (int(round(p[0] / self.res)), int(round(p[1] / self.res)))

    def _inside(self, ix, iy):
        return (0 <= ix < self.n and 0 <= iy < self.n and not self.blocked[iy, ix])

    def free_point(self, p):
        ix, iy = self._index(p)
        return self._inside(ix, iy)

    def free_line(self, a, b):
        d = dist(a, b)
        num = max(1, int(math.ceil(d / (self.res * 0.42))))
        for i in range(num + 1):
            t = i / num
            if not self.free_point((a[0] * (1.0 - t) + b[0] * t,
                                    a[1] * (1.0 - t) + b[1] * t)):
                return False
        return True

    def _closest_cell(self, p):
        ix, iy = self._index(p)
        if self._inside(ix, iy):
            return ix, iy
        # Correct a few centimetres of perception / odometry error, but
        # never pretend a deeply blocked destination is reachable.
        radius = int(math.ceil(0.10 / self.res))
        candidates = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                cx, cy = ix + dx, iy + dy
                if self._inside(cx, cy):
                    candidates.append((math.hypot(cx * self.res - p[0],
                                                   cy * self.res - p[1]), (cx, cy)))
        return min(candidates)[1] if candidates else None

    def plan(self, start, goal):
        """Return [intermediate point(s), goal], or None if no safe route."""
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
        if self.free_line(start, goal):
            return [goal]
        s = self._closest_cell(start)
        g = self._closest_cell(goal)
        if s is None or g is None:
            return None

        # Use 8-connected A* with no diagonal corner cutting.
        neighbours = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                      (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        pq = [(math.hypot(g[0] - s[0], g[1] - s[1]), 0.0, s)]
        previous = {}
        best = {s: 0.0}
        found = False
        while pq:
            _, cost, cur = heapq.heappop(pq)
            if cost > best.get(cur, float('inf')) + 1e-9:
                continue
            if cur == g:
                found = True
                break
            for dx, dy, step_cost in neighbours:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not self._inside(nx, ny):
                    continue
                if dx and dy and (not self._inside(cur[0] + dx, cur[1]) or
                                  not self._inside(cur[0], cur[1] + dy)):
                    continue
                cand = cost + step_cost
                if cand + 1e-8 >= best.get((nx, ny), float('inf')):
                    continue
                best[(nx, ny)] = cand
                previous[(nx, ny)] = cur
                priority = cand + math.hypot(g[0] - nx, g[1] - ny)
                heapq.heappush(pq, (priority, cand, (nx, ny)))
        if not found:
            return None

        indices = [g]
        while indices[-1] != s:
            indices.append(previous[indices[-1]])
        indices.reverse()
        points = [(cx * self.res, cy * self.res) for cx, cy in indices]
        points[-1] = goal if self.free_point(goal) else points[-1]

        # Visibility smoothing removes zigzags and keeps a conservative
        # clearance because it checks the *already inflated* occupancy map.
        result = []
        cur = start
        i = 0
        while i < len(points):
            far = i
            for j in range(len(points) - 1, i - 1, -1):
                if self.free_line(cur, points[j]):
                    far = j
                    break
            if far == i and not self.free_line(cur, points[i]):
                # May occur when odometry is a few cm into the safety margin.
                # Stop instead of commanding an unsafe straight-line move.
                return None
            result.append(points[far])
            cur = points[far]
            i = far + 1
        return result

    def distance(self, a, b):
        """Collision-aware route length for waypoint ordering."""
        a = (round(a[0], 3), round(a[1], 3))
        b = (round(b[0], 3), round(b[1], 3))
        key = tuple(sorted((a, b)))
        if key not in self._cache:
            path = self.plan(a, b)
            self._cache[key] = float('inf') if path is None else sum(
                dist(u, v) for u, v in zip([a] + path[:-1], path))
        return self._cache[key]
