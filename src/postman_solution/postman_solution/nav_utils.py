"""
nav_utils.py - ROS-free navigation logic shared by the tier nodes.

Contains:
  * route planning (optimal order over a small set of points, ending at home)
  * Mission: state machine (drive -> dwell -> next target -> ... -> home -> done)
"""
import math
import itertools

V_MAX = 0.30
W_MAX = 1.5
HOME = (0.0, 0.0)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def route_length(start, order, end=HOME, dist_fn=dist) -> float:
    total, cur = 0.0, start
    for p in order:
        total += dist_fn(cur, p)
        cur = p
    return total + dist_fn(cur, end)


def best_order(start, pts, end=HOME, dist_fn=dist):
    """Shortest tour start -> all pts -> end.
    Exact (brute force) for <= 8 points, nearest-neighbour + 2-opt otherwise."""
    pts = list(pts)
    if len(pts) <= 1:
        return pts
    if len(pts) <= 8:
        best, best_len = None, float('inf')
        for perm in itertools.permutations(pts):
            L = route_length(start, perm, end, dist_fn)
            if L < best_len:
                best, best_len = list(perm), L
        # Every permutation can have infinite cost if the occupancy map
        # temporarily cannot connect one of the points. Don't return None
        # (which crashes the mission when it treats queue as a list).
        return best if best is not None else pts
    # fallback heuristic
    order, cur, left = [], start, pts[:]
    while left:
        nxt = min(left, key=lambda p: dist_fn(cur, p))
        order.append(nxt)
        left.remove(nxt)
        cur = nxt
    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if route_length(start, cand, end, dist_fn) + 1e-9 < route_length(start, order, end, dist_fn):
                    order, improved = cand, True
    return order


class Mission:
    """Visit every point in the queue (dwelling at each), then go home and dwell."""

    def __init__(self, dwell_time=2.6, arrive_radius=0.05, dedupe_radius=0.08, home_radius=0.16):
        self.dwell_time = dwell_time          # task needs >= 2.0 s, margin added
        self.arrive_radius = arrive_radius    # capture limit is 0.20 m, we aim much tighter
        self.dedupe_radius = dedupe_radius
        # The robot's front bumper sticks out 0.20 m ahead of its centre, so parking the
        # *centre* on (0,0) pushes the nose through the arena wall. Capture radius is 0.20 m,
        # so park the centre 0.16 m out instead (inside the zone, nose still inside the arena).
        self.home_radius = home_radius
        self.known = []        # every point ever received
        self.queue = []        # unvisited points, in planned order
        self.visited = []
        self.state = 'DRIVE'   # DRIVE | DWELL | DONE
        self.dwell_start = None
        self.need_replan = False
        self.rotating = False
        self.dwelling_home = False

    # ---------------------------------------------------------------- input
    def add_points(self, pts) -> int:
        added = 0
        for p in pts:
            p = (float(p[0]), float(p[1]))
            if any(dist(p, k) < self.dedupe_radius for k in self.known):
                continue
            self.known.append(p)
            self.queue.append(p)
            added += 1
        if added:
            self.need_replan = True
            if self.state == 'DONE':          # late point after finishing: resume
                self.state = 'DRIVE'
        return added

    # --------------------------------------------------------------- output
    @property
    def target(self):
        return self.queue[0] if self.queue else HOME

    def update(self, t, x, y, th):
        """Return (v, w) command for the current pose at sim-time t."""
        if self.state == 'DONE':
            return 0.0, 0.0

        if self.state == 'DWELL':
            if t - self.dwell_start < self.dwell_time:
                return 0.0, 0.0
            # dwell finished
            if self.dwelling_home and not self.queue:
                self.state = 'DONE'
                return 0.0, 0.0
            self.dwelling_home = False
            self.state = 'DRIVE'
            self.rotating = False

        # ------------------------------------------------ DRIVE
        if self.need_replan:
            # Completed targets are never touched; only unvisited ones get re-ordered.
            self.queue = best_order((x, y), self.queue)
            self.need_replan = False

        tx, ty = self.target
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        homing = not self.queue
        rad = self.home_radius if homing else self.arrive_radius

        if d < rad:
            self.state = 'DWELL'
            self.dwell_start = t
            self.rotating = False
            if self.queue:
                self.visited.append(self.queue.pop(0))
                self.dwelling_home = False
            else:
                self.dwelling_home = True
            return 0.0, 0.0

        err = wrap(math.atan2(dy, dx) - th)

        # rotate in place (reverse is not allowed) with hysteresis
        if not self.rotating and abs(err) > 0.6 and d > rad * 1.5:
            self.rotating = True
        if self.rotating and abs(err) < 0.10:
            self.rotating = False

        if self.rotating:
            w = math.copysign(clamp(2.5 * abs(err), 0.35, W_MAX), err)
            return 0.0, w

        if homing:
            v = clamp(1.2 * (d - rad) + 0.05, 0.05, 0.28)   # ease in so we stop before the wall
        else:
            v = clamp(1.2 * d + 0.05, 0.05, 0.28)
        v *= max(0.25, 1.0 - abs(err) / 0.6)       # slow down when not aligned
        w = clamp(3.0 * err, -W_MAX, W_MAX)
        if d < rad + 0.10:                          # avoid wiggling right at the target
            w = clamp(w, -0.4, 0.4)
        return clamp(v, 0.0, V_MAX), w
