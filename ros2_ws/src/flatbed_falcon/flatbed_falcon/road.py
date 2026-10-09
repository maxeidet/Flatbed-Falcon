"""The closed road loop from worlds/road_centerline.csv as a Lab 3 SplinePath.

SplinePath is for open paths, so the loop is unrolled with a margin of road copied
before the start and after the end. Positions on the loop are s in [0, length);
every method here takes any s and wraps it, so s + lookahead past the seam works.
No ROS in this file, so it runs in the offline tests.
"""
import numpy as np

from flatbed_falcon.splinepath import SplinePath


class RoadLoop:
    def __init__(self, csv_path, margin=60.0):
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        s, xy, curvature, zone = data[:, 0], data[:, 1:3], data[:, 4], data[:, 5].astype(int)
        closing = np.hypot(*(xy[0] - xy[-1]))
        csv_length = s[-1] + closing

        # Unroll: [last `margin` m] + [whole loop] + [point 0 again] + [first `margin` m]
        head = s > csv_length - margin
        tail = s < margin
        points = np.vstack((xy[head], xy, xy[:1], xy[tail][1:]))
        self.spline = SplinePath(points)

        # SplinePath measures s along the chords, so take the loop length and the
        # internal position of point 0 from the same chords.
        chord_s = np.hstack(([0.0], np.cumsum(np.hypot(*np.diff(points, axis=0).T))))
        n_head = int(head.sum())
        self._s0 = chord_s[n_head]
        self.length = chord_s[n_head + len(xy)] - self._s0
        scale = self.length / csv_length

        # Curvature and landing zones from the CSV (exact: 0 on straights, 1/R on arcs),
        # on the same s scale as the spline.
        self._csv_s = s * scale
        self._curvature = curvature
        self._zone = zone
        self.zones = {}  # id -> (s_start, s_end)
        for zid in sorted(set(zone) - {-1}):
            idx = np.flatnonzero(zone == zid)
            end = self._csv_s[idx[-1] + 1] if idx[-1] + 1 < len(s) else self.length
            self.zones[zid] = (self._csv_s[idx[0]], end)

    def wrap(self, s):
        return np.mod(s, self.length)

    def _index(self, s):
        return np.clip(np.searchsorted(self._csv_s, self.wrap(s), side='right') - 1, 0, len(self._csv_s) - 1)

    def point(self, s):
        return self.spline.p(self._s0 + self.wrap(s))

    def heading(self, s):
        """Unit tangent at s."""
        return self.spline.heading(self._s0 + self.wrap(s))[0]

    def curvature(self, s):
        return self._curvature[self._index(s)]

    def project(self, p, s_guess):
        """(s, d): position of p's projection on the loop and signed distance
        (positive = left of the driving direction). Searches near s_guess."""
        s_int, d = self.spline.project(np.asarray(p, dtype=float), self._s0 + self.wrap(s_guess),
                                       ds=1.0, s_lim=20)
        return float(self.wrap(s_int - self._s0)), float(d)

    def locate(self, p):
        """Global projection (no guess): the closest CSV point, refined. For start-up."""
        csv_points = self.point(self._csv_s)
        k = int(np.argmin(np.sum((csv_points - np.asarray(p)) ** 2, axis=1)))
        return self.project(p, self._csv_s[k])

    def zone_at(self, s):
        """(zone id, metres left in it), or (-1, 0.0) outside every landing zone."""
        s = float(self.wrap(s))
        for zid, (start, end) in self.zones.items():
            if start <= s < end:
                return zid, end - s
        return -1, 0.0

    def distance_ahead(self, s_from, s_to):
        """Distance driven from s_from to s_to (always forward, so in [0, length))."""
        return float(self.wrap(s_to - s_from))
