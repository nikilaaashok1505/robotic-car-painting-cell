#!/usr/bin/env python3
"""Paint-swath visualiser for the painting cell (RViz).

Watches the real nozzle position (TF: tool0 + tool_offset along the tool axis) and, wherever the
nozzle is on the planned painting path (/painting_paths), lays down a wide flat 'paint' patch on the
surface.  Only places the robot really reached get painted, so the picture is also a coverage check.

Publishes:  /paint_swath  (MarkerArray)   - add this topic as a MarkerArray display in RViz
Resets:     ros2 topic pub --once /paint_reset std_msgs/msg/Empty "{}"
"""
import numpy as np

AXES = {'x': (1, 0, 0), '-x': (-1, 0, 0), 'y': (0, 1, 0), '-y': (0, -1, 0), 'z': (0, 0, 1), '-z': (0, 0, -1)}


def densify(points, closed_pairs, step):
    """points: (N,3); closed_pairs: list of (i, j) segments. Returns (M,3) points every <= step."""
    out = []
    for i, j in closed_pairs:
        a, b = points[i], points[j]
        n = max(int(np.ceil(np.linalg.norm(b - a) / step)), 1)
        for k in range(n):
            out.append(a + (b - a) * (k / n))
    out.append(points[closed_pairs[-1][1]])
    return np.array(out)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PaintState:
    """ROS-free logic: axis auto-detection + painted-cell bookkeeping (so it can be tested alone)."""

    def __init__(self, path_pts, path_rgb, spray_thr=0.04, lock_thr=0.03, lock_count=20, fixed_axis=None):
        self.P = np.asarray(path_pts, float)
        self.rgb = path_rgb
        self.spray_thr, self.lock_thr, self.lock_count = spray_thr, lock_thr, lock_count
        self.names = list(AXES)
        self.fixed = fixed_axis
        self.reset()

    def reset(self):
        self.counts = {n: 0 for n in self.names}
        self.pending = {n: {} for n in self.names}   # axis -> {path_idx: (normal_axis_idx, sign)}
        self.locked = self.fixed
        self.painted = {}                            # path_idx -> (normal_axis_idx, sign)

    def dists(self, p):
        return np.linalg.norm(self.P - p, axis=1)

    def update(self, tool_pos, R, tool_offset):
        """Feed one TF sample. Returns True if something new was painted."""
        new = False
        for n in self.names:
            ax = np.array(AXES[n], float)
            direction = R @ ax                       # world direction the nozzle points to
            nozzle = tool_pos + direction * tool_offset
            d = self.dists(nozzle)
            if d.min() < self.lock_thr:
                self.counts[n] += 1
            near = np.where(d < self.spray_thr)[0]
            if len(near):
                k = int(np.argmax(np.abs(direction)))
                v = (k, 1 if direction[k] > 0 else -1)
                for j in near:
                    self.pending[n][int(j)] = v
        if self.locked is None:
            best = max(self.counts, key=self.counts.get)
            others = sorted(self.counts.values())[-2]
            if self.counts[best] >= self.lock_count and self.counts[best] >= 2 * max(others, 1):
                self.locked = best
        if self.locked is not None:
            pend = self.pending[self.locked]
            if pend:
                for j, v in pend.items():
                    if j not in self.painted:
                        self.painted[j] = v
                        new = True
                pend.clear()
        return new

    def coverage(self):
        return len(self.painted) / max(len(self.P), 1)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from std_msgs.msg import Empty
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from tf2_ros import Buffer, TransformListener

    class PaintSwath(Node):
        def __init__(self):
            super().__init__('paint_swath')
            for k, v in [('frame', 'world'), ('tip_link', 'tool0'), ('tool_offset', 0.27), ('tool_axis', 'auto'),
                         ('swath', 0.16), ('push', 0.0), ('thickness', 0.012), ('spray_tol', 0.04), ('paths_topic', '/painting_paths')]:
                self.declare_parameter(k, v)
            g = lambda k: self.get_parameter(k).value
            self.frame, self.tip, self.off = g('frame'), g('tip_link'), float(g('tool_offset'))
            self.swath, self.thick = float(g('swath')), float(g('thickness'))
            self.push = float(g('push'))
            self.axis = None if g('tool_axis') == 'auto' else g('tool_axis')
            self.tol = float(g('spray_tol'))
            self.state = None
            self.dirty = False
            self.tf_buf = Buffer()
            self.tf = TransformListener(self.tf_buf, self)
            self.pub = self.create_publisher(MarkerArray, '/paint_swath', 10)
            self.create_subscription(MarkerArray, g('paths_topic'), self.on_paths, 10)
            self.create_subscription(Empty, '/paint_reset', self.on_reset, 10)
            self.create_timer(0.05, self.tick)
            self.create_timer(0.25, self.publish)
            self.create_timer(1.0, lambda: self.publish(force=True))
            self.create_timer(5.0, self.report)
            self.get_logger().info('waiting for planned paths on %s ...' % g('paths_topic'))

        def on_reset(self, _):
            if self.state:
                self.state.reset()
                self.dirty = True
                self.get_logger().info('paint cleared')

        def on_paths(self, msg):
            if self.state is not None:
                return
            pts, rgb = [], []
            for m in msg.markers:
                if m.action != Marker.ADD or len(m.points) < 2:
                    continue
                P = np.array([[p.x, p.y, p.z] for p in m.points])
                if m.type == Marker.LINE_STRIP:
                    pairs = [(i, i + 1) for i in range(len(P) - 1)]
                elif m.type == Marker.LINE_LIST:
                    pairs = [(i, i + 1) for i in range(0, len(P) - 1, 2)]
                else:
                    pairs = [(i, min(i + 1, len(P) - 1)) for i in range(len(P))]
                D = densify(P, pairs, 0.02)
                pts.append(D)
                rgb += [(m.color.r, m.color.g, m.color.b)] * len(D)
            if not pts:
                return
            self.state = PaintState(np.vstack(pts), rgb, spray_thr=self.tol, fixed_axis=self.axis)
            self.get_logger().info('planned path loaded: %d points over %d markers' % (len(self.state.P), len(msg.markers)))

        def tick(self):
            if self.state is None:
                return
            try:
                t = self.tf_buf.lookup_transform(self.frame, self.tip, Time(), timeout=Duration(seconds=0.0))
            except Exception:
                return
            tr, q = t.transform.translation, t.transform.rotation
            pos = np.array([tr.x, tr.y, tr.z])
            if self.state.update(pos, quat_to_R([q.x, q.y, q.z, q.w]), self.off):
                self.dirty = True
                if self.axis is None and self.state.locked and not getattr(self, '_said', False):
                    self.get_logger().info("tool axis detected: '%s'" % self.state.locked)
                    self._said = True

        def publish(self, force=False):
            if self.state is None or not (self.dirty or force):
                return
            self.dirty = False
            groups = {}
            for j, (k, s) in self.state.painted.items():
                rgb = tuple(round(c, 3) for c in self.state.rgb[j])
                groups.setdefault((k, rgb), []).append(j)
            arr = MarkerArray()
            clear = Marker(); clear.action = Marker.DELETEALL; clear.header.frame_id = self.frame
            arr.markers.append(clear)
            for gid, ((k, rgb), idxs) in enumerate(groups.items()):
                m = Marker()
                m.header.frame_id = self.frame
                m.ns, m.id = 'swath', gid
                m.type, m.action = Marker.CUBE_LIST, Marker.ADD
                size = [self.swath] * 3
                size[k] = self.thick
                m.scale.x, m.scale.y, m.scale.z = size
                m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
                for j in idxs:
                    _, s = self.state.painted[j]
                    p = self.state.P[j].copy()
                    p[k] += s * (self.push - self.thick * 0.5 - 0.004)   # move patch from the nozzle path onto the car surface
                    m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                arr.markers.append(m)
            self.pub.publish(arr)

        def report(self):
            if self.state:
                self.get_logger().info('painted %.1f %% of planned path (axis: %s)' %
                                       (100 * self.state.coverage(), self.state.locked or 'detecting'))

    rclpy.init()
    node = PaintSwath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
