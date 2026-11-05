'''Minimal 2D stand-in for the Gazebo course (random layouts with the same shape), used to test the user's node offline.
Course frame: robot starts at (0,0) facing +x, like the Gazebo world.'''
import math
import numpy as np

ROBOT_BODY = (-0.035, 0.18, 0.11)      # nanocar footprint from the pose origin
LIDAR_X = 0.114


def rect_segs(cx, cy, hx, hy, ang=0.0):
    c, s = math.cos(ang), math.sin(ang)
    pts = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    return [(*pts[k], *pts[(k + 1) % 4]) for k in range(4)]


class World:
    def __init__(self, rng, mode='random', swing_model='gazebo', n_static=None, door_w=None):
        self.rng = rng
        r = rng.uniform
        # ---- course outline (same topology as obstacle_course_world_fp_original.sdf)
        w1 = r(1.0, 1.4)                      # corridor 1 width (robot starts in its middle)
        xb = -r(0.45, 0.7)                    # back wall
        x1e = r(2.6, 3.4)                     # end of corridor 1
        jw = r(1.0, 1.4)                      # jog width
        xj = x1e - jw
        c2t = -w1 / 2 - r(1.0, 1.5)           # corridor 2 top (y)
        c2w = r(1.3, 1.8)
        c2b = c2t - c2w
        xd = x1e + r(0.45, 0.9)               # door wall
        sp = r(0.5, 0.75)                     # swing obstacle spacing
        xs1, xs2 = xd + sp, xd + 2 * sp
        xe = xs2 + r(0.5, 0.75)               # end wall
        poly = [(xb, w1 / 2), (x1e, w1 / 2), (x1e, c2t), (xe, c2t), (xe, c2b), (xj, c2b), (xj, -w1 / 2), (xb, -w1 / 2)]
        self.walls = [(*poly[k], *poly[(k + 1) % len(poly)]) for k in range(len(poly))]
        dw = door_w if door_w else r(0.30, 0.6)
        gy = r(c2b + 0.05, c2t - 0.05 - dw)
        self.walls += [(xd, c2b - 0.3, xd, gy), (xd, gy + dw, xd, c2t + 0.3)]
        self.door = (xd, gy, gy + dw)
        self.geom = dict(w1=w1, xb=xb, x1e=x1e, xj=xj, c2t=c2t, c2b=c2b, xd=xd, xs=(xs1, xs2), xe=xe)
        # ---- swinging obstacles: thin boxes moving along y at fixed x
        self.swing_model = swing_model
        self.swingers = []
        for xs in (xs1, xs2):
            hx = 0.025 if swing_model == 'gazebo' else r(0.03, 0.08)
            amp_lo, amp_hi = c2b + 0.2, c2t - 0.2
            if swing_model != 'gazebo':          # a real pendulum stays clear of the walls
                amp_lo, amp_hi = c2b + 0.22, c2t - 0.22
            self.swingers.append(dict(x=xs, hx=hx, hy=0.15, lo=amp_lo, hi=amp_hi, half=r(2.0, 6.0),
                                      phase=r(0, 1), T=r(2.6, 4.5), A=r(0.6, 1.0) * (amp_hi - amp_lo) / 2,
                                      released=swing_model == 'gazebo', t_rel=0.0))
        # ---- static obstacles
        self.circles, self.rects = [], []
        self._place_static(mode, n_static)

    # ------------------------------------------------------------------ static obstacles
    def _place_static(self, mode, n_static):
        g, r = self.geom, self.rng.uniform
        types = [('coke', 0.03), ('bin', 0.226), ('drawer', None), ('box', None)]
        n = n_static if n_static is not None else int(self.rng.integers(2, 9))
        for _ in range(400):
            self.circles, self.rects = [], []
            objs = []
            if mode == 'trap':          # U-shaped pocket in the jog, opening back towards the robot
                ux, uy = r(g['xj'] + 0.3, g['x1e'] - 0.3), r(g['c2t'] + 0.1, -g['w1'] / 2 - 0.1)
                objs += [('rect', ux + 0.25, uy, 0.04, 0.35, 0), ('rect', ux, uy + 0.33, 0.25, 0.04, 0), ('rect', ux, uy - 0.33, 0.25, 0.04, 0)]
            if mode == 'door':          # guard the door from the approach side
                xd, a, b = self.door
                objs += [('circle', xd - r(0.35, 0.6), r(a - 0.25, b + 0.25), 0.226)]
            if mode == 'gap':           # two obstacles leaving a narrow gap in corridor 1
                gx = r(0.5, g['xj'] - 0.3)
                gw = r(0.27, 0.36)
                gy = r(-g['w1'] / 2 + 0.05, g['w1'] / 2 - 0.05 - gw)
                objs += [('rect', gx, (-g['w1'] / 2 + gy) / 2, 0.15, (gy + g['w1'] / 2) / 2, 0),
                         ('rect', gx, (gy + gw + g['w1'] / 2) / 2, 0.15, (g['w1'] / 2 - gy - gw) / 2, 0)]
            for _k in range(n - len(objs)):
                name, rad = types[int(self.rng.integers(0, 4))]
                zone = self.rng.integers(0, 3)
                if zone == 0:
                    x, y = r(0.35, g['x1e'] - 0.2), r(-g['w1'] / 2, g['w1'] / 2)
                elif zone == 1:
                    x, y = r(g['xj'], g['x1e']), r(g['c2t'] - 0.3, -g['w1'] / 2)
                else:
                    x, y = r(g['xj'], g['xd'] - 0.15), r(g['c2b'], g['c2t'])
                if rad:
                    objs.append(('circle', x, y, rad))
                elif name == 'drawer':
                    objs.append(('rect', x, y, 0.205, 0.25, float(self.rng.choice([0, math.pi / 2, r(0, math.pi)]))))
                else:
                    objs.append(('rect', x, y, r(0.08, 0.25), r(0.08, 0.25), r(0, math.pi)))
            for o in objs:
                (self.circles if o[0] == 'circle' else self.rects).append(o[1:])
            if self._clear_of_start() and self.feasible():
                return
        raise RuntimeError('could not place a feasible layout')

    def _clear_of_start(self):
        for (x, y, rad) in self.circles:
            if math.hypot(x - 0.07, y) < rad + 0.25:
                return False
        for (x, y, hx, hy, a) in self.rects:
            if math.hypot(x - 0.07, y) < math.hypot(hx, hy) + 0.25:
                return False
        return True

    def static_segs(self):
        segs = list(self.walls)
        for rc in self.rects:
            segs += rect_segs(*rc)
        return segs

    def feasible(self, res=0.025):
        '''Is there a path for the (axis-aligned, square) robot from the start to past the swing zone?'''
        _dilate = _dilate8
        g = self.geom
        x0, y0 = g['xb'] - 0.3, g['c2b'] - 0.5
        nx, ny = int((g['xe'] + 0.3 - x0) / res), int((g['w1'] / 2 + 0.3 - y0) / res)
        occ = np.zeros((nx, ny), bool)
        for (ax, ay, bx, by) in self.static_segs():
            t = np.linspace(0, 1, int(math.hypot(bx - ax, by - ay) / 0.005) + 2)
            i = ((ax + t * (bx - ax) - x0) / res).astype(int); j = ((ay + t * (by - ay) - y0) / res).astype(int)
            k = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
            occ[i[k], j[k]] = True
        X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * res, y0 + (np.arange(ny) + 0.5) * res, indexing='ij')
        for (cx, cy, rad) in self.circles:
            occ |= np.hypot(X - cx, Y - cy) <= rad
        # every gap is at least robot width + 8 cm: grow obstacles by the robot rectangle + 4 cm
        rx = int(math.ceil(((ROBOT_BODY[1] - ROBOT_BODY[0]) / 2 + 0.04) / res)); ry = int(math.ceil((ROBOT_BODY[2] + 0.04) / res))
        s_ = np.zeros((nx + 1, ny + 1), np.int32); s_[1:, 1:] = occ.cumsum(0).cumsum(1)
        i = np.arange(nx); j = np.arange(ny)
        i0, i1 = np.clip(i - rx, 0, nx), np.clip(i + rx + 1, 0, nx); j0, j1 = np.clip(j - ry, 0, ny), np.clip(j + ry + 1, 0, ny)
        blocked = (s_[i1][:, j1] - s_[i0][:, j1] - s_[i1][:, j0] + s_[i0][:, j0]) > 0
        si, sj = int((0.0725 - x0) / res), int((0 - y0) / res)
        if blocked[si, sj]:
            return False
        seen = np.zeros_like(blocked); seen[si, sj] = True
        front = seen.copy()
        goal = X > g['xs'][1] + 0.2
        while front.any():
            front = _dilate(front) & ~blocked & ~seen
            seen |= front
            if (seen & goal).any():
                return True
        return False

    # ------------------------------------------------------------------ swinging obstacles
    def swinger_y(self, s, t):
        if self.swing_model == 'gazebo':      # linear there-and-back, like the Gazebo actors
            u = ((t / s['half'] + s['phase']) % 2.0)
            f = u if u < 1 else 2 - u
            return s['lo'] + f * (s['hi'] - s['lo'])
        mid = 0.5 * (s['lo'] + s['hi'])
        if not s['released']:
            return mid + s['A']                # held at one side until the robot reaches the door
        tt = t - s['t_rel']
        A = s['A'] * (0.85 + 0.15 * math.sin(0.37 * tt + s['phase'] * 6))  # hand pushes change the amplitude
        return float(np.clip(mid + A * math.cos(2 * math.pi * tt / s['T']), s['lo'], s['hi']))   # walls stop the swing

    def update_release(self, robot_x, t):
        for s in self.swingers:
            if not s['released'] and robot_x > self.door[0] - 0.35:
                s['released'], s['t_rel'] = True, t

    def dyn_segs(self, t):
        segs = []
        for s in self.swingers:
            segs += rect_segs(s['x'], self.swinger_y(s, t), s['hx'], s['hy'])
        return segs

    # ------------------------------------------------------------------ sensing
    def lidar(self, x, y, th, t, n=360, noise=0.0):
        ox, oy = x + LIDAR_X * math.cos(th), y + LIDAR_X * math.sin(th)
        a = th + np.arange(n) * 2 * math.pi / n
        dx, dy = np.cos(a), np.sin(a)
        S = np.array(self.static_segs() + self.dyn_segs(t))
        ax, ay, bx, by = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
        ex, ey = bx - ax, by - ay
        den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
        wx, wy = ax[None, :] - ox, ay[None, :] - oy
        with np.errstate(divide='ignore', invalid='ignore'):
            tt = (wx * ey[None, :] - wy * ex[None, :]) / den
            uu = (wx * dy[:, None] - wy * dx[:, None]) / den
        tt = np.where((np.abs(den) > 1e-12) & (uu >= 0) & (uu <= 1) & (tt > 0), tt, np.inf)
        rng = tt.min(1)
        for (cx, cy, rad) in self.circles:
            fx, fy = ox - cx, oy - cy
            b = dx * fx + dy * fy
            c = fx * fx + fy * fy - rad * rad
            disc = b * b - c
            hit = np.where(disc >= 0, -b - np.sqrt(np.maximum(disc, 0)), np.inf)
            rng = np.minimum(rng, np.where(hit > 0, hit, np.inf))
        if noise:
            rng = rng + self.rng.normal(0, noise, n)
        return np.where((rng > 0.05) & (rng < 10.0), rng, np.inf)

    # ------------------------------------------------------------------ collisions
    def collides(self, x, y, th, t, margin=0.0):
        c, s = math.cos(th), math.sin(th)
        x0, x1, w = ROBOT_BODY
        x0 -= margin; x1 += margin; w += margin

        def to_robot(px, py):
            ddx, ddy = px - x, py - y
            return c * ddx + s * ddy, -s * ddx + c * ddy

        def seg_hits(seg):
            ax, ay = to_robot(seg[0], seg[1]); bx, by = to_robot(seg[2], seg[3])
            # clip segment against the rectangle (Liang-Barsky)
            t0, t1 = 0.0, 1.0
            dx_, dy_ = bx - ax, by - ay
            for p, q in ((-dx_, ax - x0), (dx_, x1 - ax), (-dy_, ay + w), (dy_, w - ay)):
                if abs(p) < 1e-12:
                    if q < 0:
                        return False
                else:
                    rr = q / p
                    if p < 0:
                        t0 = max(t0, rr)
                    else:
                        t1 = min(t1, rr)
                    if t0 > t1:
                        return False
            return True

        for seg in self.static_segs():
            if seg_hits(seg):
                return 'static'
        for (cx, cy, rad) in self.circles:
            px, py = to_robot(cx, cy)
            if math.hypot(max(x0 - px, 0, px - x1), max(abs(py) - w, 0)) < rad:
                return 'static'
        for seg in self.dyn_segs(t):
            if seg_hits(seg):
                return 'swing'
        return None




def _dilate8(m):
    '''One step of 8-neighbour binary dilation (used by World.feasible).'''
    d = m.copy()
    d[1:, :] |= m[:-1, :]
    d[:-1, :] |= m[1:, :]
    e = d.copy()
    e[:, 1:] |= d[:, :-1]
    e[:, :-1] |= d[:, 1:]
    return e
