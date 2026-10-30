#!/usr/bin/env python3
"""
Multi-drone frontier exploration (sim; same API as the real drones).

Every drone's 2D LiDAR is ray-traced into ONE shared occupancy grid in the world frame. Drones
repeatedly pick the cheapest-to-reach frontier (free space bordering unknown space), plan a path
on the grid that keeps clear of walls and of each other, and fly it. When no reachable frontier
is left, the map is saved and all drones land.

  ros2 launch swarm sim.launch.py world:=maze num_drones:=3 y_spacing:=1.0
  rviz2 -d explore.rviz --ros-args -p use_sim_time:=true
  python3 frontier_explore.py --num-drones 3

Published for RViz (frame "map"):
  /map                         nav_msgs/OccupancyGrid   shared map
  /exploration/markers         MarkerArray              frontier cells + each drone's goal
  /<drone>/plan, /<drone>/trail  nav_msgs/Path          planned path, flown trail

Mapping uses the drones' own pose estimates (MAVROS local position + swarm offsets). In sim
those are near ground truth, so the maps of all drones line up without SLAM.

Safety (same as the other missions): any pilot override stops the mission and the other drones
land; Ctrl+C, errors, a separation breach or the time limit land every drone.
"""
import argparse
import math
import os
import sys
import time
import traceback

import numpy as np
try:
    from scipy import ndimage
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra
except ImportError:
    sys.exit("This script needs SciPy:  sudo apt install python3-scipy")

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

from swarm.drone_api import Swarm, ManualOverride
from mission_utils import MissionAbort, check_all, land_all_safely, send_world, takeoff_all

TAG = "[EXPLORE]"
LOOP_HZ = 10.0           # control / mapping loop
REPLAN_PERIOD = 1.5      # s between frontier + path updates
VIZ_PERIOD = 0.5         # s between RViz updates
GOAL_TOL = 0.5           # m, goal reached
GOAL_TIMEOUT = 45.0      # s, give up on a goal and blacklist it
MAX_TILT = 0.12          # rad, skip scans taken while tilted more than this (scan plane not level)
CLEARANCE = 0.6          # m, hard minimum distance from walls for planned paths (iris ~0.4 m radius)
SOFT_CLEARANCE = 1.0     # m, paths prefer to stay at least this far from walls
DRONE_BLOCK = 1.2        # m, other drones block the grid within this radius when planning
YIELD_DIST = 1.8         # m, lower-priority drone holds when closer than this to another drone
ABORT_SEP = 0.8          # m, horizontal separation that aborts the mission
MIN_CLUSTER = 6          # cells, smallest frontier cluster worth visiting
GAIN = 0.15              # m of path cost saved per frontier cell (prefers bigger frontiers)
CROWD_DIST = 3.0         # m, frontier goals closer than this to another drone's goal ...
CROWD_COST = 8.0         # ... cost this much extra
DONE_TIME = 6.0          # s with no reachable frontier before exploration is finished
COLORS = [(0.95, 0.25, 0.25), (0.2, 0.8, 0.3), (0.25, 0.55, 1.0), (1.0, 0.75, 0.1), (0.8, 0.3, 0.9)]


# =============================================================================
# Occupancy grid
# =============================================================================
class OccupancyMap:
    """Log-odds grid indexed [ix, iy] (ix along world x)."""
    L_HIT, L_MISS, L_MIN, L_MAX, L_OCC, L_FREE = 0.9, -0.4, -4.0, 4.0, 0.7, -0.7

    def __init__(self, xmin, xmax, ymin, ymax, res):
        self.res, self.x0, self.y0 = res, xmin, ymin
        self.nx = int(math.ceil((xmax - xmin) / res))
        self.ny = int(math.ceil((ymax - ymin) / res))
        self.L = np.zeros((self.nx, self.ny), np.float32)

    def cell(self, x, y):
        return (np.floor((np.asarray(x) - self.x0) / self.res).astype(int),
                np.floor((np.asarray(y) - self.y0) / self.res).astype(int))

    def center(self, ix, iy):
        return self.x0 + (np.asarray(ix) + 0.5) * self.res, self.y0 + (np.asarray(iy) + 0.5) * self.res

    def inside(self, ix, iy):
        return (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)

    def occupied(self):
        return self.L > self.L_OCC

    def free(self):
        return self.L < self.L_FREE

    def unknown(self):
        return ~(self.occupied() | self.free())

    def _bump(self, ix, iy, delta):
        ok = self.inside(ix, iy)
        lin = np.unique(ix[ok] * self.ny + iy[ok])
        if delta < 0:
            # rays skimming along a thin wall pass through its partly-free edge cells; don't let
            # them erase cells that are already confidently walls (keeps walls solid in RViz)
            lin = lin[self.L.flat[lin] < 2.0 * self.L_OCC]
        self.L.flat[lin] += delta

    def integrate(self, ox, oy, yaw, scan, max_range, ignore_xy=()):
        """Ray-trace one LaserScan taken at world (ox, oy) with heading yaw."""
        r = np.asarray(scan.ranges, float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment + yaw
        rmax = min(float(scan.range_max), max_range)
        valid = ~np.isnan(r) & (~np.isfinite(r) | (r >= scan.range_min))
        hit = valid & np.isfinite(r) & (r < rmax)
        length = np.where(hit, r, rmax)
        ca, sa = np.cos(a), np.sin(a)

        t = np.arange(0.0, rmax, self.res * 0.5)[None, :]
        m = valid[:, None] & (t < length[:, None] - self.res)
        fx, fy = (ox + t * ca[:, None])[m], (oy + t * sa[:, None])[m]
        self._bump(*self.cell(fx, fy), self.L_MISS)

        hx, hy = ox + length[hit] * ca[hit], oy + length[hit] * sa[hit]
        for px, py in ignore_xy:                        # don't map other drones as walls
            keep = np.hypot(hx - px, hy - py) > 0.6
            hx, hy = hx[keep], hy[keep]
        self._bump(*self.cell(hx, hy), self.L_HIT)
        np.clip(self.L, self.L_MIN, self.L_MAX, out=self.L)

    def to_msg_data(self):
        occ, free = self.occupied(), self.free()
        g = np.full(self.L.shape, -1, np.int8)
        g[free], g[occ] = 0, 100
        return g.T.ravel().tolist()                     # row-major, x fastest

    def save(self, stem):
        occ, free = self.occupied(), self.free()
        img = np.full(self.L.shape, 205, np.uint8)
        img[free], img[occ] = 254, 0
        img = np.flipud(img.T)                          # PGM row 0 = top (max y)
        with open(stem + ".pgm", "wb") as f:
            f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            f.write(img.tobytes())
        with open(stem + ".yaml", "w") as f:
            f.write(f"image: {os.path.basename(stem)}.pgm\nresolution: {self.res}\n"
                    f"origin: [{self.x0}, {self.y0}, 0.0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")


# =============================================================================
# Frontiers and planning
# =============================================================================
def frontier_cells(m: OccupancyMap):
    unk = m.unknown()
    near_unknown = np.zeros_like(unk)
    near_unknown[1:, :] |= unk[:-1, :]
    near_unknown[:-1, :] |= unk[1:, :]
    near_unknown[:, 1:] |= unk[:, :-1]
    near_unknown[:, :-1] |= unk[:, 1:]
    fr = m.free() & near_unknown
    fr[0, :] = fr[-1, :] = fr[:, 0] = fr[:, -1] = False
    return fr


def frontier_clusters(m: OccupancyMap, fr, trav):
    """List of (goal_cell, size) per frontier cluster; goal = traversable cell near its centre."""
    lab, n = ndimage.label(fr, structure=np.ones((3, 3), int))
    if n == 0:
        return []
    cells = np.argwhere(fr)
    labs = lab[fr]
    order = np.argsort(labs, kind="stable")
    cells, labs = cells[order], labs[order]
    splits = np.flatnonzero(np.diff(labs)) + 1
    out = []
    # traversable cells near frontier cells (frontier cells themselves are often too close to walls)
    near = ndimage.binary_dilation(fr, iterations=int(round(0.5 / m.res)))
    for group in np.split(cells, splits):
        if len(group) < MIN_CLUSTER:
            continue
        c = group.mean(axis=0)
        x0, y0 = group.min(axis=0) - 6
        x1, y1 = group.max(axis=0) + 7
        x0, y0 = max(x0, 0), max(y0, 0)
        sub = np.argwhere(trav[x0:x1, y0:y1] & near[x0:x1, y0:y1]) + [x0, y0]
        if len(sub) == 0:
            continue
        # pick a candidate cell close to the cluster centre and adjacent to this cluster
        d_c = np.hypot(*(sub - c).T)
        d_g = np.min(np.hypot(sub[:, None, 0] - group[None, :, 0], sub[:, None, 1] - group[None, :, 1]), axis=1) \
            if len(sub) * len(group) < 400000 else d_c
        ok = d_g * m.res <= 0.55
        if not ok.any():
            continue
        best = sub[ok][np.argmin(d_c[ok])]
        out.append(((int(best[0]), int(best[1])), len(group)))
    return out


class Planner:
    """Shortest paths on the grid (SciPy Dijkstra), soft wall-clearance cost."""

    def __init__(self, m: OccupancyMap):
        self.m = m
        self.base_trav = self.cost = None

    def update(self):
        m = self.m
        dist = ndimage.distance_transform_edt(~m.occupied()) * m.res
        self.base_trav = m.free() & (dist >= CLEARANCE)
        self.cost = 1.0 + 4.0 * np.clip((SOFT_CLEARANCE - dist) / (SOFT_CLEARANCE - CLEARANCE), 0.0, 1.0)

    def _disc(self, trav, x, y, radius, value):
        m = self.m
        r = int(math.ceil(radius / m.res))
        cx, cy = m.cell(x, y)
        xs, ys = np.arange(cx - r, cx + r + 1), np.arange(cy - r, cy + r + 1)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        inside = m.inside(X, Y) & (np.hypot(X - cx, Y - cy) * m.res <= radius)
        trav[X[inside], Y[inside]] = value

    def search(self, start_xy, others_xy):
        """Dijkstra from start over traversable cells. Returns (dist grid in m, predecessor fn)."""
        m = self.m
        trav = self.base_trav.copy()
        for ox, oy in others_xy:
            self._disc(trav, ox, oy, DRONE_BLOCK, False)
        # let the drone leave its own spot even if it is a bit too close to a wall
        start_ok = trav.copy()
        self._disc(start_ok, *start_xy, 0.45, True)
        trav = (start_ok & ~m.occupied()) | trav
        sx, sy = m.cell(*start_xy)
        if not m.inside(sx, sy):
            return None, None, None
        trav[sx, sy] = True

        ids = -np.ones(trav.shape, int)
        ids[trav] = np.arange(trav.sum())
        src, dst, w = [], [], []
        nx, ny = trav.shape
        for dx, dy in [(1, 0), (0, 1), (1, 1), (1, -1)]:
            a = (slice(max(0, -dx), nx - max(0, dx)), slice(max(0, -dy), ny - max(0, dy)))
            b = (slice(max(0, dx), nx - max(0, -dx)), slice(max(0, dy), ny - max(0, -dy)))
            both = trav[a] & trav[b]
            ia, ib = ids[a][both], ids[b][both]
            wt = m.res * math.hypot(dx, dy) * 0.5 * (self.cost[a][both] + self.cost[b][both])
            src += [ia, ib]; dst += [ib, ia]; w += [wt, wt]
        n = int(trav.sum())
        G = csr_matrix((np.concatenate(w), (np.concatenate(src), np.concatenate(dst))), shape=(n, n))
        d, pred = dijkstra(G, indices=ids[sx, sy], return_predecessors=True)
        dist = np.full(trav.shape, np.inf)
        dist[trav] = d
        coords = np.argwhere(trav)                         # id -> cell (same order as ids)

        def path_to(cell):
            i = ids[cell]
            if i < 0 or not np.isfinite(d[i]):
                return None
            out = []
            while i >= 0:
                out.append(coords[i])
                i = pred[i]
            out.reverse()
            return np.array(out)
        return dist, path_to, trav


# =============================================================================
# Drones
# =============================================================================
class Agent:
    def __init__(self, drone, idx, z_local, yaw):
        self.d, self.idx, self.z, self.yaw = drone, idx, z_local, yaw
        self.goal = None            # (x, y) world
        self.goal_t0 = 0.0
        self.path = np.zeros((0, 2))
        self.trav = None           # traversable grid from this drone's last search
        self.blacklist = []
        self.trail = []
        self.last_stamp = None
        self.holding = False

    def xy(self):
        x, y, _ = self.d.get_pos(world=True)
        return x, y

    def carrot(self, lookahead, m, others_xy):
        """Farthest point up to `lookahead` m along the path that can be reached in a straight
        line through cleared space and away from other drones (no corner cutting)."""
        if len(self.path) == 0:
            return None
        p = np.array(self.xy())
        i = int(np.argmin(np.hypot(*(self.path - p).T)))
        rest = self.path[i:]
        cum = np.r_[0.0, np.cumsum(np.hypot(*np.diff(rest, axis=0).T))] if len(rest) > 1 else np.zeros(1)
        cand = rest[cum <= lookahead]
        for q in cand[::-1]:
            if self._clear_line(p, q, m, others_xy):
                return tuple(q)
        return tuple(rest[0])

    def _clear_line(self, p, q, m, others_xy):
        n = max(2, int(np.hypot(*(q - p)) / (m.res * 0.5)) + 1)
        xs, ys = np.linspace(p[0], q[0], n), np.linspace(p[1], q[1], n)
        if self.trav is not None:
            ix, iy = m.cell(xs, ys)
            ok = m.inside(ix, iy)
            if not ok.all() or not self.trav[ix, iy].all():
                return False
        for ox, oy in others_xy:
            # only forbid getting closer to a drone that is already near the line
            if np.min(np.hypot(xs - ox, ys - oy)) < DRONE_BLOCK and \
               np.hypot(q[0] - ox, q[1] - oy) < np.hypot(p[0] - ox, p[1] - oy):
                return False
        return True


def assign_goals(agents, m, planner, now):
    """Update every agent's goal and path. Returns number of usable frontier clusters."""
    planner.update()
    fr = frontier_cells(m)
    clusters = frontier_clusters(m, fr, planner.base_trav)
    taken = []
    usable = 0
    for ag in agents:
        others = [o.xy() for o in agents if o is not ag]
        dist, path_to, ag.trav = planner.search(ag.xy(), others)
        if dist is None:
            ag.goal, ag.path = None, np.zeros((0, 2))
            continue

        # keep the current goal while it still borders unknown space and is reachable
        keep = False
        if ag.goal is not None:
            gx, gy = m.cell(*ag.goal)
            r = int(round(0.7 / m.res))
            still_frontier = fr[max(gx - r, 0):gx + r + 1, max(gy - r, 0):gy + r + 1].any()
            timed_out = now - ag.goal_t0 > GOAL_TIMEOUT
            if timed_out:
                print(f"\n{TAG} {ag.d.name}: goal ({ag.goal[0]:.1f}, {ag.goal[1]:.1f}) timed out, skipping it")
                ag.blacklist.append(ag.goal)
            keep = still_frontier and not timed_out and np.isfinite(dist[gx, gy])

        if not keep:
            best, best_score = None, np.inf
            for cell, size in clusters:
                if not np.isfinite(dist[cell]):
                    continue
                wx, wy = (float(v) for v in m.center(*cell))
                if any(math.hypot(wx - bx, wy - by) < 0.8 for bx, by in ag.blacklist):
                    continue
                score = dist[cell] - GAIN * size
                if any(math.hypot(wx - tx, wy - ty) < CROWD_DIST for tx, ty in taken):
                    score += CROWD_COST
                if score < best_score:
                    best, best_score = (wx, wy), score
            if best != ag.goal:
                ag.goal_t0 = now
            ag.goal = best

        if ag.goal is None:
            ag.path = np.zeros((0, 2))
            continue
        usable += 1
        taken.append(ag.goal)
        cells = path_to(tuple(m.cell(*ag.goal)))
        if cells is None:
            ag.path = np.zeros((0, 2))
            continue
        cx, cy = m.center(cells[:, 0], cells[:, 1])
        ag.path = np.c_[cx, cy]
    return usable, fr


# =============================================================================
# RViz output
# =============================================================================
class Viz:
    def __init__(self, names, sim_time):
        self.node = Node("frontier_explore_viz",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim_time)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.map_pub = self.node.create_publisher(OccupancyGrid, "/map", latched)
        self.mk_pub = self.node.create_publisher(MarkerArray, "/exploration/markers", 10)
        self.plan_pub = {n: self.node.create_publisher(Path, f"/{n}/plan", 10) for n in names}
        self.trail_pub = {n: self.node.create_publisher(Path, f"/{n}/trail", 10) for n in names}

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def _path(self, pts, z, stamp):
        msg = Path()
        msg.header.frame_id, msg.header.stamp = "map", stamp
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def publish(self, m: OccupancyMap, fr, agents):
        stamp = self._stamp()
        g = OccupancyGrid()
        g.header.frame_id, g.header.stamp = "map", stamp
        g.info.resolution = float(m.res)
        g.info.width, g.info.height = m.nx, m.ny
        g.info.origin.position.x, g.info.origin.position.y = float(m.x0), float(m.y0)
        g.info.origin.orientation.w = 1.0
        g.data = m.to_msg_data()
        self.map_pub.publish(g)

        arr = MarkerArray()
        mk = Marker()
        mk.header.frame_id, mk.header.stamp = "map", stamp
        mk.ns, mk.id, mk.type, mk.action = "frontiers", 0, Marker.CUBE_LIST, Marker.ADD
        mk.scale.x = mk.scale.y = float(m.res)
        mk.scale.z = 0.02
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.55, 0.0, 0.9
        mk.pose.orientation.w = 1.0
        cx, cy = m.center(*np.nonzero(fr))
        for x, y in zip(cx, cy):
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.05
            mk.points.append(p)
        arr.markers.append(mk)
        for ag in agents:
            gm = Marker()
            gm.header.frame_id, gm.header.stamp = "map", stamp
            gm.ns, gm.id, gm.type = "goals", ag.idx, Marker.SPHERE
            gm.action = Marker.ADD if ag.goal is not None else Marker.DELETE
            if ag.goal is not None:
                gm.pose.position.x, gm.pose.position.y = float(ag.goal[0]), float(ag.goal[1])
                gm.pose.position.z = 0.3
            gm.pose.orientation.w = 1.0
            gm.scale.x = gm.scale.y = gm.scale.z = 0.45
            r, gg, b = COLORS[ag.idx % len(COLORS)]
            gm.color.r, gm.color.g, gm.color.b, gm.color.a = r, gg, b, 0.95
            arr.markers.append(gm)
            name = ag.d.name
            self.plan_pub[name].publish(self._path(ag.path, 0.25, stamp))
            self.trail_pub[name].publish(self._path(ag.trail, 0.15, stamp))
        self.mk_pub.publish(arr)


# =============================================================================
# Main
# =============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="Multi-drone frontier exploration")
    p.add_argument("--num-drones", type=int, default=3, help="Drones drone0..N-1 in spawn order (default 3)")
    p.add_argument("--spacing", type=float, default=1.0, help="Spawn spacing along +y; must match launch y_spacing")
    p.add_argument("--alt", type=float, default=0.8, help="Height of drone0 above takeoff (m)")
    p.add_argument("--alt-step", type=float, default=0.2, help="Extra height per drone (m)")
    p.add_argument("--range", type=float, default=5.0, help="LiDAR range used for mapping (m)")
    p.add_argument("--res", type=float, default=0.1, help="Map resolution (m)")
    p.add_argument("--bounds", type=float, nargs=4, default=[-6.0, 16.0, -6.0, 18.0],
                   metavar=("XMIN", "XMAX", "YMIN", "YMAX"), help="Map area in the world frame (m)")
    p.add_argument("--lookahead", type=float, default=0.9, help="Carrot distance along the path (m)")
    p.add_argument("--time-limit", type=float, default=900.0, help="Stop and land after this many seconds")
    p.add_argument("--wall-top", type=float, default=1.48, help="Wall height, for the altitude check (m)")
    p.add_argument("--save", default="explore_map", help="Saved map file stem (.pgm/.yaml)")
    return p.parse_args()


def main():
    args = parse_args()
    names = [f"drone{i}" for i in range(args.num_drones)]
    heights = [args.alt + i * args.alt_step for i in range(args.num_drones)]
    if max(heights) + 0.08 >= args.wall_top - 0.05:
        sys.exit(f"{TAG} Highest LiDAR plane ({max(heights) + 0.08:.2f} m) would be at/above the walls "
                 f"({args.wall_top} m). Lower --alt / --alt-step.")
    if args.num_drones < 1:
        sys.exit(f"{TAG} Need at least one drone.")

    sim_time = os.environ.get("SWARM_REAL_HW", "0") != "1"
    swarm = Swarm(names=names, spacing=args.spacing, mapping=False)
    drones = [swarm[n] for n in names]
    viz = Viz(names, sim_time)
    swarm.executor.add_node(viz.node)

    m = OccupancyMap(*args.bounds, args.res)
    planner = Planner(m)
    agents = []
    try:
        swarm.connect_all()
        print(f"{TAG} {len(drones)} drone(s), heights {', '.join(f'{h:g}' for h in heights)} m, "
              f"map {m.nx}x{m.ny} cells @ {m.res} m")
        z_tgt = takeoff_all(drones, heights, tag=TAG)
        agents = [Agent(d, i, z, d.get_rpy()[2]) for i, (d, z) in enumerate(zip(drones, z_tgt))]

        t_start = time.monotonic()
        t_replan = t_viz = 0.0
        no_frontier_since = None
        fr = np.zeros((m.nx, m.ny), bool)
        dt = 1.0 / LOOP_HZ
        print(f"{TAG} Exploring. Ctrl+C lands all drones.")
        while True:
            tick = time.monotonic()
            now = tick - t_start
            check_all(drones)
            if now > args.time_limit:
                raise MissionAbort(f"time limit ({args.time_limit:.0f} s) reached")

            # 1. map: integrate each drone's newest level scan
            positions = [ag.xy() for ag in agents]
            for ag, pos in zip(agents, positions):
                scan = ag.d.get_scan()
                if scan is None:
                    continue
                stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
                if stamp == ag.last_stamp:
                    continue
                ag.last_stamp = stamp
                roll, pitch, yaw = ag.d.get_rpy()
                if abs(roll) > MAX_TILT or abs(pitch) > MAX_TILT:
                    continue
                others = [p for p in positions if p is not pos]
                m.integrate(pos[0], pos[1], yaw, scan, args.range, others)
                if not ag.trail or math.hypot(pos[0] - ag.trail[-1][0], pos[1] - ag.trail[-1][1]) > 0.15:
                    ag.trail.append(pos)

            # 2. frontiers, goals, paths
            if now - t_replan >= REPLAN_PERIOD:
                t_replan = now
                usable, fr = assign_goals(agents, m, planner, now)
                if usable == 0:
                    no_frontier_since = no_frontier_since or now
                    if now - no_frontier_since > DONE_TIME:
                        print(f"\n{TAG} No reachable frontiers left: exploration complete.")
                        break
                else:
                    no_frontier_since = None

            # 3. separation: abort if too close; note which higher-priority drones are near
            near_higher = [[] for _ in agents]
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    sep = math.dist(positions[i], positions[j])
                    if sep < ABORT_SEP:
                        raise MissionAbort(f"{agents[i].d.name} and {agents[j].d.name} only {sep:.2f} m apart")
                    if sep < YIELD_DIST:
                        near_higher[j].append(positions[i])

            # 4. fly: carrot along the path; a drone gives way (holds) only if its next move would
            #    bring it closer to a nearby higher-priority drone, so a queue can still unwind
            for ag, (x, y) in zip(agents, positions):
                others = [p for p in positions if p is not positions[ag.idx]]
                target = ag.carrot(args.lookahead, m, others)
                if ag.goal is not None and math.hypot(x - ag.goal[0], y - ag.goal[1]) < GOAL_TOL:
                    target = None
                ag.holding = target is not None and any(
                    math.hypot(target[0] - hx, target[1] - hy) < math.hypot(x - hx, y - hy)
                    for hx, hy in near_higher[ag.idx])
                if ag.holding:
                    target = None
                tx, ty = target if target is not None else (x, y)
                send_world(ag.d, tx, ty, ag.z, ag.yaw)

            # 5. RViz + status
            if now - t_viz >= VIZ_PERIOD:
                t_viz = now
                viz.publish(m, fr, agents)
                known = (m.occupied() | m.free()).sum() * m.res ** 2
                states = " ".join(f"{ag.d.name[-1]}:{'hold' if ag.holding else ('go' if ag.goal else 'idle')}"
                                  for ag in agents)
                print(f"\r{TAG} t={now:5.0f}s  mapped {known:6.1f} m^2  frontier cells {int(fr.sum()):4d}  "
                      f"{states}   ", end="", flush=True)

            time.sleep(max(0.0, dt - (time.monotonic() - tick)))

    except ManualOverride as e:
        print(f"\n{TAG} MANUAL OVERRIDE: {e}. Landing the other drones.")
    except MissionAbort as e:
        print(f"\n{TAG} ABORT: {e}. Landing.")
    except KeyboardInterrupt:
        print(f"\n{TAG} Interrupted by user. Landing.")
    except Exception:
        print()
        traceback.print_exc()
    finally:
        try:
            if m.free().any():
                m.save(args.save)
                viz.publish(m, np.zeros((m.nx, m.ny), bool), agents)
                print(f"{TAG} Map saved: {args.save}.pgm / {args.save}.yaml "
                      f"({(m.occupied() | m.free()).sum() * m.res ** 2:.1f} m^2 mapped)")
        except Exception as e:
            print(f"{TAG} Could not save the map: {e!r}")
        land_all_safely(drones, tag=TAG)
        swarm.close()


if __name__ == "__main__":
    main()