import os
from pathlib import Path
from types import SimpleNamespace
from math import pi
import struct

import numpy as np

import roboticstoolbox as rtb

from spatialmath import SE3

from spatialgeometry import Cuboid, Cylinder, Mesh

from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

from sinkArmDatis import armPoints as sinkArmPoints

ROOT = Path(__file__).resolve().parent
JOINT_OFFSETS = np.array([[0, 0, .330], [0, 0, 0], [0, 0, .260], [.290, 0, .015], [0, 0, 0], [0, 0, 0]], dtype=float)
JOINT_AXES = np.array([[0, 0, 1], [0, 1, 0], [0, -1, 0], [-1, 0, 0], [0, -1, 0], [-1, 0, 0]], dtype=float)
LIMITS_DEG = np.array([[-170, 170], [-110, 130], [-65, 200], [-200, 200], [-123, 123], [-455, 455]], dtype=float)
MESH_NAMES = ['base_link', 'link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
GP4_HOME = np.deg2rad([0, 0, 0, 0, 0, 0])
GP4_WAIT = np.deg2rad([83, -110, 55, 0, -76, 0])


def stl_points(filename, count=120):
    with open(filename, 'rb') as f:
        header = f.read(84)
        n = struct.unpack('<I', header[80:84])[0]
        data = f.read()
    if len(data) != n * 50:
        raise ValueError(f'Expected binary STL: {filename}')
    triangles = np.frombuffer(data, dtype=np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('flags', '<u2')]))['vertices'].reshape(-1, 3)
    stride = max(1, len(triangles) // count)
    return triangles[::stride][:count].astype(float).copy()


class GP4Robot:
    def __init__(self, base=None):
        self.base = SE3() if base is None else SE3(base)
        self.n = 6
        self.qlim = np.deg2rad(LIMITS_DEG).T
        self._q = GP4_HOME.copy()
        self.objects = []
        self.meshPoints = []
        self._rng = np.random.default_rng(7)
        for name in MESH_NAMES:
            filename = ROOT / 'GP4_reference' / 'meshes' / 'gp4' / 'collision' / f'{name}.stl'
            if not filename.is_file():
                raise FileNotFoundError(f'Missing GP4 mesh: {filename}')
            self.meshPoints.append(stl_points(filename))

    def frames(self, q):
        q = np.asarray(q, dtype=float)
        current = self.base
        frames = [current]
        for translation, axis, theta in zip(JOINT_OFFSETS, JOINT_AXES, q):
            current = current * SE3(*translation) * SE3.Rt(Rotation.from_rotvec(axis * theta).as_matrix(), [0, 0, 0])
            frames.append(current)
        return frames

    def fkine(self, q):
        return self.frames(q)[-1] * SE3(0.072, 0, 0)

    @property
    def q(self):
        return self._q.copy()

    @q.setter
    def q(self, value):
        self._q = np.asarray(value, dtype=float).copy()
        if self.objects:
            for obj, pose in zip(self.objects, self.frames(self._q)):
                obj.T = pose

    def arm_points(self, q):
        frames = self.frames(q)
        return np.vstack([pts @ f.R.T + f.t for pts, f in zip(self.meshPoints, frames)])

    def random_q(self):
        return self._rng.uniform(self.qlim[0], self.qlim[1])

    def ikine_LM(self, target, q0=None, slimit=300):
        target = SE3(target)
        seed = self.q if q0 is None else np.asarray(q0, dtype=float)
        def residual(q):
            T = self.fkine(q)
            position = T.t - target.t
            orientation = Rotation.from_matrix(target.R @ T.R.T).as_rotvec()
            return np.hstack((position, 0.25 * orientation))
        result = least_squares(residual, np.clip(seed, self.qlim[0], self.qlim[1]),
                               bounds=(self.qlim[0], self.qlim[1]), max_nfev=min(int(slimit), 220),
                               ftol=1e-7, xtol=1e-7, gtol=1e-7)
        error = residual(result.x)
        success = np.linalg.norm(error[:3]) < .012 and np.linalg.norm(error[3:]) < .035
        return SimpleNamespace(success=success, q=result.x)

    def add_to_env(self, env):
        if self.objects:
            return
        for name, pose in zip(MESH_NAMES, self.frames(self.q)):
            path = ROOT / 'GP4_reference' / 'meshes' / 'gp4' / 'collision' / f'{name}.stl'
            part = Mesh(filename=str(path), pose=pose, color=[35, 100, 185])
            part._filename = os.path.splitdrive(part.filename)[1].lstrip("\\/")
            part._collision = False
            env.add(part)
            self.objects.append(part)


def armPoints(arm, q):
    if isinstance(arm, GP4Robot):
        frames = arm.frames(q)
        return np.vstack([pts @ f.R.T + f.t for pts, f in zip(arm.meshPoints[1:], frames[1:])])
    return sinkArmPoints(arm, q)

steps = 45

dt = 0.05

collisionPad = 0.01

maxTilt = np.deg2rad(15)


def boxCorners(shape, pose):
    if isinstance(shape, Cuboid):
        half = np.asarray(shape.scale, dtype=float) / 2
    else:
        half = np.array([shape.radius, shape.radius, shape.length / 2], dtype=float)
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    T = np.asarray(pose)
    return corners @ T[:3, :3].T + T[:3, 3]


def obstacleBoxes(env, ignore, centre, radius=1.45):
    ignored = {id(obj) for obj in ignore}
    lows, highs = [], []
    for obj in env.swift_objects:
        if id(obj) in ignored or not isinstance(obj, (Cuboid, Cylinder)):
            continue
        corners = boxCorners(obj, obj.T)
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        if np.linalg.norm(np.clip(centre, lo, hi) - centre) <= radius:
            lows.append(lo)
            highs.append(hi)
    return np.asarray(lows, dtype=float).reshape(-1, 3), np.asarray(highs, dtype=float).reshape(-1, 3)


def hits(points, boxes, pad):
    low, high = boxes
    if len(low) == 0 or len(points) == 0:
        return False
    for start in range(0, len(points), 100):
        batch = points[start:start + 100]
        inside = (batch[:, None, :] > low - pad) & (batch[:, None, :] < high + pad)
        if inside.all(axis=2).any():
            return True
    return False


def jointLimits(arm, q):
    limits = np.asarray(arm.qlim, dtype=float)
    if limits.shape != (2, arm.n):
        return True
    return np.all(q >= limits[0] - 1e-6) and np.all(q <= limits[1] + 1e-6)


def closestAngles(arm, q, previous):
    q = np.asarray(q, dtype=float).copy()
    limits = np.asarray(arm.qlim, dtype=float)
    for i in range(len(q)):
        alternatives = [q[i] + 2 * pi * k for k in (-1, 0, 1)]
        allowed = [v for v in alternatives if limits[0, i] <= v <= limits[1, i]]
        if allowed:
            q[i] = min(allowed, key=lambda v: abs(v - previous[i]))
    return q


def gripPose(cupBase, cupWidth, cupHeight, yaw=pi/3, gripFraction=0.35):
    return (
        cupBase
        * SE3(0, 0, cupHeight*gripFraction)
        * SE3.Rz(yaw)
        * SE3(-cupWidth/2 - 0.01, 0, 0)
    )


def cupBaseFromParts(parts):
    base = parts[0]
    xyz = SE3(base.T).t.copy()
    xyz[2] -= float(np.asarray(base.scale)[2]) / 2
    return SE3(*xyz)


def collision(arm, q, boxes, carried, sinkTree=None, sinkClear=0.045):
    if not jointLimits(arm, q):
        return 'joint limit'
    points = armPoints(arm, q)
    if hits(points, boxes, collisionPad):
        return 'arm/environment'
    if sinkTree is not None and np.any(sinkTree.query(points, k=1)[0] < sinkClear):
        return 'arm/other arm'
    flange = arm.fkine(q)
    for index, (shape, offset) in enumerate(carried):
        pose = flange * offset
        if index == 0 and pose.R[2, 2] < np.cos(maxTilt):
            return 'kettle tilt'
        corners = boxCorners(shape, pose)
        if hits(corners, boxes, -0.002):
            return 'kettle/environment'
        if sinkTree is not None and np.any(sinkTree.query(corners, k=1)[0] < sinkClear):
            return 'kettle/other arm'
    return None


def checkTrajectory(arm, trajectory, boxes, carried, sinkTree=None):
    for q in trajectory:
        reason = collision(arm, q, boxes, carried, sinkTree)
        if reason is not None:
            return reason
    return None


def inverse(arm, target, q0):
    result = arm.ikine_LM(target, q0=q0, slimit=300)
    if not result.success:
        return None
    q = closestAngles(arm, result.q, q0)
    return q if jointLimits(arm, q) else None


def edgePath(a, b, speed=0.12):
    n = max(5, int(np.ceil(np.max(np.abs(np.asarray(b) - np.asarray(a))) / speed)) + 1)
    return rtb.jtraj(a, b, n).q


def rrtConnect(arm, goal, boxes, carried, sinkTree=None, tries=135):
    start = np.asarray(arm.q, dtype=float).copy()
    if collision(arm, start, boxes, carried, sinkTree):
        return None
    if collision(arm, goal, boxes, carried, sinkTree):
        return None
    rng = np.random.default_rng(11)
    limits = np.asarray(arm.qlim, dtype=float)
    trees = [dict(points=[start], parents=[-1], root='start'),
             dict(points=[np.asarray(goal, dtype=float)], parents=[-1], root='goal')]

    def extend(tree, target):
        points = np.asarray(tree['points'])
        parent = int(np.argmin(np.linalg.norm(points - target, axis=1)))
        previous = points[parent]
        direction = target - previous
        distance = np.linalg.norm(direction)
        if distance < 1e-7:
            return parent
        nextQ = previous + direction * min(0.24 / distance, 1.0)
        if checkTrajectory(arm, edgePath(previous, nextQ), boxes, carried, sinkTree) is not None:
            return None
        tree['points'].append(nextQ)
        tree['parents'].append(parent)
        return len(tree['points']) - 1

    def path(tree, i):
        chain = []
        while i != -1:
            chain.append(tree['points'][i])
            i = tree['parents'][i]
        return chain[::-1]

    for trial in range(tries):
        treeA, treeB = trees[trial % 2], trees[(trial + 1) % 2]
        sample = treeB['points'][0] if trial % 5 == 0 else rng.uniform(limits[0], limits[1])
        iA = extend(treeA, sample)
        if iA is None:
            continue
        meet = treeA['points'][iA]
        iB = None
        for _ in range(16):
            iNext = extend(treeB, meet)
            if iNext is None:
                break
            iB = iNext
            if np.linalg.norm(treeB['points'][iB] - meet) < 1e-5:
                break
        if iB is None or np.linalg.norm(treeB['points'][iB] - meet) >= 1e-5:
            continue
        if treeA['root'] == 'start':
            first, last = path(treeA, iA), path(treeB, iB)
        else:
            first, last = path(treeB, iB), path(treeA, iA)
        waypoints = first + last[-2::-1]
        result = np.vstack([edgePath(a, b) for a, b in zip(waypoints, waypoints[1:])])
        if checkTrajectory(arm, result, boxes, carried, sinkTree) is None:
            return result
    return None


def solveMove(arm, target, boxes, carried=(), sinkTree=None, linear=False, allowDetour=True):
    start = np.asarray(arm.q, dtype=float).copy()
    if collision(arm, start, boxes, carried, sinkTree) is not None:
        return None
    if linear:
        first = arm.fkine(start)
        dist = np.linalg.norm(target.t - first.t)
        count = max(4, int(np.ceil(dist / 0.028)))
        previous = start
        pieces = []
        for alpha in np.linspace(0, 1, count + 1)[1:]:
            T = np.eye(4)
            T[:3, :3] = target.R
            T[:3, 3] = (1 - alpha) * first.t + alpha * target.t
            pose = SE3(T)
            q = inverse(arm, pose, previous)
            if q is None:
                return None
            part = edgePath(previous, q, 0.065)
            if checkTrajectory(arm, part, boxes, carried, sinkTree) is not None:
                return None
            pieces.append(part)
            previous = q
        return np.vstack(pieces)
    seeds = [start, np.asarray(GP4_HOME, dtype=float)] + [arm.random_q() for _ in range(8 if not allowDetour else 18)]
    goals = []
    for seed in seeds:
        q = inverse(arm, target, np.asarray(seed, dtype=float))
        if q is None or collision(arm, q, boxes, carried, sinkTree) is not None:
            continue
        goals.append(q)
        trajectory = rtb.jtraj(start, q, steps).q
        if checkTrajectory(arm, trajectory, boxes, carried, sinkTree) is None:
            return trajectory
    if allowDetour:
        goals.sort(key=lambda q: np.linalg.norm(q - start))
        for q in goals[:2]:
            trajectory = rrtConnect(arm, q, boxes, carried, sinkTree)
            if trajectory is not None:
                return trajectory
    return None


def runTableArm(env, cupParts, cupFilled, cupHeight, cupWidth, mountPose, sinkArm=None,
                arm=None, otherArms=()):
    if arm is None:
        arm = GP4Robot(base=mountPose)
        arm.q = np.asarray(GP4_WAIT, dtype=float)
    ignored = list(cupParts)
    if cupFilled is not None:
        ignored.append(cupFilled)
    cupBase = cupBaseFromParts(cupParts)
    cupCenter = cupBase.t + np.array([0, 0, cupHeight / 2])
    if cupBase.t[0] > 1.34 and cupBase.t[1] < -1.38:
        arm.add_to_env(env)
        return None
    obstacles = obstacleBoxes(env, ignored, arm.base.t)
    if collision(arm, arm.q, obstacles, ()) is not None:
        rng = np.random.default_rng(8)
        original = np.asarray(arm.q, dtype=float).copy()
        valid = []
        for _ in range(120):
            candidate = original + rng.normal(0, 0.55, arm.n)
            if jointLimits(arm, candidate) and collision(arm, candidate, obstacles, ()) is None:
                valid.append(candidate)
        if not valid:
            arm.add_to_env(env)
            return None
        arm.q = min(valid, key=lambda q: np.linalg.norm(q - original))
    arm.add_to_env(env)
    avoidPoints = [armPoints(other, other.q) for other in otherArms if other is not None]
    if sinkArm is not None:
        sinkPoints = armPoints(sinkArm, sinkArm.q)
        avoidPoints.append(sinkPoints[np.linalg.norm(sinkPoints - cupCenter, axis=1) > max(0.14, cupWidth * 2.5)])
    avoidPoints = np.vstack(avoidPoints) if avoidPoints else np.empty((0, 3))
    sinkTree = cKDTree(avoidPoints) if len(avoidPoints) else None
    graspOptions = [
        (pi/3, 0.35),
        (pi/4, 0.35),
        (pi/2, 0.35),
        (pi/6, 0.35),
        (pi/3, 0.50),
        (0, 0.35),
    ]
    approachBacks = (0.10, 0.07, 0.14)
    planned = None
    startQ = np.asarray(arm.q, dtype=float).copy()
    selected = None
    for detour in (False, True):
        for yaw, fraction in (graspOptions[:3] if detour else graspOptions):
            gripTarget = gripPose(cupBase, cupWidth, cupHeight, yaw, fraction)
            for back in approachBacks:
                first = solveMove(arm, gripTarget * SE3(-back, 0, 0), obstacles,
                                  sinkTree=sinkTree, allowDetour=detour)
                if first is None:
                    continue
                arm.q = np.asarray(first[-1], dtype=float)
                second = solveMove(arm, gripTarget, obstacles, sinkTree=sinkTree,
                                   linear=True, allowDetour=False)
                arm.q = startQ.copy()
                if second is not None:
                    planned = (first, second)
                    selected = (yaw, fraction)
                    break
            if planned is not None:
                break
        if planned is not None:
            break
    arm.q = startQ.copy()
    if planned is None:
        return None
    for trajectory in planned:
        for q in trajectory:
            arm.q = q
            env.step(dt)
    pickedYaw, pickedFraction = selected
    flange = arm.fkine(arm.q)
    carried = [(shape, flange.inv() * SE3(shape.T)) for shape in ignored]
    holdPoint = np.array([0.30, -1.60, 1.15])
    holdShift = holdPoint - cupBase.t
    holdYaw = 5*pi/6
    sweepSteps = 10
    for k in range(1, sweepSteps + 1):
        f = k / sweepSteps
        candidate = SE3(f*holdShift[0], f*holdShift[1], f*holdShift[2]) * cupBase
        yaw = pickedYaw + f*(holdYaw - pickedYaw)
        target = gripPose(candidate, cupWidth, cupHeight, yaw, pickedFraction)
        path = solveMove(arm, target, obstacles, carried, sinkTree, allowDetour=False)
        if path is None:
            path = solveMove(arm, target, obstacles, carried, sinkTree, allowDetour=True)
        if path is None:
            break
        for q in path:
            arm.q = q
            flange = arm.fkine(q)
            for shape, offset in carried:
                shape.T = flange * offset
            env.step(dt)
    return arm
