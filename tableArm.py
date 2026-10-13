from math import pi

import numpy as np

import roboticstoolbox as rtb

from spatialmath import SE3

from spatialgeometry import Cuboid, Cylinder

from scipy.spatial import cKDTree

from sinkArmDatis import JakaZu5, meshCollisionPoints, armPoints, homeQ, gripYaw

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

def protectSpout(boxes, cupBase, extra=0.025):
    low, high = boxes
    low = low.copy()
    high = high.copy()
    p = cupBase.t
    span = high - low
    centre = (low + high) / 2
    spout = (
        (span[:, 0] >= 0.035)
        & (span[:, 0] <= 0.085)
        & (span[:, 1] >= 0.20)
        & (span[:, 1] <= 0.40)
        & (span[:, 2] >= 0.035)
        & (span[:, 2] <= 0.085)
        & (np.abs(centre[:, 0] - p[0]) < 0.15)
        & (np.abs(centre[:, 1] - p[1]) < 0.20)
        & (low[:, 2] > p[2] + 0.065)
        & (low[:, 2] < p[2] + 0.26)
    )
    if spout.any():
        low[spout] -= extra
        high[spout] += extra
        print(f'Table arm: tap spout protected with {extra:.3f} m clearance')
    else:
        print('Table arm: tap spout not recognised; standard obstacle collision checks remain active')
    return low, high

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

def tabletopSupport(boxes, placeBase, cupWidth):
    p = placeBase.t
    low, high = boxes
    mask = (
        (high[:, 2] >= p[2] - 0.03)
        & (high[:, 2] <= p[2] + 0.004)
        & (low[:, 0] <= p[0] - cupWidth / 2)
        & (high[:, 0] >= p[0] + cupWidth / 2)
        & (low[:, 1] <= p[1] - cupWidth / 2)
        & (high[:, 1] >= p[1] + cupWidth / 2)
    )
    return mask

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

def gripPose(cupBase, cupWidth, cupHeight, yaw=-pi/2, gripFraction=0.42):
    return (
        cupBase
        * SE3(-cupWidth/2 - 0.01, 0, cupHeight*gripFraction)
        * SE3.Rz(yaw)
        * SE3.Ry(-pi/2)
    )

def cupBaseFromParts(parts):
    base = parts[0]
    xyz = SE3(base.T).t.copy()
    xyz[2] -= float(np.asarray(base.scale)[2]) / 2
    return SE3(*xyz)

def collision(arm, q, boxes, carried, sinkTree=None, sinkClear=0.045, placeBase=None, cupWidth=0):
    if not jointLimits(arm, q):
        return 'joint limit'
    points = armPoints(arm, q)
    boxPoints = points
    if placeBase is not None:
        p = placeBase.t
        contact = (
            (np.abs(points[:, 0] - p[0]) < 0.09)
            & (np.abs(points[:, 1] - p[1]) < 0.09)
            & (points[:, 2] >= p[2] - 0.004)
            & (points[:, 2] <= p[2] + 0.11)
        )
        supports = tabletopSupport(boxes, placeBase, cupWidth)
        if supports.any():
            low, high = boxes
            if hits(points, (low[~supports], high[~supports]), collisionPad):
                return 'arm/environment'
            boxPoints = points[~contact]
            if hits(boxPoints, (low[supports], high[supports]), collisionPad):
                return 'arm/tabletop'
    else:
        if hits(boxPoints, boxes, collisionPad):
            return 'arm/environment'
    flange = arm.fkine(q)
    if sinkTree is not None:
        if np.any(sinkTree.query(points, k=1)[0] < sinkClear):
            return 'arm/sink arm'
    for shape, offset in carried:
        pose = flange * offset
        if pose.R[2, 2] < np.cos(maxTilt):
            return 'cup tilt'
        corners = boxCorners(shape, pose)
        if placeBase is not None:
            support = tabletopSupport(boxes, placeBase, cupWidth)
            if hits(corners, (boxes[0][~support], boxes[1][~support]), -0.002):
                return 'cup/environment'
            if corners[:, 2].min() < placeBase.t[2] - 0.003:
                return 'cup below table'
        elif hits(corners, boxes, -0.002):
            return 'cup/environment'
        if sinkTree is not None and np.any(sinkTree.query(corners, k=1)[0] < sinkClear):
            return 'cup/sink arm'
    return None

def checkTrajectory(arm, trajectory, boxes, carried, sinkTree=None, placeBase=None, cupWidth=0):
    for q in trajectory:
        reason = collision(arm, q, boxes, carried, sinkTree, placeBase=placeBase, cupWidth=cupWidth)
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

def rrtConnect(arm, goal, boxes, carried, sinkTree=None, placeBase=None, cupWidth=0, tries=135):
    start = np.asarray(arm.q, dtype=float).copy()
    if collision(arm, start, boxes, carried, sinkTree, placeBase=placeBase, cupWidth=cupWidth):
        return None
    if collision(arm, goal, boxes, carried, sinkTree, placeBase=placeBase, cupWidth=cupWidth):
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
        if checkTrajectory(arm, edgePath(previous, nextQ), boxes, carried,
                           sinkTree, placeBase, cupWidth) is not None:
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
        if checkTrajectory(arm, result, boxes, carried, sinkTree, placeBase, cupWidth) is None:
            return result
    return None

def solveMove(arm, target, boxes, carried=(), sinkTree=None, linear=False,
              placeBase=None, cupWidth=0, allowDetour=True):
    start = np.asarray(arm.q, dtype=float).copy()
    startError = collision(arm, start, boxes, carried, sinkTree, placeBase=placeBase, cupWidth=cupWidth)
    if startError is not None:
        return None, f'start pose: {startError}'
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
                return None, 'inverse kinematics'
            part = edgePath(previous, q, 0.065)
            error = checkTrajectory(arm, part, boxes, carried, sinkTree, placeBase, cupWidth)
            if error is not None:
                return None, error
            pieces.append(part)
            previous = q
        return np.vstack(pieces), None
    seeds = [start, np.asarray(homeQ, dtype=float)] + [arm.random_q() for _ in range(8 if not allowDetour else 18)]
    errors = []
    goals = []
    for seed in seeds:
        q = inverse(arm, target, np.asarray(seed, dtype=float))
        if q is None:
            continue
        finalError = collision(arm, q, boxes, carried, sinkTree, placeBase=placeBase, cupWidth=cupWidth)
        if finalError is not None:
            errors.append('target: ' + finalError)
            continue
        goals.append(q)
        trajectory = rtb.jtraj(start, q, steps).q
        error = checkTrajectory(arm, trajectory, boxes, carried, sinkTree, placeBase, cupWidth)
        if error is None:
            return trajectory, None
        errors.append(error)
    if allowDetour:
        goals.sort(key=lambda q: np.linalg.norm(q - start))
        for q in goals[:2]:
            trajectory = rrtConnect(arm, q, boxes, carried, sinkTree, placeBase, cupWidth)
            if trajectory is not None:
                return trajectory, None
    return None, errors[-1] if errors else 'inverse kinematics'

def move(env, arm, basePose, cupWidth, cupHeight, boxes, carried=(), sinkTree=None, linear=False, placeBase=None, yaw=-pi/2, gripFraction=0.42):
    target = gripPose(basePose, cupWidth, cupHeight, yaw, gripFraction)
    trajectory, reason = solveMove(arm, target, boxes, carried, sinkTree, linear, placeBase, cupWidth)
    if trajectory is None and linear:
        trajectory, reason = solveMove(arm, target, boxes, carried, sinkTree, False, placeBase, cupWidth)
    if trajectory is None:
        print(f'Table arm: cannot reach {np.round(basePose.t, 3)} ({reason}) - stopping')
        return False
    for q in trajectory:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True

def runTableArm(env, cupParts, cupFilled, cupHeight, cupWidth, mountPose, placeBase, sinkArm=None):
    arm = JakaZu5(base=mountPose)
    arm.q = np.asarray(homeQ, dtype=float)
    arm.meshPoints = meshCollisionPoints(arm)
    ignored = list(cupParts)
    if cupFilled is not None:
        ignored.append(cupFilled)
    cupBase = cupBaseFromParts(cupParts)
    cupCenter = cupBase.t + np.array([0, 0, cupHeight / 2])
    if cupBase.t[0] > 1.34 and cupBase.t[1] < -1.38:
        arm.add_to_env(env)
        print('Table arm: cup remains beneath the tap; waiting for a successful sink-arm handoff', flush=True)
        return None
    obstacles = protectSpout(obstacleBoxes(env, ignored, arm.base.t), cupBase)
    if collision(arm, arm.q, obstacles, ()) is not None:
        rng = np.random.default_rng(8)
        original = np.asarray(homeQ, dtype=float)
        valid = []
        for _ in range(120):
            candidate = original + rng.normal(0, 0.55, arm.n)
            if jointLimits(arm, candidate) and collision(arm, candidate, obstacles, ()) is None:
                valid.append(candidate)
        if not valid:
            print('Table arm: the starting configuration intersects the bench/workcell; check its mount')
            arm.add_to_env(env)
            return None
        arm.q = min(valid, key=lambda q: np.linalg.norm(q - original))
        print('Table arm: changed initial configuration to avoid a starting collision')
    arm.add_to_env(env)
    sinkTree = None
    if sinkArm is not None:
        sinkPoints = armPoints(sinkArm, sinkArm.q)
        farPoints = sinkPoints[np.linalg.norm(sinkPoints - cupCenter, axis=1) > max(0.14, cupWidth * 2.5)]
        if len(farPoints):
            sinkTree = cKDTree(farPoints)
    approachOffsets = [
        (-0.16, 0.16, 0.0),
        (-0.13, 0.12, -0.005),
        (-0.10, 0.10, 0.0),
        (-0.18, 0.12, -0.01),
    ]
    graspOptions = [
        (-pi/2, 0.38),
        (-pi/4, 0.38),
        (-pi/2, 0.48),
        (-pi/3, 0.42),
        (-pi/4, 0.48),
    ]
    print(f'Table arm: cup is at {np.round(cupBase.t, 3)}; planning wall-mounted pickup', flush=True)
    planned = None
    lastError = 'inverse kinematics'
    startQ = np.asarray(arm.q, dtype=float).copy()
    selected = None
    for yaw, fraction in graspOptions:
        print(f'Table arm: checking pickup yaw {np.degrees(yaw):.0f} deg, grip fraction {fraction:.2f}', flush=True)
        gripTarget = gripPose(cupBase, cupWidth, cupHeight, yaw, fraction)
        for dx, dy, dz in approachOffsets:
            approachBase = SE3(dx, dy, dz) * cupBase
            first, reason1 = solveMove(arm, gripPose(approachBase, cupWidth, cupHeight, yaw, fraction),
                                       obstacles, sinkTree=sinkTree, allowDetour=False)
            if first is None:
                lastError = f'approach {reason1}'
                continue
            arm.q = np.asarray(first[-1], dtype=float)
            second, reason2 = solveMove(arm, gripTarget, obstacles, sinkTree=sinkTree,
                                        linear=True, allowDetour=False)
            arm.q = startQ.copy()
            if second is not None:
                planned = (first, second)
                selected = (yaw, fraction)
                print(f'Table arm: low pickup planned, yaw {np.degrees(yaw):.0f} deg, cup height {fraction:.2f}, approach ({dx:+.2f}, {dy:+.2f}, {dz:+.2f})')
                break
            lastError = f'final grip {reason2}'
        if planned is not None:
            break
    if planned is None:
        print('Table arm: direct paths blocked; trying a collision-checked detour', flush=True)
        for yaw, fraction in graspOptions[:3]:
            for dx, dy, dz in approachOffsets[:2]:
                approachBase = SE3(dx, dy, dz) * cupBase
                first, reason1 = solveMove(arm, gripPose(approachBase, cupWidth, cupHeight, yaw, fraction),
                                           obstacles, sinkTree=sinkTree, allowDetour=True)
                if first is None:
                    lastError = f'approach {reason1}'
                    continue
                arm.q = np.asarray(first[-1], dtype=float)
                second, reason2 = solveMove(arm, gripPose(cupBase, cupWidth, cupHeight, yaw, fraction),
                                            obstacles, sinkTree=sinkTree, linear=True, allowDetour=False)
                arm.q = startQ.copy()
                if second is not None:
                    planned = (first, second)
                    selected = (yaw, fraction)
                    print(f'Table arm: pickup detour found, yaw {np.degrees(yaw):.0f} deg, cup height {fraction:.2f}')
                    break
                lastError = f'final grip {reason2}'
            if planned is not None:
                break
    arm.q = startQ.copy()
    if planned is None:
        print(f'Table arm: cannot complete low front-left pickup ({lastError})')
        return None
    for trajectory in planned:
        for q in trajectory:
            arm.q = q
            env.step(dt)
    pickedYaw, pickedFraction = selected
    flange = arm.fkine(arm.q)
    carried = [(shape, flange.inv() * SE3(shape.T)) for shape in ignored]
    farSinkTree = sinkTree
    print('Table arm: planning clearance move after pickup', flush=True)
    departureOptions = [
        (-0.07, -0.02, 0.02),
        (-0.10, -0.06, 0.04),
        (-0.14, -0.08, 0.06),
        (-0.06, -0.08, 0.00),
        (-0.16, -0.04, 0.02),
        (-0.20, -0.12, 0.04),
        (-0.10, 0.00, 0.07),
    ]
    departurePath = None
    departureBase = None
    departureReason = 'inverse kinematics'
    for dx, dy, dz in departureOptions:
        candidate = SE3(dx, dy, dz) * cupBase
        target = gripPose(candidate, cupWidth, cupHeight, pickedYaw, pickedFraction)
        departurePath, departureReason = solveMove(
            arm, target, obstacles, carried, farSinkTree,
            linear=False, allowDetour=False
        )
        if departurePath is not None:
            departureBase = candidate
            break
    if departurePath is None:
        for dx, dy, dz in departureOptions[:3]:
            candidate = SE3(dx, dy, dz) * cupBase
            target = gripPose(candidate, cupWidth, cupHeight, pickedYaw, pickedFraction)
            departurePath, departureReason = solveMove(
                arm, target, obstacles, carried, farSinkTree,
                linear=False, allowDetour=True
            )
            if departurePath is not None:
                departureBase = candidate
                break
    if departurePath is None:
        print(f'Table arm: no safe departure from handoff ({departureReason}); holding cup')
        return None
    print(f'Table arm: clearance move found to {np.round(departureBase.t, 3)}', flush=True)
    for q in departurePath:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    if sinkArm is not None:
        sinkTree = cKDTree(armPoints(sinkArm, sinkArm.q))
    candidateBases = [
        SE3(x, y, placeBase.t[2])
        for x, y in (
            (0.55, -1.38), (0.50, -1.30), (0.45, -1.40),
            (0.60, -1.28), (0.35, -1.32), (0.70, -1.35),
            (0.55, -1.22), (0.40, -1.35),
        )
    ]
    print('Table arm: checking right-side drop-off positions', flush=True)
    selectedDrop = None
    retreatPath = None
    currentQ = np.asarray(arm.q, dtype=float).copy()
    lastReasons = []
    for drop in candidateBases:
        for height in (0.08, 0.13):
            above = SE3(drop.t[0], drop.t[1], drop.t[2] + height)
            first, why1 = solveMove(
                arm, gripPose(above, cupWidth, cupHeight, pickedYaw, pickedFraction),
                obstacles, carried, sinkTree, allowDetour=False
            )
            if first is None:
                lastReasons.append(f'approach {why1}')
                continue
            arm.q = np.asarray(first[-1], dtype=float)
            second, why2 = solveMove(
                arm, gripPose(drop, cupWidth, cupHeight, pickedYaw, pickedFraction),
                obstacles, carried, sinkTree, linear=True,
                placeBase=drop, cupWidth=cupWidth, allowDetour=False
            )
            arm.q = currentQ.copy()
            if second is None:
                lastReasons.append(f'lowering {why2}')
                continue
            selectedDrop = drop
            retreatPath = np.asarray(second[::-1], dtype=float).copy()
            print(f'Table arm: safe bench approach at {np.round(drop.t, 3)}; lowering from {height:.2f} m', flush=True)
            for trajectory in (first, second):
                for q in trajectory:
                    arm.q = q
                    flange = arm.fkine(q)
                    for shape, offset in carried:
                        shape.T = flange * offset
                    env.step(dt)
            break
        if selectedDrop is not None:
            break
    if selectedDrop is None:
        print('Table arm: contact placement blocked; checking safe above-bench release', flush=True)
        releaseSites = [
            (0.65, -1.40), (0.60, -1.45), (0.70, -1.38),
            (0.55, -1.40), (0.75, -1.30), (0.60, -1.30),
            (0.45, -1.45), (0.50, -1.25), (0.80, -1.35),
            (0.35, -1.35),
        ]
        released = False
        for x, y in releaseSites:
            drop = SE3(x, y, placeBase.t[2])
            if not tabletopSupport(obstacles, drop, cupWidth).any():
                continue
            for hoverHeight in (0.055, 0.085, 0.12, 0.16):
                hover = SE3(x, y, placeBase.t[2] + hoverHeight)
                target = gripPose(hover, cupWidth, cupHeight, pickedYaw, pickedFraction)
                path, reason = solveMove(
                    arm, target, obstacles, carried, sinkTree, allowDetour=False
                )
                if path is None:
                    lastReasons.append(f'release approach {reason}')
                    continue
                for q in path:
                    arm.q = q
                    flange = arm.fkine(q)
                    for shape, offset in carried:
                        shape.T = flange * offset
                    env.step(dt)
                actualBase = cupBaseFromParts(cupParts)
                # After releasing, the cup and its water fall vertically together.
                # Only accept a landing whose final pieces do not intersect non-support obstacles.
                landingTransform = SE3(
                    drop.t[0] - actualBase.t[0],
                    drop.t[1] - actualBase.t[1],
                    drop.t[2] - actualBase.t[2],
                )
                support = tabletopSupport(obstacles, drop, cupWidth)
                remaining = (obstacles[0][~support], obstacles[1][~support])
                landing = [(shape, landingTransform * SE3(shape.T)) for shape in ignored]
                if any(hits(boxCorners(shape, pose), remaining, -0.002) for shape, pose in landing):
                    lastReasons.append('release landing cup/environment')
                    continue
                if actualBase.t[2] < drop.t[2] + 0.015:
                    lastReasons.append('release height too low')
                    continue
                print(f'Table arm: releasing cup above bench at {np.round(drop.t, 3)}', flush=True)
                startPoses = [SE3(shape.T) for shape in ignored]
                for alpha in np.linspace(0, 1, 13)[1:]:
                    shift = SE3(
                        alpha * (drop.t[0] - actualBase.t[0]),
                        alpha * (drop.t[1] - actualBase.t[1]),
                        alpha * (drop.t[2] - actualBase.t[2]),
                    )
                    for shape, pose in zip(ignored, startPoses):
                        shape.T = shift * pose
                    env.step(dt)
                selectedDrop = drop
                retreatPath = np.asarray(path[::-1], dtype=float).copy()
                released = True
                break
            if released:
                break
        if not released:
            print('Table arm: no safe bench release found; ' + ', '.join(dict.fromkeys(lastReasons[-8:])))
            return None
    print(f'Table arm: filled cup placed at {np.round(selectedDrop.t, 3)}')
    if retreatPath is not None:
        retreatError = checkTrajectory(arm, retreatPath, obstacles, (), sinkTree)
        if retreatError is None:
            print('Table arm: releasing cup and retreating to waiting position', flush=True)
            for q in retreatPath:
                arm.q = q
                env.step(dt)
            print('Table arm: waiting clear of the cup for the next arm', flush=True)
        else:
            print(f'Table arm: cup released; retreat blocked ({retreatError}), holding position', flush=True)
    return arm
