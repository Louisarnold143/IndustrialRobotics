from math import pi
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder
from ir_support import CylindricalDHRobotPlot

class TableBot6(rtb.DHRobot):
    def __init__(self, base=None):
        links = [
            rtb.RevoluteDH(d=0.15,  a=0,     alpha=pi/2),
            rtb.RevoluteDH(d=0,     a=-0.46, alpha=0),
            rtb.RevoluteDH(d=0,     a=-0.40, alpha=0),
            rtb.RevoluteDH(d=0.09,  a=0,     alpha=pi/2),
            rtb.RevoluteDH(d=0.08,  a=0,     alpha=-pi/2),
            rtb.RevoluteDH(d=0.06,  a=0,     alpha=0),
        ]
        super().__init__(links, name="TableBot6", base=base)

steps = 50
dt = 0.05
homeQ = [-2.55, -1.29, 1.24, 0, -0.97, -1.57]
linkRadius = 0.03
gripGap = 0.01
approachGap = 0.08
departGap = 0.06
swingLift = 0.22
retreatX = 1.18
escapeY = -1.18
liftGap = 0.38
placeApproachGap = 0.38
transferY = -1.30
gripRoll = -pi/2
ikAttempts = 30

def boxCorners(shape, T):
    if isinstance(shape, Cuboid):
        half = shape.scale / 2
    else:
        half = np.array([shape.radius, shape.radius, shape.length / 2])
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    return corners @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]

def obstacleBoxes(env, ignore):
    ignoreIds = {id(shape) for shape in ignore}
    boxes = []
    for obj in env.swift_objects:
        if isinstance(obj, (Cuboid, Cylinder)) and id(obj) not in ignoreIds:
            corners = boxCorners(obj, obj.T)
            boxes.append((corners.min(0), corners.max(0)))
    return boxes

def armPoints(arm, q):
    frames = arm.fkine_all(q)
    points = []
    for i in range(1, arm.n):
        start = frames[i].t
        elbow = start + arm.links[i].d * frames[i].R[:, 2]
        points += [*np.linspace(start, elbow, 5), *np.linspace(elbow, frames[i + 1].t, 5)]
    return np.array(points)

def robotBoxes(arm, ignoreCenter=None, ignoreRadius=0):
    points = armPoints(arm, arm.q)
    boxes = []
    for start, end in zip(points[:-1], points[1:]):
        midpoint = (start + end) / 2
        if ignoreCenter is not None and np.linalg.norm(midpoint - ignoreCenter) < ignoreRadius:
            continue
        boxes.append((np.minimum(start, end) - linkRadius, np.maximum(start, end) + linkRadius))
    return boxes

def hits(points, boxes, pad):
    return any(np.all((points > lo - pad) & (points < hi + pad), axis=1).any() for lo, hi in boxes)

def placementSupportBox(lo, hi, placeBase, clearance):
    p = placeBase.t
    return (
        p[2] - 0.05 <= hi[2] <= p[2] + 0.003
        and hi[0] >= p[0] - clearance
        and lo[0] <= p[0] + clearance
        and hi[1] >= p[1] - clearance
        and lo[1] <= p[1] + clearance
    )

def armHits(points, boxes, pad, placeBase=None, clearance=0.0):
    for lo, hi in boxes:
        checkPoints = points
        if placeBase is not None and placementSupportBox(lo, hi, placeBase, clearance):
            p = placeBase.t
            local = (
                (np.abs(points[:, 0] - p[0]) <= clearance)
                & (np.abs(points[:, 1] - p[1]) <= clearance)
                & (points[:, 2] >= p[2] - 0.005)
                & (points[:, 2] <= p[2] + 0.13)
            )
            checkPoints = points[~local]
        if len(checkPoints) and np.all((checkPoints > lo - pad) & (checkPoints < hi + pad), axis=1).any():
            return True
    return False

def pathCollides(arm, traj, boxes, carried, carriedBoxes=None, armPlaceBase=None, armPlacementClearance=0.0):
    if carriedBoxes is None:
        carriedBoxes = boxes
    for q in traj:
        if armHits(armPoints(arm, q), boxes, linkRadius, armPlaceBase, armPlacementClearance):
            return True
        flange = arm.fkine(q)
        for shape, offset in carried:
            if hits(boxCorners(shape, flange * offset), carriedBoxes, -0.002):
                return True
    return False

def nearestEquivalent(q, reference):
    q = np.asarray(q, dtype=float)
    reference = np.asarray(reference, dtype=float)
    return reference + (q - reference + pi) % (2*pi) - pi

def planPath(arm, poses, boxes, carried, carriedBoxes=None):
    for seed in (arm.q, homeQ, *[arm.random_q() for _ in range(ikAttempts)]):
        qs = []
        for pose in reversed(poses):
            sol = arm.ikine_LM(pose, q0=qs[0] if qs else seed, slimit=300)
            if not sol.success:
                break
            qs.insert(0, sol.q)
        else:
            for start in ([arm.q], [arm.q, homeQ]):
                points = start + qs
                traj = np.vstack([rtb.jtraj(a, b, steps).q for a, b in zip(points, points[1:])])
                if not pathCollides(arm, traj, boxes, carried, carriedBoxes):
                    return traj
    return None


def planLinearPath(arm, target, boxes, carried, carriedBoxes=None, segments=20, armPlaceBase=None, armPlacementClearance=0.0):
    startPose = arm.fkine(arm.q)
    qPrev = np.asarray(arm.q, dtype=float)
    pieces = []
    for alpha in np.linspace(0, 1, segments + 1)[1:]:
        t = (1 - alpha) * startPose.t + alpha * target.t
        T = np.eye(4)
        T[:3, :3] = target.R
        T[:3, 3] = t
        pose = SE3(T)
        sol = arm.ikine_LM(pose, q0=qPrev, slimit=300)
        if not sol.success:
            return None
        q = nearestEquivalent(sol.q, qPrev)
        segment = rtb.jtraj(qPrev, q, max(4, steps // 10)).q
        if pathCollides(arm, segment, boxes, carried, carriedBoxes, armPlaceBase, armPlacementClearance):
            return None
        pieces.append(segment)
        qPrev = q
    return np.vstack(pieces)

def moveLinearThrough(env, arm, target, boxes, carried=(), carriedBoxes=None, segments=20, armPlaceBase=None, armPlacementClearance=0.0):
    traj = planLinearPath(arm, target, boxes, carried, carriedBoxes, segments, armPlaceBase, armPlacementClearance)
    if traj is None:
        print(f"Table arm: collision predicted on linear path to {np.round(target.t, 3)} - stopping")
        return False
    for q in traj:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True

def swingToOtherSide(env, arm, carried, boxes, cupOffset, placeBase):
    qStart = np.asarray(arm.q, dtype=float)
    candidates = []
    for degrees in (70, 80, 90, 100, 110, -70, -80, -90, -100, -110):
        qTarget = qStart.copy()
        qTarget[0] += degrees * pi / 180
        traj = rtb.jtraj(qStart, qTarget, steps * 2).q
        if pathCollides(arm, traj, boxes, carried):
            continue
        cupPose = arm.fkine(qTarget) * cupOffset
        if cupPose.t[2] < placeBase.t[2] + 0.08:
            continue
        distance = np.linalg.norm(cupPose.t[:2] - placeBase.t[:2])
        candidates.append((distance, abs(abs(degrees) - 90), traj))
    if not candidates:
        print("Table arm: collision predicted on every base swing - stopping")
        return False
    traj = min(candidates, key=lambda item: (item[0], item[1]))[2]
    for q in traj:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True

def gripPose(cupBase, cupWidth, cupHeight):
    return cupBase * SE3(-cupWidth/2 - gripGap, 0, cupHeight/2) * SE3.Ry(pi/2) * SE3.Rz(gripRoll)

def cupBaseFromParts(cupParts):
    base = cupParts[0]
    pose = SE3(base.T)
    baseThickness = float(np.asarray(base.scale)[2])
    return SE3(pose.t[0], pose.t[1], pose.t[2] - baseThickness/2)

def moveThrough(env, arm, poses, boxes, carried=(), carriedBoxes=None):
    traj = planPath(arm, poses, boxes, carried, carriedBoxes)
    if traj is None:
        print(f"Table arm: collision predicted on every path to {np.round(poses[-1].t, 3)} - stopping")
        return False
    for q in traj:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True


def placementCarriedBoxes(boxes, placeBase, cupWidth):
    p = placeBase.t
    margin = 0.01
    filtered = []
    for lo, hi in boxes:
        support = (
            p[2] - 0.05 <= hi[2] <= p[2] + 0.003
            and hi[0] >= p[0] - cupWidth/2 - margin
            and lo[0] <= p[0] + cupWidth/2 + margin
            and hi[1] >= p[1] - cupWidth/2 - margin
            and lo[1] <= p[1] + cupWidth/2 + margin
        )
        if not support:
            filtered.append((lo, hi))
    return filtered

def runTableArm(env, cupParts, cupFilled, cupHeight, cupWidth, mountPose, placeBase, sinkArm=None):
    arm = TableBot6(base=mountPose)
    arm = CylindricalDHRobotPlot(arm, cylinder_radius=linkRadius, color="white").create_cylinders()
    arm.q = homeQ
    env.add(arm)

    ignored = list(cupParts)
    if cupFilled is not None:
        ignored.append(cupFilled)

    cupPickBase = cupBaseFromParts(cupParts)
    cupCenter = cupPickBase.t + np.array([0, 0, cupHeight/2])
    boxes = obstacleBoxes(env, ignore=ignored)
    approachBoxes = list(boxes)
    carryBoxes = list(boxes)

    if sinkArm is not None:
        handoffClearance = cupWidth + 2*linkRadius
        carryBoxes += robotBoxes(sinkArm, cupCenter, handoffClearance)

    grip = lambda cupBase: gripPose(cupBase, cupWidth, cupHeight)
    approachBase = SE3(-approachGap, 0, 0) * cupPickBase

    if not moveThrough(env, arm, [grip(approachBase), grip(cupPickBase)], approachBoxes):
        return

    flange = arm.fkine(arm.q)
    carriedShapes = list(cupParts)
    if cupFilled is not None:
        carriedShapes.append(cupFilled)
    carried = [(shape, flange.inv() * SE3(shape.T)) for shape in carriedShapes]

    departBase = SE3(-departGap, 0, 0) * cupPickBase

    if not moveLinearThrough(env, arm, grip(departBase), approachBoxes, carried, segments=10):
        return

    liftBase = SE3(departBase.t[0], departBase.t[1], departBase.t[2] + swingLift)
    if not moveLinearThrough(env, arm, grip(liftBase), approachBoxes, carried, segments=16):
        return

    flange = arm.fkine(arm.q)
    cupOffset = flange.inv() * cupBaseFromParts(cupParts)

    if not swingToOtherSide(env, arm, carried, carryBoxes, cupOffset, placeBase):
        return

    placeApproachBase = SE3(placeBase.t[0], placeBase.t[1], placeBase.t[2] + 0.18)
    if not moveLinearThrough(env, arm, grip(placeApproachBase), carryBoxes, carried, segments=18):
        return

    placeCarriedBoxes = placementCarriedBoxes(carryBoxes, placeBase, cupWidth)
    if not moveLinearThrough(env, arm, grip(placeBase), carryBoxes, carried, placeCarriedBoxes, segments=14, armPlaceBase=placeBase, armPlacementClearance=0.11):
        return

    return arm
