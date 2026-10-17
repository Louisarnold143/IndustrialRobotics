import os
from math import pi
from scipy.spatial import cKDTree
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder
from ir_support.robots.UTSMeshRobot import UTSMeshRobot

meshDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "JakaZu5")

jakaGrey = (216, 216, 216)

class JakaZu5(UTSMeshRobot):
    link_colors = [jakaGrey] * 7

    def __init__(self, base=None):
        d      = [0.12015, 0,    0,      -0.114, 0.1135, 0.107]
        a      = [0,       0.43, 0.3685, 0,      0,      0]
        alpha  = [pi/2,    0,    0,      pi/2,   -pi/2,  0]
        qlimRad = [(-6.28, 6.28), (-1.48, 4.62), (-3.05, 3.05), (-1.48, 4.62), (-6.28, 6.28), (-6.28, 6.28)]
        links = [rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlimRad[i]) for i in range(6)]

        urdfJointOrigins = [SE3(0, 0, 0.12015),
                            SE3.Rx(pi/2),
                            SE3(0.43, 0, 0),
                            SE3(0.3685, 0, -0.114),
                            SE3(0, -0.1135, 0) * SE3.Rx(pi/2),
                            SE3(0, 0.107, 0) * SE3.Rx(-pi/2)]
        meshPosesAtZero = [SE3()]
        for origin in urdfJointOrigins:
            meshPosesAtZero.append(meshPosesAtZero[-1] * origin)

        super().__init__(links, mesh_stem="JakaZu5", mesh_dir=meshDir, name="JakaZu5",
                         home_q=[0] * 6, base=base, qtest_transforms=meshPosesAtZero)

        for mesh in self.links_3d:
            mesh._filename = os.path.splitdrive(mesh.filename)[1].lstrip("\\/")
            mesh._collision = False

steps = 50
dt = 0.05
homeQ = [-1.06, 0.98, 1.48, 1.79, -0.9, 2.25]
meshPointsPerLink = 250
collisionPad = 0.01
reachRadius = 1.4
gripGap = 0.01
approachGap = 0.08
spoutGap = 0.03
gripYaw = pi/4
gripHeight = 0.7
maxCarryTilt = np.radians(15)
otherArmClear = 0.045
ikAttempts = 30
detourAttempts = 10

def stlVertices(path):
    data = open(path, "rb").read()
    count = int(np.frombuffer(data[80:84], np.uint32)[0])
    tris = np.frombuffer(data[84:84 + 50*count], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("pad", "<u2")]))
    return np.unique(tris["v"].reshape(-1, 3).astype(float), axis=0)

def meshCollisionPoints(arm):
    rng = np.random.default_rng(0)
    pointsPerLink = []
    for i in range(1, arm.n + 1):
        verts = stlVertices(os.path.join(meshDir, f"JakaZu5Link{i}.stl"))
        verts = verts[rng.choice(len(verts), min(meshPointsPerLink, len(verts)), replace=False)]
        local = arm._relation_matrices[i] @ np.c_[verts, np.ones(len(verts))].T
        pointsPerLink.append(local[:3].T)
    return pointsPerLink

def boxCorners(shape, T):
    if isinstance(shape, Cuboid):
        half = shape.scale / 2
    else:
        half = np.array([shape.radius, shape.radius, shape.length / 2])
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    return corners @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]

def obstacleBoxes(env, ignore, centre):
    ignoreIds = {id(shape) for shape in ignore}
    lows, highs = [], []
    for obj in env.swift_objects:
        if isinstance(obj, (Cuboid, Cylinder)) and id(obj) not in ignoreIds:
            corners = boxCorners(obj, obj.T)
            lo, hi = corners.min(0), corners.max(0)
            if np.linalg.norm(np.clip(centre, lo, hi) - centre) < reachRadius:
                lows.append(lo)
                highs.append(hi)
    return np.array(lows).reshape(-1, 3), np.array(highs).reshape(-1, 3)

def armPoints(arm, q):
    frames = arm._get_transforms(q)
    return np.vstack([pts @ frames[i + 1][:3, :3].T + frames[i + 1][:3, 3] for i, pts in enumerate(arm.meshPoints)])

def otherArmPoints(other):
    return other.arm_points(other.q) if hasattr(other, "arm_points") else armPoints(other, other.q)

def nearOtherArm(arm, points):
    tree = getattr(arm, "otherTree", None)
    return tree is not None and np.any(tree.query(points, k=1)[0] < otherArmClear)

def hits(points, boxes, pad):
    lo, hi = boxes
    if len(lo) == 0:
        return False
    inside = (points[:, None, :] > lo - pad) & (points[:, None, :] < hi + pad)
    return inside.all(axis=2).any()

def pathCollides(arm, traj, boxes, carried):
    for q in traj:
        if hits(armPoints(arm, q), boxes, collisionPad):
            return True
        flange = arm.fkine(q)
        for index, (shape, offset) in enumerate(carried):
            carriedPose = flange * offset
            if hits(boxCorners(shape, carriedPose), boxes, -0.002):
                return True
            if index == 0 and carriedPose.R[2, 2] < np.cos(maxCarryTilt):
                return True
    return False

def freeWaypoint(arm, boxes, carried):
    for _ in range(300):
        q = arm.random_q()
        if not pathCollides(arm, [q], boxes, carried):
            return q
    return None

def solveChain(arm, poses, seed, forward):
    order = poses if forward else poses[::-1]
    qs, q0 = [], seed
    for pose in order:
        sol = arm.ikine_LM(pose, q0=q0, slimit=300)
        if not sol.success:
            return None
        qs.append(sol.q)
        q0 = sol.q
    return qs if forward else qs[::-1]

def detours(arm, boxes, carried):
    yield []
    yield [homeQ]
    for _ in range(detourAttempts):
        q = freeWaypoint(arm, boxes, carried)
        if q is not None:
            yield [q]

def planPath(arm, poses, boxes, carried):
    chains = [(arm.q, True)] + [(seed, False) for seed in (arm.q, homeQ, *[arm.random_q() for _ in range(ikAttempts)])]
    for seed, forward in chains:
        qs = solveChain(arm, poses, seed, forward)
        if qs is None:
            continue
        for detour in detours(arm, boxes, carried):
            points = [arm.q] + detour + qs
            traj = np.vstack([rtb.jtraj(a, b, steps).q for a, b in zip(points, points[1:])])
            if not pathCollides(arm, traj, boxes, carried):
                return traj
    return None

def gripPose(cupBase, cupWidth, cupHeight):
    return cupBase * SE3(0, 0, cupHeight*gripHeight) * SE3.Rz(gripYaw) * SE3(cupWidth/2 + gripGap, 0, 0) * SE3.Ry(-pi/2)

def moveThrough(env, arm, poses, boxes, carried=()):
    traj = planPath(arm, poses, boxes, carried)
    if traj is None:
        print(f"Sink arm: collision predicted on every path to {np.round(poses[-1].t, 3)} - stopping")
        return False
    for q in traj:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True

def makeSinkArm(mountPose):
    arm = JakaZu5(base=mountPose)
    arm.q = homeQ
    arm.meshPoints = meshCollisionPoints(arm)
    return arm


def runSinkArm(env, cupParts, cupPickBase, tapSpoutEnd, cupHeight, cupWidth,
               mountPose, arm=None, otherArms=()):
    if arm is None:
        arm = makeSinkArm(mountPose)
        arm.add_to_env(env)
    others = [other for other in otherArms if other is not None]
    arm.otherTree = cKDTree(np.vstack([otherArmPoints(other) for other in others])) if others else None

    boxes = obstacleBoxes(env, ignore=cupParts, centre=arm.fkine_all(arm.q)[1].t)
    grip = lambda cupBase: gripPose(cupBase, cupWidth, cupHeight)
    kettleHoldBase = SE3(tapSpoutEnd[0], tapSpoutEnd[1] - cupWidth/2 + 0.005,
                         tapSpoutEnd[2] - spoutGap - cupHeight)
    if not moveThrough(env, arm, [grip(SE3(0, 0, approachGap) * cupPickBase),
                                  grip(cupPickBase)], boxes):
        print("Sink arm: kettle pickup path unavailable")
        return None
    flange = arm.fkine(arm.q)
    carried = [(part, flange.inv() * SE3(part.T)) for part in cupParts]

    if not moveThrough(env, arm, [grip(SE3(0, 0, 0.1) * cupPickBase),
                                  grip(SE3(0, approachGap + cupWidth, 0) * kettleHoldBase),
                                  grip(kettleHoldBase)], boxes, carried):
        print("Sink arm: kettle tap path unavailable")
        return None

    # Keep the water as a single object. Removing geometry while Swift is
    # stepping can leave a None entry in its scene list on this version.
    waterHeight = cupHeight * 0.72
    waterRadius = max(0.006, cupWidth / 2 - 0.010)
    kettleFilled = Cylinder(
        radius=waterRadius,
        length=waterHeight,
        pose=kettleHoldBase * SE3(0, 0, 0.006 + waterHeight / 2),
        color="blue",
        collision=False,
    )
    env.add(kettleFilled)
    for _ in range(32):
        env.step(dt)
    print("Sink arm: kettle water filled (open top, visible from above)")

    flange = arm.fkine(arm.q)
    filledCarry = carried + [(kettleFilled, flange.inv() * SE3(kettleFilled.T))]
    handoffBases = [SE3(1.12, -1.32, 1.10), SE3(1.16, -1.28, 1.08),
                    SE3(1.20, -1.30, 1.12)]
    exitBase = SE3(kettleHoldBase.t[0], -1.19, kettleHoldBase.t[2])
    succeeded = False
    for handoff in handoffBases:
        trajectory = planPath(arm, [grip(exitBase), grip(handoff)], boxes, filledCarry)
        if trajectory is None:
            continue
        for q in trajectory:
            arm.q = q
            newFlange = arm.fkine(q)
            for shape, offset in filledCarry:
                shape.T = newFlange * offset
            env.step(dt)
        succeeded = True
        break
    if not succeeded:
        print("Sink arm: no safe kettle handoff found")
        return None
    return arm, kettleFilled
