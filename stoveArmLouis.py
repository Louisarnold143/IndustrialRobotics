import os
from math import pi
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder
from ir_support.robots.UTSMeshRobot import UTSMeshRobot

meshDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "JakaMiniCobo")

jakaGrey = (216, 216, 216)

class JakaMiniCobo(UTSMeshRobot):
    link_colors = [jakaGrey] * 7

    def __init__(self, base=None):
        #new ones from researchGate
        d      = [0.3330,        0,      0.3160,      0,     0.3840,      0.075   ]
        a      = [0,            0,  0,  0.0825,          -0.0825,          0       ]
        alpha  = [0,    -np.pi / 2,      np.pi / 2,      np.pi / 2,  -np.pi / 2, np.pi / 2       ]

        #old ones from chat
        #d      = [0.120,        0,      0,      -0.090,     0.085,      0.075   ]
        #a      = [0,            0.280,  0.225,  0,          0,          0       ]
        #alpha  = [np.pi / 2,    0,      0,      np.pi / 2,  -np.pi / 2, 0       ]
        qlimRad = [ (-6.28, 6.28),
                    (-2.18, 2.18),
                    (-2.27, 2.27),
                    (-6.28, 6.28),
                    (-2.09, 2.09),
                    (-6.28, 6.28)]
        links = [rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlimRad[i]) for i in range(6)]

        urdfJointOrigins = [SE3(0, 0, 0.120),
                            SE3.Rx(pi/2),
                            SE3(0.280, 0, 0),
                            SE3(0.225, 0, -0.090),
                            SE3(0, -0.085, 0) * SE3.Rx(pi/2),
                            SE3(0, 0.075, 0) * SE3.Rx(-pi/2)]
        meshPosesAtZero = [SE3()]
        for origin in urdfJointOrigins:
            meshPosesAtZero.append(meshPosesAtZero[-1] * origin)

        super().__init__(links, mesh_stem="JakaMiniCobo", mesh_dir=meshDir, name="JakaMiniCobo",
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
        verts = stlVertices(os.path.join(meshDir, f"JakaMiniCoboLink{i}.stl"))
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
        for shape, offset in carried:
            carriedPose = flange * offset
            if hits(boxCorners(shape, carriedPose), boxes, -0.002):
                return True
            if carriedPose.R[2, 2] < np.cos(maxCarryTilt):
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
        print(f"Stove arm: collision predicted on every path to {np.round(poses[-1].t, 3)} - stopping")
        return False
    for q in traj:
        arm.q = q
        flange = arm.fkine(q)
        for shape, offset in carried:
            shape.T = flange * offset
        env.step(dt)
    return True

def runStoveArm(env, cupParts, cupPickBase, tapSpoutEnd, cupHeight, cupWidth, mountPose):
    arm = JakaMiniCobo(base=mountPose)
    arm.q = homeQ
    arm.meshPoints = meshCollisionPoints(arm)
    arm.add_to_env(env)

    boxes = obstacleBoxes(env, ignore=cupParts, centre=arm.fkine_all(arm.q)[1].t)
    grip = lambda cupBase: gripPose(cupBase, cupWidth, cupHeight)
    cupHoldBase = SE3(tapSpoutEnd[0], tapSpoutEnd[1] - cupWidth/2 + 0.005, tapSpoutEnd[2] - spoutGap - cupHeight)

    if not moveThrough(env, arm, [grip(SE3(0, 0, approachGap) * cupPickBase), grip(cupPickBase)], boxes):
        return
    flange = arm.fkine(arm.q)
    carried = [(part, flange.inv() * SE3(part.T)) for part in cupParts]

    if not moveThrough(env, arm, [grip(SE3(0, 0, 0.1) * cupPickBase),
                                  grip(SE3(0, approachGap + cupWidth, 0) * cupHoldBase),
                                  grip(cupHoldBase)], boxes, carried):
        return

    cupFilled = Cylinder(radius=cupWidth/2 - 0.008, length=cupHeight * 0.8,
                         pose=cupHoldBase * SE3(0, 0, cupHeight * 0.4 + 0.005),
                         color="blue", collision=False)
    env.add(cupFilled)
    env.step(dt)
    return arm, cupFilled
