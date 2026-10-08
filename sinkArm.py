from math import pi
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder
from ir_support import CylindricalDHRobotPlot

class SinkBot6(rtb.DHRobot):
    def __init__(self, base=None):
        links = [
            rtb.RevoluteDH(d=0.15,  a=0,     alpha=pi/2),
            rtb.RevoluteDH(d=0,     a=-0.40, alpha=0),
            rtb.RevoluteDH(d=0,     a=-0.35, alpha=0),
            rtb.RevoluteDH(d=0.11,  a=0,     alpha=pi/2),
            rtb.RevoluteDH(d=0.10,  a=0,     alpha=-pi/2),
            rtb.RevoluteDH(d=0.08,  a=0,     alpha=0),
        ]
        super().__init__(links, name="SinkBot6", base=base)

steps = 50
dt = 0.05
homeQ = [-2.55, -1.29, 1.24, 0, -0.97, -1.57]
linkRadius = 0.03
gripGap = 0.01
approachGap = 0.08
spoutGap = 0.03
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

def hits(points, boxes, pad):
    return any(np.all((points > lo - pad) & (points < hi + pad), axis=1).any() for lo, hi in boxes)

def pathCollides(arm, traj, boxes, carried):
    for q in traj:
        if hits(armPoints(arm, q), boxes, linkRadius):
            return True
        flange = arm.fkine(q)
        for shape, offset in carried:
            if hits(boxCorners(shape, flange * offset), boxes, -0.002):
                return True
    return False

def planPath(arm, poses, boxes, carried):
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
                if not pathCollides(arm, traj, boxes, carried):
                    return traj
    return None

def gripPose(cupBase, cupWidth, cupHeight):
    return cupBase * SE3(cupWidth/2 + gripGap, 0, cupHeight/2) * SE3.Ry(-pi/2) * SE3.Rz(gripRoll)

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

def runSinkArm(env, cupParts, cupPickBase, tapSpoutEnd, cupHeight, cupWidth, mountPose):
    arm = SinkBot6(base=mountPose)
    arm = CylindricalDHRobotPlot(arm, cylinder_radius=linkRadius, color="white").create_cylinders()
    arm.q = homeQ
    env.add(arm)

    boxes = obstacleBoxes(env, ignore=cupParts)
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