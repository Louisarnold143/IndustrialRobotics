import os
from math import pi

import numpy as np

from spatialmath import SE3

from scipy.spatial import cKDTree

from ir_support import UR3e

from tableArmArnoe import armPoints, obstacleBoxes, solveMove, cupBaseFromParts, dt

linkRadius = 0.04
ringPoints = 8
pointsAlongLink = 6
gripFraction = 0.65
gripYaws = (0, -pi/6, pi/6)
approachBacks = (0.08, 0.12)
gp4Retreat = 0.08


def linkCollisionPoints(arm):
    angles = np.linspace(0, 2*pi, ringPoints, endpoint=False)
    pointsPerLink = []
    for i, link in enumerate(arm.links):
        start = np.linalg.inv(link.A(0).A)[:3, 3]
        norm = np.linalg.norm(start)
        if norm < 1e-9:
            start = np.array([0., 0., 0.05])
            norm = np.linalg.norm(start)
        axis = start / norm
        side = np.cross(axis, [1, 0, 0] if abs(axis[0]) < 0.9 else [0, 1, 0])
        side /= np.linalg.norm(side)
        other = np.cross(axis, side)
        ring = linkRadius * (np.outer(np.cos(angles), side) + np.outer(np.sin(angles), other))
        centres = np.outer(np.linspace(0, 0.6 if i == 0 else 1, pointsAlongLink), start)
        pointsPerLink.append((centres[:, None, :] + ring[None]).reshape(-1, 3))
    return pointsPerLink


def gripPose(cupBase, cupWidth, cupHeight, yaw=0, fraction=gripFraction):
    return (
        cupBase
        * SE3(0, 0, cupHeight*fraction)
        * SE3.Rz(yaw)
        * SE3(-cupWidth/2 - 0.01, 0, 0)
        * SE3.Ry(pi/2)
    )


def otherArmTree(arms, cupCentre, keepOut):
    groups = [armPoints(other, other.q) for other in arms if other is not None]
    if not groups:
        return None
    points = np.vstack(groups)
    far = points[np.linalg.norm(points - cupCentre, axis=1) > keepOut]
    return cKDTree(far) if len(far) else None


def play(env, arm, path, carried=()):
    for q in path:
        arm.q = q
        for obj, relative in carried:
            obj.T = arm.fkine(q) * relative
        env.step(dt)


def makeUR3e(mountPose):
    arm = UR3e(base=mountPose)
    for mesh in arm.links_3d:
        mesh._filename = os.path.splitdrive(mesh.filename)[1].lstrip("\\/")
        mesh._collision = False
    arm.meshPoints = linkCollisionPoints(arm)
    return arm


def runUR3eArm(env, cupParts, cupFilled, cupHeight, cupWidth, mountPose, gp4Arm, sinkArm=None, arm=None, stoveArm=None):
    if arm is None:
        arm = makeUR3e(mountPose)
        arm.add_to_env(env)

    ignored = list(cupParts)
    if cupFilled is not None:
        ignored.append(cupFilled)
    cupBase = cupBaseFromParts(cupParts)
    cupCentre = cupBase.t + np.array([0, 0, cupHeight / 2])
    obstacles = obstacleBoxes(env, ignored, arm.base.t)
    otherTree = otherArmTree([gp4Arm], cupCentre, 0.10)

    planned = None
    startQ = np.asarray(arm.q, dtype=float).copy()
    for detour in (False, True):
        for yaw in gripYaws:
            gripTarget = gripPose(cupBase, cupWidth, cupHeight, yaw)
            for back in approachBacks:
                first = solveMove(arm, gripTarget * SE3(0, 0, -back), obstacles,
                                  sinkTree=otherTree, allowDetour=detour)
                if first is None:
                    continue
                arm.q = np.asarray(first[-1], dtype=float)
                second = solveMove(arm, gripTarget, obstacles, sinkTree=otherTree,
                                   linear=True, allowDetour=False)
                arm.q = startQ.copy()
                if second is not None:
                    planned = (first, second)
                    break
            if planned is not None:
                break
        if planned is not None:
            break
    arm.q = startQ.copy()
    if planned is None:
        print('UR3e: no safe kettle approach found')
        return None
    for trajectory in planned:
        play(env, arm, trajectory)

    gp4Obstacles = obstacleBoxes(env, ignored, gp4Arm.base.t)
    urTree = otherArmTree([arm, sinkArm], cupCentre, 0.10)
    retreat = solveMove(gp4Arm, gp4Arm.fkine(gp4Arm.q) * SE3(-gp4Retreat, 0, 0), gp4Obstacles,
                        sinkTree=urTree, linear=True, allowDetour=False)
    if retreat is None:
        print('UR3e: GP4 cannot safely retreat; transfer aborted')
        return None
    play(env, gp4Arm, retreat)

    # Keep the kettle upright while the UR3e rotates toward the stove side, but do
    # not claim a confirmed stove handoff until that path has been successfully tested.
    if stoveArm is None:
        print('UR3e: ready for future stove-side handoff but stove transfer is disabled')
        return arm

    print('UR3e: stove handoff path is experimental and still needs testing; transfer remains disabled')
    return arm
