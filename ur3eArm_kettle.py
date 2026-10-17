import os
from math import pi

import numpy as np
from spatialmath import SE3
from scipy.spatial import cKDTree
from ir_support import UR3e
from tableArmArnoe_kettle import armPoints, obstacleBoxes, solveMove, cupBaseFromParts, dt, checkTrajectory, collision, jointLimits, hits, boxCorners, armPoints as sampledArmPoints

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
        cupBase * SE3(0, 0, cupHeight*fraction) * SE3.Rz(yaw)
        * SE3(-cupWidth/2 - 0.01, 0, 0) * SE3.Ry(pi/2)
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
        mesh._filename = os.path.splitdrive(mesh.filename)[1].lstrip('\\/')
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

    # Grip the kettle in the current position: capture all offsets in flange coordinates.
    flange = arm.fkine(arm.q)
    carried = [(piece, flange.inv() * SE3(piece.T)) for piece in ignored]
    startBase = cupBaseFromParts(cupParts)
    startGrip = arm.fkine(arm.q)

    # The UR3e is to the left of the GP4 and right of the stove Jaka.
    # Keep the kettle upright while moving toward a position the stove arm can reach.
    # Begin nearer the UR3e; the old candidates at x=-1.0 were near/beyond
    # the robot's practical reach. The stove arm will still check its own reach.
    handoffPoints = [
        np.array([-0.48, -1.53, 1.17]),
        np.array([-0.55, -1.48, 1.20]),
        np.array([-0.62, -1.48, 1.19]),
        np.array([-0.40, -1.56, 1.21]),
        np.array([-0.70, -1.44, 1.19]),
    ]
    print('UR3e: GP4 clear; searching for closer upright handoff')
    print(f'UR3e: mount={np.round(arm.base.t, 3)}, kettle={np.round(startBase.t, 3)}')
    selected = None
    # Rotate the entire kettle/gripper pose about the kettle's vertical axis.
    # A translation-only target can demand an unreachable wrist configuration.
    # These yaw options preserve the upright kettle orientation.
    yawOptions = (0, pi/4, -pi/4, pi/2, -pi/2)
    for xyz in handoffPoints:
        for yaw in yawOptions:
            rotationAboutKettle = (
                SE3(*xyz) * SE3.Rz(yaw) * SE3(*(-startBase.t))
            )
            target = rotationAboutKettle * startGrip
            # Preflight an IK solution before requesting a costly planned path.
            ik = arm.ikine_LM(target, q0=arm.q, slimit=300)
            if not ik.success:
                print(f'UR3e: {np.round(xyz,3)}, yaw={np.degrees(yaw):.0f}°: IK unreachable')
                continue
            # Check goal before searching through collision-free joint paths.
            goalReason = collision(arm, ik.q, obstacles, carried)
            if goalReason is not None:
                print(f'UR3e: {np.round(xyz,3)}, yaw={np.degrees(yaw):.0f}°: goal rejected ({goalReason})')
                continue
            print(f'UR3e: {np.round(xyz,3)}, yaw={np.degrees(yaw):.0f}°: goal clear, planning route')
            path = solveMove(arm, target, obstacles, carried, sinkTree=None, allowDetour=True)
            if path is not None:
                selected = (path, xyz)
                break
            print(f'UR3e: {np.round(xyz,3)}, yaw={np.degrees(yaw):.0f}°: route unavailable despite clear goal')
        if selected is not None:
            break
    if selected is None:
        print('UR3e: no safe upright route to stove-side handoff; kettle retained at GP4 side')
        return None

    path, xyz = selected
    play(env, arm, path, carried)
    print(f'UR3e: kettle delivered to stove-side handoff at {np.round(xyz, 3)}')

    if stoveArm is None:
        print('UR3e: no stove arm supplied; pausing at handoff')
        return arm
    from stoveArmLouis_kettle import receiveKettleFromUR3e
    accepted = receiveKettleFromUR3e(env, stoveArm, cupParts, cupFilled,
                                     cupHeight, cupWidth, ur3eArm=arm)
    if not accepted:
        print('UR3e: stove could not safely accept kettle; UR3e keeps holding it')
        return arm
    print('UR3e: kettle transferred to stove arm')
    return arm
