import os
from math import pi

import numpy as np

from spatialmath import SE3

from scipy.spatial import cKDTree

from ir_support import UR3e

from tableArmArnoe import armPoints, obstacleBoxes, solveMove, cupBaseFromParts, collision, dt

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


def describeHandoffFailure(reason):
    if reason is None:
        return 'no failure'
    mapping = {
        'joint limit': 'joint-limit violation',
        'arm/environment': 'arm collision with environment',
        'kettle/environment': 'kettle collision with environment',
        'kettle/other arm': 'kettle collision with another arm',
        'arm/other arm': 'arm collision with another arm',
        'kettle tilt': 'kettle would tilt beyond safe carry limit',
    }
    return mapping.get(reason, reason)


def validateStoveHandoffPose(arm, target, boxes, carried=(), sinkTree=None, label='UR3e'):
    if arm is None:
        return None, 'no arm supplied'
    result = arm.ikine_LM(target, q0=arm.q, slimit=300)
    if not result.success:
        return None, 'unreachable pose: IK failed'
    if not np.all(np.isfinite(result.q)):
        return None, 'unreachable pose: non-finite IK result'
    limits = np.asarray(arm.qlim, dtype=float)
    if not np.all(limits[0] <= result.q) or not np.all(result.q <= limits[1]):
        return None, 'joint-limit violation'
    reason = collision(arm, result.q, boxes, carried, sinkTree)
    if reason is not None:
        return None, describeHandoffFailure(reason)
    return result.q, None


def stoveHandoffCandidates(cupBase, cupHeight, cupWidth, flangeToKettle):
    positions = []

    for x in (-0.45, -0.55, -0.65):
        for y in (-1.65, -1.50):
            for z in (1.15, 1.25):
                positions.append(SE3(x, y, z))

    candidates = []

    for base in positions:
        for yaw in (0, pi/4, -pi/4):
            desiredKettlePose = base * SE3.Rz(yaw)
            target = desiredKettlePose * flangeToKettle.inv()
            candidates.append((base, yaw, target))

    return candidates

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

    flange = arm.fkine(arm.q)
    carried = [(piece, flange.inv() * SE3(piece.T)) for piece in ignored]
    kettleBaseAtPickup = cupBaseFromParts(cupParts)
    flangeToKettle = flange.inv() * kettleBaseAtPickup
    for piece, _ in carried:
        piece.T = flange * _
    if cupFilled is not None:
        cupFilled.T = flange * (flange.inv() * SE3(cupFilled.T))
    print(f'UR3e: kettle and water attached to UR3e flange at {np.round(flange.t, 3)}')

    if stoveArm is None:
        print('UR3e: ready for future stove-side handoff but stove transfer is disabled')
        return arm

    print('UR3e: testing stove-side handoff candidates before moving the kettle')
    stoveBoxes = obstacleBoxes(env, list(cupParts) + ([cupFilled] if cupFilled is not None else []), stoveArm.base.t)
    chosen = None
    for base, yaw, target in stoveHandoffCandidates(cupBase, cupHeight, cupWidth, flangeToKettle):
        u_q, reason = validateStoveHandoffPose(arm, target, obstacles, carried=carried, sinkTree=otherArmTree([gp4Arm, sinkArm], cupCentre, 0.10), label='UR3e')
        if reason is not None:
            print(f'UR3e: candidate {np.round(base.t, 3)}, yaw={np.degrees(yaw):.0f}° rejected: {reason}')
            continue

        from stoveArmLouis import stoveReceiveCandidatePose, validateStoveReceivePose
        stoveTarget = stoveReceiveCandidatePose(base, cupWidth, cupHeight, yaw)
        s_q, s_reason = validateStoveReceivePose(stoveArm, stoveTarget, stoveBoxes, carried=(), label='Stove arm')
        if s_reason is not None:
            print(f'Stove arm: candidate {np.round(base.t, 3)}, yaw={np.degrees(yaw):.0f}° rejected: {s_reason}')
            continue

        u_path = solveMove(arm, target, obstacles, carried=carried, sinkTree=None, allowDetour=True)
        if u_path is None:
            print(f'UR3e: candidate {np.round(base.t, 3)}, yaw={np.degrees(yaw):.0f}° failed path planning for the UR3e')
            continue
        s_path = solveMove(stoveArm, stoveTarget, stoveBoxes, carried=(), sinkTree=None, allowDetour=True)
        if s_path is None:
            print(f'Stove arm: candidate {np.round(base.t, 3)}, yaw={np.degrees(yaw):.0f}° failed path planning for the stove arm')
            continue

        chosen = (base, yaw, target, u_path, stoveTarget, s_path)
        print(f'UR3e: shared handoff pose found at {np.round(base.t, 3)}, yaw={np.degrees(yaw):.0f}°')
        break

    if chosen is None:
        print('UR3e: no valid shared stove handoff pose exists under the current mount geometry, joint limits, or collision constraints; keeping the kettle on the UR3e.')
        return arm

    _, _, _, u_path, _, _ = chosen
    play(env, arm, u_path, carried=carried)
    print('UR3e: moved to the selected stove-side receive pose')

    from stoveArmLouis import receiveKettleFromUR3e
    accepted = receiveKettleFromUR3e(env, stoveArm, cupParts, cupFilled, cupHeight, cupWidth, ur3eArm=arm)
    if accepted:
        print('UR3e: stove receiver accepted the kettle transfer')
    else:
        print('UR3e: stove receiver rejected or could not complete the transfer')
    return arm
