"""Separate stove-arm receiver for the experimental lidless-kettle workflow.

The original stoveArmLouis.py remains untouched. All movements are simulated.
"""
from math import pi
import numpy as np
from spatialmath import SE3
try:
    from stoveArmLouis import makeStoveArm as _originalMakeStoveArm
except ImportError:
    from stoveArmLouis import JakaMiniCobo

    def _originalMakeStoveArm(mountPose):
        return JakaMiniCobo(base=mountPose)
from tableArmArnoe_kettle import obstacleBoxes, solveMove, cupBaseFromParts, dt


def makeStoveArm(mountPose):
    arm = _originalMakeStoveArm(mountPose)
    # Match Louis's resting joint configuration when the stove is added to Swift.
    from stoveArmLouis import homeQ, meshCollisionPoints
    arm.q = np.asarray(homeQ, dtype=float)
    arm.meshPoints = meshCollisionPoints(arm)
    return arm


def receiveKettleFromUR3e(env, arm, cupParts, water, height, width, ur3eArm=None):
    """Approach the stationary kettle, then transfer object ownership to stove arm.

    Returns False without moving the kettle if no collision-checked pickup is found.
    """
    from stoveArmLouis import gripPose as louisGripPose
    objects = list(cupParts) + ([water] if water is not None else [])
    base = cupBaseFromParts(cupParts)
    boxes = obstacleBoxes(env, objects, arm.base.t)
    # Move to one of several side grips, not the top opening or spout.
    starts = np.asarray(arm.q, dtype=float).copy()
    choices = [
        louisGripPose(base, width, height),
        base * SE3(0, 0, 0.65*height) * SE3.Rz(pi/2) * SE3(width/2 + 0.01, 0, 0) * SE3.Ry(-pi/2),
        base * SE3(0, 0, 0.65*height) * SE3.Rz(0) * SE3(width/2 + 0.01, 0, 0) * SE3.Ry(-pi/2),
    ]
    planned = None
    for target in choices:
        for retreat in (0.09, 0.13):
            # Attempt a reachable retreat point followed by a straight final pickup.
            first = solveMove(arm, target * SE3(0, 0, -retreat), boxes,
                              allowDetour=True)
            if first is None:
                continue
            arm.q = first[-1]
            second = solveMove(arm, target, boxes,
                               linear=True, allowDetour=False)
            arm.q = starts
            if second is not None:
                planned = (first, second)
                break
        if planned is not None:
            break
    if planned is None:
        print('Stove arm: unable to find a safe receiving pose')
        return False
    for path in planned:
        for q in path:
            arm.q = q
            env.step(dt)

    # Handoff: UR3e stays still while stove robot takes over kettle transforms.
    # No object deletion or recreation (avoids Swift scene-tree failures).
    flange = arm.fkine(arm.q)
    carried = [(obj, flange.inv() * SE3(obj.T)) for obj in objects]
    print('Stove arm: kettle accepted at shared handoff')

    # Small upward separation demonstrates that the stove now owns the kettle.
    lift = solveMove(arm, flange * SE3(0, 0, -0.045), boxes,
                     carried=carried, allowDetour=False)
    if lift is not None:
        for q in lift:
            arm.q = q
            tool = arm.fkine(q)
            for obj, offset in carried:
                obj.T = tool * offset
            env.step(dt)
        print('Stove arm: kettle lifted from handoff')
    else:
        print('Stove arm: kettle received; lift could not be safely planned')
    return True
