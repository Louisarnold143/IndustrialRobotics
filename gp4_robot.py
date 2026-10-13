"""Independent six-axis GP4 kinematic model based on the supplied ROS-Industrial GP4 Xacro.
Mesh geometry is from GP4_reference. Coordinates are in metres and radians.
"""
from pathlib import Path
from types import SimpleNamespace
from math import pi
import struct
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from spatialmath import SE3
from spatialgeometry import Mesh

ROOT = Path(__file__).resolve().parent
JOINT_OFFSETS = np.array([[0, 0, .330], [0, 0, 0], [0, 0, .260], [.290, 0, .015], [0, 0, 0], [0, 0, 0]], dtype=float)
JOINT_AXES = np.array([[0, 0, 1], [0, 1, 0], [0, -1, 0], [-1, 0, 0], [0, -1, 0], [-1, 0, 0]], dtype=float)
LIMITS_DEG = np.array([[-170, 170], [-110, 130], [-65, 200], [-200, 200], [-123, 123], [-455, 455]], dtype=float)
MESH_NAMES = ['base_link', 'link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t']
GP4_HOME = np.deg2rad([0, 0, 0, 0, 0, 0])


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
        self._env = None

    def frames(self, q):
        q = np.asarray(q, dtype=float)
        current = self.base
        frames = [current]
        for translation, axis, theta in zip(JOINT_OFFSETS, JOINT_AXES, q):
            current = current * SE3(*translation) * SE3.Rt(Rotation.from_rotvec(axis * theta).as_matrix(), [0, 0, 0])
            frames.append(current)
        return frames

    def fkine(self, q):
        # Link-6 frame to manufacturer flange is 72 mm along +X.
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
        return SimpleNamespace(success=success, q=result.x, residual=np.linalg.norm(error))

    def add_to_env(self, env):
        if self.objects:
            return
        for name, pose in zip(MESH_NAMES, self.frames(self.q)):
            path = ROOT / 'GP4_reference' / 'meshes' / 'gp4' / 'collision' / f'{name}.stl'
            part = Mesh(filename=str(path), pose=pose, color=[35, 100, 185])
            env.add(part)
            self.objects.append(part)
        self._env = env
