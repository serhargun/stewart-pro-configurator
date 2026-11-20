"""Test pozları üretici (çalışma uzayının en kötü noktaları)."""
import itertools

import numpy as np


def generate_poses(x_mm=0, y_mm=0, z_mm=0, roll_deg=0, pitch_deg=0, yaw_deg=0, simultaneous=False):
    """(P,6) poz DEĞİŞİMLERİ [dx, dy, dz, roll, pitch, yaw] (mm, derece); ilk satır home (sıfır).

    simultaneous=False: home + her eksen ayrı ayrı +-uç.
    simultaneous=True: yukarıdakiler + sıfırdan farklı eksenlerin tüm 2^n köşe kombinasyonları.
    dz home yüksekliğine eklenir (z_mm = home etrafında +-sınır).
    """
    lim = np.array([x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg], float)
    poses = [np.zeros(6)]
    for i in range(6):
        if lim[i] != 0:
            for s in (1.0, -1.0):
                v = np.zeros(6)
                v[i] = s * abs(lim[i])
                poses.append(v)
    if simultaneous:
        act = [i for i in range(6) if lim[i] != 0]
        for signs in itertools.product((1.0, -1.0), repeat=len(act)):
            v = np.zeros(6)
            for i, s in zip(act, signs):
                v[i] = s * abs(lim[i])
            poses.append(v)
    P = np.array(poses)
    _, idx = np.unique(np.round(P, 9), axis=0, return_index=True)
    return P[np.sort(idx)]
