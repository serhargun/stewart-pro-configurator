"""Stewart platformu kinematiği (6-6, kardan merkezleri).

Çerçeve ve kurallar
- Taban çerçevesi: Z yukarı, orijin alt kardan düzleminin merkezi. Birim: mm, derece.
- Poz = (x, y, z, roll, pitch, yaw): platform çerçevesinin orijini (üst kardan düzlemi
  merkezi) taban çerçevesinde (x,y,z); z MUTLAK yüksekliktir (home'da = home_height).
- Euler sırası ZYX (yaw-pitch-roll): R = Rz(yaw) @ Ry(pitch) @ Rx(roll), sabit (taban)
  eksenlerinde. Dönme platform orijini etrafındadır.
- Bacak i, alt kardan i ile üst kardan i'yi bağlar (make_joints sırası).
- Jacobian J (6x6): [v (mm/s); w (rad/s)] -> bacak uzama hızları (mm/s); v ve w taban
  çerçevesinde, w platform orijini etrafında. Satır i = [u_i, p_i x u_i], p_i = R*top_i.
  Statikte f = J^-T * [F (N); M (N.mm)] (sanal iş).
"""
import numpy as np

G_ACC = 9.81


def rot_zyx(roll_deg, pitch_deg, yaw_deg):
    """ZYX dönme matrisi (...,3,3); açılar derece, yayınlanır."""
    r, p, y = np.broadcast_arrays(*[np.radians(np.asarray(a, float)) for a in (roll_deg, pitch_deg, yaw_deg)])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty(r.shape + (3, 3))
    R[..., 0, 0] = cy * cp
    R[..., 0, 1] = cy * sp * sr - sy * cr
    R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp
    R[..., 1, 1] = sy * sp * sr + cy * cr
    R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp
    R[..., 2, 1] = cp * sr
    R[..., 2, 2] = cp * cr
    return R


# ---------------------------------------------------------------- geometri üretici
def _pair_angles(phase_deg, sep_deg):
    c = phase_deg + 120.0 * np.arange(3)
    return np.stack([c - sep_deg / 2.0, c + sep_deg / 2.0], axis=1).ravel()  # artan sıra


def _wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def _leg_shift(ang_b, ang_t):
    """Üst kardanları, açısal toplam mesafeyi en küçük yapan döngüsel kaymayla sırala."""
    costs = [np.abs(_wrap(ang_b - np.roll(ang_t, -s))).sum() for s in range(6)]
    return int(np.argmin(costs))


def make_joints(R_base, R_top, base_pair_sep_deg, top_pair_sep_deg, top_rot_offset_deg,
                base_phase_deg=90.0):
    """Standart 6-6 Stewart kardan noktaları, bacak sırasıyla (alt i <-> üst i).

    Alt çiftlerin merkezleri base_phase + 120k, üst çiftlerin merkezleri
    base_phase + top_rot_offset + 120k. Her çiftte iki kardan merkezden +-sep/2.
    R_base / R_top skaler veya dizi olabilir (dizi -> (...,6,3)). z = 0.
    """
    ab = _pair_angles(base_phase_deg, base_pair_sep_deg)
    at = _pair_angles(base_phase_deg + top_rot_offset_deg, top_pair_sep_deg)
    at = np.roll(at, -_leg_shift(ab, at))

    def build(R, ang):
        R = np.asarray(R, float)[..., None]
        a = np.radians(ang)
        return np.stack([R * np.cos(a), R * np.sin(a), np.zeros(R.shape[:-1] + (6,))], axis=-1)

    return build(R_base, ab), build(R_top, at)


def _wrap120(x):
    return (np.asarray(x) + 60.0) % 120.0 - 60.0


def fit_parameters(ref):
    """Referans geometriden (kardan noktaları varsa onlardan) make_joints parametreleri."""
    if ref.get("base_joints_mm") and ref.get("top_joints_mm"):
        B = np.asarray(ref["base_joints_mm"], float)
        T = np.asarray(ref["top_joints_mm"], float)

        def fit(P):
            R = float(np.mean(np.hypot(P[:, 0], P[:, 1])))
            m = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 120.0
            c = (np.degrees(np.angle(np.mean(np.exp(1j * np.radians(m * 3.0))))) / 3.0) % 120.0
            sep = 2.0 * float(np.mean(np.abs(_wrap120(m - c))))
            return R, sep, c

        Rb, sb, cb = fit(B)
        Rt, st, ct = fit(T)
        off = float(_wrap120(ct - cb))
        if off <= -60.0 + 1e-9:
            off = 60.0
        return {"R_base": Rb, "R_top": Rt, "base_pair_sep_deg": sb, "top_pair_sep_deg": st,
                "top_rot_offset_deg": off, "base_phase_deg": cb}
    return {"R_base": float(ref["R_base_mm"]), "R_top": float(ref["R_top_mm"]),
            "base_pair_sep_deg": float(ref["base_pair_separation_deg"]),
            "top_pair_sep_deg": float(ref["top_pair_separation_deg"]),
            "top_rot_offset_deg": float(ref["top_rotation_offset_deg"]), "base_phase_deg": 90.0}


# ---------------------------------------------------------------- çekirdek (toplu)
def leg_kinematics(base, top, R, t):
    """base,top (G,6,3); R (P,3,3); t (G,P,3) -> l (G,P,6), u (G,P,6,3), p (G,P,6,3), J (G,P,6,6)."""
    p = np.einsum("pij,gkj->gpki", R, top)
    d = p + t[:, :, None, :] - base[:, None, :, :]
    l = np.linalg.norm(d, axis=-1)
    u = d / l[..., None]
    J = np.concatenate([u, np.cross(p, u)], axis=-1)
    return l, u, p, J


def _prep(base, top, pose):
    base = np.asarray(base, float)[None]
    top = np.asarray(top, float)[None]
    pose = np.asarray(pose, float)
    single = pose.ndim == 1
    pose = np.atleast_2d(pose)
    R = rot_zyx(pose[:, 3], pose[:, 4], pose[:, 5])
    t = pose[None, :, :3]
    return base, top, R, t, single


# ---------------------------------------------------------------- genel API
def inverse_kinematics(base, top, pose):
    """Poz (6,) veya (P,6) -> bacak boyları (6,)/(P,6) ve birim vektörler (6,3)/(P,6,3)."""
    b, tp, R, t, single = _prep(base, top, pose)
    l, u, _, _ = leg_kinematics(b, tp, R, t)
    return (l[0, 0], u[0, 0]) if single else (l[0], u[0])


def jacobian(base, top, pose):
    """Jacobian (6,6) / (P,6,6); bkz. modül açıklaması."""
    b, tp, R, t, single = _prep(base, top, pose)
    J = leg_kinematics(b, tp, R, t)[3]
    return J[0, 0] if single else J[0]


def joint_angles(base, top, pose):
    """(alt kardan açısı, üst kardan açısı) derece; her bacak için, alt plaka / üst plaka
    normaline göre bacak ekseni açısı."""
    b, tp, R, t, single = _prep(base, top, pose)
    _, u, _, _ = leg_kinematics(b, tp, R, t)
    a_low = np.degrees(np.arccos(np.clip(u[..., 2], -1, 1)))
    n = R[:, :, 2]
    a_up = np.degrees(np.arccos(np.clip(np.einsum("gpkj,pj->gpk", u, n), -1, 1)))
    return (a_low[0, 0], a_up[0, 0]) if single else (a_low[0], a_up[0])
