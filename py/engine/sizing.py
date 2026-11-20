"""Platform boyutlandırma: silindir + servo + plaka yarıçapları seçimi (Sözleşme 4)."""
import itertools
import re
import time
from dataclasses import asdict, dataclass, fields
from typing import Optional

import numpy as np

from .catalog import load_actuators, load_reference_geometry, load_servos
from .kinematics import G_ACC, fit_parameters, leg_kinematics, make_joints, rot_zyx
from .workspace import generate_poses

ETA = 0.9                    # vidalı mil + dişli verimi
DEFAULT_JOINT_LIMIT_DEG = 40.0
CHUNK = 600
SING_TOL = 1e-4              # Hadamard oranı altında jacobian tekil sayılır
FACTORS = ["kuvvet", "hız", "strok", "kardan açısı", "motor devri", "motor torku"]
MARGIN_KEYS = ["force_pct", "speed_pct", "stroke_pct", "joint_angle_pct", "motor_rpm_pct", "motor_torque_pct"]


@dataclass
class Requirements:
    payload_kg: float = 0.0
    cog_height_mm: float = 100.0
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    simultaneous: bool = False
    lin_speed_mm_s: float = 0.0
    ang_speed_deg_s: float = 0.0
    accel_m_s2: Optional[float] = None
    safety_factor: float = 1.5
    base_max_diameter_mm: Optional[float] = None
    top_max_diameter_mm: Optional[float] = None
    supply_voltage_V: Optional[int] = None
    min_margin_pct: float = 10.0
    plate_preference: str = "reference"   # "reference" | "smallest"
    stroke_margin_pct: float = 0.0        # strok payı eşiği (diğer paylar min_margin_pct)
    end_margin_mm: float = 2.0            # strok her ucunda mekanik emniyet payı
    load_cases: Optional[list] = None     # [{payload_kg, lin_speed_mm_s, ang_speed_deg_s, accel_m_s2}]

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


# ------------------------------------------------------------------ yardımcılar
def _dirs13():
    d = [v for v in itertools.product((-1, 0, 1), repeat=3) if any(v)]
    d = [np.array(v, float) / np.linalg.norm(v) for v in d]
    out, seen = [], set()
    for v in d:  # +-v çiftlerinden birini tut
        key = tuple(np.round(v, 6))
        if tuple(np.round(-v, 6)) in seen:
            continue
        seen.add(key)
        out.append(v)
    return np.array(out)


def _size_rank(name, idx):
    m = re.search(r"(\d+)", name or "")
    return int(m.group(1)) if m else 10_000 + idx


def _num(x):
    return x is not None and not (isinstance(x, float) and np.isnan(x))


def _cases(req):
    """Yük-hız rejimleri; load_cases yoksa üst alanlardan tek rejim."""
    if req.load_cases:
        return [{"payload_kg": float(c["payload_kg"]), "lin_speed_mm_s": float(c.get("lin_speed_mm_s") or 0.0),
                 "ang_speed_deg_s": float(c.get("ang_speed_deg_s") or 0.0),
                 "accel_m_s2": c["accel_m_s2"] if c.get("accel_m_s2") is not None else req.accel_m_s2}
                for c in req.load_cases]
    return [{"payload_kg": req.payload_kg, "lin_speed_mm_s": req.lin_speed_mm_s,
             "ang_speed_deg_s": req.ang_speed_deg_s, "accel_m_s2": req.accel_m_s2}]


def _thr(req):
    t = np.full(6, float(req.min_margin_pct))
    t[2] = req.stroke_margin_pct
    return t


def _build_context(req, poses, par):
    P = len(poses)
    R = rot_zyx(poses[:, 3], poses[:, 4], poses[:, 5])
    rcog = R[:, :, 2] * req.cog_height_mm                     # (P,3) mm
    cases = _cases(req)
    Ws_l, W_l, acc = [], [], []
    for c in cases:
        m = c["payload_kg"]
        Fg = np.array([0.0, 0.0, -m * G_ACC])
        if c["accel_m_s2"]:
            dirs = np.concatenate([_dirs13(), -_dirs13()])
            F = Fg[None] - m * c["accel_m_s2"] * dirs
        else:
            F = Fg[None]
        W = np.empty((P, len(F), 6))
        W[:, :, :3] = F[None]
        W[:, :, 3:] = np.cross(rcog[:, None, :], F[None])     # N.mm
        Ws = np.empty((P, 1, 6))
        Ws[:, 0, :3] = Fg
        Ws[:, 0, 3:] = np.cross(rcog, Fg[None])
        W_l.append(W)
        Ws_l.append(Ws)
        acc.append(bool(c["accel_m_s2"]))
    return {"R": R, "W": W_l, "Ws": Ws_l, "dirs": _dirs13(), "accel": acc, "cases": cases,
            "poses": poses, "par": par}


def _solve_forces(J, W):
    """J (g,P,6,6), W (P,D,6) -> |f| maks (g,)  [N]."""
    g, P = J.shape[:2]
    Wt = np.broadcast_to(W.transpose(0, 2, 1)[None], (g, P, 6, W.shape[1]))
    f = np.linalg.solve(J.swapaxes(-1, -2), Wt)
    return np.abs(f).max(axis=(1, 2, 3))


def _eval_geoms(Rb, Rt, Lh, ctx):
    """Geometri toplu değerlendirme; her eleman için (Rb, Rt, home bacak boyu Lh)."""
    par, poses, R = ctx["par"], ctx["poses"], ctx["R"]
    Rb, Rt, Lh = (np.asarray(a, float) for a in (Rb, Rt, Lh))
    n = len(Rb)
    base, top = make_joints(Rb, Rt, par["base_pair_sep_deg"], par["top_pair_sep_deg"],
                            par["top_rot_offset_deg"], par["base_phase_deg"])
    d2 = np.mean(np.sum((top[..., :2] - base[..., :2]) ** 2, axis=-1), axis=-1)
    valid = Lh ** 2 > d2 + 1.0
    H = np.sqrt(np.maximum(Lh ** 2 - d2, 1.0))
    C = len(ctx["cases"])
    out = {k: np.zeros(n) for k in ("lmin", "lmax", "rlin", "rang", "ang")}
    for k in ("fstat", "ftot", "fhome"):
        out[k] = np.zeros((n, C))
    out["sing"] = np.zeros(n, bool)
    out["valid"] = valid
    out["H"] = H
    for s in range(0, n, CHUNK):
        sl = slice(s, min(n, s + CHUNK))
        t = np.zeros((sl.stop - sl.start, len(poses), 3))
        t[..., :2] = poses[None, :, :2]
        t[..., 2] = H[sl, None] + poses[None, :, 2]
        l, u, _, J = leg_kinematics(base[sl], top[sl], R, t)
        Js = J.copy()
        Js[..., 3:] /= Rt[sl, None, None, None]   # moment sütunlarını boyutsuzlaştır
        had = np.linalg.norm(Js, axis=-1).prod(-1)
        sing = (np.abs(np.linalg.det(Js)) / np.maximum(had, 1e-12)) < SING_TOL
        sing_any = sing.any(axis=1)
        J = np.where(sing[..., None, None], np.eye(6), J)
        out["lmin"][sl] = l.min(axis=(1, 2))
        out["lmax"][sl] = l.max(axis=(1, 2))
        out["sing"][sl] = sing_any
        for c in range(C):
            fs = _solve_forces(J, ctx["Ws"][c])
            out["fstat"][sl, c] = fs
            out["ftot"][sl, c] = _solve_forces(J, ctx["W"][c]) if ctx["accel"][c] else fs
            out["fhome"][sl, c] = _solve_forces(J[:, :1], ctx["Ws"][c][:1])   # poses[0] = home
        out["rlin"][sl] = np.abs(J[..., :3] @ ctx["dirs"].T).max(axis=(1, 2, 3))
        out["rang"][sl] = np.abs(J[..., 3:]).max(axis=(1, 2, 3))
        a_low = np.arccos(np.clip(u[..., 2], -1, 1))
        a_up = np.arccos(np.clip(np.einsum("gpkj,pj->gpk", u, R[:, :, 2]), -1, 1))
        out["ang"][sl] = np.degrees(np.maximum(a_low, a_up).max(axis=(1, 2)))
    return out


# ------------------------------------------------------------------ puanlama
def _score(ev, Rb, Rt, stroke, Lh, lead, gear, sv, ctx):
    req, cases = ctx["req"], ctx["cases"]
    n, C = len(Rb), len(cases)
    k = lead["lead_mm"] / 1000.0 / (2 * np.pi * ETA * gear)
    rr = np.where(np.isfinite(sv["rated_rpm"]), sv["rated_rpm"], np.inf)[:, None]
    pc = {key: [] for key in ("vleg", "screw", "motor_rpm", "Tc", "Tp", "Fp", "m_force", "m_speed", "mr", "mtr", "mtp")}
    for c, cs in enumerate(cases):
        vleg = np.maximum(ev["rlin"] * cs["lin_speed_mm_s"], ev["rang"] * np.radians(cs["ang_speed_deg_s"]))
        Fp = ev["ftot"][:, c] * req.safety_factor    # çalışma uzayının en kötü pozu (ivme dahil)
        Fh = ev["fhome"][:, c] * req.safety_factor   # home pozdaki statik kuvvet
        screw = vleg / lead["lead_mm"] * 60.0
        mrpm = screw * gear
        Tc, Tp = Fh * k, Fp * k
        fac = np.minimum(1.0, rr / np.maximum(mrpm[None], 1e-9))   # anma devri üstünde tork düşümü
        avT, avP = sv["rated_T"][:, None] * fac, sv["peak_T"][:, None] * fac
        for key, v in (("vleg", vleg), ("screw", screw), ("motor_rpm", mrpm), ("Tc", Tc), ("Tp", Tp), ("Fp", Fp),
                       ("m_force", (lead["max_force_N"] - Fp) / lead["max_force_N"] * 100),
                       ("m_speed", (lead["speed_limit"] - vleg) / lead["speed_limit"] * 100),
                       ("mr", (sv["max_rpm"][:, None] - mrpm[None]) / sv["max_rpm"][:, None] * 100),
                       ("mtr", (avT - Tc[None]) / avT * 100), ("mtp", (avP - Tp[None]) / avP * 100)):
            pc[key].append(v)
    pc = {key: np.array(v) for key, v in pc.items()}            # (C,n) veya (C,S,n)
    s_score = np.minimum(np.minimum(pc["mr"], pc["mtr"]), pc["mtp"]).min(axis=0)   # tüm rejimlerde yetmeli
    okm = s_score >= req.min_margin_pct
    sel = np.where(okm.any(axis=0), okm.argmax(axis=0), s_score.argmax(axis=0))
    ar = np.arange(n)
    mrpm_c = pc["mr"][:, sel, ar]
    mtq_c = np.minimum(pc["mtr"][:, sel, ar], pc["mtp"][:, sel, ar])
    Mc = np.stack([pc["m_force"], pc["m_speed"], mrpm_c, mtq_c], axis=2)   # (C,n,4)
    lc = Mc.min(axis=2).argmin(axis=0)                                      # sınırlayan rejim
    dev = np.maximum(ev["lmax"] - Lh, Lh - ev["lmin"])
    req_stroke = 2 * dev + 2 * req.end_margin_mm
    m_stroke = (stroke - req_stroke) / stroke * 100
    lim = ctx["joint_limit"]
    m_joint = (lim - ev["ang"]) / lim * 100
    M = np.stack([Mc[:, :, 0].min(axis=0), Mc[:, :, 1].min(axis=0), m_stroke, m_joint,
                  Mc[:, :, 2].min(axis=0), Mc[:, :, 3].min(axis=0)], axis=1)
    bad = (~ev["valid"]) | ev["sing"] | ~np.isfinite(M).all(axis=1)
    M = np.where(bad[:, None], -1e9, M)
    thr = _thr(req)
    sf = np.where(bad, 1e9, np.clip(thr[None] - np.maximum(M, -1000.0), 0, None).sum(axis=1))
    g = lambda key: pc[key][lc, ar]   # sınırlayan rejimin değerleri
    return {"sf": sf, "M": M, "feasible": (M >= thr[None]).all(axis=1) & ~bad, "sel": sel,
            "vleg": g("vleg"), "motor_rpm": g("motor_rpm"), "screw_rpm": g("screw"), "Tc": g("Tc"), "Tp": g("Tp"),
            "Fp": g("Fp"), "req_stroke": req_stroke, "bad": bad, "pc": pc, "Mc": Mc, "lc": lc, "cases": cases}


def _pref(ctx):
    return getattr(ctx["req"], "plate_preference", "reference")


def _ref_dev(Rb, Rt, ctx):
    """Referans R_base'e uzaklık (mm) ve R_top/R_base oranının referans orandan sapması."""
    par = ctx["par"]
    return np.abs(Rb - par["R_base"]), np.abs(Rt / Rb - par["R_top"] / par["R_base"])


def _pick(sc, ev, Rb, Rt, ctx, sv):
    """Skor dizilerinden en iyi tek elemanı seç (uygun varsa plaka/güç sıralı, yoksa en yüksek min pay)."""
    pm = ctx["plate_margin"]
    if sc["feasible"].any():
        idx = np.where(sc["feasible"])[0]
        dia_b, dia_t = 2 * (Rb[idx] + pm[0]), 2 * (Rt[idx] + pm[1])
        pw = sv["power"][sc["sel"][idx]]
        if _pref(ctx) == "reference":
            d, rd = _ref_dev(Rb[idx], Rt[idx], ctx)
            o = np.lexsort((rd, np.round(d, 1), pw))
        else:
            o = np.lexsort((dia_b + dia_t, np.maximum(dia_b, dia_t), pw))
        i = int(idx[o[0]])
    else:   # en az payı kaçıran geometriler içinden plaka tercihine göre makul olanı
        sf = sc["sf"]
        idx = np.where(sf <= sf.min() + 1.0)[0]
        if _pref(ctx) == "reference":
            d, rd = _ref_dev(Rb[idx], Rt[idx], ctx)
            o = np.lexsort((rd, np.round(d, 1)))
        else:
            o = np.lexsort((Rb[idx] + Rt[idx], np.maximum(Rb[idx], Rt[idx])))
        i = int(idx[o[0]])
    return i


def _make_rec(i, sc, ev, Rb, Rt, grp, lead, gear, stroke, Lh):
    pc, Mc, sel = sc["pc"], sc["Mc"], int(sc["sel"][i])
    per_case = []
    for c, cs in enumerate(sc["cases"]):
        mc = Mc[c, i]
        per_case.append({**cs, "max_leg_force_N": float(ev["ftot"][i, c]), "max_leg_speed_mm_s": float(pc["vleg"][c, i]),
                         "motor_rpm": float(pc["motor_rpm"][c, i]), "continuous_torque_Nm": float(pc["Tc"][c, i]),
                         "peak_torque_Nm": float(pc["Tp"][c, i]),
                         "limiting_factor": [FACTORS[j] for j in (0, 1, 4, 5)][int(np.argmin(mc))],
                         "min_margin_pct": float(mc.min())})
    return {"i": i, "model": grp, "lead": lead, "gear": gear, "stroke": stroke, "Lh": Lh,
            "Rb": float(Rb[i]), "Rt": float(Rt[i]), "M": sc["M"][i].copy(), "sf": float(sc["sf"][i]),
            "feasible": bool(sc["feasible"][i]), "servo": sel,
            "H": float(ev["H"][i]), "lmin": float(ev["lmin"][i]), "lmax": float(ev["lmax"][i]),
            "fstat": float(ev["fstat"][i].max()), "ftot": float(ev["ftot"][i].max()), "ang": float(ev["ang"][i]),
            "vleg": float(sc["vleg"][i]), "motor_rpm": float(sc["motor_rpm"][i]),
            "screw_rpm": float(sc["screw_rpm"][i]), "Tc": float(sc["Tc"][i]), "Tp": float(sc["Tp"][i]),
            "Fp": float(sc["Fp"][i]), "req_stroke": float(sc["req_stroke"][i]),
            "per_case": per_case, "lc": int(sc["lc"][i]),
            "sing": bool(ev["sing"][i]), "valid": bool(ev["valid"][i])}


# ------------------------------------------------------------------ ana fonksiyon
# ------------------------------------------------------------------ katalog uyarlama
EXCLUDED_MODELS = {
    "IMTR": "IMTR: sayfa güvenilmez (hatve/vida/hız tutarsız, ölçü tablosu XH kopyası)",
    "IMT125XH-T": "IMT125XH-T: sayfa IMT125XH ile birebir kopya içerik",
}
DESIGN_JOINT_LIMIT_DEG = 40.0


def _sv_arrays(motors):
    motors = sorted(motors, key=lambda m: (m["rated_power_W"], 0 if m.get("voltage_V") in (None, 220) else 1,
                                           m["rated_torque_Nm"]))
    return {"power": np.array([m["rated_power_W"] for m in motors], float),
            "rated_T": np.array([m["rated_torque_Nm"] for m in motors], float),
            "peak_T": np.array([m["peak_torque_Nm"] for m in motors], float),
            "max_rpm": np.array([m["max_speed_rpm"] for m in motors], float),
            "rated_rpm": np.array([m.get("rated_speed_rpm") or np.nan for m in motors], float),
            "motors": motors}


def _norm_lead(m, l, unverified):
    name = m.get("model")
    if not all(_num(l.get(k)) for k in ("lead_mm", "max_speed_mm_s", "max_force_N")):
        return None
    f = float(l["max_force_N"])
    dual = False
    alt = l.get("alt_values_imotion_com_tr") or {}
    av = [alt[k] for k in ("nominal_force_N", "dynamic_load_N") if _num(alt.get(k))]
    if av and abs(min(av) - f) > 1e-6:
        dual = True
        f = min(f, min(av))
    rf = m.get("retracted_length_formula") or {}
    base = rf.get("base_mm")
    if not _num(base):
        base = (rf.get("base_mm_by_lead") or {}).get(f"{l['lead_mm']:g}")
    if not _num(base):
        base = rf.get("base_mm_inline")
    if not _num(base):
        unverified.append(f"{name}: {l['lead_mm']:g} mm hatve için kapalı boy verisi yok, hesaba katılmadı.")
        return None
    return {"lead_mm": float(l["lead_mm"]), "max_speed_mm_s": float(l["max_speed_mm_s"]), "max_force_N": f,
            "max_input_rpm": l.get("max_input_rpm"), "force_dual": dual, "base_mm": float(base),
            "speed_limit": (float(l["lead_mm"]) * float(l["max_input_rpm"]) / 60.0 if _num(l.get("max_input_rpm"))
                            else float(l["max_speed_mm_s"]))}


def _norm_model(m, idx, unverified):
    name = m.get("model", f"model{idx}")
    leads = [x for x in (_norm_lead(m, l, unverified) for l in (m.get("lead_options") or [])) if x]
    strokes = sorted(s for s in (m.get("stroke_options_mm") or []) if _num(s))
    default = False
    if not strokes and _num(m.get("stroke_max_mm")):
        strokes = [float(s) for s in range(50, int(m["stroke_max_mm"]) + 1, 50)]
        default = True
    if not leads or not strokes:
        unverified.append(f"{name}: hatve/strok verisi eksik, hesaba katılmadı.")
        return None
    unverified.extend(f"{name}: {v}" for v in (m.get("verify") or []))
    rf = m.get("retracted_length_formula") or {}
    return {"name": name, "rank": _size_rank(name, idx), "leads": leads, "strokes": strokes,
            "gears": [g for g in (m.get("gear_ratio_options") or [1]) if _num(g)] or [1],
            "per_stroke": rf.get("per_stroke") or 1.0, "stroke_default": default,
            "used": m.get("used_before") is True,
            "flanges": [f for f in (m.get("motor_flanges_mm") or []) if _num(f)] or None}


def _model_sv(g, motors, sv):
    """Katalogda motor flanş listesi varsa yalnız o flanşlı motorlar; yoksa filtre yok."""
    if not g.get("flanges"):
        return sv
    mm = [m for m in motors if m.get("flange_mm") in g["flanges"]]
    return _sv_arrays(mm) if mm else None


def _prepare(req, cat, srv, ref, need_models=True):
    assumptions, unverified = [], []
    par = fit_parameters(ref)
    plates = ref.get("plates") or {}
    if plates.get("base_outer_diameter_mm") and plates.get("top_outer_diameter_mm"):
        pm = (plates["base_outer_diameter_mm"] / 2 - par["R_base"], plates["top_outer_diameter_mm"] / 2 - par["R_top"])
        assumptions.append("Plaka dış çapı = 2 x (kardan yarıçapı + referans geometrideki plaka payı "
                           f"(alt {pm[0]:.1f} mm, üst {pm[1]:.1f} mm)).")
    else:
        pm = (20.0, 20.0)
        assumptions.append("Referans geometride plaka çapı yok: plaka payı kardan yarıçapına 20 mm varsayıldı.")

    # kardan-kardan ofset: 55 (arka 39 + ön 16) + rod ucu/flanş ek boyu (IMTL40 referansından)
    if _num(ref.get("joint_offset_rear_mm")) and _num(ref.get("joint_offset_front_mm")):
        joff = float(ref["joint_offset_rear_mm"]) + float(ref["joint_offset_front_mm"])
        assumptions.append(f"Kardan-kardan boy = gövde boyu + ek boy + {joff:g} mm (arka {ref['joint_offset_rear_mm']:g} + ön {ref['joint_offset_front_mm']:g}, referans geometriden).")
    elif _num(ref.get("joint_offset_mm")):
        joff = 2 * float(ref["joint_offset_mm"])
        assumptions.append(f"Kardan-kardan boy = silindir boyu + {joff:g} mm (uç başına {ref['joint_offset_mm']:g}).")
    else:
        joff = 0.0
        assumptions.append("Kardan ofseti referans geometride yok: 0 mm varsayıldı.")
        unverified.append("Referans geometri: kardan ofseti teyit gerekli.")
    ext = 0.0
    ag = ref.get("actuator_geometry") or {}
    l40 = next((m for m in cat.get("models", []) if m.get("model") == "IMTL40"), None)
    if l40 and _num(ag.get("rear_flange_face_to_rod_end_mm")) and _num(ag.get("nominal_stroke_mm")) \
            and _num((l40.get("retracted_length_formula") or {}).get("base_mm_inline")):
        rf40 = l40["retracted_length_formula"]
        body = rf40["base_mm_inline"] + (rf40.get("per_stroke") or 1.0) * ag["nominal_stroke_mm"]
        ext = float(ag["rear_flange_face_to_rod_end_mm"]) - body
        assumptions.append(f"Rod ucu/flanş ek boyu {ext:g} mm: referans montajda arka yüz-rod ucu {ag['rear_flange_face_to_rod_end_mm']:g} mm "
                           f"- IMTL40 gövde formülü ({body:g} mm); tüm modellere aynı ek boy uygulandı.")
        unverified.append("Rod ucu/flanş ek boyu IMTL40 referansından alındı, diğer modellere aynen uygulandı (teyit gerekli).")
    else:
        assumptions.append("Rod ucu/flanş ek boyu hesaplanamadı (IMTL40 referansı yok): 0 mm alındı.")

    rj = cat.get("rod_end_joint") or {}
    cat_lim = rj.get("max_working_angle_deg")
    joint_limit = DESIGN_JOINT_LIMIT_DEG if not _num(cat_lim) else min(float(cat_lim), DESIGN_JOINT_LIMIT_DEG)
    if _num(cat_lim):
        assumptions.append(f"Kardan azami açısı: katalog {cat_lim:g} derece; pratik pay için tasarım sınırı {DESIGN_JOINT_LIMIT_DEG:g} derece uygulandı.")
        unverified.append("Kardan azami açısı Mario Ferri 1G kataloğundan (45 derece); parça eşleşmesi teyit gerekli.")
    else:
        assumptions.append(f"Kardan azami açısı katalogda yok: {DESIGN_JOINT_LIMIT_DEG:g} derece varsayıldı.")
        unverified.append("Kardan azami açısı katalogda yok, 40 derece varsayıldı.")

    poses = generate_poses(req.x_mm, req.y_mm, req.z_mm, req.roll_deg, req.pitch_deg, req.yaw_deg, req.simultaneous)
    ctx = _build_context(req, poses, par)
    ctx.update(req=req, plate_margin=pm, joint_limit=joint_limit)

    sv, mot = None, []
    if srv is not None:
        mot = [m for m in srv.get("motors", []) if all(_num(m.get(k)) for k in
               ("rated_power_W", "rated_torque_Nm", "peak_torque_Nm", "max_speed_rpm"))]
        sup = getattr(req, "supply_voltage_V", None)
        if sup is not None:
            mot = [m for m in mot if m.get("voltage_V") == sup]
            assumptions.append(f"Servo besleme gerilimi {sup} V olarak sınırlandı.")
        else:
            assumptions.append("Besleme gerilimi belirtilmedi: 220 V ve 380 V motorlar birlikte arandı, eşit güçte 220 V tercih edildi.")
        unverified += [f"Servo: {v}" for v in srv.get("verify", []) or []]
        sv = _sv_arrays(mot) if mot else None
    models, excluded = [], []
    if need_models:
        for idx, m in enumerate(cat.get("models", [])):
            if m.get("model") in EXCLUDED_MODELS:
                excluded.append(EXCLUDED_MODELS[m["model"]])
                continue
            g = _norm_model(m, idx, unverified)
            if g:
                g["sv"] = _model_sv(g, mot, sv)
                if g["sv"] is None:
                    unverified.append(f"{g['name']}: flanşa ({g['flanges']} mm) uyan servo yok, hesaba katılmadı.")
                    continue
                models.append(g)
        if excluded:
            assumptions.append("Seçime katılmayan (güvenilmez veri) modeller: " + "; ".join(excluded) + ".")
        dm = [g["name"] for g in models if g["stroke_default"]]
        if dm:
            assumptions.append("Standart strok listesi yayımlanmamış modellerde 50 mm adımlı 50...azami strok standart kabul edildi: " + ", ".join(dm) + ".")
            unverified.append("Standart strok adımları (50 mm) varsayım: " + ", ".join(dm))
        models.sort(key=lambda g: g["rank"])
    return {"par": par, "pm": pm, "joff": joff, "ext": ext, "joint_limit": joint_limit, "cat_limit": cat_lim,
            "poses": poses, "ctx": ctx, "sv": sv, "motors": mot, "models": models, "assumptions": assumptions,
            "unverified": unverified}


def _lhome(lead, per_stroke, stroke, P):
    return lead["base_mm"] + per_stroke * stroke + P["ext"] + P["joff"] + stroke / 2.0


def _run_combo(g, lead, gear, stroke, Rb, Rt, ev, Lh_val, P):
    Lh = np.full(len(Rb), Lh_val)
    sc = _score(ev, Rb, Rt, stroke, Lh, lead, gear, g["sv"], P["ctx"])
    i = _pick(sc, ev, Rb, Rt, P["ctx"], g["sv"])
    rec = _make_rec(i, sc, ev, Rb, Rt, g, lead, gear, stroke, float(Lh_val))
    rec["retr"] = lead["base_mm"] + g["per_stroke"] * stroke + P["ext"]
    return rec


def _rkey(rec, P):
    sv = rec["model"]["sv"]
    pm = P["pm"]
    db, dt = 2 * (rec["Rb"] + pm[0]), 2 * (rec["Rt"] + pm[1])
    if _pref(P["ctx"]) == "reference":
        d, rd = _ref_dev(rec["Rb"], rec["Rt"], P["ctx"])
        plate = (round(float(d), 1), float(rd))
    else:
        plate = (max(db, dt), db + dt)
    if rec["feasible"]:
        # kullanılmış model, boyut, strok, güç; aynı güçte hız payı büyük (büyük hatve) önde
        return (0, 0 if rec["model"]["used"] else 1, rec["model"]["rank"], rec["stroke"],
                sv["power"][rec["servo"]], -round(float(rec["M"][1])), *plate, rec["lead"]["lead_mm"], rec["gear"])
    return (1, round(rec["sf"]), rec["model"]["rank"], rec["stroke"], sv["power"][rec["servo"]], *plate,
            rec["lead"]["lead_mm"], rec["gear"])


# ------------------------------------------------------------------ ana fonksiyon
def size_platform(req, catalog=None, servos=None, ref=None):
    t0 = time.perf_counter()
    cat = catalog if catalog is not None else load_actuators()
    srv = servos if servos is not None else load_servos()
    ref = ref if ref is not None else load_reference_geometry()
    if isinstance(req, dict):
        req = Requirements.from_dict(req)
    if req.plate_preference not in ("reference", "smallest"):
        raise ValueError("plate_preference 'reference' veya 'smallest' olmalı")
    P = _prepare(req, cat, srv, ref)
    P["assumptions"].append(
        "Plaka tercihi 'referans': min payı sağlayan geometriler arasından R_base referansa (%.0f mm) en yakın, R_top/R_base oranı referansa (%.2f) yakın olan seçildi."
        % (P["par"]["R_base"], P["par"]["R_top"] / P["par"]["R_base"]) if req.plate_preference == "reference" else
        "Plaka tercihi 'en küçük': min payı sağlayan en küçük plaka seçildi.")
    assumptions, unverified, models, ctx, par, pm = (P["assumptions"], P["unverified"], P["models"],
                                                      P["ctx"], P["par"], P["pm"])
    if P["sv"] is None:
        raise ValueError("servo kataloğunda kullanılabilir motor yok")
    if not models:
        raise ValueError("silindir kataloğunda kullanılabilir model yok")
    poses = P["poses"]
    Rb_ref, Rt_ref = par["R_base"], par["R_top"]

    lo_b, hi_b, lo_t, hi_t = 0.6 * Rb_ref, 2.0 * Rb_ref, 0.6 * Rt_ref, 2.0 * Rt_ref
    cap_b = req.base_max_diameter_mm / 2 - pm[0] if req.base_max_diameter_mm else None
    cap_t = req.top_max_diameter_mm / 2 - pm[1] if req.top_max_diameter_mm else None
    n_groups = sum(len(g["strokes"]) for g in models)
    nR = int(np.clip(np.sqrt(60_000 / (n_groups * len(poses))), 6, 40))
    step_b = max(5.0, (hi_b - lo_b) / (nR - 1))
    step_t = max(5.0, (hi_t - lo_t) / (nR - 1))

    def grid(lo, hi, step, cap):
        v = np.arange(lo, hi + 1e-9, step)
        if cap is not None:
            v = v[v <= cap + 1e-9]
            if cap > 30 and (len(v) == 0 or v[-1] < cap - 1e-6):
                v = np.append(v, cap)
        return v

    gb, gt = grid(lo_b, hi_b, step_b, cap_b), grid(lo_t, hi_t, step_t, cap_t)
    if req.plate_preference == "reference":   # referans yarıçaplar ızgarada tam bulunsun
        if cap_b is None or Rb_ref <= cap_b + 1e-9:
            gb = np.unique(np.append(gb, Rb_ref))
        if cap_t is None or Rt_ref <= cap_t + 1e-9:
            gt = np.unique(np.append(gt, Rt_ref))
    if len(gb) == 0 or len(gt) == 0:
        return _fail(req, assumptions, unverified, "Plaka çapı sınırı çok küçük: kardan yerleşimi mümkün değil.")
    GB, GT = np.meshgrid(gb, gt, indexing="ij")
    GB, GT = GB.ravel(), GT.ravel()
    keep = GT <= GB + 1e-9
    GB, GT = GB[keep], GT[keep]
    if len(GB) == 0:
        return _fail(req, assumptions, unverified, "R_top <= R_base kısıtıyla geçerli yarıçap çifti yok.")

    # -- aşama 1: kaba ızgara; (model, hatve, dişli) başına ilk yeten strok
    records, n_eval = [], 0
    for g in models:
        pending = [(l, gr) for l in g["leads"] for gr in g["gears"]]
        best_inf = {}
        for stroke in g["strokes"]:
            if not pending:
                break
            cache = {}
            for combo in list(pending):
                lead, gear = combo
                Lh_val = _lhome(lead, g["per_stroke"], stroke, P)
                ck = round(Lh_val, 6)
                if ck not in cache:
                    cache[ck] = _eval_geoms(GB, GT, np.full(len(GB), Lh_val), ctx)
                    n_eval += len(GB)
                rec = _run_combo(g, lead, gear, stroke, GB, GT, cache[ck], Lh_val, P)
                key = (id(lead), gear)
                if rec["feasible"]:
                    records.append(rec)
                    pending.remove(combo)
                elif key not in best_inf or _rkey(rec, P) < _rkey(best_inf[key], P):
                    best_inf[key] = rec
        for lead, gear in pending:
            records.append(best_inf[(id(lead), gear)])

    # -- aşama 2: 5 mm ince ızgara ile yerel iyileştirme
    ob = np.arange(-step_b, step_b + 1e-9, 5.0)
    ot = np.arange(-step_t, step_t + 1e-9, 5.0)
    refined = []
    for rec in records:
        Rb = (rec["Rb"] + ob[:, None] + 0 * ot[None]).ravel()
        Rt = (rec["Rt"] + 0 * ob[:, None] + ot[None]).ravel()
        ok = (Rb >= min(lo_b, cap_b or lo_b) - 1e-9) & (Rt >= min(lo_t, cap_t or lo_t) - 1e-9)
        ok &= (Rb <= hi_b + 1e-9) & (Rt <= hi_t + 1e-9)
        ok &= Rt <= Rb + 1e-9
        if cap_b is not None:
            ok &= Rb <= cap_b + 1e-9
        if cap_t is not None:
            ok &= Rt <= cap_t + 1e-9
        Rb, Rt = Rb[ok], Rt[ok]
        if len(Rb) == 0:
            refined.append(rec)
            continue
        ev = _eval_geoms(Rb, Rt, np.full(len(Rb), rec["Lh"]), ctx)
        n_eval += len(Rb)
        new = _run_combo(rec["model"], rec["lead"], rec["gear"], rec["stroke"], Rb, Rt, ev, rec["Lh"], P)
        refined.append(new if _rkey(new, P) < _rkey(rec, P) else rec)
    records = refined
    if not records:
        return _fail(req, assumptions, unverified, "Hiçbir kombinasyon değerlendirilemedi.")

    feas = sorted([r for r in records if r["feasible"]], key=lambda r: _rkey(r, P))
    infe = sorted([r for r in records if not r["feasible"]], key=lambda r: _rkey(r, P))
    ok = bool(feas)
    if ok:
        top, seen = [], set()
        for r in feas:   # önce her silindir modelinin en iyisi (alternatif görünsün)
            if r["model"]["name"] not in seen and len(top) < 5:
                seen.add(r["model"]["name"])
                top.append(r)
        for r in feas:
            if len(top) < 5 and r not in top:
                top.append(r)
        chosen = sorted(top, key=lambda r: _rkey(r, P)) + infe[:2]
    else:
        chosen = infe[:5]

    assumptions += [
        "Statik + hız yeterli sayıldı: ivme verilmediği için eylemsizlik kuvveti hesaba katılmadı."
        if not any(c["accel_m_s2"] for c in ctx["cases"]) else
        "İvme verilen rejimlerde en kötü yönde (26 yön) m x a olarak eklendi.",
        (f"{len(ctx['cases'])} yük-hız rejimi birlikte karşılandı: her pay rejimler içindeki en küçük paydır; "
         "motor tüm rejimlerde yetecek şekilde seçildi.") if len(ctx["cases"]) > 1 else
        "Tek yük-hız rejimi kullanıldı.",
        f"Emniyet katsayısı ({req.safety_factor:g}) kuvvete ve motor torkuna uygulandı; hız ve strok payları ayrıca emniyet katsayısı içermez.",
        "Kütle yalnız payload: payload plaka ve ek aparatları kapsar; silindir/plaka kendi ağırlığı hesaba katılmadı.",
        f"Yük ağırlık merkezi yatayda ortada, yüksekliği üst kardan düzleminden {req.cog_height_mm:g} mm yukarıda (kardan düzlemi = üst plaka üst yüzeyi varsayıldı).",
        "Dönmeler üst kardan düzlemi merkezi etrafında, Euler sırası ZYX (roll-pitch-yaw, taban eksenleri).",
        "Çalışma uzayı en kötü noktaları: home + eksen uçları" + (" + tüm eksen köşe kombinasyonları (eşzamanlı)." if req.simultaneous else " (eksenler ayrı ayrı)."),
        "Hız en kötü yön: doğrusal hız x,y,z ve köşegen yönlerde, açısal hız roll/pitch/yaw eksenlerinde ayrı ayrı denendi; ikisi birleştirilmedi.",
        f"Strok her uçta {req.end_margin_mm:g} mm mekanik emniyet payı; strok payı eşiği %{req.stroke_margin_pct:g}. Bacak home'da orta strokta (kapalı boy + strok/2); kapalı boy koşulu böylece sağlanır.",
        f"Verim (vidalı mil + dişli) {ETA:g} varsayıldı; motor torku = F x hatve / (2 pi x verim x dişli oranı).",
        "Dişli/kayış oranı katalogda yoksa 1 (inline) alındı; paralel (.G/.P) varyant eklenmedi. Silindir azami kuvveti dişli oranından bağımsız.",
        "Silindir hız sınırı = hatve x 3500 d/d (datasheet azami mil devri; model sayfasına göre farklıysa o değer); sitedeki hız sütunu 3000 d/d'ya göre olduğu için yalnız bilgi amaçlıdır. 'Hız' payı bu sınıra göre, 'motor devri' payı yalnız motorun max_speed_rpm değerine göre hesaplandı.",
        "Gereken motor devri anma devrini aşarsa kullanılabilir sürekli ve tepe tork, anma devri / gereken devir oranıyla azaltıldı (sabit güç bölgesi varsayımı).",
        "Uygun olmayan adaylar: en az payı kaçıran, plaka tercihine göre makul geometri ve gereken ilk standart strok gösterilir; notta yalnız tutmayan paylar yazılır.",
        "Katalog kuvveti: iki kaynak (elektriklisilindir.com ve imotion.com.tr) farklıysa küçüğü alındı.",
        "Sürekli tork: home pozdaki (poz sıfır) statik bacak kuvveti x emniyet katsayısı; tepe tork: çalışma uzayının en kötü pozundaki toplam kuvvet (ivme girilmişse dahil) x emniyet katsayısı. Atalet oranı kontrolü yapılmadı.",
        "Alt ve üst kardan çiftleri arası açılar referans geometrideki gibi sabit; yalnız yarıçaplar tarandı "
        f"(referansın 0.6x-2.0x aralığı, kaba adım {step_b:.0f}/{step_t:.0f} mm, ardından 5 mm ince adım).",
        f"Uygunluk: kuvvet, hız, kardan açısı, motor devri ve motor torku payları en az %{req.min_margin_pct:g}; strok payı en az %{req.stroke_margin_pct:g}.",
        "Plaka yarıçapı kısıtı: R_top <= R_base (referans oranı 0.70).",
        "Sıralama: uygun olanlar önce; daha önce kullanılmış modeller (used_before), model boyutu, strok, motor gücü (en küçük), aynı güçte hız payı büyük hatve, plaka tercihi; her silindir modelinin en iyisi önce gösterilir, aynı model+hatve+dişli için tek geometri.",
    ]
    if req.base_max_diameter_mm or req.top_max_diameter_mm:
        assumptions.append("Plaka çapı sınırları yarıçap taramasına sert kısıt olarak uygulandı.")
    unverified = list(dict.fromkeys(unverified))
    cands = [_candidate(r, i + 1, req, P) for i, r in enumerate(chosen)]
    res = {"ok": ok, "candidates": cands, "assumptions": assumptions, "unverified_data": unverified,
           "search": {"evaluated_geometries": int(n_eval), "poses": int(len(poses)),
                      "elapsed_s": round(time.perf_counter() - t0, 3)}}
    if not ok:
        res["message"] = "Hiçbir aday tüm kısıtları sağlamıyor; en yakın adaylar ve gerekenler notlarda."
    return res


def _fail(req, assumptions, unverified, msg):
    return {"ok": False, "candidates": [], "assumptions": assumptions, "unverified_data": unverified,
            "message": msg}


def evaluate_config(req, actuator_row, lead_mm, stroke_mm, servo_row, geometry, gear_ratio=1,
                    catalog=None, servos=None, ref=None):
    """Tek konfigürasyonu değerlendirir; aday şemasında döner.

    actuator_row: katalogdaki model satırı; geometry: {"R_base_mm", "R_top_mm"[, "home_leg_length_mm"]};
    servo_row None ise en küçük yeten motor seçilir.
    """
    cat = catalog if catalog is not None else load_actuators()
    ref = ref if ref is not None else load_reference_geometry()
    srv = servos if servos is not None else (None if servo_row is not None else load_servos())
    if isinstance(req, dict):
        req = Requirements.from_dict(req)
    P = _prepare(req, cat, srv, ref, need_models=False)
    g = _norm_model(actuator_row, 0, P["unverified"])
    if g is None:
        raise ValueError("silindir satırı kullanılamıyor")
    lead = next((l for l in g["leads"] if abs(l["lead_mm"] - lead_mm) < 1e-9), None)
    if lead is None:
        raise ValueError(f"{g['name']} için {lead_mm} mm hatve satırı yok")
    if servo_row is not None:
        g["sv"] = _sv_arrays([servo_row])
    elif P["sv"] is not None:
        g["sv"] = _model_sv(g, P["motors"], P["sv"])
        if g["sv"] is None:
            raise ValueError("flanşa uyan servo yok")
    else:
        raise ValueError("servo yok")
    Lh_val = geometry.get("home_leg_length_mm") or _lhome(lead, g["per_stroke"], stroke_mm, P)
    Rb, Rt = np.array([geometry["R_base_mm"]], float), np.array([geometry["R_top_mm"]], float)
    ev = _eval_geoms(Rb, Rt, np.array([Lh_val], float), P["ctx"])
    rec = _run_combo(g, lead, gear_ratio, stroke_mm, Rb, Rt, ev, Lh_val, P)
    return _candidate(rec, 1, req, P)


AXES = ["x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"]


def reference_capacity(catalog=None, ref=None, cog_height_mm=100.0, lead_mm=10, stroke_mm=200, end_margin_mm=None):
    """Referans platform (IMTL40, 12x10, strok 200, gerçek geometri) kapasitesi.

    Her eksen tek başına: bacak boyu (strok payı dahil), kardan açısı ve tekillik sınırında
    ulaşılabilir max +/-; ayrıca bu sınır pozunda bacak hız ve kuvvet katsayıları.
    Home = bacaklar orta strokta (motor kuralıyla aynı).
    """
    cat = catalog if catalog is not None else load_actuators()
    ref = ref if ref is not None else load_reference_geometry()
    req = Requirements(payload_kg=1.0, cog_height_mm=cog_height_mm)
    em = req.end_margin_mm if end_margin_mm is None else end_margin_mm
    P = _prepare(req, cat, None, ref, need_models=False)
    row = next(m for m in cat["models"] if m["model"] == "IMTL40")
    g = _norm_model(row, 0, [])
    lead = next(l for l in g["leads"] if abs(l["lead_mm"] - lead_mm) < 1e-9)
    par = P["par"]
    Lh = _lhome(lead, g["per_stroke"], stroke_mm, P)
    Lc = Lh - stroke_mm / 2.0
    base, top = make_joints(par["R_base"], par["R_top"], par["base_pair_sep_deg"], par["top_pair_sep_deg"],
                            par["top_rot_offset_deg"], par["base_phase_deg"])
    d2 = float(np.mean(np.sum((top[:, :2] - base[:, :2]) ** 2, axis=-1)))
    H = float(np.sqrt(Lh ** 2 - d2))
    lim = P["joint_limit"]

    def feasible(pr):
        R = rot_zyx(pr[:, 3], pr[:, 4], pr[:, 5])
        t = np.zeros((1, len(pr), 3))
        t[..., :2] = pr[None, :, :2]
        t[..., 2] = H + pr[None, :, 2]
        l, u, _, J = leg_kinematics(base[None], top[None], R, t)
        ok = ((l >= Lc + em) & (l <= Lc + stroke_mm - em)).all(axis=(0, 2))
        ang = np.maximum(np.degrees(np.arccos(np.clip(u[..., 2], -1, 1))),
                         np.degrees(np.arccos(np.clip(np.einsum("gpkj,pj->gpk", u, R[:, :, 2]), -1, 1))))
        ok &= (ang <= lim).all(axis=(0, 2))
        Js = J.copy()
        Js[..., 3:] /= par["R_top"]
        had = np.linalg.norm(Js, axis=-1).prod(-1)
        ok &= (np.abs(np.linalg.det(Js)) / np.maximum(had, 1e-12) >= SING_TOL)[0]
        return ok

    out = {"config": {"model": "IMTL40", "lead_mm": lead_mm, "stroke_mm": stroke_mm,
                      "R_base_mm": round(par["R_base"], 2), "R_top_mm": round(par["R_top"], 2),
                      "home_height_mm": round(H, 2), "home_leg_length_mm": round(Lh, 2),
                      "closed_leg_length_mm": round(Lc, 2), "joint_angle_limit_deg": lim},
           "home_mode": "orta strok",
           "step_home_height_mm": ref.get("home_height_mm"),
           "step_home_note": "STEP montajında silindir kapalı konumda modellenmiş (bacak 471 mm = kapalı kardan-kardan boy): "
                             "STEP home yüksekliği bu değerdir; platform bu konumdan yalnız yukarı hareket edebilir. "
                             "Burada home bacaklar orta strokta alındı.",
           "axes": {}, "coefficients_note": (
               "Katsayılar sınır pozunda: leg_speed_per_lin = bacak hızı / platform doğrusal hızı (boyutsuz, en kötü yön); "
               "leg_speed_per_ang_mm_per_rad = bacak hızı (mm/s) / platform açısal hızı (rad/s); "
               "leg_force_per_kg_N = statik en kötü bacak kuvveti / payload (N/kg, CoG yüksekliği "
               f"{cog_height_mm:g} mm), emniyet katsayısız."),
           "notes": ["Home bacaklar orta strokta alındı (kapalı boy + strok/2); referans STEP'te silindir kapalı konumda modellenmiş.",
                     "Sınırlar yalnız kinematik: strok payı (her uçta " + f"{em:g}" + " mm), kardan açısı ve tekillik; kuvvet/hız/motor ayrıca denetlenir."]}
    for k, name in enumerate(AXES):
        step, smax = (0.25, 700.0) if k < 3 else (0.05, 80.0)
        s = np.arange(0.0, smax + 1e-9, step)
        res = {}
        for sign, key in ((1.0, "plus"), (-1.0, "minus")):
            pr = np.zeros((len(s), 6))
            pr[:, k] = sign * s
            bad = np.where(~feasible(pr))[0]
            if len(bad) == 0:
                res[key] = float(s[-1])
            else:
                res[key] = float(s[max(bad[0] - 1, 0)])
        mx = min(res["plus"], res["minus"])
        pose = np.zeros(6)
        pose[k] = mx
        ev = _eval_geoms(np.array([par["R_base"]]), np.array([par["R_top"]]), np.array([Lh]),
                         _build_context(req, np.atleast_2d(pose), par))
        out["axes"][name] = {"plus": round(res["plus"], 2), "minus": round(res["minus"], 2), "max": round(mx, 2),
                             "capped": bool(mx >= s[-1] - 1e-9),
                             "leg_speed_per_lin": round(float(ev["rlin"][0]), 4),
                             "leg_speed_per_ang_mm_per_rad": round(float(ev["rang"][0]), 2),
                             "leg_force_per_kg_N": round(float(ev["fstat"][0, 0]), 3)}
    return out


# ------------------------------------------------------------------ çıktı
def _why(rec, req, joint_limit):
    """Yalnız tutmayan paylar için Türkçe not ve gereken-ne."""
    M, lead, thr = rec["M"], rec["lead"], _thr(req)
    if M.min() <= -1e8:
        return ["Bu yarıçaplarda geometri geçersiz veya tekil (bacak boyu yetmiyor / jacobian tekil)."]
    det = [
        f"emniyetli bacak kuvveti {rec['Fp']:.0f} N, silindir sınırı {lead['max_force_N']:.0f} N; daha büyük silindir veya daha küçük hatve gerekir",
        f"bacak hızı {rec['vleg']:.0f} mm/s, silindir sınırı {lead['speed_limit']:.0f} mm/s; daha büyük hatve gerekir",
        f"gereken strok en az {rec['req_stroke']:.0f} mm, strok {rec['stroke']:g} mm; daha uzun strok gerekir",
        f"kardan açısı {rec['ang']:.1f} derece, sınır {joint_limit:g} derece; sabit açılarla sağlanamadı, açı değişikliği gerekebilir",
        f"motor {rec['motor_rpm']:.0f} dev/dk; daha büyük hatve veya daha düşük dişli oranı gerekir",
        f"gereken sürekli {rec['Tc']:.2f} Nm, tepe {rec['Tp']:.2f} Nm; daha büyük motor veya daha küçük hatve gerekir",
    ]
    names = ["Kuvvet", "Hız", "Strok", "Kardan açısı", "Motor devri", "Motor torku"]
    return [f"{n} payı %{m:.1f} < %{t:g} ({d})." for n, m, d, t in zip(names, M, det, thr) if m < t]


def _candidate(rec, rank, req, P):
    par, pm, joint_limit = P["par"], P["pm"], P["joint_limit"]
    g, lead = rec["model"], rec["lead"]
    mot = g["sv"]["motors"][rec["servo"]]
    base, top = make_joints(rec["Rb"], rec["Rt"], par["base_pair_sep_deg"], par["top_pair_sep_deg"],
                            par["top_rot_offset_deg"], par["base_phase_deg"])
    top = top.copy()
    top[:, 2] = rec["H"]
    notes = ["Tüm kısıtlar sağlanıyor." if rec["feasible"] else "Uygun değil."]
    notes += _why(rec, req, joint_limit)
    if lead.get("force_dual"):
        notes.append("Kuvvet: iki kaynak farklı, küçüğü alındı.")
    if P["cat_limit"]:
        notes.append(f"Kardan açısı: katalog sınırı {P['cat_limit']:g} derece, tasarım sınırı {joint_limit:g} derece uygulandı.")
    rr = g["sv"]["rated_rpm"][rec["servo"]]
    if np.isfinite(rr) and rec["motor_rpm"] > rr:
        notes.append(f"Motor devri {rec['motor_rpm']:.0f} dev/dk, anma devrinin ({rr:.0f}) üstünde, kısa süreli; bu devirde tork anma devri/gereken devir oranıyla düşürülerek kontrol edildi; sürücü eğrisinden teyit edin.")
    notes.append("Fren: bacaklar yük taşıdığı için frenli motor önerilir; bu motorda fren seçeneği "
                 + ("var." if mot.get("brake_option") is not False else "YOK (başka motor gerekebilir)."))
    if mot.get("voltage_V") == 380:
        notes.append("Motor 380 V sınıfı: 3 faz 380 V besleme gerekir.")
    lf = FACTORS[int(np.argmin(rec["M"] - _thr(req)))]
    if g.get("flanges") is None:
        notes.append("Flanş uyumu teyit gerekli (katalogda motor flanş listesi yok).")
    else:
        notes.append(f"Motor flanşı katalogdaki adaptörle uyumlu ({', '.join(f'{f:g}' for f in g['flanges'])} mm).")
        if mot.get("flange_mm") not in g["flanges"]:
            notes.append(f"UYARI: motor flanşı {mot.get('flange_mm')} mm, silindir adaptörü {g['flanges']} mm; uyumsuz.")
    if len(rec["per_case"]) > 1:
        notes.append(f"Sınırlayan yük-hız rejimi: {rec['lc'] + 1}. rejim ({rec['per_case'][rec['lc']]['payload_kg']:g} kg).")
    q = lambda v: round(float(v), 2)
    return {
        "rank": rank, "feasible": rec["feasible"], "limiting_factor": lf,
        "actuator": {"model": g["name"], "lead_mm": lead["lead_mm"], "stroke_mm": rec["stroke"],
                     "gear_ratio": rec["gear"], "retracted_length_mm": q(rec.get("retr", 0.0))},
        "servo": {"motor_code": mot.get("motor_code"), "rated_power_W": mot["rated_power_W"],
                  "drive_model": mot.get("drive_model"), "voltage_V": mot.get("voltage_V"),
                  "brake_option": mot.get("brake_option")},
        "geometry": {"R_base_mm": q(rec["Rb"]), "R_top_mm": q(rec["Rt"]), "home_height_mm": q(rec["H"]),
                     "base_joints_mm": np.round(base, 2).tolist(), "top_joints_mm": np.round(top, 2).tolist(),
                     "base_plate_diameter_mm": q(2 * (rec["Rb"] + pm[0])),
                     "top_plate_diameter_mm": q(2 * (rec["Rt"] + pm[1]))},
        "margins": {k: q(max(v, -999.0)) for k, v in zip(MARGIN_KEYS, rec["M"])},
        "worst_case": {"max_leg_force_N": q(rec["ftot"]), "max_leg_speed_mm_s": q(max(c["max_leg_speed_mm_s"] for c in rec["per_case"])),
                       "leg_length_range_mm": [q(rec["lmin"]), q(rec["lmax"])],
                       "max_joint_angle_deg": q(rec["ang"]),
                       "required_stroke_mm": q(rec["req_stroke"]),
                       "motor_rpm": q(max(c["motor_rpm"] for c in rec["per_case"])),
                       "continuous_torque_Nm": q(max(c["continuous_torque_Nm"] for c in rec["per_case"])),
                       "peak_torque_Nm": q(max(c["peak_torque_Nm"] for c in rec["per_case"])),
                       "limiting_case": rec["lc"] + 1,
                       "per_case": [{k: (q(v) if isinstance(v, float) else v) for k, v in c.items()}
                                    for c in rec["per_case"]]},
        "notes": notes,
    }
