"""Hesap motoru köprüsü: engine.sizing varsa onu, yoksa şemaya uyan örnek sonucu döndürür."""
import dataclasses
import math

try:
    from engine.sizing import Requirements, size_platform
    ENGINE_OK = True
except Exception:  # engine yok veya içe aktarma hatası -> mock
    Requirements = size_platform = None
    ENGINE_OK = False

DEFAULTS = dict(
    payload_kg=0, cog_height_mm=100, x_mm=0, y_mm=0, z_mm=0, roll_deg=0, pitch_deg=0, yaw_deg=0,
    simultaneous=False, lin_speed_mm_s=0, ang_speed_deg_s=0, accel_m_s2=None, safety_factor=1.5,
    base_max_diameter_mm=None, top_max_diameter_mm=None,
    supply_voltage_V=None, min_margin_pct=10.0, plate_preference="reference",
    stroke_margin_pct=0.0, end_margin_mm=2.0, load_cases=None,
)


def _cases(v):
    if not v:
        return None
    num = lambda x: None if x is None or x == "" else float(x)
    return [{"payload_kg": float(c["payload_kg"]), "lin_speed_mm_s": num(c.get("lin_speed_mm_s")) or 0.0,
             "ang_speed_deg_s": num(c.get("ang_speed_deg_s")) or 0.0, "accel_m_s2": num(c.get("accel_m_s2"))}
            for c in v]


def normalize(data):
    """Boş/eksik alanları varsayılanla doldurur, sayıları float'a çevirir."""
    out = dict(DEFAULTS)
    for k, v in (data or {}).items():
        if k not in DEFAULTS:
            continue
        if k == "load_cases":
            out[k] = _cases(v)
        elif k == "simultaneous":
            out[k] = bool(v)
        elif v is None or v == "":
            out[k] = DEFAULTS[k]
        elif k == "plate_preference":
            out[k] = v if v in ("reference", "smallest") else "reference"
        elif k == "supply_voltage_V":
            out[k] = int(float(v))
        else:
            out[k] = float(v)
    return out


def _ring(r, sep_deg, offset_deg, z):
    pts = []
    for k in range(3):
        for s in (-1, 1):
            a = math.radians(120 * k + offset_deg + s * sep_deg / 2)
            pts.append([round(r * math.cos(a), 3), round(r * math.sin(a), 3), z])
    return pts


def _mock_candidate(rank, model, lead, stroke, servo_w, rb, rt, feasible, limiting, notes):
    retracted = 250 + stroke
    base = _ring(rb, 20, 0, 0.0)
    top = _ring(rt, 20, 60, 0.0)
    # üst halka alt halkaya göre 60° dönük; bacak i = base[i] -> top[i] (komşu çift düzeni)
    top = [top[(i + 1) % 6] for i in range(6)]
    # home'da bacak boyu = kapalı boy + yarım strok olacak şekilde yükseklik
    d = math.hypot(base[0][0] - top[0][0], base[0][1] - top[0][1])
    h = round(math.sqrt((retracted + stroke / 2) ** 2 - d * d), 1)
    for p in top:
        p[2] = h
    pct = (lambda v: v) if feasible else (lambda v: -abs(v) / 3)
    return {
        "rank": rank, "feasible": feasible, "limiting_factor": limiting,
        "actuator": {"model": model, "lead_mm": lead, "stroke_mm": stroke, "gear_ratio": 1,
                     "retracted_length_mm": retracted},
        "servo": {"motor_code": f"MS1H-ÖRNEK-{servo_w}", "rated_power_W": servo_w,
                  "drive_model": "SV660P-ÖRNEK"},
        "geometry": {"R_base_mm": rb, "R_top_mm": rt, "home_height_mm": h,
                     "base_joints_mm": base, "top_joints_mm": top,
                     "base_plate_diameter_mm": round(2 * rb + 40, 1),
                     "top_plate_diameter_mm": round(2 * rt + 40, 1)},
        "margins": {"force_pct": pct(55), "speed_pct": pct(30), "stroke_pct": pct(42),
                    "joint_angle_pct": pct(25), "motor_rpm_pct": pct(38), "motor_torque_pct": pct(60)},
        "worst_case": {"max_leg_force_N": 1200, "max_leg_speed_mm_s": 400,
                       "leg_length_range_mm": [retracted + 30, retracted + stroke - 20],
                       "max_joint_angle_deg": 32},
        "notes": notes,
    }


def mock_result(req):
    return {
        "ok": True, "mock": True,
        "candidates": [
            _mock_candidate(1, "IMT40", 10, 200, 750, 150, 100, True, "hız", ["Örnek veri: gerçek hesap değildir."]),
            _mock_candidate(2, "IMT40", 5, 300, 400, 180, 120, True, "strok", ["Örnek veri: gerçek hesap değildir."]),
            _mock_candidate(3, "IMT32", 5, 100, 200, 120, 80, False, "kuvvet",
                            ["Uygunsuz (örnek): bacak kuvveti silindir sınırını aşıyor."]),
        ],
        "assumptions": ["Hesap motoru bağlı değil; sonuçlar örnek veridir."],
        "unverified_data": ["Tüm katalog değerleri (örnek)"],
        "request_echo": req,
    }


def size(data):
    req = normalize(data)
    if not ENGINE_OK:
        return mock_result(req)
    names = {f.name for f in dataclasses.fields(Requirements)}
    result = size_platform(Requirements(**{k: v for k, v in req.items() if k in names}))
    result["mock"] = False
    return result


_ref_cache = None


def reference_capacity():
    """Referans platform kapasitesi; engine yoksa None. Sonuç sabit olduğundan önbelleğe alınır."""
    global _ref_cache
    if not ENGINE_OK:
        return None
    if _ref_cache is None:
        from engine.sizing import reference_capacity as rc
        _ref_cache = rc()
    return _ref_cache
