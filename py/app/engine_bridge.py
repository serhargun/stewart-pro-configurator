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


def plate_details(geo, ref_plates=None):
    """Sipariş için plaka ölçüleri; referans plakalardan (kalınlık, alt plakanın düz kesik oranı)."""
    p = ref_plates if ref_plates is not None else _ref_plates()
    bd, td = geo.get("base_plate_diameter_mm"), geo.get("top_plate_diameter_mm")
    if not bd or not td:
        return None
    flat_ratio = p.get("base_flat_half_width_mm", 310.0) / (p.get("base_outer_diameter_mm", 650.0) / 2)
    bd_i, td_i = math.ceil(bd - 1e-6), math.ceil(td - 1e-6)   # yukarı yuvarla: kardan payı azalmasın
    return {
        "base": {"diameter_mm": bd_i, "thickness_mm": p.get("base_thickness_mm", 10.0),
                 "flat_half_width_mm": round(bd_i / 2 * flat_ratio), "hole_circle_diameter_mm": round(2 * geo["R_base_mm"], 1)},
        "top": {"diameter_mm": td_i, "thickness_mm": p.get("top_thickness_mm", 10.0),
                "hole_circle_diameter_mm": round(2 * geo["R_top_mm"], 1)},
        "note": "Kalınlık ve alt plakanın iki yandaki düz kesiği referans montajdan (Ø650, ±310) orantılı alındı; "
                "üst plaka referansta köşeleri yuvarlatılmış çokgen, burada dış çap verildi.",
    }


_plates_cache = None


def _ref_plates():
    global _plates_cache
    if _plates_cache is None:
        try:
            from engine.catalog import load_reference_geometry
            _plates_cache = dict(load_reference_geometry().get("plates") or {})
        except Exception:
            _plates_cache = {}
    return _plates_cache


ENC_PER_REV = 8388608      # 23 bit seri enkoder (sahadaki MS1H4'te H00-05 ile teyitli)
PULSE_PER_REV = 6600       # sahadaki düzen: myRIO pulse modu, H05-17 = 1650 (x4 = 6600)


def drive_settings(cand, follow_err_mm=10.0):
    """SV660P pulse modu için hatveden türetilen ayar önerisi (sahada doğrulanan düzen)."""
    a = (cand or {}).get("actuator") or {}
    lead, gear = a.get("lead_mm"), a.get("gear_ratio") or 1
    if not lead:
        return None
    mm_per_rev = lead / gear
    ppmm = PULSE_PER_REV / mm_per_rev
    vmax = ((cand.get("worst_case") or {}).get("max_leg_speed_mm_s")) or 0.0
    fmax = vmax * ppmm
    return {
        "pulse_per_rev": PULSE_PER_REV, "electronic_gear": f"{ENC_PER_REV} / {PULSE_PER_REV}",
        "pulse_per_mm": round(ppmm, 3), "resolution_um": round(1000 / ppmm, 2),
        "max_pulse_kHz": round(fmax / 1000, 1), "high_speed_input": fmax > 200e3,
        "H05_17": PULSE_PER_REV // 4,
        "H0A_10": round(follow_err_mm / mm_per_rev * ENC_PER_REV), "follow_err_mm": follow_err_mm,
        "note": "Değerler sahada çalışan bir kurulumda (IMT63, 16 mm hatve: 412,5 pulse/mm, H05-17 = 1650, H0A-10 = 5242880) "
                "doğrulanan düzenden hatveye göre ölçeklendi. 23 bit enkoder MS1H4 için teyitli; diğer motorlarda H00-05'ten "
                "teyit edin. 200 kHz sınırı ve yüksek hızlı pulse girişi (H05-01 = 1) sürücü kılavuzundan teyit edilmeli.",
    }


_prices_cache = False


def _prices():
    """data/prices_internal.json (yalnız yerel sürüm); yoksa None."""
    global _prices_cache
    if _prices_cache is False:
        import json
        from pathlib import Path
        p = Path(__file__).resolve().parent.parent / "data" / "prices_internal.json"
        try:
            _prices_cache = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            _prices_cache = None
    return _prices_cache


def cost_estimate(cand, prices=None, n=6):
    """Teklif fiyatlarıyla kaba malzeme maliyeti; para birimleri çevrilmeden ayrı toplanır."""
    P = prices if prices is not None else _prices()
    if not P or not cand:
        return None
    a, s = cand.get("actuator", {}), cand.get("servo", {})
    # hesap notu bacaklar yük taşıdığı için frenli motor öneriyor; fren seçeneği varsa frenli fiyat (IMT63 alımı da frenli)
    brake = bool(s.get("brake_option"))
    lines, missing = [], []

    def add(name, qty, row, note=""):
        lines.append({"name": name, "qty": qty, "unit": row["price"], "currency": row["currency"],
                      "total": round(qty * row["price"], 2), "source": f"{row.get('supplier', '')} · {row.get('source', '')}".strip(" ·"),
                      "note": note or row.get("note", "")})

    act = [r for r in P.get("actuators", []) if r["model"] == a.get("model") and r["stroke_mm"] == a.get("stroke_mm")]
    act.sort(key=lambda r: r.get("lead_mm") != a.get("lead_mm"))
    if act:
        r = act[0]
        add(f"Elektrikli silindir {r['code']}", n, r,
            "" if r.get("lead_mm") == a.get("lead_mm") else f"teklif hatvesi {r.get('lead_mm') or 'belirtilmemiş'}, seçilen {a.get('lead_mm'):g} mm")
    else:
        missing.append(f"Silindir {a.get('model')} strok {a.get('stroke_mm'):g}")

    code = s.get("motor_code") or ""
    mot = [r for r in P.get("motors", []) if r["motor_code"] == code]
    if not mot and code[:5] == "MS1H1":   # aynı güç/flanşta MS1H4 teklifi muadil olarak gösterilir
        mot = [r for r in P.get("motors", []) if r["motor_code"] == "MS1H4" + code[5:]]
        note = f"{code} teklifi yok; aynı güç ve flanştaki {mot[0]['motor_code']} fiyatı" if mot else ""
    else:
        note = ""
    mot.sort(key=lambda r: r.get("brake") != brake)
    if mot:
        alt = [r for r in mot[1:] if r.get("brake") != mot[0].get("brake")]
        if alt:
            note = "; ".join(x for x in (note, f"{'frenli' if alt[0].get('brake') else 'frensiz'}: {alt[0]['price']:g} {alt[0]['currency']}") if x)
        if brake and not mot[0].get("brake"):
            note = "; ".join(x for x in (note, "frenli fiyat yok, frensiz fiyat yazıldı") if x)
            missing.append(f"Frenli motor fiyatı ({mot[0]['motor_code']})")
        add(f"Servo motor {mot[0]['motor_code']}{' frenli' if mot[0].get('brake') else ''}", n, mot[0], note)
    else:
        missing.append(f"Servo motor {code}")

    dm = (s.get("drive_model") or "").replace("-INT", "")
    drv = [r for r in P.get("drives", []) if r["drive_model"] == dm]
    if drv:
        add(f"Servo sürücü {dm}", n, drv[0])
    else:
        missing.append(f"Servo sürücü {dm}")
    for r in P.get("cables", []):
        if r.get("optional") or (r.get("brake") is not None and r["brake"] != brake):
            continue
        add(r["name"], n, r)

    sets = []
    for fs in P.get("fixed_sets", []):
        tot = round(sum(q * u for _, q, u in fs["lines"]), 2)
        sets.append({"name": fs["name"], "currency": fs["currency"], "total": tot, "source": fs["source"],
                     "note": fs.get("note", "")})
    totals = {}
    for x in lines + sets:
        totals[x["currency"]] = round(totals.get(x["currency"], 0) + x["total"], 2)
    return {"lines": lines, "sets": sets, "missing": missing, "totals": totals,
            "note": "Tedarikçi tekliflerinden (KDV hariç); kur çevrimi yapılmadı. Mekanik/elektronik setler IMTL40 boyundaki "
                    "sistemden, ölçeklenmedi. Teklif geçerlilikleri kısa: sipariş öncesi güncel fiyat alın."}


def size(data):
    req = normalize(data)
    if not ENGINE_OK:
        return mock_result(req)
    names = {f.name for f in dataclasses.fields(Requirements)}
    result = size_platform(Requirements(**{k: v for k, v in req.items() if k in names}))
    result["mock"] = False
    for c in result.get("candidates", []):
        c["plates"] = plate_details(c.get("geometry", {}))
        c["drive_settings"] = drive_settings(c)
        cost = cost_estimate(c)
        if cost:
            c["cost"] = cost
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
