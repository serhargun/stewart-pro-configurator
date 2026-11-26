"""JSON giriş/çıkışlı küçük arayüz: Flask sunucusu ve tarayıcıda Pyodide aynı kodu kullanır.

Pyodide'da dizin düzeni /app/{app,engine,data}; sys.path'e /app eklenir.
"""
import json
from pathlib import Path

from . import engine_bridge

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def load_catalog(data_dir=None):
    out, warnings = {"actuators": [], "servos": [], "actuators_meta": {}, "servos_meta": {}}, []
    d = Path(data_dir) if data_dir else DATA_DIR
    for key, fn, lst in (("actuators", "actuators.json", "models"), ("servos", "servos.json", "motors")):
        try:
            j = json.loads((d / fn).read_text(encoding="utf-8"))
            out[key] = j.get(lst, [])
            out[key + "_meta"] = {k: v for k, v in j.items() if k != lst}
        except (OSError, ValueError):
            warnings.append(f"{fn} bulunamadı veya okunamadı")
    out["warnings"] = warnings
    prices = engine_bridge._prices()   # yalnız yerel sürümde var
    if prices:
        out["prices"] = prices
    return out


def make_snapshot(cand, catalog, taken_at):
    """Seçilen silindir ve motor satırlarının o anki kopyası."""
    snap = {"taken_at": taken_at, "actuator": None, "servo": None}
    if not cand:
        return snap
    a, s = cand.get("actuator") or {}, cand.get("servo") or {}
    snap["actuator"] = next((m for m in catalog["actuators"] if m.get("model") == a.get("model")), None)
    snap["servo"] = next((m for m in catalog["servos"] if m.get("motor_code") == s.get("motor_code")), None)
    return snap


def suggest_bom(cand):
    if not cand:
        return []
    a, s, geo = cand.get("actuator", {}), cand.get("servo", {}), cand.get("geometry", {})
    sup = "elektriklisilindir.com"
    nt = "tedarikçi sipariş kodu teyit gerekli"
    kn = "büyük silindirlerde kardan boyutu teyit gerekli"
    rows = [
        (f"Elektrikli silindir {a.get('model')}, strok {a.get('stroke_mm'):g}, hatve {a.get('lead_mm'):g}", "", 6, sup, nt),
        ("Servo motor", s.get("motor_code") or "", 6, "Inovance", ""),
        ("Servo sürücü SV660P", s.get("drive_model") or "", 6, "Inovance", ""),
        ("Kardan mafsal Mario Ferri 1G (alt)", "", 6, "Mario Ferri", kn),
        ("Kardan mafsal Mario Ferri 1G (üst)", "", 6, "Mario Ferri", kn),
    ]
    pl = cand.get("plates")
    if pl:
        b, t = pl["base"], pl["top"]
        rows += [("Alt plaka", f"Ø{b['diameter_mm']} × {b['thickness_mm']:g} mm, düz kesik ±{b['flat_half_width_mm']}", 1, "",
                  f"kardan delik dairesi Ø{b['hole_circle_diameter_mm']:g}"),
                 ("Üst plaka", f"Ø{t['diameter_mm']} × {t['thickness_mm']:g} mm", 1, "",
                  f"kardan delik dairesi Ø{t['hole_circle_diameter_mm']:g}")]
    else:
        rows += [("Alt plaka", f"Ø{geo.get('base_plate_diameter_mm', '')}", 1, "", ""),
                 ("Üst plaka", f"Ø{geo.get('top_plate_diameter_mm', '')}", 1, "", "")]
    return [dict(part_name=n, part_code=c, qty=q, supplier=sp, note=note) for n, c, q, sp, note in rows]


def default_plate(cand):
    geo, pl = (cand or {}).get("geometry", {}), (cand or {}).get("plates")
    if pl:
        return {"base_diameter_mm": pl["base"]["diameter_mm"], "top_diameter_mm": pl["top"]["diameter_mm"],
                "thickness_mm": pl["base"]["thickness_mm"], "material": "", "holes": []}
    return {"base_diameter_mm": geo.get("base_plate_diameter_mm"),
            "top_diameter_mm": geo.get("top_plate_diameter_mm"),
            "thickness_mm": None, "material": "", "holes": []}


# ---- JSON metin giriş/çıkışlı sarmalayıcılar (Pyodide worker bunları çağırır) ----
def size(req_json):
    return json.dumps(engine_bridge.size(json.loads(req_json)), ensure_ascii=False)


def reference_capacity(_=None):
    res = engine_bridge.reference_capacity()
    return json.dumps(res, ensure_ascii=False)


def catalog(_=None):
    return json.dumps(load_catalog(), ensure_ascii=False)


def order_parts(cand_json, taken_at):
    """Yeni sipariş için otomatik BOM, snapshot ve plaka önerisi."""
    cand = json.loads(cand_json)
    return json.dumps({"bom": suggest_bom(cand), "snapshot": make_snapshot(cand, load_catalog(), taken_at),
                       "plate": default_plate(cand)}, ensure_ascii=False)
