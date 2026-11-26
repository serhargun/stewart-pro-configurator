"use strict";
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const esc = v => String(v ?? "").replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const TBC = '<span class="tag">teyit gerekli</span>';
const num = v => (v === "" || v == null ? null : Number(v));
const isText = el => el.tagName === "SELECT" && el.name === "plate_preference";

const STATIC = !!(window.APP_CONFIG && window.APP_CONFIG.static);

async function api(path, opt = {}) {
  if (STATIC) return staticApi(path, opt);   // api-static.js: Pyodide + IndexedDB
  const init = { ...opt };
  if (opt.json !== undefined) {
    init.method = init.method || "POST";
    init.headers = { "Content-Type": "application/json" };
    init.body = JSON.stringify(opt.json);
    delete init.json;
  }
  const r = await fetch("/api" + path, init);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}

/* ---------- sekmeler ---------- */
$$("nav button").forEach(b => b.onclick = () => {
  $$("nav button").forEach(x => x.classList.toggle("active", x === b));
  ["cfg", "orders", "catalog"].forEach(t => $("#tab-" + t).hidden = t !== b.dataset.tab);
  if (b.dataset.tab === "orders") loadOrders();
  if (b.dataset.tab === "catalog") loadCatalog();
  if (b.dataset.tab === "cfg") viewer.resize();
});

let STATUSES = [];
api("/status").then(s => {
  STATUSES = s.statuses;
  $("#ostatus").innerHTML += STATUSES.map(x => `<option>${x}</option>`).join("");
});

/* ---------- 3D / 2D görünüm ---------- */
// Euler sırası ZYX: R = Rz(yaw)·Ry(pitch)·Rx(roll) — engine ile aynı olmalı.
function rotZYX(rollD, pitchD, yawD) {
  const [a, b, c] = [rollD, pitchD, yawD].map(d => d * Math.PI / 180);
  const [ca, sa, cb, sb, cc, sc] = [Math.cos(a), Math.sin(a), Math.cos(b), Math.sin(b), Math.cos(c), Math.sin(c)];
  return [
    [cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
    [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
    [-sb, cb * sa, cb * ca],
  ];
}
const mulv = (R, v) => R.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);

const viewer = (() => {
  const box = $("#view3d");
  const has3d = typeof THREE !== "undefined" && THREE.OrbitControls;
  let cand = null, pose = [0, 0, 0, 0, 0, 0], ztop = 0, tloc = [], blen = 0;
  let renderer, scene, camera, controls, topGroup, legs = [], base = [], svg;

  function legGeometry() {
    const g = cand.geometry;
    base = g.base_joints_mm;
    ztop = g.top_joints_mm.reduce((s, p) => s + p[2], 0) / 6;
    tloc = g.top_joints_mm.map(p => [p[0], p[1], p[2] - ztop]);
    blen = (cand.actuator.retracted_length_mm || 300) * 0.55;
  }

  // her bacak için üst kardan noktası (taban çerçevesi) ve uzunluk
  function solve() {
    const R = rotZYX(pose[3], pose[4], pose[5]), p = [pose[0], pose[1], ztop + pose[2]];
    return tloc.map((t, i) => {
      const w = mulv(R, t), top = [p[0] + w[0], p[1] + w[1], p[2] + w[2]];
      const d = top.map((v, k) => v - base[i][k]);
      return { top, len: Math.hypot(...d), dir: d };
    });
  }

  function build() {
    legGeometry();
    const g = cand.geometry;
    if (has3d) {
      if (!renderer) {
        renderer = new THREE.WebGLRenderer({ antialias: true });
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0xeef1f6);
        camera = new THREE.PerspectiveCamera(40, 1, 1, 20000);
        camera.up.set(0, 0, 1);
        scene.add(new THREE.AmbientLight(0xffffff, 0.7));
        const dl = new THREE.DirectionalLight(0xffffff, 0.8);
        dl.position.set(500, -800, 1200);
        scene.add(dl);
        box.innerHTML = "";
        box.appendChild(renderer.domElement);
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        (function loop() { requestAnimationFrame(loop); if (box.offsetParent) renderer.render(scene, camera); })();
      }
      scene.children.filter(o => o.userData.dyn).forEach(o => scene.remove(o));
      const mk = (geo, color) => new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color }));
      const disc = (d, color) => {
        const m = mk(new THREE.CylinderGeometry(d / 2, d / 2, 12, 48).rotateX(Math.PI / 2), color);
        m.material.transparent = true; m.material.opacity = 0.75;
        return m;
      };
      // referans alt plaka: iki yanı düz kesik daire (yerel Y ±310, R 325 oranında)
      const R0 = g.base_plate_diameter_mm / 2, ymax = R0 * 310 / 325, shp = new THREE.Shape();
      for (let k = 0; k <= 96; k++) {
        const a = 2 * Math.PI * k / 96, pt = [R0 * Math.cos(a), Math.max(-ymax, Math.min(ymax, R0 * Math.sin(a)))];
        k ? shp.lineTo(...pt) : shp.moveTo(...pt);
      }
      const bp = mk(new THREE.ExtrudeGeometry(shp, { depth: 12, bevelEnabled: false }).translate(0, 0, -12), 0x13315c);
      bp.material.transparent = true; bp.material.opacity = 0.75; bp.userData.dyn = true; scene.add(bp);
      topGroup = new THREE.Group(); topGroup.userData.dyn = true;
      const tp = disc(g.top_plate_diameter_mm, 0x5aa9ff); tp.position.z = 6; topGroup.add(tp);
      scene.add(topGroup);
      base.forEach(b => { const s = mk(new THREE.SphereGeometry(10, 16, 12), 0xc9372c); s.position.set(...b); s.userData.dyn = true; scene.add(s); });
      tloc.forEach(t => { const s = mk(new THREE.SphereGeometry(10, 16, 12), 0xc9372c); s.position.set(...t); topGroup.add(s); });
      legs = base.map(() => {
        const body = mk(new THREE.CylinderGeometry(14, 14, 1, 20), 0x8794a8);
        const rod = mk(new THREE.CylinderGeometry(7, 7, 1, 16), 0xdfe5ee);
        body.userData.dyn = rod.userData.dyn = true;
        scene.add(body, rod);
        return { body, rod };
      });
      const r = g.base_plate_diameter_mm;
      camera.position.set(r * 1.6, -r * 1.8, r * 1.2 + ztop);
      controls.target.set(0, 0, ztop / 2);
      controls.update();
      resize();
    } else {
      box.innerHTML = '<svg viewBox="-400 -400 800 800"></svg>';
      svg = $("svg", box);
    }
    update();
  }

  function place(mesh, a, b) {
    const d = new THREE.Vector3(...b).sub(new THREE.Vector3(...a));
    const len = d.length();
    mesh.scale.set(1, len, 1);
    mesh.position.set(...a.map((v, k) => (v + b[k]) / 2));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  }

  function update() {
    if (!cand) return;
    const s = solve();
    if (has3d && renderer) {
      const R = rotZYX(pose[3], pose[4], pose[5]);
      topGroup.matrix.set(R[0][0], R[0][1], R[0][2], pose[0], R[1][0], R[1][1], R[1][2], pose[1],
        R[2][0], R[2][1], R[2][2], ztop + pose[2], 0, 0, 0, 1);
      topGroup.matrixAutoUpdate = false;
      s.forEach((l, i) => {
        const u = l.dir.map(v => v / l.len), k = Math.min(blen, l.len * 0.5);
        const mid = base[i].map((v, j) => v + u[j] * k);
        place(legs[i].body, base[i], mid);
        place(legs[i].rod, mid, l.top);
      });
    } else if (svg) {
      let h = "";
      [cand.geometry.base_plate_diameter_mm, cand.geometry.top_plate_diameter_mm].forEach((d, i) =>
        h += `<circle r="${d / 2}" fill="none" stroke="${i ? "#5aa9ff" : "#13315c"}" stroke-width="3"/>`);
      s.forEach((l, i) => h += `<line x1="${base[i][0]}" y1="${-base[i][1]}" x2="${l.top[0]}" y2="${-l.top[1]}" stroke="#8794a8" stroke-width="6"/>` +
        `<circle cx="${base[i][0]}" cy="${-base[i][1]}" r="8" fill="#c9372c"/><circle cx="${l.top[0]}" cy="${-l.top[1]}" r="8" fill="#c9372c"/>`);
      svg.innerHTML = h;
    }
    const lens = s.map(l => l.len), a = cand.actuator, lmin = a.retracted_length_mm, lmax = lmin + a.stroke_mm;
    const bad = lmin && lens.some(l => l < lmin || l > lmax);
    $("#legstat").innerHTML = `Bacak boyu: ${Math.min(...lens).toFixed(1)} – ${Math.max(...lens).toFixed(1)} mm` +
      (lmin ? ` (silindir ${lmin}–${lmax} mm)` : "") +
      (bad ? ' <span class="tag" style="background:#fde2df;color:#c9372c;border-color:#e8a9a3">strok dışı</span>' : "");
  }

  function resize() {
    if (!renderer) return;
    const w = box.clientWidth, h = box.clientHeight || 440;
    if (!w) return;
    renderer.setSize(w, h);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  window.addEventListener("resize", resize);

  return { show(c) { cand = c; pose = [0, 0, 0, 0, 0, 0]; build(); }, setPose(i, v) { pose[i] = v; update(); }, resize };
})();

/* ---------- konfigüratör ---------- */
let lastReq = null, lastResult = null, selected = null;
const MARGIN_KEYS = [["force_pct", "K", "Kuvvet"], ["speed_pct", "H", "Hız"], ["stroke_pct", "S", "Strok"],
  ["joint_angle_pct", "A", "Kardan açısı"], ["motor_rpm_pct", "D", "Motor devri"], ["motor_torque_pct", "T", "Motor torku"]];

function reqFromForm() {
  const f = $("#reqform"), r = {};
  $$("input[name], select[name]", f).forEach(i => { r[i.name] = i.type === "checkbox" ? i.checked : isText(i) ? i.value : num(i.value); });
  r.load_cases = $$("tr[data-case]", f).map(tr => {
    const g = k => num($(`[data-k="${k}"]`, tr).value);
    return { payload_kg: g("payload_kg") ?? 0, lin_speed_mm_s: g("lin_speed_mm_s") ?? 0, ang_speed_deg_s: g("ang_speed_deg_s") ?? 0, accel_m_s2: g("accel_m_s2") };
  });
  return r;
}

/* yük–hız rejimleri tablosu */
const newCase = () => ({ payload_kg: 100, lin_speed_mm_s: 200, ang_speed_deg_s: 0, accel_m_s2: null });
let caseRows = [newCase()];
function renderCaseTable() {
  const c = (k, v) => `<td><input data-k="${k}" type="number" step="any" value="${esc(v ?? "")}"></td>`;
  $("#casetable").innerHTML = "<tr><th>#</th><th>Yük (kg)</th><th>Doğrusal hız (mm/s)</th><th>Açısal hız (°/s)</th><th>İvme (m/s²)</th><th></th></tr>" +
    caseRows.map((r, i) => `<tr data-case="${i}"><td>${i + 1}</td>${c("payload_kg", r.payload_kg)}${c("lin_speed_mm_s", r.lin_speed_mm_s)}${c("ang_speed_deg_s", r.ang_speed_deg_s)}${c("accel_m_s2", r.accel_m_s2)}<td>${caseRows.length > 1 ? `<button type="button" class="danger" data-delcase="${i}">Sil</button>` : ""}</td></tr>`).join("");
  $$("[data-delcase]").forEach(b => b.onclick = () => { caseRows = reqFromForm().load_cases; caseRows.splice(+b.dataset.delcase, 1); renderCaseTable(); });
}
$("#addcase").onclick = () => { caseRows = reqFromForm().load_cases; caseRows.push(newCase()); renderCaseTable(); };
renderCaseTable();

// gerçek iki siparişin isterleri (tests/test_engine_real_orders.py ile aynı)
const PRESETS = {
  imt63: { cases: [{ payload_kg: 100, lin_speed_mm_s: 750, ang_speed_deg_s: 165, accel_m_s2: null }, { payload_kg: 200, lin_speed_mm_s: 200, ang_speed_deg_s: 80, accel_m_s2: null }],
    f: { cog_height_mm: 100, x_mm: 150, y_mm: 150, z_mm: 150, roll_deg: 25, pitch_deg: 25, yaw_deg: 40 } },
  imtl40: { cases: [{ payload_kg: 40, lin_speed_mm_s: 250, ang_speed_deg_s: 0, accel_m_s2: null }],
    f: { cog_height_mm: 100, x_mm: 75, y_mm: 75, z_mm: 75, roll_deg: 20, pitch_deg: 20, yaw_deg: 20 } },
};
$$("[data-preset]").forEach(b => b.onclick = () => {
  const p = PRESETS[b.dataset.preset];
  caseRows = p.cases.map(c => ({ ...c }));
  renderCaseTable();
  Object.entries(p.f).forEach(([k, v]) => { $(`#reqform [name="${k}"]`).value = v; });
  $("#reqform [name=simultaneous]").checked = false;
  $("#sizemsg").textContent = "Örnek yüklendi; Hesapla'ya basın." + (b.dataset.preset === "imtl40" ? " (Çalışma uzayı siparişte yoktu, varsayımdır.)" : "");
});

$("#reqform").onsubmit = async e => {
  e.preventDefault();
  $("#sizemsg").textContent = "Hesaplanıyor…";
  $("#sizebtn").disabled = true;
  try {
    lastReq = reqFromForm();
    lastResult = await api("/size", { json: lastReq });
    $("#sizemsg").textContent = "";
    $("#mockbar").hidden = !lastResult.mock;
    renderCandidates();
    // 3D görünüm boyutlanırken düzen kayıyor; kaydırmayı düzen oturduktan sonra yap
    setTimeout(() => ($("#viewcard").hidden ? $("#results") : $("#viewcard")).scrollIntoView({ block: "start" }), 150);
  } catch (x) { $("#sizemsg").textContent = x.message; }
  $("#sizebtn").disabled = false;
};

// statik modda Pyodide hazır olana dek Hesapla pasif
if (STATIC) {
  $("#sizebtn").disabled = true;
  $("#enginebar").hidden = false;
  $("#staticwarn").hidden = false;
  $("#iobar").hidden = false;
  engine.watch(st => {
    if (st.ready) { $("#enginebar").hidden = true; $("#sizebtn").disabled = false; }
    else if (st.error) { $("#enginebar").textContent = "Hesap motoru yüklenemedi: " + st.error; }
  });
}

const pct = v => Number(v).toFixed(1).replace(".", ",");
const MARGIN_HINT = {
  force_pct: "bacak kuvveti silindirin yük sınırına yakın", speed_pct: "bacak hızı silindirin hız sınırına yakın",
  stroke_pct: "istenen hareket strokun tamamını kullanıyor", joint_angle_pct: "kardan açısı sınıra yakın",
  motor_rpm_pct: "motor devri sınıra yakın", motor_torque_pct: "motor torku sınıra yakın",
};

function limitingText(c) {
  let best = null;
  MARGIN_KEYS.forEach(([k, , name]) => {
    const v = (c.margins || {})[k];
    if (v != null && (best === null || v < best.v)) best = { v, name, hint: v < 0 ? "sınır aşılıyor" : MARGIN_HINT[k] };
  });
  if (!best) return c.limiting_factor ? `Sınırlayan etken: ${c.limiting_factor}` : "";
  return `En dar pay: ${best.name.toLowerCase()} %${pct(best.v)} — ${best.hint}`;
}

function marginBars(m) {
  return MARGIN_KEYS.map(([k, , name]) => {
    const v = m[k];
    if (v == null) return `<div class="mrow"><span>${name}</span><div class="mbar"></div><b>—</b></div>`;
    // strok payının eşiği ayrı (varsayılan %0); kırmızı yalnız eşiğin altı
    const thr = k === "stroke_pct" ? (lastReq?.stroke_margin_pct ?? 0) : (lastReq?.min_margin_pct ?? 10);
    const c = v < thr ? "var(--bad)" : v < thr + 20 ? "var(--warn)" : "var(--ok)";
    return `<div class="mrow"><span>${name}</span><div class="mbar"><i style="width:${Math.max(0, Math.min(100, v))}%;background:${c}"></i></div><b>%${pct(v)}</b></div>`;
  }).join("");
}

const shortReason = c => (c.notes || []).find(n => n && !n.startsWith("Uygun değil")) || "";

function renderCandidates() {
  const res = lastResult, cs = res.candidates || [];
  $("#results").hidden = false;
  selected = null;
  if (!cs.length) {
    $("#viewcard").hidden = true;
    $("#candtable").innerHTML = '<tr><td class="small">Hiç aday bulunamadı; isterleri gevşetin.</td></tr>';
  } else {
    $("#candtable").innerHTML = "<tr><th>#</th><th>Silindir</th><th>Motor</th><th>Plaka Ø alt / üst</th><th>Durum</th><th></th></tr>" +
      cs.map((c, i) => {
        const a = c.actuator, g = c.geometry, s = c.servo;
        const st = c.feasible ? '<span class="ok">Uygun</span>' : `<span class="bad">Uygun değil</span><div class="small">${esc(shortReason(c))}</div>`;
        return `<tr data-i="${i}" class="${c.feasible ? "" : "infeasible"}"><td>${c.rank}</td><td>${esc(a.model)} · strok ${a.stroke_mm} · hatve ${a.lead_mm}</td>` +
          `<td>${s.rated_power_W} W</td><td>${g.base_plate_diameter_mm} / ${g.top_plate_diameter_mm}</td><td>${st}</td><td><button type="button" data-pick="${i}"></button></td></tr>`;
      }).join("");
    $$("#candtable [data-pick]").forEach(b => b.onclick = () => select(+b.dataset.pick, true));
    select(0, false);
  }
  const list = (t, a, tag) => a && a.length ? `<details class="box"><summary><b>${t} (${a.length})</b> ${tag || ""}</summary><ul>${a.map(x => `<li>${esc(x)}</li>`).join("")}</ul></details>` : "";
  $("#notes").innerHTML = list("Varsayımlar", res.assumptions) + list("Teyit gereken katalog verisi", res.unverified_data, TBC);
}

/* ---------- referans kapasite ---------- */
const AXES = { x_mm: ["X", "mm"], y_mm: ["Y", "mm"], z_mm: ["Z", "mm"], roll_deg: ["Roll", "°"], pitch_deg: ["Pitch", "°"], yaw_deg: ["Yaw", "°"] };
api("/reference-capacity").then(r => {
  const c = r.config;
  $("#refcap").hidden = false;
  $("#refcaptitle").textContent = `(${c.model}, strok ${c.stroke_mm}, hatve ${c.lead_mm}, R ${c.R_base_mm}/${c.R_top_mm})`;
  $("#refcapbody").innerHTML = `<div class="small">Home yüksekliği ${c.home_height_mm} mm · home bacak boyu ${c.home_leg_length_mm} mm · kardan sınırı ${c.joint_angle_limit_deg}°. Her eksen tek başına, yalnız kinematik sınır.</div>
    <table><tr><th>Eksen</th><th>+ yön</th><th>− yön</th><th>Max ±</th><th>Bacak hızı / doğrusal hız</th><th>Bacak hızı (mm/s) / açısal (rad/s)</th><th>Bacak kuvveti (N/kg)</th></tr>` +
    Object.keys(AXES).filter(k => r.axes[k]).map(k => [k, r.axes[k]]).map(([k, a]) => `<tr><td>${AXES[k]?.[0] || k}</td><td>${a.plus}</td><td>${a.minus}</td><td><b>${a.max} ${AXES[k]?.[1] || ""}</b>${a.capped ? ' <span class="small">(tarama üst sınırı)</span>' : ""}</td><td>${a.leg_speed_per_lin}</td><td>${a.leg_speed_per_ang_mm_per_rad}</td><td>${a.leg_force_per_kg_N}</td></tr>`).join("") +
    `</table><div class="small">${esc(r.coefficients_note || "")}</div>` + (r.notes || []).map(n => `<div class="small">• ${esc(n)}</div>`).join("");
}).catch(() => {});

function platesTable(c) {
  const p = c.plates, g = c.geometry;
  if (!p) return `<h3>Plakalar</h3><div class="small">Alt Ø${g.base_plate_diameter_mm} / üst Ø${g.top_plate_diameter_mm} mm · kardan yarıçapı alt ${g.R_base_mm} / üst ${g.R_top_mm} mm</div>`;
  const b = p.base, t = p.top;
  return `<h3>Plakalar</h3><div class="tablewrap"><table class="plates"><tr><th></th><th>Dış çap</th><th>Kalınlık</th><th>Kardan delik dairesi</th><th>Biçim</th></tr>
    <tr><td>Alt plaka</td><td><b>Ø${b.diameter_mm} mm</b></td><td>${b.thickness_mm} mm</td><td>Ø${b.hole_circle_diameter_mm} mm (R ${g.R_base_mm})</td><td>daire, iki yanı düz kesik ±${b.flat_half_width_mm} mm</td></tr>
    <tr><td>Üst plaka</td><td><b>Ø${t.diameter_mm} mm</b></td><td>${t.thickness_mm} mm</td><td>Ø${t.hole_circle_diameter_mm} mm (R ${g.R_top_mm})</td><td>köşeleri yuvarlatılmış çokgen (dış çap çevrel çember)</td></tr></table></div>
    <div class="small">${esc(p.note)}</div>`;
}

function driveBox(c) {
  const d = c.drive_settings;
  if (!d) return "";
  const r = (k, v) => `<tr><td>${k}</td><td><b>${v}</b></td></tr>`;
  return `<details class="box"><summary><b>Sürücü ayar önerisi (SV660P, pulse modu)</b> <span class="small">${String(d.pulse_per_mm).replace(".", ",")} pulse/mm</span></summary>
    <div class="tablewrap"><table>
    ${r("Elektronik dişli (enkoder / pulse)", d.electronic_gear)}
    ${r("Motor turu başına pulse", d.pulse_per_rev)}
    ${r("Bacak çözünürlüğü", `${String(d.pulse_per_mm).replace(".", ",")} pulse/mm (1 pulse ≈ ${String(d.resolution_um).replace(".", ",")} µm)`)}
    ${r("En yüksek pulse frekansı", `${String(d.max_pulse_kHz).replace(".", ",")} kHz` + (d.high_speed_input ? " → yüksek hızlı pulse girişi (H05-01 = 1)" : ""))}
    ${r("Enkoder çıkışı H05-17", `${d.H05_17} (×4 = ${d.pulse_per_rev} sayım/tur)`)}
    ${r(`Takip hatası eşiği H0A-10 (${d.follow_err_mm} mm)`, d.H0A_10)}
    </table></div><div class="small">${esc(d.note)}</div></details>`;
}

const money = (v, cur) => Number(v).toLocaleString("tr-TR", { maximumFractionDigits: 2 }) + " " + ({ TRY: "TL" }[cur] || cur);
function costBox(c) {
  const k = c.cost;
  if (!k) return "";
  const tot = Object.entries(k.totals).map(([cur, v]) => `<b>${money(v, cur)}</b>`).join(" + ") +
    (k.missing.length ? ` <span class="bad">· eksik: ${k.missing.length} kalem fiyatsız, toplam tam değil</span>` : "");
  const tr = x => `<tr><td>${esc(x.name)}${x.note ? `<div class="small">${esc(x.note)}</div>` : ""}</td><td>${x.qty ?? ""}</td><td>${x.unit != null ? money(x.unit, x.currency) : ""}</td><td>${money(x.total, x.currency)}</td><td class="small">${esc(x.source)}</td></tr>`;
  return `<details class="box cost"><summary><b>Tahmini malzeme maliyeti</b> <span class="tag">iç kullanım</span> ${tot}</summary>
    <div class="tablewrap"><table><tr><th>Kalem</th><th>Adet</th><th>Birim</th><th>Toplam</th><th>Kaynak</th></tr>${k.lines.map(tr).join("")}${k.sets.map(tr).join("")}</table></div>
    ${k.missing.length ? `<div class="small bad">Fiyatı bulunamayan: ${k.missing.map(esc).join(", ")}</div>` : ""}
    <div class="small">${esc(k.note)}</div></details>`;
}

const SLIDERS = [["x", "X (mm)"], ["y", "Y (mm)"], ["z", "Z (mm)"], ["roll", "Roll (°)"], ["pitch", "Pitch (°)"], ["yaw", "Yaw (°)"]];
function select(i, scroll) {
  selected = lastResult.candidates[i];
  $$("#candtable tr[data-i]").forEach(tr => tr.classList.toggle("sel", +tr.dataset.i === i));
  $$("#candtable [data-pick]").forEach(b => { b.textContent = +b.dataset.pick === i ? "Seçili" : "Bunu seç"; b.disabled = +b.dataset.pick === i; });
  const c = selected, g = c.geometry, a = c.actuator, s = c.servo;
  $("#viewcard").hidden = false;
  $("#rectitle").textContent = i === 0 ? "Önerilen konfigürasyon" : `Seçilen konfigürasyon (alternatif #${c.rank})`;
  const brake = s.brake_option == null ? "fren bilgisi yok" : s.brake_option ? "frenli seçeneği var" : "frenli seçeneği yok";
  const row = (k, v) => `<div class="dimrow"><span>${k}</span><b>${v}</b></div>`;
  const reasons = (c.notes || []).filter(n => n && !n.startsWith("Uygun değil"));
  $("#recbox").innerHTML =
    (c.feasible ? "" : `<div class="badhead">Bu konfigürasyon uygun değil</div><ul class="badlist">${reasons.slice(0, 4).map(n => `<li>${esc(n)}</li>`).join("")}</ul>`) +
    row("Elektrikli silindir", `${esc(a.model)}, strok ${a.stroke_mm} mm, hatve ${a.lead_mm} mm ${TBC.replace("teyit gerekli", "sipariş kodu teyit gerekli")}`) +
    row("Servo motor", `${esc(s.motor_code || "—")} · ${s.rated_power_W} W${s.voltage_V ? " · " + s.voltage_V + " V" : ""} · ${brake}`) +
    row("Servo sürücü", esc(s.drive_model || "—")) +
    row("Home yüksekliği", `${g.home_height_mm} mm`) +
    `<div class="limit">${esc(limitingText(c))}</div>` + platesTable(c) + driveBox(c) + costBox(c);
  $("#margins").innerHTML = "<h3>Pay yüzdeleri</h3>" + marginBars(c.margins || {});
  const keys = ["x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"];
  $("#sliders").innerHTML = SLIDERS.map(([, label], n) => {
    const lim = Math.max(Math.abs(lastReq[keys[n]] || 0), n < 3 ? 50 : 10);
    return `<div class="srow"><span>${label}</span><input type="range" min="${-lim}" max="${lim}" step="0.5" value="0" data-n="${n}"><span id="sv${n}">0</span></div>`;
  }).join("");
  $$("#sliders input").forEach(el => el.oninput = () => { $("#sv" + el.dataset.n).textContent = el.value; viewer.setPose(+el.dataset.n, +el.value); });
  renderCases(c);
  const nn = c.notes || [];
  $("#cnotes").innerHTML = nn.length ? `<details class="box"><summary><b>Aday notları (${nn.length})</b></summary><ul>${nn.map(n => `<li>${esc(n)}</li>`).join("")}</ul></details>` : "";
  viewer.show(c);
  if (scroll) setTimeout(() => $("#viewcard").scrollIntoView({ behavior: "smooth", block: "start" }), 50);
}

function renderCases(c) {
  const w = c.worst_case || {}, pc = w.per_case, f = (x, d = 1) => x == null ? "—" : Number(x).toFixed(d);
  let h = "";
  if (pc && pc.length) {
    h = `<div class="tablewrap"><table><tr><th>#</th><th>Yük (kg)</th><th>Doğr. hız (mm/s)</th><th>Açısal hız (°/s)</th><th>Max bacak kuvveti (N)</th><th>Max bacak hızı (mm/s)</th><th>Motor devri (rpm)</th><th>Sürekli tork (home) Nm</th><th>Tepe tork (en kötü poz) Nm</th><th>Sınırlayan</th><th>Min. pay (%)</th></tr>` +
      pc.map((r, i) => `<tr class="${i === w.limiting_case ? "sel" : ""}"><td>${i + 1}${i === w.limiting_case ? ' <span class="tag">sınırlayan</span>' : ""}</td><td>${f(r.payload_kg, 0)}</td><td>${f(r.lin_speed_mm_s, 0)}</td><td>${f(r.ang_speed_deg_s, 0)}</td><td>${f(r.max_leg_force_N, 0)}</td><td>${f(r.max_leg_speed_mm_s, 0)}</td><td>${f(r.motor_rpm, 0)}</td><td>${f(r.continuous_torque_Nm, 2)}</td><td>${f(r.peak_torque_Nm, 2)}</td><td>${esc(r.limiting_factor || "")}</td><td>${f(r.min_margin_pct, 1)}</td></tr>`).join("") + "</table></div>";
  } else if (w.continuous_torque_Nm != null) {
    h = `<div class="small">Sürekli tork (home): ${f(w.continuous_torque_Nm, 2)} Nm · Tepe tork (en kötü poz): ${f(w.peak_torque_Nm, 2)} Nm</div>`;
  }
  $("#cases").innerHTML = h ? `<details class="box"><summary><b>Yük–hız rejimleri</b></summary>${h}</details>` : "";
}
$("#resetpose").onclick = () => { $$("#sliders input").forEach(s => { s.value = 0; s.oninput(); }); };

$("#toorder").onclick = () => { $("#orderform").hidden = false; $("#ocust").focus(); };
$("#ocancel").onclick = () => { $("#orderform").hidden = true; };
$("#ocreate").onclick = async () => {
  if (!selected) return;
  try {
    const o = await api("/orders", { json: { customer: $("#ocust").value.trim(), requirements: lastReq, selected_candidate: selected } });
    $("#orderform").hidden = true; $("#ocust").value = "";
    document.querySelector('nav button[data-tab="orders"]').click();
    openOrder(o.id);
  } catch (x) { alert(x.message); }
};

/* ---------- siparişler ---------- */
let searchTimer;
$("#oq").oninput = () => { clearTimeout(searchTimer); searchTimer = setTimeout(loadOrders, 250); };
$("#ostatus").onchange = loadOrders;

async function loadOrders() {
  $("#orderlist").hidden = false; $("#orderdetail").hidden = true;
  const os = await api(`/orders?q=${encodeURIComponent($("#oq").value)}&status=${encodeURIComponent($("#ostatus").value)}`);
  $("#ordertable").innerHTML = "<tr><th>Sipariş no</th><th>Müşteri</th><th>Konfig</th><th>Durum</th><th>Güncelleme</th></tr>" +
    (os.map(o => `<tr class="clickable" data-id="${o.id}"><td>${esc(o.order_no)}</td><td>${esc(o.customer)}</td><td>${esc(o.config)}</td>` +
      `<td><span class="pill">${esc(o.status)}</span></td><td>${esc(o.updated_at.replace("T", " "))}</td></tr>`).join("") ||
      '<tr><td colspan="5" class="small">Kayıt yok.</td></tr>');
  $$("#ordertable tr.clickable").forEach(tr => tr.onclick = () => openOrder(+tr.dataset.id));
}

async function downloadFile(oid, fid) {
  if (!STATIC) { location.href = `/api/orders/${oid}/files/${fid}`; return; }
  const f = await staticApi(`/orders/${oid}/files/${fid}`), u = URL.createObjectURL(f.blob), a = document.createElement("a");
  a.href = u; a.download = f.filename; a.click();
  setTimeout(() => URL.revokeObjectURL(u), 5000);
}

// statik mod: tüm siparişleri (ekler base64) JSON olarak yedekle / geri yükle
$("#oexport").onclick = async () => {
  const data = await exportAllOrders(), a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: "application/json" }));
  a.download = `siparisler-${new Date().toISOString().slice(0, 10)}.json`; a.click();
  $("#iomsg").textContent = `${data.orders.length} sipariş dışa aktarıldı.`;
};
$("#oimport").onclick = () => $("#ofile").click();
$("#ofile").onchange = async e => {
  const f = e.target.files[0]; e.target.value = "";
  if (!f) return;
  try {
    const r = await importAllOrders(JSON.parse(await f.text()));
    $("#iomsg").textContent = `${r.added} sipariş eklendi, ${r.skipped} tanesi zaten var (atlandı).`;
    loadOrders();
  } catch (x) { $("#iomsg").textContent = x.message; }
};

let cur = null;
async function openOrder(id) {
  cur = await api("/orders/" + id);
  $("#orderlist").hidden = true; $("#orderdetail").hidden = false;
  renderOrder();
}

function collect() {
  const d = $("#orderdetail"), v = n => $(`[data-f="${n}"]`, d)?.value;
  Object.assign(cur, { order_no: v("order_no"), customer: v("customer"), contact: v("contact"), status: v("status"), notes: v("notes") });
  cur.plate = { ...(cur.plate || {}), base_diameter_mm: num(v("pb")), top_diameter_mm: num(v("pt")), thickness_mm: num(v("pth")), material: v("pm") };
  cur.bom = $$("tr[data-bom]", d).map(tr => {
    const g = k => $(`[data-k="${k}"]`, tr).value;
    return { part_name: g("part_name"), part_code: g("part_code"), qty: num(g("qty")), supplier: g("supplier"), note: g("note") };
  });
  cur.plate.holes = $$("tr[data-hole]", d).map(tr => {
    const g = k => $(`[data-k="${k}"]`, tr).value;
    return { desc: g("desc"), qty: num(g("qty")), dia_mm: num(g("dia_mm")), pos: g("pos") };
  });
}

/* yazdırılabilir teknik föy: ekrandaki (kaydedilmemiş olabilir) sipariş verisinden */
function openSheet(o) {
  const c = o.selected_candidate, r = o.requirements || {}, p = o.plate || {};
  const AX = [["x_mm", "X", "mm"], ["y_mm", "Y", "mm"], ["z_mm", "Z", "mm"], ["roll_deg", "Roll", "°"], ["pitch_deg", "Pitch", "°"], ["yaw_deg", "Yaw", "°"]];
  const tbl = (head, rows) => `<table><tr>${head.map(h => `<th>${h}</th>`).join("")}</tr>${rows.map(rw => `<tr>${rw.map(x => `<td>${x ?? ""}</td>`).join("")}</tr>`).join("")}</table>`;
  let h = `<h1>Stewart PRO — Teknik föy</h1><div class="meta">Sipariş ${esc(o.order_no)}${o.customer ? " · " + esc(o.customer) : ""} · durum: ${esc(o.status)} · ${new Date().toLocaleDateString("tr-TR")}</div>`;
  h += `<h2>İsterler</h2>` + tbl(["Yük (kg)", "Doğrusal hız (mm/s)", "Açısal hız (°/s)", "İvme (m/s²)"],
    (r.load_cases || []).map(x => [x.payload_kg, x.lin_speed_mm_s ?? 0, x.ang_speed_deg_s ?? 0, x.accel_m_s2 ?? "—"])) +
    `<p>Çalışma uzayı (±): ${AX.map(([k, n, u]) => `${n} ${r[k] ?? 0} ${u}`).join(" · ")}${r.simultaneous ? " · eksenler aynı anda" : ""} · ağırlık merkezi ${r.cog_height_mm ?? "—"} mm · emniyet katsayısı ${r.safety_factor ?? "—"}</p>`;
  if (c) {
    const a = c.actuator, s = c.servo, g = c.geometry, pl = c.plates, d = c.drive_settings;
    h += `<h2>Konfigürasyon</h2>` + tbl(["Kalem", "Değer"], [
      ["Elektrikli silindir (6)", `${esc(a.model)}, strok ${a.stroke_mm} mm, hatve ${a.lead_mm} mm, kapalı boy ${a.retracted_length_mm ?? "—"} mm`],
      ["Servo motor (6)", `${esc(s.motor_code || "—")} · ${s.rated_power_W} W`], ["Servo sürücü (6)", esc(s.drive_model || "—")],
      ["Kardan yarıçapı", `alt ${g.R_base_mm} / üst ${g.R_top_mm} mm`], ["Home yüksekliği (kardan–kardan)", `${g.home_height_mm} mm`],
      ["Durum", c.feasible ? "Uygun" : "Uygun değil"], ["En dar pay", esc(limitingText(c).replace(/^En dar pay:\s*/, ""))]]);
    h += `<h2>Plakalar</h2>` + tbl(["", "Dış çap", "Kalınlık", "Kardan delik dairesi", "Biçim"], [
      ["Alt", `Ø${p.base_diameter_mm ?? pl?.base.diameter_mm ?? "—"} mm`, `${p.thickness_mm ?? pl?.base.thickness_mm ?? "—"} mm`, pl ? `Ø${pl.base.hole_circle_diameter_mm} mm` : `R ${g.R_base_mm}`, pl ? `iki yanı düz kesik ±${pl.base.flat_half_width_mm} mm` : ""],
      ["Üst", `Ø${p.top_diameter_mm ?? pl?.top.diameter_mm ?? "—"} mm`, `${p.thickness_mm ?? pl?.top.thickness_mm ?? "—"} mm`, pl ? `Ø${pl.top.hole_circle_diameter_mm} mm` : `R ${g.R_top_mm}`, pl ? "köşeleri yuvarlatılmış çokgen (dış çap çevrel çember)" : ""]]) +
      (p.material ? `<p>Malzeme: ${esc(p.material)}</p>` : "");
    // ustaya: delik dairesi çapı + açı (plakanın +X ekseninden saat yönü tersine)
    const ang = p => { const a = Math.atan2(p[1], p[0]) * 180 / Math.PI; return ((a % 360) + 360) % 360; };
    const fmt = a => (Math.round(a * 10) / 10).toString().replace(".", ",") + "°";
    h += `<h3>Kardan bağlantı noktaları</h3><p>Alt plaka: Ø${2 * g.R_base_mm} mm daire üzerinde · Üst plaka: Ø${2 * g.R_top_mm} mm daire üzerinde. Açılar plakanın +X ekseninden saat yönünün tersine.</p>` +
      tbl(["Bacak", "Alt plaka açısı", "Üst plaka açısı"], g.base_joints_mm.map((b, i) => [i + 1, fmt(ang(b)), fmt(ang(g.top_joints_mm[i]))]));
    if (d) h += `<h2>Sürücü ayarları (SV660P, pulse modu)</h2>` + tbl(["Parametre", "Değer"], [
      ["Elektronik dişli", d.electronic_gear], ["Pulse / mm", d.pulse_per_mm], ["En yüksek pulse frekansı", `${d.max_pulse_kHz} kHz${d.high_speed_input ? " (H05-01 = 1)" : ""}`],
      ["H05-17 enkoder çıkışı", d.H05_17], [`H0A-10 takip hatası (${d.follow_err_mm} mm)`, d.H0A_10]]) + `<p class="small">${esc(d.note)}</p>`;
  }
  if ((p.holes || []).length) h += `<h2>Özel delikler</h2>` + tbl(["Açıklama", "Adet", "Çap (mm)", "Konum"], p.holes.map(x => [esc(x.desc), x.qty, x.dia_mm, esc(x.pos)]));
  h += `<h2>Malzeme listesi</h2>` + tbl(["Parça", "Kod", "Adet", "Tedarikçi", "Not"], (o.bom || []).map(b => [esc(b.part_name), esc(b.part_code), b.qty, esc(b.supplier), esc(b.note)]));
  if (o.notes) h += `<h2>Notlar</h2><p style="white-space:pre-wrap">${esc(o.notes)}</p>`;
  const w = window.open("", "_blank");
  if (!w) { alert("Açılır pencere engellendi; tarayıcıda bu siteye izin verin."); return; }
  w.document.write(`<!doctype html><html lang="tr"><head><meta charset="utf-8"><title>Teknik föy ${esc(o.order_no)}</title><style>
    body{font:13px/1.45 system-ui,Segoe UI,Arial,sans-serif;color:#111;margin:24px;max-width:900px}
    h1{font-size:20px;margin:0}h2{font-size:15px;margin:18px 0 6px;border-bottom:1px solid #999}h3{font-size:13px;margin:12px 0 4px}
    .meta{color:#555;margin:4px 0 8px}.small{font-size:11px;color:#555}table{border-collapse:collapse;width:100%;margin:4px 0}
    th,td{border:1px solid #bbb;padding:3px 6px;text-align:left;vertical-align:top}th{background:#eee}
    @media print{body{margin:10mm}button{display:none}}</style></head><body>
    <button onclick="print()" style="float:right">Yazdır</button>${h}</body></html>`);
  w.document.close();
}

function renderOrder() {
  const o = cur, p = o.plate || {}, c = o.selected_candidate, r = o.requirements || {};
  const inp = (f, v, t = "text") => `<input data-f="${f}" type="${t}" value="${esc(v)}">`;
  const cell = (k, v, t = "text") => `<td><input data-k="${k}" type="${t}" step="any" value="${esc(v)}"></td>`;
  const a = c?.actuator, s = c?.servo;
  $("#orderdetail").innerHTML = `
  <div class="card"><div class="row-actions" style="margin:0 0 10px"><button id="back">← Liste</button><h2 style="margin:0">${esc(o.order_no)}</h2><button id="sheet" title="Kaydedilmiş bilgilerle yazdırılabilir sayfa açar">Teknik föy (yazdır)</button></div>
    <div class="cols">
      <label>Sipariş no${inp("order_no", o.order_no)}</label>
      <label>Müşteri${inp("customer", o.customer)}</label>
      <label>İletişim${inp("contact", o.contact)}</label>
      <label>Durum<select data-f="status">${STATUSES.map(x => `<option ${x === o.status ? "selected" : ""}>${x}</option>`).join("")}</select></label>
    </div>
    <div class="small">Oluşturma: ${esc(o.created_at)} · Güncelleme: ${esc(o.updated_at)}</div></div>
  <div class="card"><h2>İsterler (salt okunur)</h2><pre class="kv">${esc(Object.entries(r).filter(([k, v]) => k !== "load_cases" && v !== null && v !== "").map(([k, v]) => `${k}: ${v}`).join("\n")) || "—"}${(r.load_cases || []).length ? "\n\nYük–hız rejimleri:\n" + esc(r.load_cases.map((c, i) => `${i + 1}) ${c.payload_kg} kg, ${c.lin_speed_mm_s ?? 0} mm/s, ${c.ang_speed_deg_s ?? 0} °/s, ivme: ${c.accel_m_s2 ?? "—"}`).join("\n")) : ""}</pre></div>
  <div class="card"><h2>Seçilen konfigürasyon</h2>${c ? `<div>${esc(a.model)} · strok ${a.stroke_mm} mm · hatve ${a.lead_mm} mm · motor ${esc(s.motor_code || "")} ${s.rated_power_W} W · sürücü ${esc(s.drive_model || "")}</div>
      <div class="small">Plaka Ø ${c.geometry.base_plate_diameter_mm} / ${c.geometry.top_plate_diameter_mm} mm · home yüksekliği ${c.geometry.home_height_mm} mm · sınırlayan: ${esc(c.limiting_factor)}</div>
      <div class="small">Katalog snapshot: ${o.catalog_snapshot?.taken_at ? esc(o.catalog_snapshot.taken_at) : "—"}${o.catalog_snapshot?.actuator ? "" : " · silindir satırı katalogda bulunamadı"}</div>` : "Aday seçilmemiş."}</div>
  <div class="card"><h2>Malzeme listesi (BOM)</h2>
    <table><tr><th>Parça</th><th>Kod</th><th style="width:70px">Adet</th><th>Tedarikçi</th><th>Not</th><th></th></tr>
    ${o.bom.map((b, i) => `<tr data-bom="${i}">${cell("part_name", b.part_name)}${cell("part_code", b.part_code)}${cell("qty", b.qty, "number")}${cell("supplier", b.supplier)}${cell("note", b.note)}<td><button class="danger" data-delbom="${i}">Sil</button></td></tr>`).join("")}</table>
    <div class="row-actions"><button id="addbom">+ Satır ekle</button></div></div>
  <div class="card"><h2>Plaka bilgisi</h2>
    <div class="cols">
      <label>Alt plaka çapı (mm)${inp("pb", p.base_diameter_mm ?? "", "number")}</label>
      <label>Üst plaka çapı (mm)${inp("pt", p.top_diameter_mm ?? "", "number")}</label>
      <label>Kalınlık (mm)${inp("pth", p.thickness_mm ?? "", "number")}</label>
      <label>Malzeme${inp("pm", p.material ?? "")}</label>
    </div>
    <h3>Özel delikler</h3>
    <table><tr><th>Açıklama</th><th style="width:70px">Adet</th><th style="width:90px">Çap (mm)</th><th>Konum notu</th><th></th></tr>
    ${(p.holes || []).map((h, i) => `<tr data-hole="${i}">${cell("desc", h.desc)}${cell("qty", h.qty, "number")}${cell("dia_mm", h.dia_mm, "number")}${cell("pos", h.pos)}<td><button class="danger" data-delhole="${i}">Sil</button></td></tr>`).join("")}</table>
    <div class="row-actions"><button id="addhole">+ Delik ekle</button></div></div>
  <div class="card"><h2>Dosyalar</h2>
    <table>${o.files.map(f => `<tr><td>${f.kind === "STEP" ? '<span class="tag" style="background:#e0ecff;color:#13315c;border-color:#9fb0c8">STEP</span>' : esc(f.kind)}</td>
      <td><a href="#" data-dl="${f.id}">${esc(f.filename)}</a></td><td>${(f.size / 1024).toFixed(1)} KB</td><td>${esc(f.uploaded_at.replace("T", " "))}</td>
      <td><button class="danger" data-delfile="${f.id}">Kaldır</button></td></tr>`).join("") || '<tr><td class="small">Dosya yok.</td></tr>'}</table>
    <div class="drop" id="drop">Dosyaları buraya sürükleyin veya <label style="display:inline;color:var(--navy2);cursor:pointer;text-decoration:underline">seçin<input id="filepick" type="file" multiple hidden></label>
      <div class="small">.step .stp .pdf .png .jpg .dxf .xlsx .docx · en fazla 200 MB</div></div></div>
  <div class="card"><h2>Notlar</h2><textarea data-f="notes" rows="5">${esc(o.notes)}</textarea>
    <div class="row-actions"><button class="primary" id="saveorder">Kaydet</button><button class="danger" id="cancelorder">Siparişi iptal et</button><span id="savemsg" class="small"></span></div></div>`;

  const d = $("#orderdetail");
  $("#back", d).onclick = loadOrders;
  $("#sheet", d).onclick = () => { collect(); openSheet(cur); };
  $("#addbom", d).onclick = () => { collect(); cur.bom.push({ part_name: "", part_code: "", qty: 1, supplier: "", note: "" }); renderOrder(); };
  $("#addhole", d).onclick = () => { collect(); cur.plate.holes.push({ desc: "", qty: 1, dia_mm: null, pos: "" }); renderOrder(); };
  $$("[data-delbom]", d).forEach(b => b.onclick = () => { collect(); cur.bom.splice(+b.dataset.delbom, 1); renderOrder(); });
  $$("[data-delhole]", d).forEach(b => b.onclick = () => { collect(); cur.plate.holes.splice(+b.dataset.delhole, 1); renderOrder(); });
  $$("[data-dl]", d).forEach(a => a.onclick = e => { e.preventDefault(); downloadFile(cur.id, +a.dataset.dl); });
  $$("[data-delfile]", d).forEach(b => b.onclick = async () => {
    if (!confirm("Dosya kayıttan kaldırılsın mı? (Disk üzerinde _trash klasörüne taşınır)")) return;
    collect(); await api(`/orders/${cur.id}/files/${b.dataset.delfile}`, { method: "DELETE" });
    cur.files = (await api("/orders/" + cur.id)).files; renderOrder();
  });
  $("#saveorder", d).onclick = async () => {
    collect();
    try {
      const { id, files } = cur;
      cur = await api("/orders/" + id, { method: "PUT", json: { order_no: cur.order_no, customer: cur.customer, contact: cur.contact, status: cur.status, notes: cur.notes, plate: cur.plate, bom: cur.bom } });
      renderOrder(); $("#savemsg").textContent = "Kaydedildi.";
    } catch (x) { $("#savemsg").textContent = x.message; }
  };
  $("#cancelorder", d).onclick = async () => {
    if (!confirm("Sipariş 'iptal' durumuna alınsın mı? (Kayıt silinmez)")) return;
    cur = await api("/orders/" + cur.id, { method: "DELETE" }); renderOrder();
  };
  const upload = async files => {
    if (!files.length) return;
    collect();
    const fd = new FormData();
    [...files].forEach(f => fd.append("file", f));
    try {
      await api(`/orders/${cur.id}/files`, { method: "POST", body: fd });
      cur.files = (await api("/orders/" + cur.id)).files; renderOrder();
    } catch (x) { alert(x.message); }
  };
  const drop = $("#drop", d);
  ["dragover", "dragenter"].forEach(ev => drop.addEventListener(ev, e => { e.preventDefault(); drop.classList.add("over"); }));
  ["dragleave", "drop"].forEach(ev => drop.addEventListener(ev, e => { e.preventDefault(); drop.classList.remove("over"); }));
  drop.addEventListener("drop", e => upload(e.dataTransfer.files));
  $("#filepick", d).onchange = e => upload(e.target.files);
}

/* ---------- katalog ---------- */
async function loadCatalog() {
  const c = await api("/catalog");
  const v = x => x == null ? TBC : esc(typeof x === "object" ? JSON.stringify(x) : x);
  const link = (u, t) => /^https?:\/\//.test(u) ? `<a href="${esc(u)}" target="_blank" rel="noopener">${esc(t || "kaynak")}</a>` : esc(u || "");
  // kaynak metnindeki URL'ler tıklanabilir olsun
  const srcs = t => !t ? TBC : (String(t).match(/https?:\/\/[^\s;,)]+/g) || []).map((u, i) => link(u, new URL(u).hostname + (i ? ` (${i + 1})` : ""))).join("<br>") || esc(t);
  // iki kaynak farklıysa küçük imotion bilgisi
  const dual = (l, k) => {
    const alt = (l.alt_values_imotion_com_tr || {})[k];
    return v(l[k]) + (alt != null && alt !== l[k] ? `<div class="small">imotion: ${esc(alt)}</div>` : "");
  };
  const list = a => a && a.length ? `<ul class="small">${a.map(x => `<li>${esc(x)}</li>`).join("")}</ul>` : "";
  let h = (c.warnings || []).map(w => `<div class="mockbar">${esc(w)}</div>`).join("");
  const pr = c.prices;
  if (pr) {
    const row = (name, x) => `<tr><td>${name}</td><td><b>${Number(x.price).toLocaleString("tr-TR")} ${x.currency === "TRY" ? "TL" : esc(x.currency)}</b></td><td class="small">${esc([x.supplier, x.source].filter(Boolean).join(" · "))}${x.note ? " — " + esc(x.note) : ""}</td></tr>`;
    h += `<details class="card" open><summary><b>Bilinen teklif fiyatları</b> <span class="tag">iç kullanım</span> <span class="small">KDV hariç, birim fiyat</span></summary>
      <div class="small">${esc(pr.note || "")}</div><div class="tablewrap"><table><tr><th>Kalem</th><th>Birim fiyat</th><th>Kaynak</th></tr>` +
      pr.actuators.map(x => row(`${esc(x.code)} <span class="small">(${esc(x.model)}, strok ${x.stroke_mm}${x.lead_mm ? ", hatve " + x.lead_mm : ""})</span>`, x)).join("") +
      pr.motors.map(x => row(`${esc(x.motor_code)}${x.brake ? " frenli" : ""}`, x)).join("") +
      pr.drives.map(x => row(esc(x.drive_model), x)).join("") +
      pr.cables.map(x => row(esc(x.name), x)).join("") + `</table></div></details>`;
  }
  const am = c.actuators_meta || {};
  h += `<div class="card"><h2>Silindir modelleri</h2><div class="small">Kaynak: ${srcs(am.source)} · alındı ${esc(am.retrieved || "")}</div>
    <div class="tablewrap"><table><tr><th>Model</th><th>Seri</th><th>Strok</th><th>Max strok</th><th>Kapalı boy</th><th>Max ivme (m/s²)</th><th>Kütle (kg)</th><th>Kaynaklar</th><th>Teyit</th></tr>` +
    c.actuators.map(m => {
      const f = m.retracted_length_formula || {}, base = f.base_mm ?? (f.base_mm_inline != null ? `${f.base_mm_inline} (inline) / ${f.base_mm_parallel} (paralel)` : null);
      return `<tr><td><b>${esc(m.model)}</b>${m.used_before ? ' <span class="tag" title="Daha önce siparişte kullanıldı">kullanıldı</span>' : ""}</td><td>${v(m.series)}</td><td>${v(m.stroke_options_mm && m.stroke_options_mm.join(", "))}</td><td>${v(m.stroke_max_mm)}</td>
        <td>${base == null ? TBC : `${base} + ${f.per_stroke}×strok`}</td><td>${v(m.max_acceleration_m_s2)}</td><td>${v(m.mass_kg)}</td>
        <td>${(m.source_urls || []).map((u, i) => link(u, "kaynak " + (i + 1))).join("<br>")}</td><td>${m.verify?.length ? TBC + `<details><summary class="small">${m.verify.length} madde</summary>${list(m.verify)}</details>` : ""}</td></tr>`;
    }).join("") + "</table></div></div>";
  h += `<div class="card"><h2>Hatve seçenekleri (vida, hız, yük)</h2><div class="small">Küçük "imotion" notu: üretici sitesi farklı değer veriyor. Max kuvvet = min(anma, dinamik) olarak türetilmiş.</div>
    <div class="tablewrap"><table><tr><th>Model</th><th>Vida</th><th>Hatve (mm)</th><th>Max hız (mm/s)</th><th>Anma kuvvet (N)</th><th>Dinamik yük (N)</th><th>Statik yük (N)</th><th>Max kuvvet (N)</th><th>Max devir (rpm)</th><th>Kaynak</th></tr>` +
    c.actuators.flatMap(m => (m.lead_options || []).map(l => `<tr><td>${esc(m.model)}</td><td>${v(l.screw_dia_x_lead)}</td><td>${v(l.lead_mm)}</td><td>${dual(l, "max_speed_mm_s")}</td><td>${dual(l, "nominal_force_N")}</td>
      <td>${dual(l, "dynamic_load_N")}</td><td>${dual(l, "static_load_N")}</td><td>${v(l.max_force_N)}</td><td>${v(l.max_input_rpm)}</td><td>${srcs(l.source)}${l.datasheet_pdf ? "<br>" + link(l.datasheet_pdf, "datasheet") : ""}${l.note ? `<div class="small">${esc(l.note)}</div>` : ""}</td></tr>`)).join("") +
    `</table></div></div>`;
  const rj = am.rod_end_joint;
  if (rj) h += `<div class="card"><h2>Rod ucu / kardan mafsal</h2><div>${esc(rj.reference_part)} — max çalışma açısı ${v(rj.max_working_angle_deg)}°, max devir ${v(rj.max_speed_rpm)} d/d</div>
    <div class="small">${esc(rj.interpretation || "")} ${TBC}</div><div class="small">${esc(rj.note || "")}</div><div class="small">${esc(rj.cylinder_side_rod_end_note || "")}</div>
    <div class="small">1G ölçüleri: ${esc(JSON.stringify(rj["1G_dimensions_mm"] || {}))}</div><div class="small">${srcs(rj.source)}</div></div>`;
  const sm = c.servos_meta || {};
  h += `<div class="card"><h2>Servo motorlar ve sürücüler (${esc(sm.drive_series || "")})</h2><div class="small">${esc(sm.scope_note || "")}</div><div class="small">${esc(sm.drive_model_naming || "")}</div>
    <div class="tablewrap"><table><tr><th>Motor</th><th>Gerilim</th><th>Güç (W)</th><th>Nominal tork (Nm)</th><th>Tepe tork (Nm)</th><th>Nominal / max devir</th><th>Flanş (mm)</th><th>Atalet (kgcm²)</th><th>Fren</th><th>Kütle (kg)</th><th>Sürücü</th><th>Sürücü akımı (A)</th><th>Kaynak</th></tr>` +
    c.servos.map(m => `<tr><td>${v(m.motor_code)}</td><td>${v(m.voltage_V)} V</td><td>${v(m.rated_power_W)}</td><td>${v(m.rated_torque_Nm)}</td><td>${v(m.peak_torque_Nm)}</td><td>${v(m.rated_speed_rpm)} / ${v(m.max_speed_rpm)}</td>
      <td>${v(m.flange_mm)}</td><td>${v(m.inertia_kgcm2)}</td><td>${m.brake_option == null ? TBC : m.brake_option ? "var" : "yok"}</td><td>${v(m.mass_kg)}</td><td>${v(m.drive_model)}</td><td>${v(m.drive_rated_current_A)}</td><td>${srcs(m.source)}</td></tr>`).join("") +
    `</table></div>${(sm.verify || []).length ? `<div class="box"><b>Teyit gerekli</b> ${TBC}${list(sm.verify)}</div>` : ""}</div>`;
  $("#catalogbox").innerHTML = h;
}
