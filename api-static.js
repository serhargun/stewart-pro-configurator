"use strict";
// Statik mod: Flask API'sinin şekillerini taklit eder. Hesap = Pyodide worker, siparişler = IndexedDB.
const STATUSES_STATIC = ["teklif", "onaylandı", "üretimde", "teslim edildi", "iptal"];
const ALLOWED_EXT = [".step", ".stp", ".pdf", ".png", ".jpg", ".dxf", ".xlsx", ".docx", ".zip", ".sldprt", ".sldasm", ".slddrw", ".igs", ".iges", ".x_t"];
const CAD_EXT = [".zip", ".sldprt", ".sldasm", ".slddrw", ".igs", ".iges", ".x_t"];

/* ---- worker ---- */
const engine = (() => {
  const w = new Worker("engine-worker.js");
  const pending = new Map();
  let seq = 0, onState = () => {};
  const state = { ready: false, error: null };
  w.onmessage = e => {
    const m = e.data;
    if ("ready" in m && !("id" in m)) { state.ready = m.ready; state.error = m.error || null; onState(state); return; }
    const p = pending.get(m.id); pending.delete(m.id);
    m.error ? p.reject(new Error(m.error)) : p.resolve(m.result);
  };
  w.onerror = e => { state.error = e.message; onState(state); };
  return {
    state,
    watch(cb) { onState = cb; cb(state); },
    call(fn, ...args) { return new Promise((resolve, reject) => { const id = ++seq; pending.set(id, { resolve, reject }); w.postMessage({ id, fn, args }); }); },
  };
})();

/* ---- IndexedDB ---- */
const dbp = new Promise((resolve, reject) => {
  const r = indexedDB.open("stw-configurator", 1);
  r.onupgradeneeded = () => {
    r.result.createObjectStore("orders", { keyPath: "id", autoIncrement: true });
    r.result.createObjectStore("files", { keyPath: "id", autoIncrement: true }).createIndex("order_id", "order_id");
  };
  r.onsuccess = () => resolve(r.result);
  r.onerror = () => reject(r.error);
});
async function tx(store, mode, fn) {
  const db = await dbp;
  return new Promise((resolve, reject) => {
    const t = db.transaction(store, mode), s = t.objectStore(store);
    let out;
    Promise.resolve(fn(s)).then(v => { out = v; });
    t.oncomplete = () => resolve(out);
    t.onerror = t.onabort = () => reject(t.error);
  });
}
const req = r => new Promise((res, rej) => { r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error); });
const allOrders = () => tx("orders", "readonly", s => req(s.getAll()));
const allFiles = orderId => tx("files", "readonly", s => req(s.index("order_id").getAll(orderId)));
const nowIso = () => new Date().toISOString().slice(0, 19);

const fileMeta = f => ({ id: f.id, kind: f.kind, filename: f.filename, size: f.size, uploaded_at: f.uploaded_at, note: f.note });
async function fullOrder(o) {
  return { ...o, files: (await allFiles(o.id)).map(fileMeta) };
}
function summary(o) {
  const a = (o.selected_candidate || {}).actuator || {};
  const { id, order_no, customer, contact, created_at, updated_at, status, notes } = o;
  return { id, order_no, customer, contact, created_at, updated_at, status, notes,
    config: `${a.model ?? ""} ${a.stroke_mm ?? ""}/${a.lead_mm ?? ""}`.trim().replace(/^\/|\/$/g, "") };
}
async function getOrder(id) {
  const o = await tx("orders", "readonly", s => req(s.get(id)));
  if (!o) throw new Error("Sipariş bulunamadı");
  return o;
}
const putOrder = o => tx("orders", "readwrite", s => req(s.put(o)));

async function nextOrderNo(orders) {
  const prefix = `STW-${new Date().getFullYear()}-`, used = new Set(orders.map(o => o.order_no));
  let n = orders.filter(o => o.order_no.startsWith(prefix)).length + 1;
  while (used.has(prefix + String(n).padStart(3, "0"))) n++;
  return prefix + String(n).padStart(3, "0");
}
const withIds = lines => (lines || []).map((l, i) => ({ id: i + 1, part_name: l.part_name || "", part_code: l.part_code || "", qty: l.qty || 0, supplier: l.supplier || "", note: l.note || "" }));

// Flask'taki secure_filename benzeri: aksan atılır, boşluk _, yalnız [A-Za-z0-9._-]
function safeName(name, ext) {
  let n = name.normalize("NFKD").replace(/[̀-ͯ]/g, "").replace(/ı/g, "i").replace(/\s+/g, "_").replace(/[^A-Za-z0-9._-]/g, "").replace(/^[._]+/, "");
  if (!n.toLowerCase().endsWith(ext)) n = (n.replace(/\.[^.]*$/, "") || "dosya") + ext;
  return n;
}
const kindOf = ext => ({ ".step": "STEP", ".stp": "STEP", ".pdf": "çizim", ".dxf": "çizim", ".png": "görsel", ".jpg": "görsel" })[ext] || (CAD_EXT.includes(ext) ? "CAD" : "diğer");

/* ---- yönlendirme ---- */
async function staticApi(path, opt = {}) {
  const url = new URL(path, "http://x"), p = url.pathname, method = (opt.method || (opt.json !== undefined || opt.body ? "POST" : "GET")).toUpperCase();
  const body = opt.json;
  let m;
  if (p === "/status") return { engine: true, statuses: STATUSES_STATIC };
  if (p === "/catalog") return JSON.parse(await engine.call("catalog"));
  if (p === "/reference-capacity") return JSON.parse(await engine.call("reference_capacity"));
  if (p === "/size") return JSON.parse(await engine.call("size", JSON.stringify(body || {})));

  if (p === "/orders" && method === "GET") {
    const q = (url.searchParams.get("q") || "").toLowerCase(), st = url.searchParams.get("status") || "";
    return (await allOrders()).filter(o => (!q || [o.order_no, o.customer, o.notes].some(x => (x || "").toLowerCase().includes(q))) && (!st || o.status === st))
      .sort((a, b) => b.id - a.id).map(summary);
  }
  if (p === "/orders" && method === "POST") {
    const d = body || {}, orders = await allOrders(), cand = d.selected_candidate ?? null;
    const orderNo = (d.order_no || "").trim() || await nextOrderNo(orders);
    if (orders.some(o => o.order_no === orderNo)) throw new Error("Bu sipariş no zaten var");
    const status = d.status || "teklif";
    if (!STATUSES_STATIC.includes(status)) throw new Error("Geçersiz durum");
    const t = nowIso();
    // BOM, snapshot ve plaka önerisi sunucudaki (Flask) ile aynı Python kodundan gelir
    const parts = JSON.parse(await engine.call("order_parts", JSON.stringify(cand), t));
    const o = { order_no: orderNo, customer: d.customer || "", contact: d.contact || "", created_at: t, updated_at: t, status,
      requirements: d.requirements ?? null, selected_candidate: cand, catalog_snapshot: parts.snapshot,
      plate: d.plate ?? parts.plate, notes: d.notes || "", bom: withIds(d.bom ?? parts.bom) };
    o.id = await putOrder(o);
    return fullOrder(o);
  }
  if ((m = p.match(/^\/orders\/(\d+)$/))) {
    const id = +m[1], o = await getOrder(id);
    if (method === "GET") return fullOrder(o);
    if (method === "DELETE") { o.status = "iptal"; o.updated_at = nowIso(); await putOrder(o); return fullOrder(o); }
    if (method === "PUT") {
      const d = body || {};
      if ("status" in d && !STATUSES_STATIC.includes(d.status)) throw new Error("Geçersiz durum");
      if ("order_no" in d && d.order_no !== o.order_no && (await allOrders()).some(x => x.order_no === d.order_no)) throw new Error("Bu sipariş no zaten var");
      for (const k of ["order_no", "customer", "contact", "status", "notes", "requirements", "plate"]) if (k in d) o[k] = d[k];
      if ("selected_candidate" in d) {
        if (JSON.stringify(d.selected_candidate) !== JSON.stringify(o.selected_candidate)) {
          // snapshot yalnız seçilen aday değişirse yenilenir
          o.catalog_snapshot = JSON.parse(await engine.call("order_parts", JSON.stringify(d.selected_candidate), nowIso())).snapshot;
        }
        o.selected_candidate = d.selected_candidate;
      }
      if ("bom" in d) o.bom = withIds(d.bom);
      o.updated_at = nowIso();
      await putOrder(o);
      return fullOrder(o);
    }
  }
  if ((m = p.match(/^\/orders\/(\d+)\/files$/)) && method === "POST") {
    const oid = +m[1];
    await getOrder(oid);
    const files = opt.body.getAll("file").concat(opt.body.getAll("files"));
    if (!files.length) throw new Error("Dosya gönderilmedi");
    const existing = (await allFiles(oid)).map(f => f.filename), saved = [];
    for (const f of files) {
      const ext = (f.name.match(/\.[^.]*$/) || [""])[0].toLowerCase();
      if (!ALLOWED_EXT.includes(ext)) throw new Error("Desteklenmeyen dosya türü: " + (ext || f.name));
      if (f.size > 200 * 1024 * 1024) throw new Error("Dosya 200 MB sınırını aşıyor");
      let name = safeName(f.name, ext);
      const stem = name.slice(0, name.length - ext.length);
      for (let n = 1; existing.includes(name);) name = `${stem}_v${++n}${ext}`;
      existing.push(name);
      const rec = { order_id: oid, kind: opt.body.get("kind") || kindOf(ext), filename: name, size: f.size, uploaded_at: nowIso(), note: opt.body.get("note") || "", blob: f };
      rec.id = await tx("files", "readwrite", s => req(s.add(rec)));
      saved.push(fileMeta(rec));
    }
    return saved;
  }
  if ((m = p.match(/^\/orders\/(\d+)\/files\/(\d+)$/))) {
    const oid = +m[1], fid = +m[2], f = await tx("files", "readonly", s => req(s.get(fid)));
    if (!f || f.order_id !== oid) throw new Error("Dosya bulunamadı");
    if (method === "GET") return f;   // {filename, blob, ...}: indirme app.js'te object URL ile
    if (method === "DELETE") { await tx("files", "readwrite", s => req(s.delete(fid))); return { ok: true }; }
  }
  throw new Error("Bilinmeyen istek: " + method + " " + path);
}

/* ---- dışa / içe aktarma (ekler base64) ---- */
const blobToB64 = b => new Promise(res => { const r = new FileReader(); r.onload = () => res(r.result.split(",")[1] || ""); r.readAsDataURL(b); });
const b64ToBlob = s => { const bin = atob(s), a = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i); return new Blob([a]); };

async function exportAllOrders() {
  const out = { format: "stw-configurator-orders", version: 1, exported_at: nowIso(), orders: [] };
  for (const o of await allOrders()) {
    const files = [];
    for (const f of await allFiles(o.id)) files.push({ ...fileMeta(f), data_b64: await blobToB64(f.blob) });
    out.orders.push({ ...o, files });
  }
  return out;
}

async function importAllOrders(data) {
  if (!data || data.format !== "stw-configurator-orders") throw new Error("Geçersiz dışa aktarım dosyası");
  const have = new Set((await allOrders()).map(o => o.order_no));
  let added = 0, skipped = 0;
  for (const src of data.orders) {
    if (have.has(src.order_no)) { skipped++; continue; }
    const { files, id, ...o } = src;
    const newId = await putOrder(o);
    for (const f of files || []) {
      const blob = b64ToBlob(f.data_b64);
      await tx("files", "readwrite", s => req(s.add({ order_id: newId, kind: f.kind, filename: f.filename, size: blob.size, uploaded_at: f.uploaded_at, note: f.note || "", blob })));
    }
    have.add(o.order_no); added++;
  }
  return { added, skipped };
}
