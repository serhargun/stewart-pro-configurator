// Hesap motoru: Pyodide içinde engine/*.py (yalnız numpy). Dosyalar py/manifest.json'dan okunur.
importScripts("https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js");

const ready = (async () => {
  const py = await loadPyodide();
  await py.loadPackage("numpy");
  const manifest = await (await fetch("py/manifest.json")).json();
  for (const f of manifest.files) {
    const path = "/app/" + f;
    py.FS.mkdirTree(path.slice(0, path.lastIndexOf("/")));
    py.FS.writeFile(path, await (await fetch("py/" + f)).text());
  }
  py.runPython("import sys; sys.path.insert(0, '/app')");
  const mod = py.pyimport("app.web_entry");
  postMessage({ ready: true });
  return mod;
})();
ready.catch(e => postMessage({ ready: false, error: String(e.message || e) }));

onmessage = async e => {
  const { id, fn, args } = e.data;
  try {
    const mod = await ready;
    postMessage({ id, result: mod[fn](...args) });
  } catch (err) {
    // Python hatasında yalnız son satır (ValueError: ...) kullanıcıya gösterilir
    const lines = String(err.message || err).trim().split("\n");
    postMessage({ id, error: lines[lines.length - 1] });
  }
};
