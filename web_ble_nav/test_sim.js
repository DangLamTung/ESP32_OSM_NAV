// test_sim.js — verify the web_ble_nav path-following simulation math.
const fs = require("fs");
const html = fs.readFileSync("/tmp/ESP32_OSM_NAV/web_ble_nav/index.html", "utf8");
const js = html.slice(html.indexOf("<script>") + 8, html.lastIndexOf("</script>"));

// Minimal DOM/console stubs so the page's script evaluates without errors.
const el = new Proxy({}, { get: () => () => {}, set: () => true });
global.document = {
  getElementById: () => el,
  createElement: () => el,
};
global.navigator = { bluetooth: { requestDevice: async () => { throw new Error("no bt"); } } };
const origLog = console.log;
global.console = { log: () => {}, error: () => {} };

try {
  eval(js); // must not throw -> page JS is valid
  origLog("PAGE JS: OK (evaluated without syntax errors)");
} catch (e) {
  origLog("PAGE JS ERROR: " + e.message);
  process.exit(1);
}
global.console = { log: origLog, error: origLog };

// ---- now test the pure sim math against a rectangular route ----
function parseRoutePts(xml) {
  const out = [];
  const re = /<p\s+lat="([-\d.]+)"\s+lon="([-\d.]+)"\s*\/>/g;
  let m;
  while ((m = re.exec(xml))) out.push({ lat: parseFloat(m[1]), lon: parseFloat(m[2]) });
  return out;
}
function distMeters(a, b) {
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLon = (b.lon - a.lon) * Math.PI / 180;
  const la1 = a.lat * Math.PI / 180, la2 = b.lat * Math.PI / 180;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(la1) * Math.cos(la2) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}
function heading(a, b) {
  const f1 = a.lat * Math.PI / 180, f2 = b.lat * Math.PI / 180;
  const dl = (b.lon - a.lon) * Math.PI / 180;
  const x = Math.sin(dl) * Math.cos(f2);
  const y = Math.cos(f1) * Math.sin(f2) - Math.sin(f1) * Math.cos(f2) * Math.cos(dl);
  return Math.round((Math.atan2(x, y) * 180 / Math.PI + 360) % 360);
}
function buildSim(pts) {
  const seg = [0];
  let acc = 0;
  for (let i = 0; i < pts.length - 1; i++) { acc += distMeters(pts[i], pts[i + 1]); seg.push(acc); }
  return { pts, seg, total: acc };
}
function simAt(state, d) {
  const { pts, seg, total } = state;
  d = ((d % total) + total) % total;
  let i = 0;
  while (i < seg.length - 2 && seg[i + 1] < d) i++;
  const a = pts[i], b = pts[i + 1];
  const t = (d - seg[i]) / Math.max(seg[i + 1] - seg[i], 1e-9);
  return { lat: a.lat + (b.lat - a.lat) * t, lon: a.lon + (b.lon - a.lon) * t, hdg: heading(a, b) };
}

// Build a closed rectangle (like the sample): 13 cols bottom->top, top right->left, etc.
const c = { lat: 10.7718, lon: 106.6982 }, dLat = 0.012, dLon = 0.018, n = 12, pts = [];
const push = (la, lo) => pts.push({ lat: la, lon: lo });
for (let i = 0; i <= n; i++) push(c.lat - dLat + (2 * dLat) * i / n, c.lon - dLon);
for (let i = 1; i <= n; i++) push(c.lat + dLat, c.lon - dLon + (2 * dLon) * i / n);
for (let i = 1; i <= n; i++) push(c.lat + dLat - (2 * dLat) * i / n, c.lon + dLon);
for (let i = 1; i < n; i++) push(c.lat - dLat, c.lon + dLon - (2 * dLon) * i / n);
const st = buildSim(pts);
console.log("route pts:", pts.length, " total length:", st.total.toFixed(0), "m");

// drive at 30 km/h * 30 = 900 km/h = 250 m/s; show first ticks + samples around loop
console.log("--- first 5 seconds (250 m/s) ---");
for (let s = 0; s < 5; s++) { const p = simAt(st, s * 250); console.log(`  s=${s} lat=${p.lat.toFixed(5)} lon=${p.lon.toFixed(5)} hdg=${p.hdg}`); }
console.log("--- 8 samples around the loop (heading should be 90/0/270/180-ish) ---");
for (let k = 0; k < 8; k++) { const p = simAt(st, st.total * k / 8); console.log(`  ${(100 * k / 8).toFixed(0)}% lat=${p.lat.toFixed(4)} lon=${p.lon.toFixed(4)} hdg=${p.hdg}`); }
