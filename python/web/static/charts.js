// DroneGuard - small helpers and a canvas line chart, shared by both pages.
// No libraries: the charts are a few lines each and must redraw 10x a second.

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (x, d = 0) => Number(x).toLocaleString("en-IN", { maximumFractionDigits: d, minimumFractionDigits: d });

function legend(id, items, title) {
  $(id).innerHTML = `<span style="color:var(--ink);font-weight:600">${title}</span>` +
    items.map(([l, c, dash]) => `<span><i class="${dash ? "dash" : ""}" style="${dash ? `border-color:${c}` : `background:${c}`}"></i>${l}</span>`).join("");
}

// series: [{x, y, color, width?, dash?, alpha?}]   o: {yMin, yMax, xMax, lines: [{y, label}], marks: [{x, label}], xlabel}
function chart(id, series, o = {}) {
  const cv = $(id), dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, Hh = cv.clientHeight;
  if (!W || !Hh) return;
  cv.width = W * dpr; cv.height = Hh * dpr;
  const ctx = cv.getContext("2d"); ctx.scale(dpr, dpr); ctx.clearRect(0, 0, W, Hh);
  const pad = { l: 44, r: 12, t: 10, b: o.xlabel ? 34 : 24 };
  const xs = series.flatMap((s) => s.x), ys = series.flatMap((s) => s.y);
  const xMax = o.xMax ?? (Math.max(...xs, 1) || 1);
  let yMin = o.yMin ?? Math.min(0, ...ys);
  let yMax = o.yMax ?? Math.max(...ys, ...(o.lines || []).map((l) => l.y), 1) * 1.08;
  if (yMax <= yMin) yMax = yMin + 1;
  const X = (x) => pad.l + (x / xMax) * (W - pad.l - pad.r);
  const Y = (y) => pad.t + (1 - (y - yMin) / (yMax - yMin)) * (Hh - pad.t - pad.b);
  const ink = css("--muted"), grid = css("--grid");
  ctx.font = `11px ${css("--mono")}`; ctx.fillStyle = ink; ctx.strokeStyle = grid; ctx.lineWidth = 1;
  for (const t of ticks(yMin, yMax, 5)) {
    ctx.beginPath(); ctx.moveTo(pad.l, Y(t)); ctx.lineTo(W - pad.r, Y(t)); ctx.stroke();
    ctx.textAlign = "right"; ctx.textBaseline = "middle"; ctx.fillText(fmt(t, t % 1 ? 1 : 0), pad.l - 6, Y(t));
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (const t of ticks(0, xMax, Math.max(3, Math.floor(W / 90)))) ctx.fillText(fmt(t), X(t), Hh - pad.b + 6);
  if (o.xlabel) ctx.fillText(o.xlabel, (pad.l + W - pad.r) / 2, Hh - 14);
  for (const l of o.lines || []) {
    ctx.save(); ctx.setLineDash([4, 4]); ctx.strokeStyle = ink;
    ctx.beginPath(); ctx.moveTo(pad.l, Y(l.y)); ctx.lineTo(W - pad.r, Y(l.y)); ctx.stroke(); ctx.restore();
    ctx.textAlign = "right"; ctx.textBaseline = "bottom"; ctx.fillText(l.label, W - pad.r - 2, Y(l.y) - 2);
  }
  for (const m of o.marks || []) {
    ctx.save(); ctx.strokeStyle = css("--nogo"); ctx.globalAlpha = 0.7;
    ctx.beginPath(); ctx.moveTo(X(m.x), pad.t); ctx.lineTo(X(m.x), Hh - pad.b); ctx.stroke(); ctx.restore();
    ctx.save(); ctx.fillStyle = css("--nogo"); ctx.textAlign = "left"; ctx.textBaseline = "top";
    ctx.fillText(m.label, X(m.x) + 4, pad.t); ctx.restore();
  }
  for (const s of series) {
    if (!s.x.length) continue;
    ctx.save(); ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 2; ctx.lineJoin = "round";
    ctx.globalAlpha = s.alpha ?? 1;
    if (s.dash) ctx.setLineDash([6, 4]);
    // Never draw more than ~2 points per pixel - long live runs stay cheap.
    const step = Math.max(1, Math.floor(s.x.length / (2 * W)));
    ctx.beginPath();
    for (let i = 0; i < s.x.length; i += step) {
      i ? ctx.lineTo(X(s.x[i]), Y(s.y[i])) : ctx.moveTo(X(s.x[i]), Y(s.y[i]));
    }
    const j = s.x.length - 1; ctx.lineTo(X(s.x[j]), Y(s.y[j]));
    ctx.stroke(); ctx.restore();
  }
}

function ticks(lo, hi, n) {
  const span = hi - lo, raw = span / n, mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((k) => k * mag).find((s) => s >= raw);
  const out = []; for (let t = Math.ceil(lo / step) * step; t <= hi + 1e-9; t += step) out.push(+t.toFixed(6));
  return out;
}
