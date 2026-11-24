// Interfaz del Roomba de cuadrícula (sin dependencias externas)
'use strict';

const N = 6, CS = 50;                      // 6×6 celdas, 50 px por celda en el SVG
const HEAD = ['N', 'E', 'S', 'O'];
const HEAD_ANGLE = { N: 90, E: 0, S: -90, O: 180 };
const COLORS = ['#1f6feb', '#d63384', '#1a7f37', '#b26b00', '#6f42c1', '#0aa2c0', '#c62828', '#5c6b00', '#8a4baf', '#2b7a78', '#a0522d', '#4a6fa5', '#777'];
const $ = (id) => document.getElementById(id);

const st = {
  tool: 'inicio',
  start: { f: 0, c: 0, h: 'E' },
  obst: new Set(),
  goals: [],
  plan: null,
  tele: null,
  ws: null,
  anim: null,
  expCells: new Set(),
  history: loadHistory(),
  runT0: 0,
  prevEstado: '',
};

// ---------------------------------------------------------------- utilidades
const key = (f, c) => f + ',' + c;
const cx = (c) => c * CS + CS / 2;
const cy = (f) => (N - 1 - f) * CS + CS / 2;

function toast(msg, ms = 2600) {
  const t = $('toast');
  t.textContent = msg;
  t.hidden = false;
  clearTimeout(toast.h);
  toast.h = setTimeout(() => (t.hidden = true), ms);
}

async function api(path, body) {
  const opt = body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  };
  try {
    const r = await fetch(path, opt);
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) throw new Error(j.error || ('HTTP ' + r.status));
    return j;
  } catch (e) {
    toast('Error: ' + e.message, 4000);
    throw e;
  }
}

function loadHistory() {
  try { return JSON.parse(localStorage.getItem('historial') || '[]'); } catch { return []; }
}
function saveHistory() {
  try { localStorage.setItem('historial', JSON.stringify(st.history)); } catch { /* sin almacenamiento */ }
}

// ---------------------------------------------------------------- dibujo
const svg = $('grid');
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs, parent = svg) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  parent.appendChild(e);
  return e;
}

function draw() {
  svg.textContent = '';
  for (let f = 0; f < N; f++) {
    for (let c = 0; c < N; c++) {
      const k = key(f, c);
      let cls = 'cell';
      if (st.obst.has(k)) cls += ' obst';
      else if (st.expCells.has(k)) cls += ' exp';
      const r = el('rect', { x: c * CS, y: (N - 1 - f) * CS, width: CS, height: CS, class: cls });
      r.addEventListener('click', () => onCell(f, c));
    }
  }
  for (let i = 0; i < N; i++) {
    el('text', { x: cx(i), y: N * CS + 15, class: 'lbl' }).textContent = 'c' + i;
    el('text', { x: -14, y: cy(i) + 4, class: 'lbl' }).textContent = 'f' + i;
  }
  el('text', { x: N * CS / 2, y: -1, class: 'lbl' }).textContent = 'N ↑';

  // Rutas del plan
  const p = st.plan;
  if (p && p.ok && p.rutas) {
    p.rutas.forEach((ruta, i) => {
      if (ruta.length < 2) return;
      const off = (i % 3 - 1) * 3;  // separa tramos que se enciman
      const pts = ruta.map(([f, c]) => (cx(c) + off) + ',' + (cy(f) + off)).join(' ');
      el('polyline', { points: pts, class: 'route', stroke: COLORS[i % COLORS.length] });
    });
  }

  // Metas (número = orden en que las marcaste; el orden de visita va en la tabla)
  const unreach = new Set((p && p.inalcanzables) || []);
  st.goals.forEach(([f, c], i) => {
    el('circle', { cx: cx(c), cy: cy(f), r: 17, class: 'goal' + (unreach.has(i) ? ' unreach' : '') });
    el('text', { x: cx(c), y: cy(f), class: 'goaltxt' }).textContent = i + 1;
  });

  // Inicio con flecha de orientación
  const a = HEAD_ANGLE[st.start.h];
  const g = el('g', { transform: `translate(${cx(st.start.c)},${cy(st.start.f)}) rotate(${-a})` });
  el('polygon', { points: '-14,-11 16,0 -14,11 -7,0', class: 'start' }, g);

  // Robot en su pose estimada
  const t = st.tele;
  if (t && typeof t.x === 'number') {
    const rx = t.x * 100, ry = N * CS - t.y * 100;
    const rg = el('g', { transform: `translate(${rx},${ry}) rotate(${-t.theta})` });
    el('circle', { cx: 0, cy: 0, r: 13, class: 'robot' }, rg);
    el('polygon', { points: '4,-6 14,0 4,6', fill: '#fff' }, rg);
  }
}

function onCell(f, c) {
  const k = key(f, c);
  const gi = st.goals.findIndex(([a, b]) => a === f && b === c);
  switch (st.tool) {
    case 'inicio':
      if (st.start.f === f && st.start.c === c) {
        st.start.h = HEAD[(HEAD.indexOf(st.start.h) + 1) % 4];  // N → E → S → O
      } else {
        st.start = { f, c, h: st.start.h };
        st.obst.delete(k);
      }
      break;
    case 'obstaculo':
      if (st.start.f === f && st.start.c === c) return toast('El inicio no puede ser obstáculo');
      if (st.obst.has(k)) st.obst.delete(k);
      else { st.obst.add(k); if (gi >= 0) st.goals.splice(gi, 1); }
      break;
    case 'meta':
      if (gi >= 0) st.goals.splice(gi, 1);
      else if (st.obst.has(k)) return toast('Esa celda es obstáculo');
      else if (st.goals.length >= 12) return toast('Máximo 12 metas');
      else st.goals.push([f, c]);
      break;
    case 'borrar':
      st.obst.delete(k);
      if (gi >= 0) st.goals.splice(gi, 1);
      break;
  }
  st.expCells.clear();
  draw();
}

document.querySelectorAll('.tool').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.tool').forEach((x) => x.classList.remove('on'));
  b.classList.add('on');
  st.tool = b.dataset.tool;
}));

// ---------------------------------------------------------------- plan
function obstaclesArray() {
  return [...st.obst].map((k) => k.split(',').map(Number));
}

function showPlan(p) {
  st.plan = p;
  const info = $('planInfo');
  if (!p || (!p.ok && !p.error)) { draw(); return; }
  if (!p.ok) {
    info.textContent = 'No se pudo planear: ' + (p.error || '');
    info.className = 'info bad';
  } else {
    const nombres = { astar: 'A*', bfs: 'BFS', dfs: 'DFS', ucs: 'UCS' };
    let txt = `Plan con ${nombres[p.algoritmo] || p.algoritmo}` + (p.exacto ? ' (orden óptimo, Held–Karp).' : ' (orden por vecino más cercano + 2-opt).');
    if (p.inalcanzables && p.inalcanzables.length)
      txt += ` Metas inalcanzables: ${p.inalcanzables.map((i) => i + 1).join(', ')}. Al ejecutar se pedirá confirmar.`;
    info.textContent = txt;
    info.className = 'info' + (p.inalcanzables && p.inalcanzables.length ? ' bad' : '');
  }
  $('mOrden').textContent = p.orden ? p.orden.map((i) => i + 1).join(' → ') + (p.regresar ? ' → inicio' : '') : '—';
  $('mCeldas').textContent = p.celdas ?? '—';
  $('mGiros').textContent = p.giros ?? '—';
  $('mCosto').textContent = p.costo != null ? Number(p.costo).toFixed(2) : '—';
  $('mExp').textContent = p.expandidos != null ? `${p.expandidos} en los tramos (+${p.expandidos_matriz} en la matriz de costos)` : '—';
  $('mTiempo').textContent = p.tiempo_us != null ? (p.tiempo_us / 1000).toFixed(2) + ' ms' : '—';
  $('mPrim').textContent = p.primitivas && p.primitivas.length ? p.primitivas.join(', ') : '—';
  draw();
}

function addHistory(p) {
  if (!p || !p.ok) return;
  st.history.push({
    alg: p.algoritmo, metas: p.metas.length, celdas: p.celdas, giros: p.giros,
    costo: +Number(p.costo).toFixed(2), exp: p.expandidos, us: p.tiempo_us, ejec: null,
  });
  saveHistory();
  renderHistory();
}

function renderHistory() {
  const tb = $('hist').querySelector('tbody');
  tb.textContent = '';
  st.history.forEach((h, i) => {
    const tr = document.createElement('tr');
    [i + 1, h.alg, h.metas, h.celdas, h.giros, h.costo, h.exp, h.us, h.ejec ?? '—'].forEach((v) => {
      const td = document.createElement('td');
      td.textContent = v;
      tr.appendChild(td);
    });
    tb.appendChild(tr);
  });
}

let waitingPlan = false;
$('btnPlan').addEventListener('click', async () => {
  if (!st.goals.length) return toast('Marca al menos una meta');
  waitingPlan = true;
  $('planInfo').textContent = 'Planeando…';
  $('planInfo').className = 'info';
  try {
    await api('/api/plan', {
      algoritmo: $('alg').value,
      heuristica: $('heur').value,
      inicio: [st.start.f, st.start.c],
      orientacion: st.start.h,
      metas: st.goals,
      regresar: $('regresar').checked,
      obstaculos: obstaclesArray(),
    });
    // Respaldo por si el WebSocket no está conectado
    setTimeout(async () => {
      if (!waitingPlan) return;
      try { onPlanMessage(await api('/api/plan')); } catch { /* ya se avisó */ }
    }, 800);
  } catch { waitingPlan = false; }
});

function onPlanMessage(p) {
  showPlan(p);
  if (waitingPlan) { waitingPlan = false; addHistory(p); }
}

$('btnEjecutar').addEventListener('click', async () => {
  const p = st.plan;
  let confirmar = false;
  if (p && p.inalcanzables && p.inalcanzables.length) {
    confirmar = window.confirm('Hay metas inalcanzables. ¿Continuar sin ellas?');
    if (!confirmar) return;
  }
  await api('/api/ejecutar', { confirmar });
  st.runT0 = performance.now();
  toast('Ejecutando');
});

$('btnPausa').addEventListener('click', async () => { const r = await api('/api/pausa', {}); toast(r.mensaje); });

function paro() {
  try { if (st.ws && st.ws.readyState === 1) st.ws.send('{"tipo":"paro"}'); } catch { /* sigue por HTTP */ }
  fetch('/api/paro', { method: 'POST' }).catch(() => toast('¡No se pudo enviar el paro! Usa el botón físico.', 6000));
  toast('PARO enviado');
}
$('btnParo').addEventListener('click', paro);
document.addEventListener('keydown', (e) => { if (e.code === 'Space' && e.target === document.body) { e.preventDefault(); paro(); } });

$('btnAnimar').addEventListener('click', () => {
  const p = st.plan;
  if (!p || !p.expandidos_orden) return toast('Primero planea');
  clearInterval(st.anim);
  const seq = p.expandidos_orden.flat();
  st.expCells.clear();
  let i = 0;
  st.anim = setInterval(() => {
    if (i >= seq.length) { clearInterval(st.anim); return; }
    const idx = seq[i++];
    st.expCells.add(key(Math.floor(idx / N), idx % N));
    draw();
  }, 40);
});

$('alg').addEventListener('change', () => { $('heurBox').style.display = $('alg').value === 'astar' ? '' : 'none'; });

$('btnLimpiarMetas').addEventListener('click', () => { st.goals = []; draw(); });
$('btnLimpiarObs').addEventListener('click', () => { st.obst.clear(); draw(); });
$('btnGuardarMapa').addEventListener('click', async () => {
  await api('/api/mapa', { obstaculos: obstaclesArray() });
  toast('Mapa guardado en el robot');
});

$('btnCSV').addEventListener('click', () => {
  const rows = [['n', 'algoritmo', 'metas', 'celdas', 'giros', 'costo', 'expandidos', 'tiempo_us', 'ejecucion_s']];
  st.history.forEach((h, i) => rows.push([i + 1, h.alg, h.metas, h.celdas, h.giros, h.costo, h.exp, h.us, h.ejec ?? '']));
  const blob = new Blob([rows.map((r) => r.join(',')).join('\n')], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'historial-roomba.csv';
  a.click();
  URL.revokeObjectURL(a.href);
});
$('btnBorrarHist').addEventListener('click', () => { st.history = []; saveHistory(); renderHistory(); });

// ---------------------------------------------------------------- pruebas
document.querySelectorAll('[data-prueba]').forEach((b) => b.addEventListener('click', async () => {
  await api('/api/prueba', JSON.parse(b.dataset.prueba));
  toast('Prueba: ' + b.textContent);
}));
$('btnPWM').addEventListener('click', async () => {
  await api('/api/prueba', { tipo: 'pwm', izq: +$('pwmL').value, der: +$('pwmR').value, ms: +$('pwmMs').value });
  toast('Prueba de PWM en curso');
});
$('btnCero').addEventListener('click', async () => {
  await api('/api/imu/cero', { celda: [st.start.f, st.start.c], orientacion: st.start.h });
  toast(`Robot colocado en f${st.start.f} c${st.start.c} mirando al ${st.start.h}`);
});
$('btnBarredora').addEventListener('click', async () => {
  const r = await api('/api/barredora', { encendida: !(st.tele && st.tele.barredora) });
  toast(r.mensaje);
});
$('btnCalibrar').addEventListener('click', async () => { const r = await api('/api/calibrar', { tipo: 'imu' }); toast(r.mensaje, 6000); });

// ---------------------------------------------------------------- ajustes
const CFG_LABELS = {
  cuentas_por_vuelta: 'Cuentas por vuelta', radio_rueda: 'Radio rueda (m)', separacion_ruedas: 'Separación ruedas (m)',
  rueda_kp: 'Rueda Kp', rueda_ki: 'Rueda Ki', rueda_kff: 'Rueda Kff', pwm_min: 'PWM mínimo',
  rumbo_k: 'Kθ rumbo', lateral_k: 'Ky lateral', giro_kp: 'Giro Kp', giro_kd: 'Giro Kd',
  v_max: 'v máx (m/s)', aceleracion: 'Aceleración (m/s²)', v_aproximacion: 'v aproximación (m/s)',
  giro_w_max: 'ω máx giro (°/s)', giro_alfa: 'α giro (°/s²)', costo_giro: 'Costo de giro c_g', beta_giro: 'β giroscopio (0 = solo encoders)',
};
function fillConfig(c) {
  const form = $('cfg');
  form.textContent = '';
  for (const k in c) {
    if (k === 'ok') continue;
    const lab = document.createElement('label');
    lab.textContent = CFG_LABELS[k] || (k === 'paro_sin_latido' ? 'Frenar si se pierde la conexión' : k === 'barredora_auto' ? 'Barredora automática al ejecutar' : k);
    const inp = document.createElement('input');
    inp.name = k;
    if (typeof c[k] === 'boolean') { inp.type = 'checkbox'; inp.checked = c[k]; lab.className = 'check'; }
    else { inp.type = 'number'; inp.step = 'any'; inp.value = +Number(c[k]).toPrecision(5); }
    lab.appendChild(inp);
    form.appendChild(lab);
  }
}
async function loadConfig() { try { fillConfig(await api('/api/config')); } catch { /* sin robot */ } }
$('btnCfgGuardar').addEventListener('click', async (e) => {
  e.preventDefault();
  const body = {};
  $('cfg').querySelectorAll('input').forEach((i) => { body[i.name] = i.type === 'checkbox' ? i.checked : +i.value; });
  fillConfig(await api('/api/config', body));
  toast('Ajustes guardados');
});
$('btnCfgReset').addEventListener('click', async (e) => {
  e.preventDefault();
  if (!window.confirm('¿Volver a los valores de roomba_config.h?')) return;
  fillConfig(await api('/api/config', { restablecer: true }));
});
$('btnWifi').addEventListener('click', async () => {
  await api('/api/wifi', { modo: $('wifiModo').value, ssid: $('wifiSsid').value, clave: $('wifiClave').value });
  toast('Reiniciando el robot…', 5000);
});

// ---------------------------------------------------------------- telemetría
const fmt = (v, d = 2) => (typeof v === 'number' ? v.toFixed(d) : '—');

function onTelemetry(t) {
  st.tele = t;
  $('estado').textContent = t.estado;
  $('estado').className = 'chip ' + ({ EJECUTANDO: 'on', FALLA: 'off', PAUSADO: 'warn', PLANEANDO: 'warn' }[t.estado] || '');
  $('tEstado').textContent = t.estado;
  $('tMensaje').textContent = t.mensaje || '—';
  $('tPrim').textContent = t.primitiva;
  $('tPaso').textContent = t.pasos ? `${Math.min(t.paso + 1, t.pasos)} de ${t.pasos} · metas: ${t.metas_alcanzadas}` : '—';
  $('tProg').value = t.progreso || 0;
  $('tXY').textContent = `${fmt(t.x)} m, ${fmt(t.y)} m`;
  $('tTh').textContent = fmt(t.theta, 1) + '°';
  $('tCelda').textContent = t.celda ? `f${t.celda[0]} c${t.celda[1]}` : '—';
  $('tMov').textContent = t.movimiento + (t.motores_activos ? ' · motores activos' : ' · rueda libre') +
    (t.barredora ? ' · barredora ON' : '');
  $('btnBarredora').textContent = t.barredora ? 'Apagar barredora' : 'Probar barredora';
  $('tV').textContent = `${fmt(t.v_izq)} / ${fmt(t.v_der)} m/s`;
  $('tTicks').textContent = t.cuentas ? t.cuentas.join(' / ') : '—';
  $('tSlip').textContent = `${t.deslizamientos} · frenado ${fmt(t.dist_frenado_mm, 1)} mm`;
  const c = t.calib || {};
  $('tCal').textContent = (t.imu_ok ? '' : 'SIN DATOS · ') +
    `b_z ${fmt(c.sesgo_dps, 3)} °/s · ω ${fmt(t.omega, 1)} °/s · ` +
    (c.calibrando ? 'calibrando…' : c.resultado === 1 ? 'calibrado' : c.resultado === -1 ? 'se movió, repite' :
      c.calibrado ? `al arrancar: ${c.arranque}` : 'SIN CALIBRAR');
  $('tRed').textContent = t.red;

  // Tiempo real de ejecución para el historial
  if (st.prevEstado === 'EJECUTANDO' && t.estado === 'COMPLETADO' && st.runT0 && st.history.length) {
    const last = st.history[st.history.length - 1];
    if (last.ejec == null) { last.ejec = +((performance.now() - st.runT0) / 1000).toFixed(1); saveHistory(); renderHistory(); }
    st.runT0 = 0;
  }
  st.prevEstado = t.estado;
  draw();
}

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  st.ws = ws;
  ws.onopen = () => { $('conn').textContent = 'conectado'; $('conn').className = 'chip on'; };
  ws.onclose = () => {
    $('conn').textContent = 'sin conexión';
    $('conn').className = 'chip off';
    setTimeout(connect, 1500);
  };
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.tipo === 'telemetria') onTelemetry(m);
    else if (m.tipo === 'plan') onPlanMessage(m);
    else if (m.tipo === 'evento') toast('⚠ ' + m.texto, 6000);
  };
}
// Latido cada segundo: si el robot deja de recibirlo 2 s durante una misión, frena
setInterval(() => { if (st.ws && st.ws.readyState === 1) st.ws.send('{"tipo":"latido"}'); }, 1000);

// ---------------------------------------------------------------- arranque
async function init() {
  renderHistory();
  draw();
  try {
    const m = await api('/api/mapa');
    (m.obstaculos || []).forEach(([f, c]) => st.obst.add(key(f, c)));
    const p = await api('/api/plan');
    if (p.ok) {
      st.start = { f: p.inicio[0], c: p.inicio[1], h: p.orientacion };
      st.goals = p.metas;
      showPlan(p);
    }
  } catch { /* sin robot todavía */ }
  draw();
  loadConfig();
  connect();
}
init();
