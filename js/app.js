/* Newton's Laws Simulator: user interface, animation and diagrams. */
(function () {
  'use strict';

  const P = window.Physics;
  const DEG = Math.PI / 180;
  const $ = (id) => document.getElementById(id);

  /* ======================================================================
   * State
   * ==================================================================== */

  const G_OPTIONS = [
    [9.8, 'Earth, 9.8 m/s²'],
    [9.81, 'Earth, 9.81 m/s²'],
    [10, 'Earth, rounded to 10 m/s²'],
    [1.62, 'Moon, 1.62 m/s²'],
    [3.71, 'Mars, 3.71 m/s²'],
    [24.79, 'Jupiter, 24.79 m/s²'],
  ];
  const MAX_BLOCKS = { modified: 3, incline: 3, horizontal: 4 };
  const SCENARIOS = ['atwood', 'modified', 'incline', 'horizontal'];

  const blk = (m) => ({ m, mus: 0.4, muk: 0.3 });
  function surfaceDefaults(o) {
    return Object.assign(
      {
        theta: 0,
        blocks: [blk(4)],
        links: [],
        friction: false,
        sameMu: true,
        mus: 0.4,
        muk: 0.3,
        hanging: { on: false, m: 2 },
        applied: { on: false, F: 20, angle: 0, block: 0 },
      },
      o
    );
  }
  function defaults() {
    return {
      scenario: 'atwood',
      g: 9.8,
      atwood: { m1: 3, m2: 5 },
      modified: surfaceDefaults({ blocks: [blk(4)], hanging: { on: true, m: 2 } }),
      incline: surfaceDefaults({ theta: 30, blocks: [blk(5)], hanging: { on: false, m: 3 } }),
      horizontal: surfaceDefaults({
        blocks: [blk(2), blk(3)],
        links: ['contact'],
        applied: { on: true, F: 20, angle: 0, block: 0 },
      }),
    };
  }

  let state = defaults();
  let analysis = null; // physics result
  let sim = null; // geometry + motion for the current analysis
  let labels = null; // names for link forces
  const anim = { t: 0, playing: false, done: false, last: 0 };
  const ui = {
    tab: 'check',
    graph: 'x',
    track: 0,
    strobe: false,
    speed: 1,
    showFbdValues: false,
    fbdSameScale: false,
    solutionOpen: false,
    revealed: false,
    pred: {},
    checkSig: '',
  };

  const cur = () => state[state.scenario];
  const isAtwood = () => state.scenario === 'atwood';

  /* ======================================================================
   * Formatting helpers
   * ==================================================================== */

  const SUBS = '₀₁₂₃₄₅₆₇₈₉';
  const sub = (n) =>
    String(n)
      .split('')
      .map((d) => SUBS[+d] || d)
      .join('');
  function fmt(x, d = 2) {
    if (!isFinite(x)) return '—';
    const s = (Math.abs(x) < 0.5 * Math.pow(10, -d) ? 0 : x).toFixed(d);
    return s.replace('-', '−');
  }
  const signed = (x) => (x < 0 ? ' − ' + fmt(-x) : ' + ' + fmt(x));
  const trimNum = (x) => String(Math.round(x * 1000) / 1000);
  const esc = (s) =>
    String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  function blockSpan(s, e, bodies) {
    if (s === e) return bodies[s].name;
    return 'Blocks ' + (s + 1) + '–' + (e + 1);
  }

  function dirWords() {
    const sc = state.scenario;
    const hang = analysis && analysis.kind === 'chain' && analysis.bodies.some((b) => b.kind === 'hanging');
    if (sc === 'incline') return { pos: 'up the incline', neg: 'down the incline', hang };
    return { pos: hang ? 'to the right (toward the pulley)' : 'to the right', neg: 'to the left', hang };
  }

  /* ======================================================================
   * Building the physics input
   * ==================================================================== */

  function physicsInput() {
    const sc = state.scenario;
    const c = cur();
    const blocks = c.blocks.map((b) => {
      const mus = c.sameMu ? c.mus : b.mus;
      const muk = Math.min(c.sameMu ? c.muk : b.muk, mus);
      return { m: b.m, mus, muk };
    });
    const hang = sc === 'modified' || (sc === 'incline' && c.hanging.on);
    return {
      theta: sc === 'incline' ? c.theta : 0,
      g: state.g,
      friction: c.friction,
      blocks,
      links: c.links.slice(),
      hanging: hang ? { m: c.hanging.m } : null,
      applied: c.applied.on
        ? { F: c.applied.F, angle: c.applied.angle, block: Math.min(c.applied.block, blocks.length - 1) }
        : null,
    };
  }

  function computeLabels(an) {
    if (an.kind === 'atwood') return { links: [{ onLeft: 'T', onRight: 'T', name: 'T' }] };
    const nStrings = an.links.filter((l) => l.type === 'string').length;
    let k = 0;
    const links = an.links.map((l) => {
      const L = l.left + 1;
      const R = l.right + 1;
      if (l.type === 'string') {
        k++;
        const T = nStrings > 1 ? 'T' + sub(k) : 'T';
        return { onLeft: T, onRight: T, name: T };
      }
      return {
        onLeft: 'F' + sub(R) + '→' + sub(L),
        onRight: 'F' + sub(L) + '→' + sub(R),
        name: 'F' + sub(L) + '→' + sub(R),
      };
    });
    return { links };
  }

  /* ======================================================================
   * Controls
   * ==================================================================== */

  function numField(label, path, value, unit, o = {}) {
    const id = 'in-' + state.scenario + '-' + path.replace(/\./g, '-');
    return (
      '<label class="field" for="' + id + '"><span>' + label +
      (unit ? ' <span class="unit">(' + unit + ')</span>' : '') +
      '</span><input type="number" id="' + id + '" data-path="' + path + '" value="' + trimNum(value) +
      '" min="' + (o.min ?? 0) + '" max="' + (o.max ?? 1000) + '" step="' + (o.step ?? 0.1) + '" inputmode="decimal"></label>'
    );
  }
  function checkField(label, path, checked) {
    const id = 'ck-' + state.scenario + '-' + path.replace(/\./g, '-');
    return (
      '<label class="check" for="' + id + '"><input type="checkbox" id="' + id + '" data-path="' + path + '"' +
      (checked ? ' checked' : '') + ' data-rerender="1"> ' + label + '</label>'
    );
  }

  function renderControls() {
    const sc = state.scenario;
    const c = cur();
    let h = '';

    if (sc === 'atwood') {
      h += '<div class="group"><h2>Masses</h2>';
      h += '<div class="blk"><span class="tag" style="background:var(--b1)">1</span>' +
        numField('Mass 1', 'm1', c.m1, 'kg', { min: 0.01, step: 0.1 }) + '<span></span></div>';
      h += '<div class="blk"><span class="tag" style="background:var(--b2)">2</span>' +
        numField('Mass 2', 'm2', c.m2, 'kg', { min: 0.01, step: 0.1 }) + '<span></span></div>';
      h += '<p class="note">Both masses hang from one string over the pulley.</p></div>';
    } else {
      if (sc === 'incline') {
        h += '<div class="group"><h2>Incline</h2>' +
          numField('Angle θ', 'theta', c.theta, '°', { min: 0, max: 85, step: 1 }) +
          '<input type="range" id="rng-theta" data-path="theta" min="0" max="85" step="1" value="' + c.theta + '" aria-label="Angle of the incline">' +
          '</div>';
      }

      const maxB = MAX_BLOCKS[sc];
      h += '<div class="group"><h2>' + (sc === 'modified' ? 'Blocks on the table' : 'Blocks') + '</h2>';
      h += '<p class="note">' +
        (sc === 'incline' ? 'Block 1 is lowest on the slope.' : 'Block 1 is on the left.') +
        ' Choose whether neighbors touch or are tied with a string.</p>';
      const perMu = c.friction && !c.sameMu;
      c.blocks.forEach((b, i) => {
        if (i > 0) {
          const t = c.links[i - 1];
          h += '<div class="link-row"><span class="rl">' + (i) + '–' + (i + 1) + '</span><div class="seg" role="group" aria-label="Connection between Block ' + i + ' and Block ' + (i + 1) + '">' +
            '<button type="button" data-action="link" data-index="' + (i - 1) + '" data-type="contact" class="' + (t === 'contact' ? 'on' : '') + '" aria-pressed="' + (t === 'contact') + '">Touching</button>' +
            '<button type="button" data-action="link" data-index="' + (i - 1) + '" data-type="string" class="' + (t === 'string' ? 'on' : '') + '" aria-pressed="' + (t === 'string') + '">String</button>' +
            '</div></div>';
        }
        h += '<div class="blk"><span class="tag" style="background:var(--b' + (i + 1) + ')">' + (i + 1) + '</span><div class="vals">' +
          '<label>Mass (kg)<input type="number" id="in-' + sc + '-b' + i + '-m" data-path="blocks.' + i + '.m" value="' + trimNum(b.m) + '" min="0.01" max="1000" step="0.1" inputmode="decimal"></label>' +
          (perMu
            ? '<label>μs<input type="number" id="in-' + sc + '-b' + i + '-mus" data-path="blocks.' + i + '.mus" value="' + trimNum(b.mus) + '" min="0" max="2" step="0.05"></label>' +
              '<label>μk<input type="number" id="in-' + sc + '-b' + i + '-muk" data-path="blocks.' + i + '.muk" value="' + trimNum(b.muk) + '" min="0" max="2" step="0.05"></label>'
            : '') +
          '</div><button type="button" class="icon-btn" data-action="remove" data-index="' + i + '" aria-label="Remove Block ' + (i + 1) + '"' + (c.blocks.length < 2 ? ' disabled' : '') + '>×</button></div>';
      });
      if (c.blocks.length < maxB) {
        h += '<div class="btn-row"><button type="button" class="btn btn-small" data-action="add">+ Add a block</button></div>';
      }
      h += '</div>';

      const nb = c.blocks.length;
      if (sc === 'modified') {
        h += '<div class="group"><h2>Hanging block</h2>' +
          '<div class="blk"><span class="tag" style="background:var(--b' + (nb + 1) + ')">' + (nb + 1) + '</span>' +
          numField('Block ' + (nb + 1) + ' mass', 'hanging.m', c.hanging.m, 'kg', { min: 0.01 }) + '<span></span></div>' +
          '<p class="note">Tied to Block ' + nb + ' by a string over the pulley at the table edge.</p></div>';
      } else if (sc === 'incline') {
        h += '<div class="group"><h2>Pulley</h2>' + checkField('Hang a block over a pulley at the top', 'hanging.on', c.hanging.on);
        if (c.hanging.on) {
          h += '<div class="blk"><span class="tag" style="background:var(--b' + (nb + 1) + ')">' + (nb + 1) + '</span>' +
            numField('Block ' + (nb + 1) + ' mass', 'hanging.m', c.hanging.m, 'kg', { min: 0.01 }) + '<span></span></div>';
        }
        h += '</div>';
      }

      h += '<div class="group"><h2>Friction</h2>' + checkField('Include friction with the surface', 'friction', c.friction);
      if (c.friction) {
        if (c.sameMu || nb < 2) {
          h += numField('Static coefficient μs', 'mus', c.mus, '', { min: 0, max: 2, step: 0.05 });
          h += numField('Kinetic coefficient μk', 'muk', c.muk, '', { min: 0, max: 2, step: 0.05 });
        }
        if (nb > 1) h += checkField('Different μ for each block', 'sameMuInv', !c.sameMu);
        const bad = c.sameMu ? c.muk > c.mus : c.blocks.some((b) => b.muk > b.mus);
        if (bad) h += '<p class="warn">μk should not be larger than μs. The simulator uses μk = μs where needed.</p>';
      }
      h += '</div>';

      h += '<div class="group"><h2>Applied force</h2>' + checkField('Push or pull with a force Fₐ', 'applied.on', c.applied.on);
      if (c.applied.on) {
        h += numField('Size of Fₐ', 'applied.F', c.applied.F, 'N', { min: 0, max: 10000, step: 1 });
        h += numField('Direction', 'applied.angle', c.applied.angle, '°', { min: -180, max: 180, step: 5 });
        h += '<label class="field" for="sel-applied-block"><span>Acts on</span><select id="sel-applied-block" data-path="applied.block">' +
          c.blocks.map((b, i) => '<option value="' + i + '"' + (c.applied.block === i ? ' selected' : '') + '>Block ' + (i + 1) + '</option>').join('') +
          '</select></label>';
        h += '<p class="note">0° points ' + (sc === 'incline' ? 'up the slope' : 'right') + ' along the surface and 180° points ' +
          (sc === 'incline' ? 'down the slope' : 'left') + '. Positive angles tilt Fₐ away from the surface; negative angles press it into the surface.</p>';
      }
      h += '</div>';
    }

    h += '<div class="group"><h2>Gravity</h2><label class="field wide" for="sel-g"><span>g</span><select id="sel-g" data-global="g">' +
      G_OPTIONS.map(([v, t]) => '<option value="' + v + '"' + (v === state.g ? ' selected' : '') + '>' + t + '</option>').join('') +
      '</select></label></div>';

    h += '<div class="group"><h2>Practice</h2><div class="btn-row">' +
      '<button type="button" class="btn btn-small" data-action="random">New random problem</button>' +
      '<button type="button" class="btn btn-small" data-action="share">Copy link to this setup</button>' +
      '<button type="button" class="btn btn-small btn-quiet" data-action="defaults">Start over</button>' +
      '</div><p class="note" id="share-msg" hidden></p></div>';

    $('controls').innerHTML = h;
  }

  function setPath(obj, path, value) {
    const parts = path.split('.');
    let o = obj;
    for (let i = 0; i < parts.length - 1; i++) o = o[parts[i]];
    o[parts[parts.length - 1]] = value;
  }
  function getPath(obj, path) {
    return path.split('.').reduce((o, k) => (o == null ? o : o[k]), obj);
  }

  function onControlInput(e) {
    const el = e.target;
    if (el.dataset.global === 'g') {
      state.g = parseFloat(el.value);
      changed();
      return;
    }
    const path = el.dataset.path;
    if (!path) return;
    const c = cur();
    if (el.type === 'checkbox') {
      if (path === 'sameMuInv') {
        c.sameMu = !el.checked;
        if (!c.sameMu) c.blocks.forEach((b) => { b.mus = c.mus; b.muk = c.muk; });
      } else setPath(c, path, el.checked);
      renderControls();
      changed();
      return;
    }
    if (el.tagName === 'SELECT') {
      setPath(c, path, parseInt(el.value, 10));
      changed();
      return;
    }
    const v = parseFloat(el.value);
    const min = parseFloat(el.min);
    const max = parseFloat(el.max);
    const positive = /\.m$|^m\d$|^m$/.test(path) || path === 'hanging.m';
    const ok = isFinite(v) && v >= min && v <= max && !(positive && v <= 0);
    el.setAttribute('aria-invalid', ok ? 'false' : 'true');
    if (!ok) return;
    setPath(c, path, v);
    if (path === 'theta') {
      const other = el.type === 'range' ? $('in-incline-theta') : $('rng-theta');
      if (other) other.value = v;
    }
    if (/mu[sk]$/.test(path) && e.type === 'change') renderControls();
    changed();
  }

  function onControlClick(e) {
    const btn = e.target.closest('[data-action]');
    if (!btn) return;
    const c = cur();
    const act = btn.dataset.action;
    if (act === 'add') {
      if (c.blocks.length >= MAX_BLOCKS[state.scenario]) return;
      const last = c.blocks[c.blocks.length - 1];
      c.blocks.push({ m: 2, mus: last ? last.mus : c.mus, muk: last ? last.muk : c.muk });
      c.links.push(state.scenario === 'horizontal' ? 'contact' : 'string');
    } else if (act === 'remove') {
      const i = +btn.dataset.index;
      if (c.blocks.length < 2) return;
      c.blocks.splice(i, 1);
      c.links.splice(Math.max(0, i - 1), 1);
      if (c.applied.block >= c.blocks.length) c.applied.block = c.blocks.length - 1;
    } else if (act === 'link') {
      c.links[+btn.dataset.index] = btn.dataset.type;
    } else if (act === 'random') {
      randomize();
    } else if (act === 'share') {
      share();
      return;
    } else if (act === 'defaults') {
      state[state.scenario] = defaults()[state.scenario];
      ui.pred = {};
    } else return;
    renderControls();
    changed();
  }

  function randomize() {
    const r = (a, b, step) => Math.round((a + Math.random() * (b - a)) / step) * step;
    const c = cur();
    if (isAtwood()) {
      c.m1 = r(1, 10, 0.5);
      do c.m2 = r(1, 10, 0.5); while (c.m2 === c.m1);
    } else {
      c.blocks.forEach((b) => {
        b.m = r(1, 10, 0.5);
        b.mus = r(0.2, 0.6, 0.05);
        b.muk = Math.max(0.05, Math.round((b.mus - r(0.05, 0.2, 0.05)) * 100) / 100);
      });
      c.hanging.m = r(1, 8, 0.5);
      if (state.scenario === 'incline') c.theta = r(15, 50, 5);
      c.mus = r(0.2, 0.6, 0.05);
      c.muk = Math.max(0.05, Math.round((c.mus - r(0.05, 0.2, 0.05)) * 100) / 100);
      if (c.applied.on) c.applied.F = r(10, 80, 5);
    }
    ui.pred = {};
  }

  /* ---------- share link ---------- */
  function encodeState() {
    const data = { v: 1, scenario: state.scenario, g: state.g, c: cur() };
    return btoa(unescape(encodeURIComponent(JSON.stringify(data))))
      .replace(/\+/g, '-')
      .replace(/\//g, '_')
      .replace(/=+$/, '');
  }
  function share() {
    const url = location.href.split('#')[0] + '#s=' + encodeState();
    const msg = $('share-msg');
    const done = (ok) => {
      msg.hidden = false;
      msg.innerHTML = ok
        ? 'Link copied. Anyone who opens it gets this exact setup.'
        : 'Copy this link: <input type="text" readonly id="share-url" value="' + esc(url) + '" style="width:100%">';
      if (!ok) {
        const inp = $('share-url');
        inp.focus();
        inp.select();
      }
    };
    try {
      navigator.clipboard.writeText(url).then(() => done(true), () => done(false));
    } catch (err) {
      done(false);
    }
  }
  function loadFromHash() {
    const m = /#s=([A-Za-z0-9_-]+)/.exec(location.hash);
    if (!m) return;
    try {
      const b64 = m[1].replace(/-/g, '+').replace(/_/g, '/');
      const data = JSON.parse(decodeURIComponent(escape(atob(b64))));
      if (!SCENARIOS.includes(data.scenario)) return;
      const base = defaults();
      const num = (v, d, lo, hi) => (typeof v === 'number' && isFinite(v) && v >= lo && v <= hi ? v : d);
      state.scenario = data.scenario;
      state.g = num(data.g, 9.8, 0.1, 100);
      const c = data.c || {};
      if (data.scenario === 'atwood') {
        state.atwood = { m1: num(c.m1, 3, 0.01, 1000), m2: num(c.m2, 5, 0.01, 1000) };
      } else {
        const d = base[data.scenario];
        const blocks = Array.isArray(c.blocks) && c.blocks.length
          ? c.blocks.slice(0, MAX_BLOCKS[data.scenario]).map((b) => ({
              m: num(b && b.m, 2, 0.01, 1000),
              mus: num(b && b.mus, 0.4, 0, 2),
              muk: num(b && b.muk, 0.3, 0, 2),
            }))
          : d.blocks;
        const links = blocks.slice(1).map((_, j) =>
          Array.isArray(c.links) && (c.links[j] === 'string' || c.links[j] === 'contact') ? c.links[j] : 'contact'
        );
        const ap = c.applied || {};
        const hg = c.hanging || {};
        state[data.scenario] = {
          theta: num(c.theta, d.theta, 0, 85),
          blocks,
          links,
          friction: !!c.friction,
          sameMu: c.sameMu !== false,
          mus: num(c.mus, 0.4, 0, 2),
          muk: num(c.muk, 0.3, 0, 2),
          hanging: { on: data.scenario === 'modified' ? true : !!hg.on, m: num(hg.m, 2, 0.01, 1000) },
          applied: {
            on: !!ap.on,
            F: num(ap.F, 20, 0, 10000),
            angle: num(ap.angle, 0, -180, 180),
            block: Math.min(Math.max(0, Math.round(num(ap.block, 0, 0, 10))), blocks.length - 1),
          },
        };
      }
    } catch (err) {
      /* ignore a damaged link and keep the defaults */
    }
  }

  /* ======================================================================
   * Recompute after any change
   * ==================================================================== */

  function changed() {
    analysis = isAtwood()
      ? P.solveAtwood({ m1: state.atwood.m1, m2: state.atwood.m2, g: state.g })
      : P.solveChain(physicsInput());
    labels = computeLabels(analysis);
    sim = buildSim(analysis);
    anim.t = 0;
    anim.playing = false;
    anim.done = false;
    ui.revealed = false;
    renderScenarioTabs();
    renderTrack();
    renderCheck();
    renderLegendAndPairs();
    renderSolution();
    updatePlayButton();
    updateStatus();
    drawAll();
  }

  /* ======================================================================
   * Simulation geometry and motion (closed form: constant accelerations)
   * ==================================================================== */

  const REST_RUN = 3; // seconds the clock runs when nothing moves
  const MAX_RUN = 20;

  function findStop(violation) {
    let prev = 0;
    const dt = 0.005;
    for (let t = dt; t <= MAX_RUN + 1e-9; t += dt) {
      const v = violation(t);
      if (v) {
        let lo = prev;
        let hi = t;
        for (let k = 0; k < 40; k++) {
          const mid = (lo + hi) / 2;
          if (violation(mid)) hi = mid;
          else lo = mid;
        }
        return { t: lo, reason: violation(hi) };
      }
      prev = t;
    }
    return { t: MAX_RUN, reason: null };
  }

  function buildSim(an) {
    if (!an.ok) return { error: true };
    return an.kind === 'atwood' ? buildAtwoodSim(an) : buildSurfaceSim(an);
  }

  function buildAtwoodSim(an) {
    const g = { pc: { x: 0, y: 3.0 }, r: 0.3, w: 0.44, h: 0.38, ceil: 3.36 };
    g.dMin = g.r + 0.06;
    g.dMax = g.pc.y - g.h;
    const mid = (g.dMin + g.dMax) / 2;
    if (an.a > 1e-12) {
      g.d2 = g.dMin + 0.06;
      g.d1 = g.dMax - 0.06;
    } else if (an.a < -1e-12) {
      g.d1 = g.dMin + 0.06;
      g.d2 = g.dMax - 0.06;
    } else {
      g.d1 = g.d2 = mid;
    }
    g.bbox = { x0: -1.2, x1: 1.05, y0: -0.12, y1: 3.55 };
    const s = (t) => 0.5 * an.a * t * t;
    const pos = (t) => ({ d1: g.d1 - s(t), d2: g.d2 + s(t) });
    const violation = (t) => {
      const p = pos(t);
      if (p.d1 < g.dMin) return 'Mass 1 reached the pulley';
      if (p.d2 < g.dMin) return 'Mass 2 reached the pulley';
      if (p.d1 > g.dMax) return 'Mass 1 hit the floor';
      if (p.d2 > g.dMax) return 'Mass 2 hit the floor';
      return null;
    };
    const stop = an.atRest ? { t: REST_RUN, reason: null } : findStop(violation);
    return { kind: 'atwood', geom: g, pos, tStop: stop.t, reason: stop.reason };
  }

  function buildSurfaceSim(an) {
    const th = an.theta * DEG;
    const u = { x: Math.cos(th), y: Math.sin(th) };
    const n = { x: -Math.sin(th), y: Math.cos(th) };
    const blocks = an.bodies.filter((b) => b.kind === 'block');
    const hangBody = an.bodies.find((b) => b.kind === 'hanging');
    const nb = blocks.length;
    const w = 0.42;
    const h = 0.3;
    const hw = 0.3;
    const strLen = 0.4;
    const gaps = an.links.filter((l) => !l.hanging).map((l) => (l.type === 'string' ? strLen : 0));
    const chain = nb * w + gaps.reduce((a, b) => a + b, 0);
    const Ls = chain + (an.theta > 0 ? 2.0 : 2.6);
    const H = hangBody ? (an.theta > 0 ? 1.0 : 1.4) : an.theta > 0 ? 0.25 : 0.5;
    const S0 = { x: 0, y: H };
    const E = { x: S0.x + Ls * u.x, y: S0.y + Ls * u.y };

    let mom = 0;
    for (const b of an.bodies) mom += b.m * b.a;
    const dir = Math.abs(mom) < 1e-12 ? 0 : Math.sign(mom);
    const margin = 0.06;
    const back0 = dir > 0 ? margin : dir < 0 ? Ls - chain - margin : (Ls - chain) / 2;
    const x0 = [];
    let cursor = back0;
    for (let i = 0; i < nb; i++) {
      x0.push(cursor + w / 2);
      cursor += w + (gaps[i] || 0);
    }

    const geom = { th, u, n, w, h, hw, strLen, gaps, Ls, H, S0, E, x0, nb, hang: !!hangBody };
    if (hangBody) {
      const rp = 0.11;
      const k = (0.09 + (h / 2 - rp) * Math.sin(th)) / Math.cos(th) + 0.01;
      const C = { x: E.x + u.x * k + n.x * (h / 2 - rp), y: E.y + u.y * k + n.y * (h / 2 - rp) };
      geom.rp = rp;
      geom.C = C;
      geom.T1 = { x: C.x + n.x * rp, y: C.y + n.y * rp };
      geom.T2 = { x: C.x + rp, y: C.y };
      geom.dMin = 0.06;
      geom.dMax = geom.T2.y - hw;
      geom.d0 = dir > 0 ? geom.dMin + 0.04 : dir < 0 ? geom.dMax - 0.02 : (geom.dMin + geom.dMax) / 2;
    }
    const right = Math.max(E.x, hangBody ? geom.T2.x + hw / 2 : 0) + 0.25;
    const top = Math.max(E.y + h + 0.25, hangBody ? geom.C.y + 0.3 : 0, 0.9);
    geom.bbox = { x0: -0.75, x1: right + 0.35, y0: -0.12, y1: top + 0.25 };

    const pos = (t) => {
      const xs = blocks.map((b, i) => x0[i] + 0.5 * b.a * t * t);
      const d = hangBody ? geom.d0 + 0.5 * hangBody.a * t * t : 0;
      return { xs, d };
    };
    const end = hangBody ? 'the pulley' : an.theta > 0 ? 'the top of the ramp' : 'the end of the track';
    const start = an.theta > 0 ? 'the bottom of the ramp' : 'the end of the track';
    const violation = (t) => {
      const p = pos(t);
      if (p.xs[0] - w / 2 < -1e-9) return 'Block 1 reached ' + start;
      if (p.xs[nb - 1] + w / 2 > Ls + 1e-9) return 'Block ' + nb + ' reached ' + end;
      for (let i = 0; i < nb - 1; i++) {
        if (p.xs[i + 1] - p.xs[i] < w - 1e-9) return 'Block ' + (i + 1) + ' ran into Block ' + (i + 2) + ' after the string between them went slack';
      }
      if (hangBody) {
        if (p.d < geom.dMin) return 'Block ' + (nb + 1) + ' reached the pulley';
        if (p.d > geom.dMax) return 'Block ' + (nb + 1) + ' hit the floor';
      }
      return null;
    };
    const stop = an.atRest ? { t: REST_RUN, reason: null } : findStop(violation);
    return { kind: 'surface', geom, pos, tStop: stop.t, reason: stop.reason };
  }

  // Displacement and velocity of a body along its own + direction.
  function bodyMotion(i, t) {
    const a = analysis.bodies[i].a;
    return { s: 0.5 * a * t * t, v: a * t, a };
  }

  /* ======================================================================
   * Colors from CSS tokens
   * ==================================================================== */

  let C = {};
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    const names = ['ink', 'muted', 'line', 'panel', 'scene', 'grid', 'bench', 'bench-edge', 'accent',
      'f-w', 'f-n', 'f-f', 'f-a', 'l1', 'l2', 'l3', 'l4', 'b1', 'b2', 'b3', 'b4', 'b5'];
    C = {};
    names.forEach((nm) => (C[nm] = cs.getPropertyValue('--' + nm).trim() || '#888'));
    C.fontBody = cs.getPropertyValue('--font-body').trim() || 'sans-serif';
    C.fontMono = cs.getPropertyValue('--font-mono').trim() || 'monospace';
  }
  const linkColor = (j) => C['l' + ((j % 4) + 1)];
  const bodyColor = (i) => C['b' + ((i % 5) + 1)];

  /* ======================================================================
   * Canvas helpers
   * ==================================================================== */

  function prepCanvas(canvas, cssW, cssH) {
    const dpr = window.devicePixelRatio || 1;
    const r = canvas.getBoundingClientRect();
    const W = cssW || r.width;
    const H = cssH || r.height;
    const pw = Math.max(1, Math.round(W * dpr));
    const ph = Math.max(1, Math.round(H * dpr));
    if (canvas.width !== pw || canvas.height !== ph) {
      canvas.width = pw;
      canvas.height = ph;
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    return { ctx, W, H };
  }

  function arrow(ctx, x1, y1, x2, y2, color, width = 2.5, head = 9, dashed = false) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const L = Math.hypot(dx, dy);
    if (L < 0.5) return;
    const ux = dx / L;
    const uy = dy / L;
    const hd = Math.min(head, L * 0.6);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = 'round';
    if (dashed) ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2 - ux * hd * 0.8, y2 - uy * hd * 0.8);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - ux * hd - uy * hd * 0.5, y2 - uy * hd + ux * hd * 0.5);
    ctx.lineTo(x2 - ux * hd + uy * hd * 0.5, y2 - uy * hd - ux * hd * 0.5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function fitView(bbox, W, H, pad = 16) {
    const bw = bbox.x1 - bbox.x0;
    const bh = bbox.y1 - bbox.y0;
    const k = Math.min((W - 2 * pad) / bw, (H - 2 * pad) / bh);
    const ox = (W - bw * k) / 2 - bbox.x0 * k;
    const oy = (H + bh * k) / 2 + bbox.y0 * k;
    return { k, P: (x, y) => [ox + x * k, oy - y * k] };
  }

  function drawGrid(ctx, W, H, view) {
    const step = 0.25 * view.k;
    if (step < 6) return;
    ctx.save();
    ctx.strokeStyle = C.grid;
    ctx.lineWidth = 1;
    const [ox, oy] = view.P(0, 0);
    ctx.beginPath();
    for (let x = ox % step; x < W; x += step) {
      ctx.moveTo(Math.round(x) + 0.5, 0);
      ctx.lineTo(Math.round(x) + 0.5, H);
    }
    for (let y = oy % step; y < H; y += step) {
      ctx.moveTo(0, Math.round(y) + 0.5);
      ctx.lineTo(W, Math.round(y) + 0.5);
    }
    ctx.stroke();
    ctx.restore();
  }

  function blockPath(ctx, view, c, u, n, w, h) {
    const pts = [
      [c.x - u.x * w / 2 - n.x * h / 2, c.y - u.y * w / 2 - n.y * h / 2],
      [c.x + u.x * w / 2 - n.x * h / 2, c.y + u.y * w / 2 - n.y * h / 2],
      [c.x + u.x * w / 2 + n.x * h / 2, c.y + u.y * w / 2 + n.y * h / 2],
      [c.x - u.x * w / 2 + n.x * h / 2, c.y - u.y * w / 2 + n.y * h / 2],
    ].map(([x, y]) => view.P(x, y));
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < 4; i++) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.closePath();
  }

  function drawBlock(ctx, view, c, u, n, w, h, idx, mass, ghost = false) {
    blockPath(ctx, view, c, u, n, w, h);
    ctx.save();
    if (ghost) {
      ctx.globalAlpha = 0.3;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1;
      ctx.stroke();
      const [px, py] = view.P(c.x, c.y);
      ctx.fillStyle = C.ink;
      ctx.beginPath();
      ctx.arc(px, py, 2.2, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      return;
    }
    ctx.fillStyle = bodyColor(idx);
    ctx.fill();
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.6;
    ctx.stroke();
    const [px, py] = view.P(c.x, c.y);
    const big = Math.max(10, Math.min(17, view.k * 0.12));
    ctx.translate(px, py);
    ctx.rotate(-Math.atan2(u.y, u.x));
    ctx.fillStyle = C.ink;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const roomy = w * view.k > 52 && h * view.k > 34;
    ctx.font = '700 ' + big + 'px ' + C.fontMono;
    ctx.fillText(String(idx + 1), 0, roomy ? -big * 0.45 : 0);
    if (roomy) {
      ctx.font = '400 ' + Math.round(big * 0.72) + 'px ' + C.fontMono;
      ctx.fillText(trimNum(mass) + ' kg', 0, big * 0.6);
    }
    ctx.restore();
  }

  /* ======================================================================
   * Scene
   * ==================================================================== */

  function drawScene() {
    const canvas = $('scene');
    const { ctx, W, H } = prepCanvas(canvas);
    ctx.fillStyle = C.scene;
    ctx.fillRect(0, 0, W, H);
    if (!sim || sim.error) {
      ctx.fillStyle = C.muted;
      ctx.font = '15px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.fillText('Fix the setup to see the motion.', W / 2, H / 2);
      return;
    }
    if (W < 40 || H < 40) return;
    const view = fitView(sim.geom.bbox, W, H);
    drawGrid(ctx, W, H, view);
    if (sim.kind === 'atwood') drawAtwood(ctx, view, anim.t);
    else drawSurface(ctx, view, anim.t);
  }

  function strobeTimes(t) {
    if (!ui.strobe) return [];
    const out = [];
    for (let k = 0; k * 0.1 <= t + 1e-9 && out.length < 250; k++) out.push(k * 0.1);
    return out;
  }

  function drawFloor(ctx, view, x0, x1) {
    const [a, y] = view.P(x0, 0);
    const [b] = view.P(x1, 0);
    ctx.save();
    ctx.strokeStyle = C['bench-edge'];
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(a, y);
    ctx.lineTo(b, y);
    ctx.stroke();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = a; x < b; x += 10) {
      ctx.moveTo(x, y + 1);
      ctx.lineTo(x - 6, y + 7);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawAtwood(ctx, view, t) {
    const g = sim.geom;
    const an = analysis;
    const P = view.P;
    const up = { x: 1, y: 0 };
    const nn = { x: 0, y: 1 };
    drawFloor(ctx, view, -1.1, 1.0);

    // meter stick
    ctx.save();
    const [sx, sy0] = P(-0.82, 0);
    const [, sy1] = P(-0.82, 3.0);
    ctx.fillStyle = C.panel;
    ctx.strokeStyle = C['bench-edge'];
    ctx.fillRect(sx - 4, sy1, 14, sy0 - sy1);
    ctx.strokeRect(sx - 4, sy1, 14, sy0 - sy1);
    ctx.fillStyle = C.muted;
    ctx.strokeStyle = C.muted;
    ctx.font = '10px ' + C.fontMono;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 30; i++) {
      const [, y] = P(0, i * 0.1);
      const len = i % 5 === 0 ? 9 : 5;
      ctx.beginPath();
      ctx.moveTo(sx + 10, y);
      ctx.lineTo(sx + 10 - len, y);
      ctx.stroke();
      if (i % 5 === 0) ctx.fillText((i / 10).toFixed(1), sx - 7, y);
    }
    ctx.textAlign = 'center';
    ctx.fillText('height (m)', sx + 3, sy1 - 16);
    ctx.restore();

    // ceiling and pulley
    const [cx0, cy] = P(-0.7, g.ceil);
    const [cx1] = P(0.7, g.ceil);
    ctx.save();
    ctx.fillStyle = C.bench;
    ctx.strokeStyle = C['bench-edge'];
    ctx.fillRect(cx0, cy - 8, cx1 - cx0, 8);
    ctx.strokeRect(cx0, cy - 8, cx1 - cx0, 8);
    const [px, py] = P(g.pc.x, g.pc.y);
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(px, cy);
    ctx.lineTo(px, py);
    ctx.stroke();
    const pos = sim.pos(t);
    // strings
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.arc(px, py, g.r * view.k, Math.PI, 0);
    const [l1x, l1y] = P(-g.r, g.pc.y);
    const [l1x2, l1y2] = P(-g.r, g.pc.y - pos.d1);
    ctx.moveTo(l1x, l1y);
    ctx.lineTo(l1x2, l1y2);
    const [r1x, r1y] = P(g.r, g.pc.y);
    const [r1x2, r1y2] = P(g.r, g.pc.y - pos.d2);
    ctx.moveTo(r1x, r1y);
    ctx.lineTo(r1x2, r1y2);
    ctx.stroke();
    // pulley wheel
    ctx.fillStyle = C.panel;
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(px, py, g.r * view.k - 2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = C.ink;
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();

    for (const ts of strobeTimes(t)) {
      const p = sim.pos(ts);
      drawBlock(ctx, view, { x: -g.r, y: g.pc.y - p.d1 - g.h / 2 }, up, nn, g.w, g.h, 0, 0, true);
      drawBlock(ctx, view, { x: g.r, y: g.pc.y - p.d2 - g.h / 2 }, up, nn, g.w, g.h, 1, 0, true);
    }
    drawBlock(ctx, view, { x: -g.r, y: g.pc.y - pos.d1 - g.h / 2 }, up, nn, g.w, g.h, 0, an.bodies[0].m);
    drawBlock(ctx, view, { x: g.r, y: g.pc.y - pos.d2 - g.h / 2 }, up, nn, g.w, g.h, 1, an.bodies[1].m);
  }

  function drawSurface(ctx, view, t) {
    const g = sim.geom;
    const an = analysis;
    const P = view.P;
    const { u, n, w, h } = g;
    const pos = sim.pos(t);
    const blocks = an.bodies.filter((b) => b.kind === 'block');
    const hangIdx = an.bodies.findIndex((b) => b.kind === 'hanging');

    drawFloor(ctx, view, g.bbox.x0 + 0.1, g.bbox.x1 - 0.1);

    // bench / ramp
    ctx.save();
    ctx.fillStyle = C.bench;
    ctx.strokeStyle = C['bench-edge'];
    ctx.lineWidth = 1.5;
    if (g.th === 0) {
      const slab = 0.08;
      const [ax, ay] = P(0, g.H);
      const [bx, by] = P(g.Ls, g.H - slab);
      ctx.fillRect(ax, ay, bx - ax, by - ay);
      ctx.strokeRect(ax, ay, bx - ax, by - ay);
      for (const lx of [0.12, g.Ls - 0.2]) {
        const [qx, qy] = P(lx, g.H - slab);
        const [rx, ry] = P(lx + 0.08, 0);
        ctx.fillRect(qx, qy, rx - qx, ry - qy);
        ctx.strokeRect(qx, qy, rx - qx, ry - qy);
      }
    } else {
      const pts = [P(0, 0), P(g.E.x, 0), P(g.E.x, g.E.y), P(g.S0.x, g.S0.y)];
      ctx.beginPath();
      ctx.moveTo(pts[0][0], pts[0][1]);
      pts.slice(1).forEach((p) => ctx.lineTo(p[0], p[1]));
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      // angle marker
      const [bx, by] = P(0, g.H);
      const rr = Math.min(70, 0.55 * view.k);
      ctx.strokeStyle = C.muted;
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(bx, by);
      ctx.lineTo(bx + rr * 1.5, by);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.arc(bx, by, rr, -g.th, 0);
      ctx.stroke();
      ctx.fillStyle = C.ink;
      ctx.font = '600 13px ' + C.fontMono;
      ctx.textBaseline = 'middle';
      // label sits inside the ramp, below the surface, so a block at the bottom never covers it
      const la = g.th * 0.35;
      ctx.textAlign = 'left';
      ctx.fillText('θ = ' + trimNum(an.theta) + '°', bx + (rr + 8) * Math.cos(la), by - (rr + 8) * Math.sin(la) + 8);
    }
    ctx.restore();

    // ruler along the surface
    ctx.save();
    ctx.strokeStyle = C.muted;
    ctx.fillStyle = C.muted;
    ctx.lineWidth = 1;
    ctx.font = '10px ' + C.fontMono;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const nTicks = Math.floor(g.Ls * 10 + 1e-9);
    for (let i = 0; i <= nTicks; i++) {
      const s = i * 0.1;
      const base = { x: g.S0.x + u.x * s, y: g.S0.y + u.y * s };
      const len = i % 10 === 0 ? 0.07 : i % 5 === 0 ? 0.05 : 0.025;
      const [ax, ay] = P(base.x, base.y);
      const [bx, by] = P(base.x - n.x * len, base.y - n.y * len);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      if (i % 5 === 0 && view.k > 60 && !(g.th > 0 && i < 10)) {
        const [tx, ty] = P(base.x - n.x * 0.14, base.y - n.y * 0.14);
        ctx.fillText(i % 10 === 0 ? i / 10 + (i === 0 ? '' : ' m') : (i / 10).toFixed(1), tx, ty);
      }
    }
    ctx.restore();

    // end stops
    ctx.save();
    ctx.fillStyle = C['bench-edge'];
    const stops = [0];
    if (!g.hang) stops.push(g.Ls);
    for (const s of stops) {
      const base = { x: g.S0.x + u.x * s, y: g.S0.y + u.y * s };
      const dirOut = s === 0 ? -1 : 1;
      const c = { x: base.x + u.x * dirOut * 0.03 + n.x * 0.06, y: base.y + u.y * dirOut * 0.03 + n.y * 0.06 };
      blockPath(ctx, view, c, u, n, 0.06, 0.12);
      ctx.fill();
    }
    ctx.restore();

    // pulley and hanging string
    if (g.hang) {
      const [cx, cy] = P(g.C.x, g.C.y);
      const [ex, ey] = P(g.E.x - n.x * 0.12, g.E.y - n.y * 0.12);
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(cx, cy);
      ctx.stroke();
      const last = blocks.length - 1;
      const front = { x: g.S0.x + u.x * (pos.xs[last] + w / 2) + n.x * h / 2, y: g.S0.y + u.y * (pos.xs[last] + w / 2) + n.y * h / 2 };
      const slack = !an.links[an.links.length - 1].engaged;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.6;
      if (slack) ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const [fx, fy] = P(front.x, front.y);
      const [t1x, t1y] = P(g.T1.x, g.T1.y);
      ctx.moveTo(fx, fy);
      ctx.lineTo(t1x, t1y);
      const angN = Math.atan2(n.y, n.x);
      ctx.arc(cx, cy, g.rp * view.k, -angN, 0, false);
      const [t2x, t2y] = P(g.T2.x, g.T2.y - pos.d);
      ctx.lineTo(t2x, t2y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = C.panel;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, g.rp * view.k - 1.5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = C.ink;
      ctx.beginPath();
      ctx.arc(cx, cy, 2.5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
    }

    // strings between blocks
    ctx.save();
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.6;
    an.links.forEach((l, j) => {
      if (l.hanging || l.type !== 'string') return;
      const a = pos.xs[j] + w / 2;
      const b = pos.xs[j + 1] - w / 2;
      const A = { x: g.S0.x + u.x * a + n.x * h / 2, y: g.S0.y + u.y * a + n.y * h / 2 };
      const B = { x: g.S0.x + u.x * b + n.x * h / 2, y: g.S0.y + u.y * b + n.y * h / 2 };
      const [ax, ay] = P(A.x, A.y);
      const [bx, by] = P(B.x, B.y);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      if (l.engaged) ctx.lineTo(bx, by);
      else {
        const sag = Math.max(0, g.strLen - (b - a)) * 0.5 + 0.05;
        const M = { x: (A.x + B.x) / 2 - n.x * sag, y: (A.y + B.y) / 2 - n.y * sag };
        const [mx, my] = P(M.x, M.y);
        ctx.quadraticCurveTo(mx, my, bx, by);
      }
      ctx.stroke();
    });
    ctx.restore();

    const centerOf = (s) => ({ x: g.S0.x + u.x * s + n.x * h / 2, y: g.S0.y + u.y * s + n.y * h / 2 });
    const hangCenter = (d) => ({ x: g.T2.x, y: g.T2.y - d - g.hw / 2 });
    const ux = { x: 1, y: 0 };
    const uy = { x: 0, y: 1 };

    for (const ts of strobeTimes(t)) {
      const p = sim.pos(ts);
      blocks.forEach((b, i) => drawBlock(ctx, view, centerOf(p.xs[i]), u, n, w, h, i, 0, true));
      if (g.hang) drawBlock(ctx, view, hangCenter(p.d), ux, uy, g.hw, g.hw, hangIdx, 0, true);
    }
    blocks.forEach((b, i) => drawBlock(ctx, view, centerOf(pos.xs[i]), u, n, w, h, i, b.m));
    if (g.hang) drawBlock(ctx, view, hangCenter(pos.d), ux, uy, g.hw, g.hw, hangIdx, an.bodies[hangIdx].m);

    // applied force
    const ap = physicsInput().applied;
    if (ap && ap.F > 0) {
      const i = ap.block;
      const c = centerOf(pos.xs[i]);
      const ang = ap.angle * DEG;
      const d = { x: u.x * Math.cos(ang) + n.x * Math.sin(ang), y: u.y * Math.cos(ang) + n.y * Math.sin(ang) };
      const dl = { a: Math.cos(ang), b: Math.sin(ang) }; // local (along, perpendicular)
      const tEdge = Math.min(
        Math.abs(dl.a) > 1e-9 ? w / 2 / Math.abs(dl.a) : Infinity,
        Math.abs(dl.b) > 1e-9 ? h / 2 / Math.abs(dl.b) : Infinity
      );
      const len = 0.55;
      let tail;
      let tip;
      if (dl.b > 1e-6) {
        // pulls away from the surface: draw from the face it leaves
        tail = { x: c.x + d.x * tEdge, y: c.y + d.y * tEdge };
        tip = { x: tail.x + d.x * len, y: tail.y + d.y * len };
      } else {
        tip = { x: c.x - d.x * (tEdge + 0.01), y: c.y - d.y * (tEdge + 0.01) };
        tail = { x: tip.x - d.x * len, y: tip.y - d.y * len };
      }
      const [x1, y1] = P(tail.x, tail.y);
      const [x2, y2] = P(tip.x, tip.y);
      arrow(ctx, x1, y1, x2, y2, C['f-a'], 3.5, 12);
      ctx.save();
      ctx.fillStyle = C['f-a'];
      ctx.font = '700 13px ' + C.fontMono;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const lab = dl.b > 1e-6 ? [x2 + (x2 - x1) * 0.25, y2 + (y2 - y1) * 0.25 - 12] : [x1, y1 - 14];
      ctx.fillText('Fₐ = ' + trimNum(ap.F) + ' N', lab[0], lab[1]);
      ctx.restore();
    }
  }

  /* ======================================================================
   * Free-body diagrams
   * ==================================================================== */

  // Returns, for each body, a list of force arrows (world direction + magnitude).
  function forceSets() {
    const an = analysis;
    if (an.kind === 'atwood') {
      return an.bodies.map((b, i) => ({
        body: b,
        idx: i,
        tilt: 0,
        arrows: [
          { dir: { x: 0, y: -1 }, mag: b.W, color: C['f-w'], label: 'W' },
          { dir: { x: 0, y: 1 }, mag: an.T, color: linkColor(0), label: 'T' },
        ],
      }));
    }
    const th = an.theta * DEG;
    const u = { x: Math.cos(th), y: Math.sin(th) };
    const n = { x: -Math.sin(th), y: Math.cos(th) };
    const ap = physicsInput().applied;
    return an.bodies.map((b, i) => {
      const along = (v) => (b.kind === 'hanging' ? { x: 0, y: -Math.sign(v) } : { x: u.x * Math.sign(v), y: u.y * Math.sign(v) });
      const arrows = [{ dir: { x: 0, y: -1 }, mag: b.W, color: C['f-w'], label: 'W' }];
      if (b.kind === 'block') {
        if (b.N > 1e-9) arrows.push({ dir: n, mag: b.N, color: C['f-n'], label: 'N' });
        if (Math.abs(b.f) > 1e-9) arrows.push({ dir: along(b.f), mag: Math.abs(b.f), color: C['f-f'], label: b.fType === 'static' ? 'fₛ' : 'fₖ' });
        if (ap && ap.block === i && ap.F > 0) {
          const ang = ap.angle * DEG;
          arrows.push({ dir: { x: u.x * Math.cos(ang) + n.x * Math.sin(ang), y: u.y * Math.cos(ang) + n.y * Math.sin(ang) }, mag: ap.F, color: C['f-a'], label: 'Fₐ' });
        }
      }
      an.links.forEach((l, j) => {
        if (l.F <= 1e-9 && !l.indeterminate) return;
        const sg = P.linkSigns(l.type);
        if (l.left === i) arrows.push({ dir: along(sg.left), mag: l.F, color: linkColor(j), label: labels.links[j].onLeft, dashed: l.indeterminate });
        if (l.right === i) arrows.push({ dir: along(sg.right), mag: l.F, color: linkColor(j), label: labels.links[j].onRight, dashed: l.indeterminate });
      });
      return { body: b, idx: i, tilt: b.kind === 'block' ? th : 0, arrows };
    });
  }

  function drawFBD() {
    const canvas = $('fbd');
    const wrap = canvas.parentElement;
    if (!analysis || !analysis.ok || $('panel-fbd').hidden) {
      if (analysis && !analysis.ok) {
        const { ctx, W, H } = prepCanvas(canvas, wrap.clientWidth, 300);
        canvas.style.width = W + 'px';
        ctx.fillStyle = C.muted;
        ctx.font = '15px ' + C.fontBody;
        ctx.textAlign = 'center';
        ctx.fillText('Fix the setup to see the diagrams.', W / 2, H / 2);
      }
      return;
    }
    const sets = forceSets();
    const nCells = sets.length;
    const cellW = Math.max(200, wrap.clientWidth / nCells);
    const totalW = Math.floor(cellW * nCells);
    const maxArrows = Math.max(...sets.map((s) => s.arrows.filter((a) => a.mag > 1e-9).length));
    const baseH = 290;
    const fullH = baseH + (ui.showFbdValues ? maxArrows * 17 + 8 : 0);
    canvas.style.width = totalW + 'px';
    canvas.style.height = fullH + 'px';
    const { ctx } = prepCanvas(canvas, totalW, fullH);
    let maxMag = 0;
    sets.forEach((s) => s.arrows.forEach((a) => (maxMag = Math.max(maxMag, a.mag))));
    const Lmax = Math.min(cellW / 2 - 42, 100);

    sets.forEach((s, ci) => {
      const cx = cellW * ci + cellW / 2;
      const cy = 132;
      let setMax = 0;
      s.arrows.forEach((a) => (setMax = Math.max(setMax, a.mag)));
      const ref = ui.fbdSameScale ? maxMag : setMax;
      const scale = ref > 0 ? Lmax / ref : 0;
      if (ci > 0) {
        ctx.save();
        ctx.strokeStyle = C.line;
        ctx.beginPath();
        ctx.moveTo(cellW * ci + 0.5, 12);
        ctx.lineTo(cellW * ci + 0.5, fullH - 12);
        ctx.stroke();
        ctx.restore();
      }
      // surface hint line and box
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(-s.tilt);
      if (s.body.kind === 'block') {
        ctx.strokeStyle = C.muted;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(-60, 20);
        ctx.lineTo(60, 20);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.fillStyle = bodyColor(s.idx);
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.4;
      ctx.fillRect(-26, -20, 52, 40);
      ctx.strokeRect(-26, -20, 52, 40);
      ctx.restore();

      // group arrows sharing a direction so they sit side by side
      const groups = {};
      s.arrows.forEach((a) => {
        if (a.mag <= 1e-9) return;
        const key = Math.round(Math.atan2(a.dir.y, a.dir.x) / (5 * DEG));
        (groups[key] = groups[key] || []).push(a);
      });
      Object.values(groups).forEach((list) => {
        list.forEach((a, k) => {
          const off = (k - (list.length - 1) / 2) * 11;
          const dx = a.dir.x;
          const dy = -a.dir.y;
          const px = -dy * off;
          const py = dx * off;
          const L = Math.max(22, a.mag * scale);
          const x1 = cx + px;
          const y1 = cy + py;
          const x2 = x1 + dx * L;
          const y2 = y1 + dy * L;
          arrow(ctx, x1, y1, x2, y2, a.color, 2.6, 9, a.dashed);
          const text = a.label + (a.dashed ? ' ?' : '');
          ctx.save();
          ctx.font = '700 12px ' + C.fontMono;
          ctx.fillStyle = a.color;
          ctx.textBaseline = 'middle';
          const tw = ctx.measureText(text).width;
          let lx = x2 + dx * 10;
          let ly = y2 + dy * 12;
          if (Math.abs(dx) > 0.5) ctx.textAlign = dx > 0 ? 'left' : 'right';
          else {
            ctx.textAlign = 'center';
            if (list.length > 1) lx += (k - (list.length - 1) / 2) * 0;
          }
          if (ctx.textAlign === 'left') lx = Math.min(lx, cellW * (ci + 1) - tw - 4);
          if (ctx.textAlign === 'right') lx = Math.max(lx, cellW * ci + tw + 4);
          if (Math.abs(dx) > 0.5 && list.length > 1) ly += (k - (list.length - 1) / 2) * 6;
          ctx.fillText(text, lx, ly);
          ctx.restore();
        });
      });
      // center dot
      ctx.fillStyle = C.ink;
      ctx.beginPath();
      ctx.arc(cx, cy, 3, 0, 2 * Math.PI);
      ctx.fill();

      ctx.fillStyle = C.ink;
      ctx.font = '700 13px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'alphabetic';
      ctx.fillText(s.body.name.replace(' (hanging)', '') + ' · ' + trimNum(s.body.m) + ' kg', cx, baseH - 22);
      if (ui.showFbdValues) {
        ctx.font = '600 12px ' + C.fontMono;
        s.arrows
          .filter((a) => a.mag > 1e-9)
          .forEach((a, k) => {
            ctx.fillStyle = a.color;
            ctx.fillText(a.label + ' = ' + (a.dashed ? '?' : fmt(a.mag) + ' N'), cx, baseH + k * 17);
          });
      }
    });
  }

  function renderLegendAndPairs() {
    const legend = $('fbd-legend');
    const pairs = $('fbd-pairs');
    if (!analysis.ok) {
      legend.innerHTML = '';
      pairs.innerHTML = '';
      return;
    }
    const chip = (color, text) => '<span><i style="background:' + color + '"></i>' + text + '</span>';
    let h = chip('var(--f-w)', 'W: weight') ;
    if (analysis.kind === 'chain') {
      h += chip('var(--f-n)', 'N: normal force') + chip('var(--f-f)', 'fₛ / fₖ: static / kinetic friction') + chip('var(--f-a)', 'Fₐ: applied force');
    }
    analysis.kind === 'atwood'
      ? (h += chip('var(--l1)', 'T: string tension'))
      : analysis.links.forEach((l, j) => {
          const nm = labels.links[j];
          h += chip('var(--l' + ((j % 4) + 1) + ')', l.type === 'string' ? nm.name + ': tension' : nm.onRight + ' / ' + nm.onLeft + ': contact pair');
        });
    if (analysis.kind === 'chain' && analysis.links.some((l) => l.indeterminate)) h += '<span>Dashed “?”: not uniquely determined</span>';
    legend.innerHTML = h;

    const showV = ui.showFbdValues || ui.revealed;
    const val = (F) => (showV ? ' (' + fmt(F) + ' N each)' : '');
    let items = [];
    if (analysis.kind === 'chain') {
      const dw = dirWords();
      analysis.links.forEach((l, j) => {
        const nm = labels.links[j];
        const A = analysis.bodies[l.left];
        const B = analysis.bodies[l.right];
        const sw = '<span class="swatch" style="background:var(--l' + ((j % 4) + 1) + ')"></span>';
        if (l.type === 'contact') {
          if (!l.engaged) items.push(sw + A.name + ' and ' + B.name + ' are moving apart, so they do not push on each other. The contact force is zero.');
          else
            items.push(sw + '<b>' + nm.onRight + '</b> (' + A.name + ' pushes ' + B.name + ' ' + dw.pos + ') and <b>' + nm.onLeft + '</b> (' + B.name + ' pushes ' + A.name + ' ' + dw.neg +
              ') are a <b>Newton’s third-law pair</b>: equal in size' + val(l.F) + ', opposite in direction, and acting on different objects.');
        } else {
          const where = l.hanging ? 'over the pulley' : '';
          if (!l.engaged) items.push(sw + 'String ' + nm.name + ' is slack, so its tension is zero.');
          else
            items.push(sw + 'String <b>' + nm.name + '</b> ' + where + ' pulls ' + A.name + ' and ' + B.name + ' toward each other with the same tension' + val(l.F) +
              '. These two pulls are not a third-law pair with each other. Each one pairs with the block’s pull back on the string. They are equal because the string is massless.');
        }
      });
    } else {
      items.push('<span class="swatch" style="background:var(--l1)"></span>The string pulls up on Mass 1 and up on Mass 2 with the same tension T' + val(analysis.T) +
        '. Those two pulls are not a third-law pair; each pairs with the mass pulling back down on the string.');
    }
    items.push('<b>Weight:</b> Earth pulls each block down, and each block pulls Earth up just as hard. The partner force acts on Earth, so it is not on these diagrams.');
    if (analysis.kind === 'chain')
      items.push('<b>Normal force:</b> the surface pushes on the block and the block pushes back on the surface. W and N on the same block are <b>not</b> a third-law pair: they act on the same object and can have different sizes.');
    pairs.innerHTML = '<h3>Action–reaction pairs</h3><ul>' + items.map((x) => '<li>' + x + '</li>').join('') + '</ul>';
  }

  /* ======================================================================
   * Motion graph
   * ==================================================================== */

  function niceStep(range, target) {
    const raw = range / target;
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const m = raw / p;
    return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
  }

  function drawGraph() {
    if ($('panel-graph').hidden) return;
    const canvas = $('graph');
    const { ctx, W, H } = prepCanvas(canvas);
    if (!analysis || !analysis.ok) return;
    const i = Math.min(ui.track, analysis.bodies.length - 1);
    const tEnd = sim.tStop;
    const fn = (t) => {
      const m = bodyMotion(i, t);
      return ui.graph === 'x' ? m.s : ui.graph === 'v' ? m.v : m.a;
    };
    let ymin = 0;
    let ymax = 0;
    for (let k = 0; k <= 50; k++) {
      const y = fn((tEnd * k) / 50);
      ymin = Math.min(ymin, y);
      ymax = Math.max(ymax, y);
    }
    if (ymax - ymin < 1e-9) {
      ymax += 1;
      ymin -= ui.graph === 'a' ? 1 : 0;
    }
    const padY = (ymax - ymin) * 0.08;
    ymax += padY;
    if (ymin < 0) ymin -= padY;
    const L = 62;
    const R = 16;
    const T = 16;
    const B = 40;
    const X = (t) => L + ((W - L - R) * t) / tEnd;
    const Y = (y) => T + ((H - T - B) * (ymax - y)) / (ymax - ymin);

    ctx.font = '11px ' + C.fontMono;
    ctx.fillStyle = C.muted;
    ctx.strokeStyle = C.grid;
    ctx.lineWidth = 1;
    const xs = niceStep(tEnd, 8);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let t = 0; t <= tEnd + 1e-9; t += xs) {
      ctx.beginPath();
      ctx.moveTo(Math.round(X(t)) + 0.5, T);
      ctx.lineTo(Math.round(X(t)) + 0.5, H - B);
      ctx.stroke();
      ctx.fillText(+t.toFixed(3) + '', X(t), H - B + 6);
    }
    const ys = niceStep(ymax - ymin, 5);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let y = Math.ceil(ymin / ys) * ys; y <= ymax + 1e-9; y += ys) {
      ctx.beginPath();
      ctx.moveTo(L, Math.round(Y(y)) + 0.5);
      ctx.lineTo(W - R, Math.round(Y(y)) + 0.5);
      ctx.stroke();
      ctx.fillText(fmt(y, ys < 0.1 ? 2 : 1), L - 8, Y(y));
    }
    ctx.strokeStyle = C.muted;
    ctx.beginPath();
    ctx.moveTo(L, T);
    ctx.lineTo(L, H - B);
    ctx.moveTo(L, Y(0));
    ctx.lineTo(W - R, Y(0));
    ctx.stroke();
    ctx.fillStyle = C.ink;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.font = '600 12px ' + C.fontBody;
    ctx.fillText('time t (s)', L + (W - L - R) / 2, H - 4);
    ctx.save();
    ctx.translate(14, T + (H - T - B) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textBaseline = 'middle';
    ctx.fillText(ui.graph === 'x' ? 'displacement (m)' : ui.graph === 'v' ? 'velocity (m/s)' : 'acceleration (m/s²)', 0, 0);
    ctx.restore();

    if (anim.t <= 0) {
      ctx.fillStyle = C.muted;
      ctx.font = '13px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Press Play to draw the graph', L + (W - L - R) / 2, T + (H - T - B) / 2);
      return;
    }
    const tNow = Math.min(anim.t, tEnd);
    ctx.strokeStyle = C.accent;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    const steps = 160;
    for (let k = 0; k <= steps; k++) {
      const t = (tNow * k) / steps;
      const x = X(t);
      const y = Y(fn(t));
      k ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.stroke();
    ctx.fillStyle = C.accent;
    ctx.beginPath();
    ctx.arc(X(tNow), Y(fn(tNow)), 4.5, 0, 2 * Math.PI);
    ctx.fill();
  }

  /* ======================================================================
   * Predict & check
   * ==================================================================== */

  function quantities() {
    const an = analysis;
    const q = [];
    if (!an.ok) return q;
    if (an.kind === 'atwood') {
      q.push({ key: 'a', group: 'main', label: 'Acceleration of the system', unit: 'm/s²', value: Math.abs(an.a) });
      q.push({ key: 'T', group: 'main', label: 'Tension in the string, T', unit: 'N', value: an.T });
      an.bodies.forEach((b, i) =>
        q.push({ key: 'net' + i, group: 'more', label: 'Net force on ' + b.name, unit: 'N', value: Math.abs(b.Fnet) })
      );
      return q;
    }
    const gr = an.groups;
    const sameA = gr.every((g) => Math.abs(g.a - gr[0].a) < 1e-9);
    if (sameA) q.push({ key: 'a', group: 'main', label: 'Acceleration of the system', unit: 'm/s²', value: Math.abs(gr[0].a) });
    else
      gr.forEach((g) =>
        q.push({ key: 'a' + g.start + '-' + g.end, group: 'main', label: 'Acceleration of ' + blockSpan(g.start, g.end, an.bodies), unit: 'm/s²', value: Math.abs(g.a) })
      );
    an.links.forEach((l, j) => {
      const nm = labels.links[j];
      const A = l.left + 1;
      const B = l.right + 1;
      const row = { key: 'link' + j + l.type, group: 'main', unit: 'N', value: l.F };
      if (l.type === 'string') {
        row.label = 'Tension ' + nm.name;
        row.sub = l.hanging ? 'string from Block ' + A + ' over the pulley to Block ' + B : 'string between Block ' + A + ' and Block ' + B;
      } else {
        row.label = 'Contact force between Block ' + A + ' and Block ' + B;
        row.sub = '|' + nm.onRight + '| = |' + nm.onLeft + '|';
      }
      if (l.indeterminate) row.na = 'Not uniquely determined: static friction can be shared between the blocks in more than one way.';
      q.push(row);
    });
    an.bodies.forEach((b, i) => {
      if (b.kind === 'block') q.push({ key: 'N' + i, group: 'more', label: 'Normal force on ' + b.name, unit: 'N', value: b.N });
      if (an.friction && b.kind === 'block') q.push({ key: 'f' + i, group: 'more', label: 'Friction force on ' + b.name, unit: 'N', value: Math.abs(b.f) });
      q.push({ key: 'net' + i, group: 'more', label: 'Net force on ' + b.name, unit: 'N', value: Math.abs(b.Fnet) });
    });
    return q;
  }

  function renderCheck() {
    const qs = quantities();
    const sig = state.scenario + '|' + qs.map((q) => q.key + ':' + q.label + ':' + (q.na ? 1 : 0)).join('|');
    $('score').textContent = '';
    if (sig !== ui.checkSig) {
      ui.checkSig = sig;
      if (!qs.length) {
        $('check-rows').innerHTML = '<p class="hint">Fix the setup first.</p>';
        return;
      }
      const row = (q) => {
        const id = 'pred-' + q.key;
        return '<div class="qrow' + (q.na ? ' na' : '') + '" data-key="' + q.key + '"><label class="ql" for="' + id + '">' + esc(q.label) +
          (q.sub ? '<small>' + esc(q.sub) + '</small>' : '') + '</label>' +
          '<input type="number" step="any" id="' + id + '" data-key="' + q.key + '" inputmode="decimal"' + (q.na ? ' disabled' : '') +
          ' value="' + esc(ui.pred[q.key] ?? '') + '" aria-label="Your answer for ' + esc(q.label) + '">' +
          '<span class="qu">' + q.unit + '</span><span class="qr" aria-live="polite">' + (q.na ? esc(q.na) : '') + '</span></div>';
      };
      const main = qs.filter((q) => q.group === 'main');
      const more = qs.filter((q) => q.group === 'more');
      $('check-rows').innerHTML =
        '<div class="check-group"><h3>System</h3>' + main.map(row).join('') + '</div>' +
        (more.length ? '<details class="check-group"><summary>' + (analysis.kind === 'atwood' ? 'More: net force on each mass' : (analysis.friction ? 'More: normal, friction and net force on each block' : 'More: normal and net force on each block')) + '</summary>' + more.map(row).join('') + '</details>' : '');
    } else {
      document.querySelectorAll('.qrow').forEach((r) => {
        r.classList.remove('good', 'bad');
        const q = qs.find((x) => x.key === r.dataset.key);
        r.querySelector('.qr').textContent = q && q.na ? q.na : '';
      });
    }
  }

  function gradeAll(reveal) {
    const qs = quantities();
    let right = 0;
    let tried = 0;
    qs.forEach((q) => {
      const r = document.querySelector('.qrow[data-key="' + q.key + '"]');
      if (!r || q.na) return;
      const inp = r.querySelector('input');
      const out = r.querySelector('.qr');
      r.classList.remove('good', 'bad');
      const raw = inp.value.trim();
      const ans = q.value;
      if (raw === '') {
        out.textContent = reveal ? 'answer: ' + fmt(ans) : '';
        return;
      }
      tried++;
      const x = Math.abs(parseFloat(raw));
      const ok = isFinite(x) && Math.abs(x - ans) <= Math.max(0.02 * Math.abs(ans), 0.01);
      if (ok) right++;
      r.classList.add(ok ? 'good' : 'bad');
      out.textContent = ok ? '✓ correct' + (reveal ? ' (' + fmt(ans) + ')' : '') : reveal ? '✗ answer: ' + fmt(ans) : x > ans ? '✗ too high' : '✗ too low';
    });
    $('score').textContent = tried ? right + ' of ' + tried + ' correct' : reveal ? '' : 'Enter at least one answer.';
  }

  /* ======================================================================
   * Worked solution
   * ==================================================================== */

  function alongTerms(b) {
    const t = [];
    if (b.kind === 'hanging') t.push({ v: b.W, d: 'weight m' + sub(b.index + 1) + 'g' });
    else {
      if (Math.abs(b.Wpar) > 1e-12) t.push({ v: -b.Wpar, d: 'gravity along the slope, m' + sub(b.index + 1) + 'g sin θ' });
      if (Math.abs(b.Fpar) > 1e-12) t.push({ v: b.Fpar, d: 'applied force along the surface' });
    }
    return t;
  }

  function solutionHTML() {
    const an = analysis;
    if (!an.ok) return '<p>' + esc(an.error) + '</p>';
    const g = fmt(an.g);
    let h = '';
    if (an.kind === 'atwood') {
      const [b1, b2] = an.bodies;
      h += '<section><h3>1. Choose a positive direction</h3><p>Take + as <b>down for Mass 2</b> and <b>up for Mass 1</b>. The string keeps them moving together, so they share one acceleration a.</p></section>';
      h += '<section><h3>2. Newton’s second law for each mass</h3>' +
        '<div class="eq">Mass 1:  T − m₁g = m₁a\nMass 2:  m₂g − T = m₂a</div></section>';
      h += '<section><h3>3. Add the equations so T cancels</h3>' +
        '<div class="eq">(m₂ − m₁) g = (m₁ + m₂) a\na = (m₂ − m₁) g / (m₁ + m₂)\n  = (' + fmt(b2.m) + ' − ' + fmt(b1.m) + ')(' + g + ') / (' + fmt(b1.m) + ' + ' + fmt(b2.m) + ')\n  = ' + fmt(an.a) + ' m/s²</div>' +
        '<p>' + (Math.abs(an.a) < 1e-12 ? 'The masses are equal, so nothing accelerates.' : an.a > 0 ? 'a is positive: Mass 2 falls and Mass 1 rises.' : 'a is negative: Mass 1 falls and Mass 2 rises.') + '</p></section>';
      h += '<section><h3>4. Solve for the tension</h3><div class="eq">T = m₁(g + a) = ' + fmt(b1.m) + ' × (' + g + signed(an.a) + ') = ' + fmt(an.T) + ' N\nCheck: T = m₂(g − a) = ' + fmt(b2.m) + ' × (' + g + signed(-an.a) + ') = ' + fmt(b2.m * (an.g - an.a)) + ' N</div>' +
        '<p>The tension is between the two weights (' + fmt(b1.W) + ' N and ' + fmt(b2.W) + ' N), as it must be when one mass speeds up going up and the other speeds up going down.</p></section>';
      h += '<section><h3>5. Net force on each mass</h3><div class="eq">F_net,1 = m₁a = ' + fmt(b1.m) + ' × ' + fmt(an.a) + ' = ' + fmt(b1.Fnet) + ' N\nF_net,2 = m₂a = ' + fmt(b2.m) + ' × ' + fmt(an.a) + ' = ' + fmt(b2.Fnet) + ' N</div></section>';
      return h;
    }

    const dw = dirWords();
    const fr = an.friction;
    h += '<section><h3>1. Choose a positive direction</h3><p>Take + as <b>' + dw.pos + '</b>.' +
      (dw.hang ? ' For the hanging block, + is <b>down</b>. With the string taut, every object then has the same acceleration a.' : '') + '</p></section>';

    // forces table
    let t = '<div class="table-wrap"><table><thead><tr><th>Object</th><th>m (kg)</th><th>W = mg (N)</th><th>Along the line, before friction (N)</th><th>N (N)</th>' +
      (fr ? '<th>μs·N (N)</th><th>μk·N (N)</th>' : '') + '</tr></thead><tbody>';
    an.bodies.forEach((b) => {
      const tm = alongTerms(b);
      t += '<tr><td>' + b.name + '</td><td>' + fmt(b.m) + '</td><td>' + fmt(b.W) + '</td><td>' + (tm.length ? tm.map((x) => fmt(x.v)).join(' + ').replace(/\+ −/g, '− ') : '0.00') + '</td><td>' +
        (b.kind === 'block' ? fmt(b.N) : '—') + '</td>' + (fr ? '<td>' + (b.kind === 'block' ? fmt(b.mus * b.N) : '—') + '</td><td>' + (b.kind === 'block' ? fmt(b.muk * b.N) : '—') + '</td>' : '') + '</tr>';
    });
    t += '</tbody></table></div>';
    let notes = '';
    if (an.theta > 0) notes += '<p>On the incline, gravity along the slope is mg sin θ (pointing down the slope) and gravity into the slope is mg cos θ, with θ = ' + trimNum(an.theta) + '°. ' + (fr ? 'With no other perpendicular forces, N = mg cos θ.' : '') + '</p>';
    const ap = an.bodies.find((b) => Math.abs(b.Fperp) > 1e-9 || Math.abs(b.Fpar) > 1e-9);
    if (ap) {
      const inp = physicsInput().applied;
      notes += '<p>Fₐ = ' + trimNum(inp.F) + ' N at ' + trimNum(inp.angle) + '° on ' + ap.name + ': along the surface Fₐ cos ' + trimNum(inp.angle) + '° = ' + fmt(ap.Fpar) + ' N' +
        (Math.abs(ap.Fperp) > 1e-9 ? ', and ' + (ap.Fperp > 0 ? 'away from' : 'into') + ' the surface Fₐ sin ' + trimNum(Math.abs(inp.angle)) + '° = ' + fmt(Math.abs(ap.Fperp)) + ' N, so N = ' + fmt(ap.Wperp) + (ap.Fperp > 0 ? ' − ' : ' + ') + fmt(Math.abs(ap.Fperp)) + ' = ' + fmt(ap.N) + ' N' : '') + '.</p>';
    }
    h += '<section><h3>2. Forces on each object</h3>' + t + notes + '</section>';

    let step = 3;
    const broken = an.links.map((l, j) => ({ l, j })).filter((x) => !x.l.engaged);
    if (an.groups.length > 1) {
      h += '<section><h3>' + step++ + '. Which strings and contacts hold?</h3><p>' +
        broken.map(({ l, j }) => (l.type === 'string' ? 'String ' + labels.links[j].name + ' goes slack (the objects it joins move toward each other).' : an.bodies[l.left].name + ' and ' + an.bodies[l.right].name + ' move apart, so they stop touching.')).join(' ') +
        ' Each connected part moves on its own, so solve each part separately.</p></section>';
    }

    an.groups.forEach((gr) => {
      const name = blockSpan(gr.start, gr.end, an.bodies);
      const terms = [];
      for (let i = gr.start; i <= gr.end; i++) alongTerms(an.bodies[i]).forEach((x) => terms.push(x));
      let fTot = 0;
      for (let i = gr.start; i <= gr.end; i++) fTot += an.bodies[i].f;
      const termStr = terms.length ? terms.map((x, k) => (k ? signed(x.v) : fmt(x.v))).join('') : '0';
      let s = '<section><h3>' + step++ + '. Acceleration' + (an.groups.length > 1 || gr.end > gr.start ? ' of ' + name : '') + '</h3>';
      if (gr.end > gr.start) s += '<p>Treat ' + name + ' as one system. Forces between them are internal and cancel in pairs (Newton’s third law), so only outside forces matter. Total mass M = ' + fmt(gr.M) + ' kg.</p>';
      s += '<div class="eq">Net outside force before friction:\nD = ' + (terms.length > 1 ? termStr + ' = ' : '') + fmt(gr.D) + ' N</div>';
      if (fr && gr.cap + gr.kin > 0) {
        if (gr.state !== 'sliding') {
          s += '<p>Maximum static friction is Σ μs·N = ' + fmt(gr.cap) + ' N. Since |D| = ' + fmt(Math.abs(gr.D)) + ' N is not larger, static friction holds and the system <b>stays at rest</b>: a = 0. Static friction supplies ' + fmt(Math.abs(gr.D)) + ' N' +
            (Math.abs(gr.D) > 1e-9 ? ', pointing ' + (gr.D > 0 ? dw.neg : dw.pos) : '') + '.</p>';
        } else {
          s += '<p>Maximum static friction is Σ μs·N = ' + fmt(gr.cap) + ' N. Since |D| = ' + fmt(Math.abs(gr.D)) + ' N is larger, the system slides ' + (gr.D > 0 ? dw.pos : dw.neg) +
            '. Kinetic friction opposes the motion with Σ μk·N = ' + fmt(gr.kin) + ' N.</p>';
          s += '<div class="eq">a = (D' + (fTot < 0 ? ' − fₖ' : ' + fₖ') + ') / M = (' + fmt(gr.D) + signed(fTot) + ') / ' + fmt(gr.M) + ' = ' + fmt(gr.a) + ' m/s²</div>';
        }
      } else if (Math.abs(gr.D) < 1e-9) {
        s += '<p>No net force, so a = 0. Starting from rest, it stays at rest.</p>';
      } else {
        s += '<div class="eq">a = D / M = ' + fmt(gr.D) + ' / ' + fmt(gr.M) + ' = ' + fmt(gr.a) + ' m/s²</div>';
      }
      if (gr.state === 'sliding') s += '<p>' + (gr.a > 0 ? 'Positive, so the motion is ' + dw.pos + '.' : 'Negative, so the motion is ' + dw.neg + (dw.hang && gr.end === an.bodies.length - 1 ? ' (the hanging block rises)' : '') + '.') + ' Magnitude |a| = ' + fmt(Math.abs(gr.a)) + ' m/s².</p>';
      h += s + '</section>';
    });

    const engagedLinks = an.links.map((l, j) => ({ l, j })).filter((x) => x.l.engaged || x.l.indeterminate);
    if (engagedLinks.length) {
      let s = '<section><h3>' + step++ + '. Tensions and contact forces</h3><p>Isolate the objects on one side of each string or contact and apply ΣF = ma to just that part.</p>';
      engagedLinks.forEach(({ l, j }) => {
        const nm = labels.links[j];
        const gr = an.groups.find((g) => g.start <= l.left && g.end >= l.right);
        if (l.indeterminate) {
          s += '<p><b>' + (l.type === 'string' ? 'Tension ' + nm.name : 'Contact ' + nm.name) + ':</b> not uniquely determined. The blocks are held at rest and static friction on each side can take different shares of the load, so Newton’s laws alone do not fix this force.</p>';
          return;
        }
        // Isolate the simpler side: the hanging block if there is one, otherwise
        // the side with fewer forces on it (ties go to the right-hand side).
        const sideParts = (a, b) => {
          const out = [];
          let m = 0;
          for (let i = a; i <= b; i++) {
            const bd = an.bodies[i];
            m += bd.m;
            alongTerms(bd).forEach((x) => out.push(x.v));
            if (Math.abs(bd.f) > 1e-9) out.push(bd.f);
          }
          return { parts: out, M: m };
        };
        const L = sideParts(gr.start, l.left);
        const R = sideParts(l.right, gr.end);
        const nL = l.left - gr.start + 1;
        const nR = gr.end - l.right + 1;
        const right =
          an.bodies[l.right].kind === 'hanging' ||
          R.parts.length < L.parts.length ||
          (R.parts.length === L.parts.length && nR <= nL);
        const [s0, s1] = right ? [l.right, gr.end] : [gr.start, l.left];
        const sg = P.linkSigns(l.type)[right ? 'right' : 'left'];
        const lab = right ? nm.onRight : nm.onLeft;
        const { parts, M } = right ? R : L;
        const who = blockSpan(s0, s1, an.bodies).replace(' (hanging)', '');
        const lhs = (sg > 0 ? '' : '−') + lab + parts.map((v) => signed(v)).join('');
        // e.g. "T − 19.60 = (2.00)(3.27)" then "T = 13.07 N"
        s += '<p><b>' + (l.type === 'string' ? 'Tension ' + nm.name : 'Contact ' + nm.name + ' and ' + nm.onLeft) + '</b>: isolate ' + who + '.</p>' +
          '<div class="eq">' + lhs + ' = (' + fmt(M) + ')(' + fmt(gr.a) + ')\n' + lab + ' = ' + fmt(l.F) + ' N</div>';
      });
      broken.forEach(({ l, j }) => {
        s += '<p><b>' + (l.type === 'string' ? 'Tension ' + labels.links[j].name : 'Contact ' + labels.links[j].name) + '</b> = 0 N, because ' + (l.type === 'string' ? 'the string is slack.' : 'the blocks are not pressing on each other.') + '</p>';
      });
      h += s + '</section>';
    }

    let net = '<div class="eq">' + an.bodies.map((b) => 'F_net on ' + b.name + ' = m a = ' + fmt(b.m) + ' × ' + fmt(b.a) + ' = ' + fmt(b.Fnet) + ' N').join('\n') + '</div>';
    if (fr) net += '<div class="eq">' + an.bodies.filter((b) => b.kind === 'block').map((b) => 'Friction on ' + b.name + ': ' + (b.fType === 'static' ? 'static, ' : b.fType === 'kinetic' ? 'kinetic, μk·N = ' : '') + fmt(Math.abs(b.f)) + ' N').join('\n') + '</div>';
    h += '<section><h3>' + step++ + '. Net force on each object</h3>' + net + '</section>';
    return h;
  }

  function renderSolution() {
    const open = ui.solutionOpen;
    $('solution-lock').hidden = open;
    $('solution').hidden = !open;
    if (open) $('solution').innerHTML = solutionHTML();
  }

  /* ======================================================================
   * Readouts, status and animation
   * ==================================================================== */

  function renderScenarioTabs() {
    document.querySelectorAll('.scenarios button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.scenario === state.scenario)));
  }

  function renderTrack() {
    const sel = $('sel-track');
    if (!analysis.ok) {
      sel.innerHTML = '';
      return;
    }
    if (ui.track >= analysis.bodies.length) ui.track = 0;
    sel.innerHTML = analysis.bodies.map((b, i) => '<option value="' + i + '"' + (i === ui.track ? ' selected' : '') + '>' + b.name + '</option>').join('');
  }

  function plusWord() {
    if (!analysis.ok) return '';
    const b = analysis.bodies[ui.track];
    if (analysis.kind === 'atwood') return ui.track === 0 ? '+ = up' : '+ = down';
    if (b.kind === 'hanging') return '+ = down';
    return state.scenario === 'incline' ? '+ = up the slope' : '+ = right';
  }

  function updateReadouts() {
    if (!analysis.ok) {
      $('ro-t').textContent = '—';
      $('ro-d').textContent = '—';
      $('ro-v').textContent = '—';
      return;
    }
    const m = bodyMotion(ui.track, anim.t);
    $('ro-t').textContent = anim.t.toFixed(2) + ' s';
    $('ro-d').textContent = fmt(m.s) + ' m';
    $('ro-v').textContent = fmt(m.v) + ' m/s';
    $('ro-d').title = plusWord();
    $('ro-v').title = plusWord();
  }

  function updateStatus() {
    const st = $('status');
    st.classList.remove('error');
    if (!analysis.ok) {
      st.classList.add('error');
      st.textContent = analysis.error;
      return;
    }
    const pw = plusWord();
    if (anim.t === 0 && !anim.playing) {
      st.textContent = 'Ready. Press Play to release the system from rest. Readouts use ' + pw + ' for the tracked object.';
      return;
    }
    if (anim.playing) {
      st.textContent = 'Running…';
      return;
    }
    if (anim.done) {
      if (analysis.atRest) {
        st.textContent = analysis.kind === 'chain' && analysis.friction ? 'Nothing moves: static friction holds the system at rest.' : 'Nothing moves: the forces are balanced.';
      } else {
        let msg = (sim.reason ? sim.reason : 'Stopped') + ' at t = ' + sim.tStop.toFixed(2) + ' s.';
        if (analysis.kind === 'chain') {
          const slack = analysis.links.filter((l) => !l.engaged && !l.indeterminate);
          if (slack.length) msg += ' ' + slack.map((l) => (l.type === 'string' ? 'A string went slack.' : 'Two blocks separated.')).filter((x, i, a) => a.indexOf(x) === i).join(' ');
        }
        st.textContent = msg + ' Press Reset to run it again.';
      }
      return;
    }
    st.textContent = 'Paused at t = ' + anim.t.toFixed(2) + ' s.';
  }

  function updatePlayButton() {
    const b = $('btn-play');
    b.disabled = !analysis.ok;
    b.textContent = anim.playing ? 'Pause' : anim.done ? 'Replay' : anim.t > 0 ? 'Resume' : 'Play';
  }

  function drawAll() {
    drawScene();
    updateReadouts();
    drawFBD();
    drawGraph();
  }

  function frame(ts) {
    if (!anim.playing) return;
    const dt = Math.min(0.05, (ts - anim.last) / 1000) * ui.speed;
    anim.last = ts;
    anim.t += dt;
    if (anim.t >= sim.tStop) {
      anim.t = sim.tStop;
      anim.playing = false;
      anim.done = true;
      updatePlayButton();
      updateStatus();
    }
    drawScene();
    updateReadouts();
    drawGraph();
    if (anim.playing) requestAnimationFrame(frame);
  }

  function play() {
    if (!analysis.ok) return;
    if (anim.playing) {
      anim.playing = false;
    } else {
      if (anim.done) {
        anim.t = 0;
        anim.done = false;
      }
      anim.playing = true;
      anim.last = performance.now();
      requestAnimationFrame(frame);
    }
    updatePlayButton();
    updateStatus();
  }

  function reset() {
    anim.t = 0;
    anim.playing = false;
    anim.done = false;
    updatePlayButton();
    updateStatus();
    drawAll();
  }

  /* ======================================================================
   * Tabs and wiring
   * ==================================================================== */

  function showTab(name) {
    ui.tab = name;
    document.querySelectorAll('.tabs button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.tab === name)));
    ['check', 'fbd', 'graph', 'solution'].forEach((t) => ($('panel-' + t).hidden = t !== name));
    if (name === 'fbd') drawFBD();
    if (name === 'graph') drawGraph();
  }

  function init() {
    loadFromHash();
    readColors();

    document.querySelectorAll('.scenarios button').forEach((b) =>
      b.addEventListener('click', () => {
        if (state.scenario === b.dataset.scenario) return;
        state.scenario = b.dataset.scenario;
        ui.pred = {};
        ui.solutionOpen = false;
        ui.track = 0;
        renderControls();
        changed();
      })
    );
    const controls = $('controls');
    controls.addEventListener('input', (e) => {
      if (e.target.type !== 'checkbox' && e.target.tagName !== 'SELECT') onControlInput(e);
    });
    controls.addEventListener('change', (e) => {
      if (e.target.type === 'checkbox' || e.target.tagName === 'SELECT') onControlInput(e);
      else if (/mu[sk]$/.test(e.target.dataset.path || '')) onControlInput(e);
    });
    controls.addEventListener('click', onControlClick);

    $('btn-play').addEventListener('click', play);
    $('btn-reset').addEventListener('click', reset);
    $('sel-speed').addEventListener('change', (e) => (ui.speed = parseFloat(e.target.value)));
    $('chk-strobe').addEventListener('change', (e) => {
      ui.strobe = e.target.checked;
      drawScene();
    });
    $('sel-track').addEventListener('change', (e) => {
      ui.track = parseInt(e.target.value, 10);
      updateReadouts();
      updateStatus();
      drawGraph();
    });
    $('chk-fbd-scale').addEventListener('change', (e) => {
      ui.fbdSameScale = e.target.checked;
      drawFBD();
    });
    $('chk-fbd-values').addEventListener('change', (e) => {
      ui.showFbdValues = e.target.checked;
      renderLegendAndPairs();
      drawFBD();
    });
    document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
    document.querySelectorAll('[data-graph]').forEach((b) =>
      b.addEventListener('click', () => {
        ui.graph = b.dataset.graph;
        document.querySelectorAll('[data-graph]').forEach((x) => x.classList.toggle('on', x === b));
        drawGraph();
      })
    );

    $('check-form').addEventListener('submit', (e) => {
      e.preventDefault();
      gradeAll(ui.revealed);
      ui.solutionOpen = true;
      renderSolution();
    });
    $('check-rows').addEventListener('input', (e) => {
      if (e.target.dataset.key) ui.pred[e.target.dataset.key] = e.target.value;
    });
    $('btn-reveal').addEventListener('click', () => {
      ui.revealed = true;
      ui.solutionOpen = true;
      gradeAll(true);
      renderSolution();
      renderLegendAndPairs();
    });
    $('btn-clear').addEventListener('click', () => {
      ui.pred = {};
      ui.revealed = false;
      document.querySelectorAll('#check-rows input').forEach((i) => (i.value = ''));
      renderCheck();
    });
    $('btn-unlock').addEventListener('click', () => {
      ui.solutionOpen = true;
      renderSolution();
    });

    const redraw = () => drawAll();
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(redraw);
      ro.observe(document.querySelector('.canvas-wrap'));
      ro.observe(document.querySelector('.graph-wrap'));
      ro.observe(document.querySelector('.fbd-scroll'));
    } else window.addEventListener('resize', redraw);
    const recolor = () => {
      readColors();
      drawAll();
    };
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', recolor);
    new MutationObserver(recolor).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(recolor);

    renderControls();
    changed();
    showTab('check');
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
