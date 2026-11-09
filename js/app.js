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
  const SCENARIOS = ['atwood', 'modified', 'incline', 'horizontal', 'elevator', 'car', 'body', 'statics', 'lab', 'puzzles'];

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
        longString: false,
        stack: { on: false, base: 0, m: 1, mus: 0.5, muk: 0.4 },
        launch: { on: false, d: 2, H: 1, phi: 0 },
        mystery: { on: false, target: 'b0' },
      },
      o
    );
  }
  function defaults() {
    return {
      scenario: 'atwood',
      g: 9.8,
      atwood: { m1: 3, m2: 5, mystery: { on: false, target: 'm1' } },
      modified: surfaceDefaults({ blocks: [blk(4)], hanging: { on: true, m: 2 } }),
      incline: surfaceDefaults({ theta: 30, blocks: [blk(5)], hanging: { on: false, m: 3 } }),
      horizontal: surfaceDefaults({
        blocks: [blk(2), blk(3)],
        links: ['contact'],
        applied: { on: true, F: 20, angle: 0, block: 0 },
      }),
      elevator: ELEV.defaults(),
      body: BODY.defaults(),
      statics: STATICS.defaults(),
      lab: LAB.defaults(),
      car: CAR.defaults(),
      puzzles: PUZ.defaults(),
    };
  }

  let state; // created in init(), once every module is defined
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
    mystery: { key: '', value: 0 }, // the hidden mass, never put in share links
  };

  const cur = () => state[state.scenario];
  const isAtwood = () => state.scenario === 'atwood';

  // Launching off the table needs one block on the ramp with nothing holding it back.
  function launchBlocked(c) {
    if (state.scenario !== 'incline') return 'not incline';
    if (c.blocks.length > 1) return 'Launching works with one block on the ramp. Remove the other blocks.';
    if (c.hanging.on) return 'Launching works without a pulley. Turn off the hanging block.';
    if (c.stack.on) return 'Launching works without a stacked block. Turn off the block on top.';
    return '';
  }
  const launchActive = () => state.scenario === 'incline' && cur().launch.on && !launchBlocked(cur());

  const mysteryOn = () => !!(cur().mystery && cur().mystery.on) && mysteryTargets().length > 0;
  function mysteryTargets() {
    const sc = state.scenario;
    const c = cur();
    if (sc === 'atwood') return [['m1', 'Mass 1'], ['m2', 'Mass 2']];
    if (sc === 'elevator') return [['person', 'Person']];
    if (sc === 'body' || sc === 'statics' || sc === 'lab' || sc === 'puzzles' || sc === 'car') return [];
    const out = c.blocks.map((b, i) => ['b' + i, 'Block ' + (i + 1)]);
    const nb = c.blocks.length;
    const hang = sc === 'modified' || (sc === 'incline' && c.hanging.on);
    if (hang) out.push(['hang', 'Block ' + (nb + 1) + ' (hanging)']);
    if (c.stack.on) out.push(['top', 'Block ' + (nb + (hang ? 2 : 1)) + ' (on top)']);
    return out;
  }
  function mysteryTarget() {
    const t = cur().mystery.target;
    const list = mysteryTargets();
    if (!list.length) return '';
    return list.some((x) => x[0] === t) ? t : list[0][0];
  }
  const isMystery = (target) => mysteryOn() && mysteryTarget() === target;
  const massOf = (target, given) => (isMystery(target) ? ui.mystery.value : given);
  // Which body index in the analysis is the mystery one?
  function mysteryBodyIndex() {
    if (!mysteryOn() || !analysis) return -1;
    const t = mysteryTarget();
    if (t === 'm1' || t === 'person') return 0;
    if (t === 'm2') return 1;
    if (t[0] === 'b') return +t.slice(1);
    if (!analysis.bodies) return -1;
    return analysis.bodies.findIndex((b) => b.kind === (t === 'hang' ? 'hanging' : 'top'));
  }
  const massText = (i, m) => (i === mysteryBodyIndex() && !ui.revealed ? '?' : trimNum(m));

  // Pick a hidden mass that makes the system actually move.
  function newMystery() {
    const key = state.scenario + ':' + mysteryTarget();
    for (let k = 0; k < 40; k++) {
      const value = state.scenario === 'elevator' ? Math.round(40 + Math.random() * 60) : Math.round((0.5 + Math.random() * 9.5) * 2) / 2;
      ui.mystery = { key, value };
      const an = solveCurrent();
      if (an.ok && !an.atRest) return;
    }
  }
  function ensureMystery() {
    if (!mysteryOn()) return;
    if (ui.mystery.key !== state.scenario + ':' + mysteryTarget()) newMystery();
  }
  function solveCurrent() {
    const mod = scenarioModule();
    if (mod) return mod.solve(cur());
    return isAtwood() ? P.solveAtwood(atwoodInput()) : P.solveChain(physicsInput());
  }
  const atwoodInput = () => ({ m1: massOf('m1', state.atwood.m1), m2: massOf('m2', state.atwood.m2), g: state.g });

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
  // "Block 2", "Blocks 1–3" or "Blocks 1 and 3"
  function nameOf(members, bodies) {
    const m = members.slice().sort((x, y) => x - y);
    if (m.length === 1) return bodies[m[0]].name;
    if (m[m.length - 1] - m[0] === m.length - 1) return blockSpan(m[0], m[m.length - 1], bodies);
    return 'Blocks ' + m.slice(0, -1).map((i) => i + 1).join(', ') + ' and ' + (m[m.length - 1] + 1);
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
    const blocks = c.blocks.map((b, i) => {
      const mus = c.sameMu ? c.mus : b.mus;
      const muk = Math.min(c.sameMu ? c.muk : b.muk, mus);
      return { m: massOf('b' + i, b.m), mus, muk };
    });
    const hang = sc === 'modified' || (sc === 'incline' && c.hanging.on);
    return {
      theta: sc === 'incline' ? c.theta : 0,
      g: state.g,
      friction: c.friction,
      blocks,
      links: c.links.slice(),
      longString: sc === 'horizontal' && !!c.longString && blocks.length >= 3,
      hanging: hang ? { m: massOf('hang', c.hanging.m) } : null,
      stack: c.stack.on && !launchActive()
        ? { base: Math.min(c.stack.base, blocks.length - 1), m: massOf('top', c.stack.m), mus: c.stack.mus, muk: c.stack.muk }
        : null,
      applied: c.applied.on
        ? {
            F: c.applied.F,
            angle: c.applied.angle,
            block: c.applied.block === 'top' ? (c.stack.on ? 'top' : 0) : Math.min(c.applied.block, blocks.length - 1),
          }
        : null,
    };
  }

  function computeLabels(an) {
    if (an.kind === 'atwood') return { links: [{ onLeft: 'T', onRight: 'T', name: 'T' }] };
    if (!an.links) return { links: [] };
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
      if (l.type === 'stack') {
        // left = base block, right = top block
        return {
          onLeft: 'f' + sub(R) + '→' + sub(L),
          onRight: 'f' + sub(L) + '→' + sub(R),
          nOnLeft: 'N' + sub(R) + '→' + sub(L),
          nOnRight: 'N' + sub(L) + '→' + sub(R),
          name: 'f' + sub(L) + '→' + sub(R),
        };
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

  const hiddenMass = (label) =>
    '<label class="field"><span>' + label + ' <span class="unit">(kg)</span></span><input type="text" value="?" disabled aria-label="' +
    label + ' is the mystery mass"></label>';
  const selectField = (label, id, path, options, value, stacked) =>
    '<label class="field' + (stacked ? ' stack' : '') + '" for="' + id + '"><span>' + label + '</span><select id="' + id + '" data-path="' + path + '">' +
    options.map(([v, t]) => '<option value="' + v + '"' + (String(v) === String(value) ? ' selected' : '') + '>' + t + '</option>').join('') +
    '</select></label>';

  function renderControls() {
    const sc = state.scenario;
    const c = cur();
    ensureMystery();
    let h = '';

    if (scenarioModule()) {
      h += scenarioModule().controls(c);
    } else if (sc === 'atwood') {
      h += '<div class="group"><h2>Masses</h2>';
      h += '<div class="blk"><span class="tag" style="background:var(--b1)">1</span>' +
        (isMystery('m1') ? hiddenMass('Mass 1') : numField('Mass 1', 'm1', c.m1, 'kg', { min: 0.01, step: 0.1 })) + '<span></span></div>';
      h += '<div class="blk"><span class="tag" style="background:var(--b2)">2</span>' +
        (isMystery('m2') ? hiddenMass('Mass 2') : numField('Mass 2', 'm2', c.m2, 'kg', { min: 0.01, step: 0.1 })) + '<span></span></div>';
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
        ' Choose whether neighbors touch, are tied with a string, or sit apart.</p>';
      const perMu = c.friction && !c.sameMu;
      c.blocks.forEach((b, i) => {
        if (i > 0) {
          const t = c.links[i - 1];
          h += '<div class="link-row"><span class="rl">' + (i) + '–' + (i + 1) + '</span><div class="seg" role="group" aria-label="Connection between Block ' + i + ' and Block ' + (i + 1) + '">' +
            '<button type="button" data-action="link" data-index="' + (i - 1) + '" data-type="contact" class="' + (t === 'contact' ? 'on' : '') + '" aria-pressed="' + (t === 'contact') + '">Touching</button>' +
            '<button type="button" data-action="link" data-index="' + (i - 1) + '" data-type="string" class="' + (t === 'string' ? 'on' : '') + '" aria-pressed="' + (t === 'string') + '">String</button>' +
            '<button type="button" data-action="link" data-index="' + (i - 1) + '" data-type="none" class="' + (t === 'none' ? 'on' : '') + '" aria-pressed="' + (t === 'none') + '">Apart</button>' +
            '</div></div>';
        }
        h += '<div class="blk"><span class="tag" style="background:var(--b' + (i + 1) + ')">' + (i + 1) + '</span><div class="vals">' +
          (isMystery('b' + i)
            ? '<label>Mass (kg)<input type="text" value="?" disabled aria-label="Block ' + (i + 1) + ' is the mystery mass"></label>'
            : '<label>Mass (kg)<input type="number" id="in-' + sc + '-b' + i + '-m" data-path="blocks.' + i + '.m" value="' + trimNum(b.m) + '" min="0.01" max="1000" step="0.1" inputmode="decimal"></label>') +
          (perMu
            ? '<label>μs<input type="number" id="in-' + sc + '-b' + i + '-mus" data-path="blocks.' + i + '.mus" value="' + trimNum(b.mus) + '" min="0" max="2" step="0.05"></label>' +
              '<label>μk<input type="number" id="in-' + sc + '-b' + i + '-muk" data-path="blocks.' + i + '.muk" value="' + trimNum(b.muk) + '" min="0" max="2" step="0.05"></label>'
            : '') +
          '</div><button type="button" class="icon-btn" data-action="remove" data-index="' + i + '" aria-label="Remove Block ' + (i + 1) + '"' + (c.blocks.length < 2 ? ' disabled' : '') + '>×</button></div>';
      });
      if (c.blocks.length < maxB) {
        h += '<div class="btn-row"><button type="button" class="btn btn-small" data-action="add">+ Add a block</button></div>';
      }
      if (sc === 'horizontal' && c.blocks.length >= 3) {
        const last = c.blocks.length;
        h += checkField('Tie a string from Block 1 to Block ' + last + ', passing over the blocks between', 'longString', c.longString);
        if (c.longString && c.links.every((t) => t !== 'none')) {
          h += '<p class="warn">With this string, set at least one pair of neighbors to Apart. Otherwise the connections form a closed loop.</p>';
        }
      }
      h += '</div>';

      const nb = c.blocks.length;
      if (sc === 'modified') {
        h += '<div class="group"><h2>Hanging block</h2>' +
          '<div class="blk"><span class="tag" style="background:var(--b' + (nb + 1) + ')">' + (nb + 1) + '</span>' +
          (isMystery('hang') ? hiddenMass('Block ' + (nb + 1) + ' mass') : numField('Block ' + (nb + 1) + ' mass', 'hanging.m', c.hanging.m, 'kg', { min: 0.01 })) + '<span></span></div>' +
          '<p class="note">Tied to Block ' + nb + ' by a string over the pulley at the table edge.</p></div>';
      } else if (sc === 'incline') {
        h += '<div class="group"><h2>Pulley</h2>' + checkField('Hang a block over a pulley at the top', 'hanging.on', c.hanging.on);
        if (c.hanging.on) {
          h += '<div class="blk"><span class="tag" style="background:var(--b' + (nb + 1) + ')">' + (nb + 1) + '</span>' +
            (isMystery('hang') ? hiddenMass('Block ' + (nb + 1) + ' mass') : numField('Block ' + (nb + 1) + ' mass', 'hanging.m', c.hanging.m, 'kg', { min: 0.01 })) + '<span></span></div>';
        }
        h += '</div>';

        // projectile launch off the table at the bottom of the ramp
        const why = launchBlocked(c);
        h += '<div class="group"><h2>Launch</h2>' + checkField('Fly off the table at the bottom of the ramp', 'launch.on', c.launch.on);
        if (c.launch.on && why) h += '<p class="warn">' + why + '</p>';
        if (c.launch.on && !why) {
          h += numField('Distance slid down the ramp, d', 'launch.d', c.launch.d, 'm', { min: 0.2, max: 5, step: 0.1 });
          h += numField('Table height, H', 'launch.H', c.launch.H, 'm', { min: 0.1, max: 3, step: 0.1 });
          h += numField('Launch angle φ (above horizontal)', 'launch.phi', c.launch.phi || 0, '°', { min: 0, max: 75, step: 1 });
          h += '<p class="note">' + ((c.launch.phi || 0) > 0
            ? 'A small kicker ramp at the bottom turns the block to φ above the horizontal without changing its speed. Its size is small enough to ignore.'
            : 'A smooth curve at the bottom turns the block horizontal without changing its speed.') +
            ' Then it flies as a projectile. Fₐ stops acting once the block leaves the ramp.</p>';
        }
        h += '</div>';
      }

      // a block resting on top of another block
      const hangNow = sc === 'modified' || (sc === 'incline' && c.hanging.on);
      const topNum = nb + (hangNow ? 2 : 1);
      if (!launchActive()) {
        h += '<div class="group"><h2>Stacked block</h2>' + checkField('Put a block on top of another block', 'stack.on', c.stack.on);
        if (c.stack.on) {
          h += '<div class="blk"><span class="tag" style="background:var(--b' + ((topNum - 1) % 5 + 1) + ')">' + topNum + '</span><div class="vals">' +
            (isMystery('top')
              ? '<label>Mass (kg)<input type="text" value="?" disabled aria-label="The top block is the mystery mass"></label>'
              : '<label>Mass (kg)<input type="number" id="in-' + sc + '-top-m" data-path="stack.m" value="' + trimNum(c.stack.m) + '" min="0.01" max="1000" step="0.1" inputmode="decimal"></label>') +
            '</div><span></span></div>';
          if (nb > 1) h += selectField('Sits on', 'sel-stack-base', 'stack.base', c.blocks.map((b, i) => [i, 'Block ' + (i + 1)]), c.stack.base);
          h += numField('μs between the blocks', 'stack.mus', c.stack.mus, '', { min: 0, max: 2, step: 0.05 });
          h += numField('μk between the blocks', 'stack.muk', c.stack.muk, '', { min: 0, max: 2, step: 0.05 });
          if (c.stack.muk > c.stack.mus) h += '<p class="warn">μk should not be larger than μs. The simulator uses μk = μs.</p>';
          h += '<p class="note">Block ' + topNum + ' rides along only if static friction between the blocks is strong enough. Otherwise it slips.</p>';
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
        const targets = c.blocks.map((b, i) => [i, 'Block ' + (i + 1)]);
        if (c.stack.on && !launchActive()) targets.push(['top', 'Block ' + topNum + ' (on top)']);
        h += selectField('Acts on', 'sel-applied-block', 'applied.block', targets, c.applied.block);
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
      '</div><p class="note" id="share-msg" hidden></p>' +
      (mysteryTargets().length ? checkField('Mystery mass', 'mystery.on', mysteryOn()) : '');
    if (mysteryOn()) {
      h += selectField('Hide the mass of', 'sel-mystery', 'mystery.target', mysteryTargets(), mysteryTarget());
      h += '<div class="btn-row"><button type="button" class="btn btn-small" data-action="mystery">New mystery mass</button></div>';
      h += '<p class="note">Measure the motion with the clock, ruler and graphs, then work out the hidden mass. Values and the solution stay hidden until you choose Show answers. A shared link gives each student a different mystery mass.</p>';
    }
    h += '</div>';

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
      if (path === 'longString' && el.checked && c.links.every((t) => t !== 'none')) c.links[0] = 'none';
      if (path === 'mystery.on') {
        ui.pred = {};
        if (el.checked) newMystery();
      }
      if (path === 'stack.on' && !el.checked && c.applied.block === 'top') c.applied.block = 0;
      if (state.scenario === 'lab' && path === 'hidden') {
        ui.labSecret = null;
        ui.labTrials = [];
        LAB.renderData();
      }
      renderControls();
      changed();
      return;
    }
    if (el.tagName === 'SELECT') {
      const v = el.value;
      setPath(c, path, /^-?\d+$/.test(v) ? parseInt(v, 10) : v);
      if (path === 'mystery.target') {
        ui.pred = {};
        newMystery();
        renderControls();
      }
      if (path === 'mode' || path === 'motion' || path === 'p' || path === 'type' || path === 'tackle.motion') {
        ui.pred = {};
        ui.solutionOpen = false;
        ui.graph = 'x';
        document.querySelectorAll('[data-graph]').forEach((x) => x.classList.toggle('on', x.dataset.graph === 'x'));
        renderControls();
      }
      changed();
      return;
    }
    const v = parseFloat(el.value);
    const min = parseFloat(el.min);
    const max = parseFloat(el.max);
    const positive = /\.m$|^m\d$|^m$/.test(path) || path === 'hanging.m' || path === 'stack.m';
    const ok = isFinite(v) && v >= min && v <= max && !(positive && v <= 0);
    el.setAttribute('aria-invalid', ok ? 'false' : 'true');
    if (!ok) return;
    setPath(c, path, v);
    if (path === 'theta') {
      const other = el.type === 'range' ? $('in-incline-theta') : $('rng-theta');
      if (other) other.value = v;
    }
    if ((/mu[sk]$/.test(path) || path === 'launch.phi') && e.type === 'change') renderControls();
    if (isFma() && (path === 'M' || path === 'f') && (ui.labTrials || []).length) {
      ui.labTrials = [];
      LAB.renderData();
      if (e.type === 'change') renderControls();
    }
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
      if (typeof c.applied.block === 'number' && c.applied.block >= c.blocks.length) c.applied.block = c.blocks.length - 1;
      if (c.stack.base >= c.blocks.length) c.stack.base = c.blocks.length - 1;
    } else if (act === 'link') {
      c.links[+btn.dataset.index] = btn.dataset.type;
    } else if (act === 'random') {
      randomize();
    } else if (act === 'newcar') {
      ui.carSecret = null;
      ui.pred = {};
    } else if (act === 'newtilt') {
      ui.tiltSecret = null;
      ui.pred = {};
    } else if (act === 'newlab') {
      ui.labSecret = null;
      ui.labTrials = [];
      ui.pred = {};
      LAB.renderData();
    } else if (act === 'labclear') {
      ui.labTrials = [];
      LAB.renderData();
    } else if (act === 'mystery') {
      ui.pred = {};
      newMystery();
    } else if (act === 'share') {
      share();
      return;
    } else if (act === 'defaults') {
      state[state.scenario] = defaults()[state.scenario];
      if (state.scenario === 'lab') {
        ui.labTrials = [];
        LAB.renderData();
      }
      ui.pred = {};
    } else return;
    renderControls();
    changed();
  }

  function randomize() {
    const r = (a, b, step) => Math.round((a + Math.random() * (b - a)) / step) * step;
    const c = cur();
    if (scenarioModule()) {
      scenarioModule().randomize(c, r);
    } else if (isAtwood()) {
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
      c.stack.m = r(0.5, 5, 0.5);
      c.stack.mus = r(0.2, 0.7, 0.05);
      c.stack.muk = Math.max(0.05, Math.round((c.stack.mus - r(0.05, 0.2, 0.05)) * 100) / 100);
      if (state.scenario === 'incline') {
        c.launch.d = r(1, 3, 0.1);
        c.launch.H = r(0.5, 1.5, 0.1);
        if (c.launch.phi > 0) c.launch.phi = r(10, 50, 5);
      }
    }
    ui.pred = {};
    if (mysteryOn()) newMystery();
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
        ? 'Link copied. Anyone who opens it gets this exact setup' + (mysteryOn() ? ', with their own mystery mass.' : '.')
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
      const my = c.mystery || {};
      const mystery = { on: !!my.on, target: typeof my.target === 'string' ? my.target : data.scenario === 'atwood' ? 'm1' : 'b0' };
      const mod = MODULES[data.scenario];
      if (mod) {
        state[data.scenario] = Object.assign(mod.load(c, num), data.scenario === 'elevator' ? { mystery: { on: !!my.on, target: 'person' } } : {});
      } else if (data.scenario === 'atwood') {
        state.atwood = { m1: num(c.m1, 3, 0.01, 1000), m2: num(c.m2, 5, 0.01, 1000), mystery };
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
          Array.isArray(c.links) && ['string', 'contact', 'none'].includes(c.links[j]) ? c.links[j] : 'contact'
        );
        const ap = c.applied || {};
        const hg = c.hanging || {};
        const st = c.stack || {};
        const la = c.launch || {};
        state[data.scenario] = {
          theta: num(c.theta, d.theta, 0, 85),
          blocks,
          links,
          friction: !!c.friction,
          longString: !!c.longString,
          sameMu: c.sameMu !== false,
          mus: num(c.mus, 0.4, 0, 2),
          muk: num(c.muk, 0.3, 0, 2),
          hanging: { on: data.scenario === 'modified' ? true : !!hg.on, m: num(hg.m, 2, 0.01, 1000) },
          applied: {
            on: !!ap.on,
            F: num(ap.F, 20, 0, 10000),
            angle: num(ap.angle, 0, -180, 180),
            block: ap.block === 'top' ? 'top' : Math.min(Math.max(0, Math.round(num(ap.block, 0, 0, 10))), blocks.length - 1),
          },
          stack: {
            on: !!st.on,
            base: Math.min(Math.max(0, Math.round(num(st.base, 0, 0, 10))), blocks.length - 1),
            m: num(st.m, 1, 0.01, 1000),
            mus: num(st.mus, 0.5, 0, 2),
            muk: num(st.muk, 0.4, 0, 2),
          },
          launch: { on: !!la.on, d: num(la.d, 2, 0.2, 5), H: num(la.H, 1, 0.1, 3), phi: num(la.phi, 0, 0, 75) },
          mystery,
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
    ensureMystery();
    analysis = solveCurrent();
    labels = computeLabels(analysis);
    sim = buildSim(analysis);
    anim.t = 0;
    anim.playing = false;
    anim.done = false;
    ui.revealed = false;
    renderScenarioTabs();
    syncModes();
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
    if (xsFor(an)) return xsFor(an).buildSim(an);
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
    // The incline rises to the left, so its low end (and any launch) faces right.
    const dirX = state.scenario === 'incline' ? -1 : 1;
    const u = { x: dirX * Math.cos(th), y: Math.sin(th) }; // + along the surface (up the slope)
    const n = { x: -dirX * Math.sin(th), y: Math.cos(th) }; // away from the surface
    const blocks = an.bodies.filter((b) => b.kind === 'block');
    const hangBody = an.bodies.find((b) => b.kind === 'hanging');
    const topBody = an.bodies.find((b) => b.kind === 'top');
    const nb = blocks.length;
    const w = 0.42;
    const h = 0.3;
    const hw = 0.3;
    const wt = 0.3;
    const ht = 0.24;
    const strLen = 0.4;
    const apart = 0.6;
    const gaps = an.neighbors.map((t) => (t === 'string' ? strLen : t === 'none' ? apart : 0));
    const chain = nb * w + gaps.reduce((a, b) => a + b, 0);
    const L = launchActive() ? cur().launch : null;
    const Ls = L ? L.d + w / 2 + 0.25 : chain + (state.scenario === 'incline' ? 2.0 : 2.6);
    const H = L ? L.H : hangBody ? (an.theta > 0 ? 1.0 : 1.4) : an.theta > 0 ? 0.25 : 0.5;
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
    if (L) x0[0] = L.d;

    const geom = { th, dirX, u, n, w, h, hw, wt, ht, strLen, gaps, Ls, H, S0, E, x0, nb, hang: !!hangBody, top: !!topBody, launch: !!L };
    if (hangBody) {
      const rp = 0.11;
      const k = (0.09 + (h / 2 - rp) * Math.sin(th)) / Math.cos(th) + 0.01;
      const C = { x: E.x + u.x * k + n.x * (h / 2 - rp), y: E.y + u.y * k + n.y * (h / 2 - rp) };
      geom.rp = rp;
      geom.C = C;
      geom.T1 = { x: C.x + n.x * rp, y: C.y + n.y * rp };
      geom.T2 = { x: C.x + dirX * rp, y: C.y };
      geom.dMin = 0.06;
      geom.dMax = geom.T2.y - hw;
      geom.d0 = dir > 0 ? geom.dMin + 0.04 : dir < 0 ? geom.dMax - 0.02 : (geom.dMin + geom.dMax) / 2;
    }

    // Projectile flight after the ramp
    let fl = null;
    if (L) {
      const b = blocks[0];
      fl = P.launch({ a: Math.max(0, -b.a), d: L.d, H: L.H, g: an.g, phi: L.phi || 0 });
      geom.flight = fl;
    }

    const xs0 = [S0.x, E.x, hangBody ? geom.T2.x - dirX * hw / 2 : S0.x];
    if (fl) xs0.push(fl.R + 0.4);
    const xMin = Math.min(...xs0);
    const xMax = Math.max(...xs0);
    const top = Math.max(E.y + h + (topBody ? ht : 0) + 0.4, hangBody ? geom.C.y + 0.3 : 0, fl ? H + h + fl.hMax + 0.35 : 0, 0.9);
    geom.bbox = { x0: xMin - (dirX > 0 ? 0.75 : 0.4), x1: xMax + (dirX > 0 ? 0.35 : 0.6), y0: -0.12, y1: top + 0.25 };

    const tRamp = fl ? fl.tRamp : Infinity;
    const pos = (t) => {
      const tr = Math.min(t, tRamp);
      const xs = blocks.map((b, i) => x0[i] + 0.5 * b.a * tr * tr);
      const d = hangBody ? geom.d0 + 0.5 * hangBody.a * t * t : 0;
      const xt = topBody ? x0[topBody.base] + 0.5 * topBody.a * t * t : 0;
      let air = null;
      if (fl && t > tRamp) {
        const tau = t - tRamp;
        air = { x: S0.x + fl.vx * tau, y: H + h / 2 + fl.vy0 * tau - 0.5 * an.g * tau * tau, tau };
      }
      return { xs, d, xt, air };
    };
    const end = hangBody ? 'the pulley' : an.theta > 0 ? 'the top of the ramp' : 'the end of the track';
    const start = an.theta > 0 ? 'the bottom of the ramp' : 'the end of the track';
    const violation = (t) => {
      const p = pos(t);
      if (fl) {
        if (p.air && p.air.y - h / 2 <= 0) return 'Block 1 landed ' + fmt(fl.R) + ' m from the table edge';
        return null;
      }
      if (p.xs[0] - w / 2 < -1e-9) return 'Block 1 reached ' + start;
      if (p.xs[nb - 1] + w / 2 > Ls + 1e-9) return 'Block ' + nb + ' reached ' + end;
      for (let i = 0; i < nb - 1; i++) {
        if (p.xs[i + 1] - p.xs[i] < w - 1e-9)
          return an.neighbors[i] === 'string'
            ? 'Block ' + (i + 1) + ' and Block ' + (i + 2) + ' collided after the string between them went slack'
            : 'Block ' + (i + 1) + ' and Block ' + (i + 2) + ' collided';
      }
      if (hangBody) {
        if (p.d < geom.dMin) return 'Block ' + (nb + 1) + ' reached the pulley';
        if (p.d > geom.dMax) return 'Block ' + (nb + 1) + ' hit the floor';
      }
      if (topBody && Math.abs(p.xt - p.xs[topBody.base]) > w / 2) {
        return topBody.name.replace(' (on top)', '') + ' slid off Block ' + (topBody.base + 1);
      }
      return null;
    };
    let stop;
    if (fl) stop = { t: fl.tRamp + fl.tAir, reason: 'Block 1 landed ' + fmt(fl.R) + ' m from the table edge' };
    else stop = an.atRest ? { t: REST_RUN, reason: null } : findStop(violation);
    return { kind: 'surface', geom, pos, tStop: stop.t, reason: stop.reason };
  }

  // Displacement and velocity of a body along its own + direction.
  // For a launch, x is horizontal distance from the table edge and v splits into vx, vy.
  function bodyMotion(i, t) {
    if (xsFor(analysis)) return xsFor(analysis).motion(analysis, t);
    const b = analysis.bodies[i];
    const a = b.a;
    const fl = sim && sim.geom && sim.geom.flight;
    if (fl) {
      const th = sim.geom.th;
      const d = sim.geom.x0[0];
      if (t <= fl.tRamp) {
        const sAlong = d + 0.5 * a * t * t; // distance up the ramp from the edge
        const sp = -a * t; // speed down the ramp
        return { s: 0.5 * a * t * t, v: a * t, a, x: -sAlong * Math.cos(th), vx: sp * Math.cos(th), vy: -sp * Math.sin(th), air: false, speed: sp };
      }
      const tau = t - fl.tRamp;
      const vy = fl.vy0 - analysis.g * tau;
      return { s: 0, v: 0, a: 0, x: fl.vx * tau, vx: fl.vx, vy, air: true, speed: Math.hypot(fl.vx, vy) };
    }
    return { s: 0.5 * a * t * t, v: a * t, a };
  }

  /* ======================================================================
   * Colors from CSS tokens
   * ==================================================================== */

  let C = {};
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    const names = ['ink', 'muted', 'line', 'panel', 'scene', 'grid', 'bench', 'bench-edge', 'accent',
      'f-w', 'f-n', 'f-f', 'f-a', 'l1', 'l2', 'l3', 'l4', 'b1', 'b2', 'b3', 'b4', 'b5',
      'fig-skin', 'fig-shirt', 'fig-shorts', 'fig-shoe', 'fig-hair', 'track', 'horse'];
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
    const big = Math.max(12, Math.min(20, view.k * 0.13));
    ctx.translate(px, py);
    // keep labels upright whichever way the surface slopes
    ctx.rotate(-(u.x < 0 ? Math.atan2(-u.y, -u.x) : Math.atan2(u.y, u.x)));
    ctx.fillStyle = C.ink;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const roomy = w * view.k > 42 && h * view.k > 30;
    ctx.font = '700 ' + big + 'px ' + C.fontMono;
    ctx.fillText(String(idx + 1), 0, roomy ? -big * 0.45 : 0);
    if (roomy) {
      ctx.font = '400 ' + Math.round(big * 0.72) + 'px ' + C.fontMono;
      ctx.fillText((typeof mass === 'string' ? mass : trimNum(mass)) + ' kg', 0, big * 0.6);
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
    if (xsFor(analysis)) {
      xsFor(analysis).draw(ctx, W, H, anim.t);
      return;
    }
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
    ctx.font = '12px ' + C.fontMono;
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
    drawBlock(ctx, view, { x: -g.r, y: g.pc.y - pos.d1 - g.h / 2 }, up, nn, g.w, g.h, 0, massText(0, an.bodies[0].m));
    drawBlock(ctx, view, { x: g.r, y: g.pc.y - pos.d2 - g.h / 2 }, up, nn, g.w, g.h, 1, massText(1, an.bodies[1].m));
  }

  function drawSurface(ctx, view, t) {
    const g = sim.geom;
    const an = analysis;
    const P = view.P;
    const { u, n, w, h, dirX } = g;
    const pos = sim.pos(t);
    const blocks = an.bodies.filter((b) => b.kind === 'block');
    const hangIdx = an.bodies.findIndex((b) => b.kind === 'hanging');
    const topIdx = an.bodies.findIndex((b) => b.kind === 'top');
    const at = (sAlong, up) => ({ x: g.S0.x + u.x * sAlong + n.x * up, y: g.S0.y + u.y * sAlong + n.y * up });

    drawFloor(ctx, view, g.bbox.x0 + 0.1, g.bbox.x1 - 0.1);

    // bench / ramp / launch table
    ctx.save();
    ctx.fillStyle = C.bench;
    ctx.strokeStyle = C['bench-edge'];
    ctx.lineWidth = 1.5;
    const poly = (pts) => {
      const q = pts.map(([x, y]) => P(x, y));
      ctx.beginPath();
      ctx.moveTo(q[0][0], q[0][1]);
      q.slice(1).forEach((p) => ctx.lineTo(p[0], p[1]));
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    };
    if (g.launch) {
      // a table whose right edge is the bottom of the ramp, with the ramp on top
      const slab = 0.08;
      const left = g.E.x - 0.15;
      poly([[left, g.H], [g.S0.x, g.H], [g.S0.x, g.H - slab], [left, g.H - slab]]);
      for (const lx of [left + 0.08, g.S0.x - 0.16]) poly([[lx, g.H - slab], [lx + 0.08, g.H - slab], [lx + 0.08, 0], [lx, 0]]);
      if (g.th > 0) poly([[g.S0.x, g.S0.y], [g.E.x, g.H], [g.E.x, g.E.y]]);
      const phiDeg = g.flight ? g.flight.phi : 0;
      const [lx2, ly2] = P(g.S0.x, g.S0.y);
      if (phiDeg > 0) {
        // a small kicker ramp: curves up from the slope to the launch angle at the table edge
        const ph = phiDeg * DEG;
        const len = 0.2;
        const back = 0.28;
        const start = { x: g.S0.x - back * Math.cos(g.th), y: g.S0.y + back * Math.sin(g.th) };
        const tip = { x: g.S0.x + len * Math.cos(ph), y: g.S0.y + len * Math.sin(ph) };
        const ctrl = { x: g.S0.x - 0.02, y: g.S0.y };
        const [sx, sy] = P(start.x, start.y);
        const [cx2, cy2] = P(ctrl.x, ctrl.y);
        const [tx2, ty2] = P(tip.x, tip.y);
        const [, baseY] = P(0, g.H);
        ctx.fillStyle = C['bench-edge'];
        ctx.strokeStyle = C.ink;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.quadraticCurveTo(cx2, cy2, tx2, ty2);
        ctx.lineTo(tx2, baseY);
        ctx.lineTo(sx, baseY);
        ctx.closePath();
        ctx.fill();
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.quadraticCurveTo(cx2, cy2, tx2, ty2);
        ctx.stroke();
        // launch angle marker
        ctx.save();
        ctx.strokeStyle = C.muted;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(tx2, ty2);
        ctx.lineTo(tx2 + 0.45 * view.k, ty2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(tx2, ty2);
        ctx.lineTo(tx2 + 0.45 * view.k * Math.cos(ph), ty2 - 0.45 * view.k * Math.sin(ph));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.arc(tx2, ty2, 0.3 * view.k, -ph, 0);
        ctx.stroke();
        ctx.restore();
        ctx.fillStyle = C.ink;
        ctx.font = '600 15px ' + C.fontMono;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText('φ = ' + trimNum(phiDeg) + '°', tx2 + 0.34 * view.k * Math.cos(ph / 2) + 4, ty2 - 0.34 * view.k * Math.sin(ph / 2) - 6);
      } else {
        // smooth lip at the bottom
        ctx.strokeStyle = C.ink;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(lx2 - 10, ly2 - 10 * Math.tan(g.th));
        ctx.quadraticCurveTo(lx2 - 4, ly2, lx2, ly2);
        ctx.stroke();
      }
    } else if (g.th === 0) {
      const slab = 0.08;
      poly([[0, g.H], [g.E.x, g.H], [g.E.x, g.H - slab], [0, g.H - slab]]);
      for (const lx of [0.12, g.Ls - 0.2]) poly([[lx, g.H - slab], [lx + 0.08, g.H - slab], [lx + 0.08, 0], [lx, 0]]);
    } else {
      poly([[g.S0.x, 0], [g.E.x, 0], [g.E.x, g.E.y], [g.S0.x, g.S0.y]]);
    }
    if (g.th > 0) {
      // angle marker at the bottom of the slope, inside the ramp
      const [bx, by] = P(g.S0.x, g.S0.y);
      const rr = Math.min(70, 0.55 * view.k);
      ctx.strokeStyle = C.muted;
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(bx, by);
      ctx.lineTo(bx + dirX * rr * 1.5, by);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath();
      if (dirX > 0) ctx.arc(bx, by, rr, -g.th, 0);
      else ctx.arc(bx, by, rr, -Math.PI, -Math.PI + g.th);
      ctx.stroke();
      ctx.fillStyle = C.ink;
      ctx.font = '600 15px ' + C.fontMono;
      ctx.textBaseline = 'middle';
      const la = g.th * 0.35;
      ctx.textAlign = dirX > 0 ? 'left' : 'right';
      ctx.fillText('θ = ' + trimNum(an.theta) + '°', bx + dirX * (rr + 8) * Math.cos(la), by - (rr + 8) * Math.sin(la) + 8);
    }
    ctx.restore();

    // ruler along the surface
    ctx.save();
    ctx.strokeStyle = C.muted;
    ctx.fillStyle = C.muted;
    ctx.lineWidth = 1;
    ctx.font = '12px ' + C.fontMono;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const nTicks = Math.floor(g.Ls * 10 + 1e-9);
    for (let i = 0; i <= nTicks; i++) {
      const s2 = i * 0.1;
      const base = at(s2, 0);
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
    // landing ruler on the floor for a launch
    if (g.launch && g.flight) {
      const R = g.flight.R;
      const maxX = Math.ceil((R + 0.3) * 10);
      for (let i = 0; i <= maxX; i++) {
        const [ax, ay] = P(g.S0.x + i * 0.1, 0);
        const len = i % 10 === 0 ? 9 : i % 5 === 0 ? 6 : 3;
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(ax, ay - len);
        ctx.stroke();
        if (i % 5 === 0 && view.k > 50) ctx.fillText(i % 10 === 0 ? i / 10 + ' m' : (i / 10).toFixed(1), ax, ay + 14);
      }
      // dashed line down from the table edge
      const [ex, ey] = P(g.S0.x, g.H);
      const [, fy] = P(g.S0.x, 0);
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex, fy);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.restore();

    // end stops
    ctx.save();
    ctx.fillStyle = C['bench-edge'];
    const stops = [];
    if (!g.launch) stops.push(0);
    if (!g.hang) stops.push(g.Ls);
    for (const s2 of stops) {
      const dirOut = s2 === 0 ? -1 : 1;
      const base = at(s2, 0);
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
      const front = at(pos.xs[last] + w / 2, h / 2);
      const slack = !an.links.find((l) => l.hanging).engaged;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.6;
      if (slack) ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const [fx, fy] = P(front.x, front.y);
      const [t1x, t1y] = P(g.T1.x, g.T1.y);
      ctx.moveTo(fx, fy);
      ctx.lineTo(t1x, t1y);
      const angN = Math.atan2(n.y, n.x);
      if (dirX > 0) ctx.arc(cx, cy, g.rp * view.k, -angN, 0, false);
      else ctx.arc(cx, cy, g.rp * view.k, -angN, -Math.PI, true);
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
    an.links.forEach((l) => {
      if (l.hanging || l.type !== 'string') return;
      if (l.long) {
        // runs over the blocks between, on two short posts (higher if a block is stacked)
        const y = h + (g.top ? g.ht : 0) + 0.1;
        const A0 = pos.xs[l.left];
        const B0 = pos.xs[l.right];
        const pts = [[A0, h], [A0, y], [B0, y], [B0, h]].map(([sAlong, up]) => {
          const q = at(sAlong, up);
          return P(q.x, q.y);
        });
        ctx.save();
        if (!l.engaged) ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(pts[0][0], pts[0][1]);
        pts.slice(1).forEach((q) => ctx.lineTo(q[0], q[1]));
        ctx.stroke();
        ctx.restore();
        return;
      }
      const a = pos.xs[l.left] + w / 2;
      const b = pos.xs[l.right] - w / 2;
      const A = at(a, h / 2);
      const B = at(b, h / 2);
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

    const centerOf = (s2) => at(s2, h / 2);
    const topCenter = (s2) => at(s2, h + g.ht / 2);
    const hangCenter = (d) => ({ x: g.T2.x, y: g.T2.y - d - g.hw / 2 });
    const ux = { x: 1, y: 0 };
    const uy = { x: 0, y: 1 };
    const block1 = (p) => (p.air ? { c: { x: p.air.x, y: p.air.y }, uu: ux, nn: uy } : { c: centerOf(p.xs[0]), uu: u, nn: n });

    for (const ts of strobeTimes(t)) {
      const p = sim.pos(ts);
      blocks.forEach((b, i) => {
        if (i === 0) {
          const q = block1(p);
          drawBlock(ctx, view, q.c, q.uu, q.nn, w, h, 0, 0, true);
        } else drawBlock(ctx, view, centerOf(p.xs[i]), u, n, w, h, i, 0, true);
      });
      if (g.hang) drawBlock(ctx, view, hangCenter(p.d), ux, uy, g.hw, g.hw, hangIdx, 0, true);
      if (g.top) drawBlock(ctx, view, topCenter(p.xt), u, n, g.wt, g.ht, topIdx, 0, true);
    }
    blocks.forEach((b, i) => {
      if (i === 0) {
        const q = block1(pos);
        drawBlock(ctx, view, q.c, q.uu, q.nn, w, h, 0, massText(0, b.m));
      } else drawBlock(ctx, view, centerOf(pos.xs[i]), u, n, w, h, i, massText(i, b.m));
    });
    if (g.hang) drawBlock(ctx, view, hangCenter(pos.d), ux, uy, g.hw, g.hw, hangIdx, massText(hangIdx, an.bodies[hangIdx].m));
    if (g.top) drawBlock(ctx, view, topCenter(pos.xt), u, n, g.wt, g.ht, topIdx, massText(topIdx, an.bodies[topIdx].m));

    // applied force (only while the block is on the surface)
    const ap = physicsInput().applied;
    if (ap && ap.F > 0 && !(pos.air && ap.block === 0)) {
      const onTop = ap.block === 'top';
      const i = onTop ? -1 : ap.block;
      const c = onTop ? topCenter(pos.xt) : centerOf(pos.xs[i]);
      const bw = onTop ? g.wt : w;
      const bh = onTop ? g.ht : h;
      const ang = ap.angle * DEG;
      const d = { x: u.x * Math.cos(ang) + n.x * Math.sin(ang), y: u.y * Math.cos(ang) + n.y * Math.sin(ang) };
      const dl = { a: Math.cos(ang), b: Math.sin(ang) }; // local (along, perpendicular)
      const tEdge = Math.min(
        Math.abs(dl.a) > 1e-9 ? bw / 2 / Math.abs(dl.a) : Infinity,
        Math.abs(dl.b) > 1e-9 ? bh / 2 / Math.abs(dl.b) : Infinity
      );
      let len = 0.55;
      // keep a push arrow out of the neighbor behind it
      if (!onTop && !(dl.b > 1e-6) && Math.abs(dl.a) > 0.5) {
        const back = dl.a > 0 ? i - 1 : i + 1;
        if (back >= 0 && back < g.nb) {
          const room = Math.abs(pos.xs[i] - pos.xs[back]) - w - 0.04;
          len = Math.max(0.12, Math.min(len, room / Math.abs(dl.a)));
        }
      }
      let tail;
      let tip;
      if (dl.b > 1e-6 || onTop) {
        // pulls from the face it leaves
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
      ctx.font = '700 15px ' + C.fontMono;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const lift = an.links.some((l) => l.long) ? 0.2 : 0.06;
      const [, topY] = P(c.x + n.x * (bh / 2 + lift), c.y + n.y * (bh / 2 + lift));
      const pull = dl.b > 1e-6 || onTop;
      // a push label sits just above its arrow, ending at the arrow tip
      const lab = pull ? [x2 + (x2 - x1) * 0.25, Math.min(y2, topY) - 12] : [x2 - 8, Math.min(y1, y2) - 11];
      if (!pull) ctx.textAlign = x2 >= x1 ? 'right' : 'left';
      if (!pull && x2 < x1) lab[0] = x2 + 8;
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
    if (xsFor(an)) return xsFor(an).forceSets(an, anim.t);
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
    const dirX = state.scenario === 'incline' ? -1 : 1;
    const u = { x: dirX * Math.cos(th), y: Math.sin(th) };
    const n = { x: -dirX * Math.sin(th), y: Math.cos(th) };
    const ap = physicsInput().applied;
    return an.bodies.map((b, i) => {
      const along = (v) => (b.kind === 'hanging' ? { x: 0, y: -Math.sign(v) } : { x: u.x * Math.sign(v), y: u.y * Math.sign(v) });
      const arrows = [{ dir: { x: 0, y: -1 }, mag: b.W, color: C['f-w'], label: 'W' }];
      if (b.kind === 'block') {
        if (b.N > 1e-9) arrows.push({ dir: n, mag: b.N, color: C['f-n'], label: 'N' });
        if (Math.abs(b.f) > 1e-9) arrows.push({ dir: along(b.f), mag: Math.abs(b.f), color: C['f-f'], label: b.fType === 'static' ? 'fₛ' : 'fₖ' });
      }
      const apHere = ap && ap.F > 0 && (ap.block === i || (ap.block === 'top' && b.kind === 'top'));
      if (apHere) {
        const ang = ap.angle * DEG;
        arrows.push({ dir: { x: u.x * Math.cos(ang) + n.x * Math.sin(ang), y: u.y * Math.cos(ang) + n.y * Math.sin(ang) }, mag: ap.F, color: C['f-a'], label: 'Fₐ' });
      }
      an.links.forEach((l, j) => {
        const nm = labels.links[j];
        if (l.type === 'stack') {
          // two third-law pairs: the normal pair and the friction pair
          if (l.right === i) {
            if (l.N > 1e-9) arrows.push({ dir: n, mag: l.N, color: linkColor(j), label: nm.nOnRight });
            if (l.F > 1e-9) arrows.push({ dir: along(l.Fs), mag: l.F, color: linkColor(j), label: nm.onRight, note: l.fType === 'static' ? 'static' : 'kinetic' });
          }
          if (l.left === i) {
            if (l.N > 1e-9) arrows.push({ dir: { x: -n.x, y: -n.y }, mag: l.N, color: linkColor(j), label: nm.nOnLeft });
            if (l.F > 1e-9) arrows.push({ dir: along(-l.Fs), mag: l.F, color: linkColor(j), label: nm.onLeft, note: l.fType === 'static' ? 'static' : 'kinetic' });
          }
          return;
        }
        if (l.F <= 1e-9 && !l.indeterminate) return;
        const sg = P.linkSigns(l.type);
        if (l.left === i) arrows.push({ dir: along(sg.left), mag: l.F, color: linkColor(j), label: nm.onLeft, dashed: l.indeterminate });
        if (l.right === i) arrows.push({ dir: along(sg.right), mag: l.F, color: linkColor(j), label: nm.onRight, dashed: l.indeterminate });
      });
      return { body: b, idx: i, tilt: b.kind === 'hanging' ? 0 : dirX * th, arrows };
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
    const cellW = Math.max(230, wrap.clientWidth / nCells);
    const totalW = Math.floor(cellW * nCells);
    const maxArrows = Math.max(...sets.map((s) => s.arrows.filter((a) => a.mag > 1e-9).length));
    const baseH = 320;
    const fullH = baseH + (ui.showFbdValues ? maxArrows * 20 + 10 : 0);
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
      if (s.body.kind !== 'hanging' && !s.pose) {
        ctx.strokeStyle = C.muted;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(-60, 20);
        ctx.lineTo(60, 20);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (!s.pose) {
        ctx.fillStyle = bodyColor(s.idx);
        ctx.strokeStyle = C.ink;
        ctx.lineWidth = 1.4;
        ctx.fillRect(-26, -20, 52, 40);
        ctx.strokeRect(-26, -20, 52, 40);
      }
      ctx.restore();
      if (s.pose) drawFigureInCell(ctx, s.pose, cx, cy);

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
          const L = Math.max(32, a.mag * scale);
          const x1 = cx + px;
          const y1 = cy + py;
          const x2 = x1 + dx * L;
          const y2 = y1 + dy * L;
          arrow(ctx, x1, y1, x2, y2, a.color, 2.6, 9, a.dashed);
          const text = a.label ? a.label + (a.dashed ? ' ?' : '') : '';
          ctx.save();
          ctx.font = '700 14px ' + C.fontMono;
          ctx.fillStyle = a.color;
          ctx.textBaseline = 'middle';
          const tw = ctx.measureText(text).width;
          let lx = x2 + dx * 10;
          let ly = y2 + dy * 12;
          if (Math.abs(dx) > 0.5) ctx.textAlign = dx > 0 ? 'left' : 'right';
          else if (list.length > 1) {
            const leftSide = k < (list.length - 1) / 2;
            ctx.textAlign = leftSide ? 'right' : 'left';
            lx = x2 + (leftSide ? -7 : 7);
            ly = y2;
          } else ctx.textAlign = 'center';
          if (ctx.textAlign === 'left') lx = Math.min(lx, cellW * (ci + 1) - tw - 4);
          if (ctx.textAlign === 'right') lx = Math.max(lx, cellW * ci + tw + 4);
          if (Math.abs(dx) > 0.5 && list.length > 1) ly += (k - (list.length - 1) / 2) * 17;
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
      ctx.font = '700 15px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'alphabetic';
      ctx.fillText(s.body.name.replace(/ \((hanging|on top)\)/, '') + (s.body.m == null ? '' : ' · ' + massText(s.idx, s.body.m) + ' kg'), cx, baseH - 22);
      if (ui.showFbdValues) {
        ctx.font = '600 14px ' + C.fontMono;
        s.arrows
          .filter((a) => a.mag > 1e-9 && a.label)
          .forEach((a, k) => {
            ctx.fillStyle = a.color;
            ctx.fillText(a.label + ' = ' + (a.dashed ? '?' : fmt(a.mag) + ' N') + (a.note ? ' (' + a.note + ')' : ''), cx, baseH + k * 20);
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
    if (xsFor(analysis)) {
      const out = xsFor(analysis).legend(analysis, (ui.showFbdValues || ui.revealed) && !(mysteryOn() && !ui.revealed));
      legend.innerHTML = out.legend;
      pairs.innerHTML = out.pairs;
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
          h += chip(
            'var(--l' + ((j % 4) + 1) + ')',
            l.type === 'string' ? nm.name + ': tension' : l.type === 'stack' ? nm.nOnRight + ', ' + nm.onRight + ' and their partners: between stacked blocks' : nm.onRight + ' / ' + nm.onLeft + ': contact pair'
          );
        });
    if (analysis.kind === 'chain' && analysis.links.some((l) => l.indeterminate)) h += '<span>Dashed “?”: not uniquely determined</span>';
    legend.innerHTML = h;

    const showV = (ui.showFbdValues || ui.revealed) && !(mysteryOn() && !ui.revealed);
    const val = (F) => (showV ? ' (' + fmt(F) + ' N each)' : '');
    let items = [];
    if (analysis.kind === 'chain') {
      const dw = dirWords();
      analysis.links.forEach((l, j) => {
        const nm = labels.links[j];
        const A = analysis.bodies[l.left];
        const B = analysis.bodies[l.right];
        const sw = '<span class="swatch" style="background:var(--l' + ((j % 4) + 1) + ')"></span>';
        if (l.type === 'stack') {
          const kind = l.fType === 'static' ? 'Static' : 'Kinetic';
          items.push(sw + '<b>' + nm.nOnRight + '</b> (' + A.name + ' pushes up on ' + B.name.replace(' (on top)', '') + ') and <b>' + nm.nOnLeft + '</b> (' + B.name.replace(' (on top)', '') + ' pushes down on ' + A.name +
            ') are a <b>third-law pair</b>' + val(l.N) + '. That is why ' + A.name + '’s normal force from the surface is larger than its own weight.');
          if (l.F > 1e-9)
            items.push(sw + kind + ' friction <b>' + nm.onRight + '</b> on the top block and <b>' + nm.onLeft + '</b> on the block below are a <b>third-law pair</b>' + val(l.F) +
              ': same size, opposite directions. ' + (l.engaged ? 'The top block rides along without slipping.' : 'The top block slips, so the friction is kinetic.'));
          else items.push(sw + 'No friction acts between the stacked blocks.');
          return;
        }
        if (l.type === 'contact') {
          if (!l.engaged) items.push(sw + A.name + ' and ' + B.name + ' are moving apart, so they do not push on each other. The contact force is zero.');
          else
            items.push(sw + '<b>' + nm.onRight + '</b> (' + A.name + ' pushes ' + B.name + ' ' + dw.pos + ') and <b>' + nm.onLeft + '</b> (' + B.name + ' pushes ' + A.name + ' ' + dw.neg +
              ') are a <b>Newton’s third-law pair</b>: equal in size' + val(l.F) + ', opposite in direction, and acting on different objects.');
        } else {
          const where = l.hanging ? 'over the pulley' : l.long ? '(over the blocks between)' : '';
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
    if (sim && sim.geom && sim.geom.launch) items.push('These diagrams show the block on the ramp. Once it flies off the table, the only force on it is its weight W, so it accelerates straight down at g.');
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
    if (isFma()) {
      LAB.drawGraph(ctx, W, H);
      return;
    }
    const i = Math.min(ui.track, analysis.bodies.length - 1);
    const tEnd = sim.tStop;
    const launching = !!(sim.geom && sim.geom.flight);
    const xmod = xsFor(analysis);
    const fn = (t) => {
      if (xmod) return xmod.graphValue(analysis, ui.graph, t);
      const m = bodyMotion(i, t);
      if (launching) return ui.graph === 'x' ? m.x : ui.graph === 'v' ? m.vx : m.vy;
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
    const L = 72;
    const R = 16;
    const T = 16;
    const B = 46;
    const X = (t) => L + ((W - L - R) * t) / tEnd;
    const Y = (y) => T + ((H - T - B) * (ymax - y)) / (ymax - ymin);

    ctx.font = '13px ' + C.fontMono;
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
    ctx.font = '600 14px ' + C.fontBody;
    ctx.fillText('time t (s)', L + (W - L - R) / 2, H - 4);
    ctx.save();
    ctx.translate(14, T + (H - T - B) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textBaseline = 'middle';
    const xsLabels = xmod ? (typeof xmod.graphLabel === 'function' ? xmod.graphLabel() : xmod.graphLabel) : null;
    ctx.fillText(
      xsLabels
        ? xsLabels[ui.graph]
        : launching
        ? ui.graph === 'x' ? 'x from table edge (m)' : ui.graph === 'v' ? 'vₓ (m/s)' : 'v_y (m/s)'
        : ui.graph === 'x' ? 'displacement (m)' : ui.graph === 'v' ? 'velocity (m/s)' : 'acceleration (m/s²)',
      0,
      0
    );
    ctx.restore();

    if (anim.t <= 0) {
      ctx.fillStyle = C.muted;
      ctx.font = '15px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Press Play to draw the graph', L + (W - L - R) / 2, T + (H - T - B) / 2);
      return;
    }
    const tNow = Math.min(anim.t, tEnd);
    if (launching) {
      const tl = sim.geom.flight.tRamp;
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(Math.round(X(tl)) + 0.5, T);
      ctx.lineTo(Math.round(X(tl)) + 0.5, H - B);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = C.muted;
      ctx.font = '13px ' + C.fontBody;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText('leaves the table', X(tl) + 4, T + 2);
      ctx.restore();
    }
    ctx.strokeStyle = C.accent;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    const steps = 160;
    const tl = launching ? sim.geom.flight.tRamp : Infinity;
    for (let k = 0; k <= steps; k++) {
      const t = (tNow * k) / steps;
      const tPrev = (tNow * (k - 1)) / steps;
      const x = X(t);
      const y = Y(fn(t));
      if (k && tPrev <= tl && t > tl) {
        ctx.lineTo(X(tl), Y(fn(tl)));
        ctx.moveTo(X(tl), Y(fn(tl + 1e-9)));
      }
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
    const mi = mysteryBodyIndex();
    if (mi >= 0) {
      q.push({ key: 'mystery', group: 'main', label: 'Mystery mass: ' + an.bodies[mi].name, sub: 'Measure the motion, then work back to the mass (within 5% counts as correct)', unit: 'kg', value: an.bodies[mi].m, tol: 0.05 });
    }
    if (xsFor(an)) {
      const extra = xsFor(an).quantities(an);
      // the mystery row joins the first group shown
      if (q.length) {
        q[0].group = 'mys';
        q[0].groupTitle = 'The mystery mass';
      }
      return q.concat(extra);
    }
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
        q.push({ key: 'a' + g.members.join('-'), group: 'main', label: 'Acceleration of ' + nameOf(g.members, an.bodies), unit: 'm/s²', value: Math.abs(g.a) })
      );
    an.links.forEach((l, j) => {
      const nm = labels.links[j];
      const A = l.left + 1;
      const B = l.right + 1;
      const row = { key: 'link' + j + l.type, group: 'main', unit: 'N', value: l.F };
      if (l.type === 'stack') {
        const top = an.bodies[l.right].name.replace(' (on top)', '');
        q.push({ key: 'stackN' + j, group: 'main', unit: 'N', value: l.N, label: 'Normal force between Block ' + A + ' and ' + top, sub: '|' + nm.nOnRight + '| = |' + nm.nOnLeft + '|' });
        row.label = 'Friction between Block ' + A + ' and ' + top;
        row.sub = '|' + nm.onRight + '| = |' + nm.onLeft + '|';
        q.push(row);
        return;
      }
      if (l.type === 'string') {
        row.label = 'Tension ' + nm.name;
        row.sub = l.hanging
          ? 'string from Block ' + A + ' over the pulley to Block ' + B
          : l.long
            ? 'string from Block ' + A + ' to Block ' + B + ', over the blocks between'
            : 'string between Block ' + A + ' and Block ' + B;
      } else {
        row.label = 'Contact force between Block ' + A + ' and Block ' + B;
        row.sub = '|' + nm.onRight + '| = |' + nm.onLeft + '|';
      }
      if (l.indeterminate) row.na = 'Not uniquely determined: static friction can be shared between the blocks in more than one way.';
      q.push(row);
    });
    an.bodies.forEach((b, i) => {
      if (b.kind === 'block') q.push({ key: 'N' + i, group: 'more', label: 'Normal force on ' + b.name, unit: 'N', value: b.N });
      if (an.friction && b.kind === 'block') q.push({ key: 'f' + i, group: 'more', label: 'Friction force on ' + b.name + ' from the surface', unit: 'N', value: Math.abs(b.f) });
      q.push({ key: 'net' + i, group: 'more', label: 'Net force on ' + b.name, unit: 'N', value: Math.abs(b.Fnet) });
    });
    const fl = sim && sim.geom && sim.geom.flight;
    if (fl) {
      q.push({ key: 'tRamp', group: 'launch', label: 'Time to slide down the ramp', unit: 's', value: fl.tRamp });
      q.push({ key: 'v0', group: 'launch', label: 'Speed at the bottom of the ramp', sub: fl.phi > 0 ? 'the speed it leaves the table with, at angle φ' : 'the speed it leaves the table with, horizontally', unit: 'm/s', value: fl.v0 });
      if (fl.phi > 0) {
        q.push({ key: 'vx', group: 'launch', label: 'Horizontal part of the launch velocity, vₓ', unit: 'm/s', value: fl.vx });
        q.push({ key: 'vy0', group: 'launch', label: 'Vertical part of the launch velocity, v_y', unit: 'm/s', value: fl.vy0 });
        q.push({ key: 'hMax', group: 'launch', label: 'Highest point above the table', unit: 'm', value: fl.hMax });
      }
      q.push({ key: 'tAir', group: 'launch', label: 'Time in the air', unit: 's', value: fl.tAir });
      q.push({ key: 'R', group: 'launch', label: 'Landing distance from the table edge', sub: 'horizontal distance', unit: 'm', value: fl.R });
      q.push({ key: 'vLand', group: 'launch', label: 'Speed just before it lands', unit: 'm/s', value: fl.vLand });
    }
    return q;
  }

  function mcQuestions() {
    const xs = xsFor(analysis);
    return analysis && analysis.ok && xs && xs.mcq ? xs.mcq(analysis) || [] : [];
  }
  function renderMC() {
    const mc = mcQuestions();
    return mc
      .map((m) =>
        '<fieldset class="mcq" data-mc="' + m.key + '"><legend>' + esc(m.q) + '</legend>' +
        m.options.map((o, i) => {
          const id = 'mc-' + m.key + '-' + i;
          return '<label class="mc-opt" for="' + id + '"><input type="radio" name="mc-' + m.key + '" id="' + id + '" value="' + i + '"' + (ui.pred['mc:' + m.key] === String(i) ? ' checked' : '') + '> <span>' + esc(o) + '</span></label>';
        }).join('') +
        '<p class="mc-out" aria-live="polite"></p></fieldset>')
      .join('');
  }
  function renderCheck() {
    const qs = quantities();
    const mcSig = mcQuestions().map((m) => m.key + ':' + m.correct).join(',');
    const sig = state.scenario + '|' + mcSig + '|' + qs.map((q) => q.key + ':' + q.label + ':' + (q.sub || '') + ':' + (q.na ? 1 : 0)).join('|');
    $('score').textContent = '';
    if (sig !== ui.checkSig) {
      ui.checkSig = sig;
      if (!qs.length && !mcQuestions().length) {
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
      const more = qs.filter((q) => q.group === 'more');
      const lau = qs.filter((q) => q.group === 'launch');
      // other groups in the order they first appear
      const order = [];
      qs.forEach((q) => {
        if (q.group !== 'more' && q.group !== 'launch' && !order.includes(q.group)) order.push(q.group);
      });
      const titleOf = (gname) => {
        const withTitle = qs.find((q) => q.group === gname && q.groupTitle);
        if (withTitle) return withTitle.groupTitle;
        return lau.length ? 'On the ramp' : xsFor(analysis) ? 'Your answers' : 'System';
      };
      const xs = xsFor(analysis);
      const moreTitle = xs
        ? xs.moreTitle
        : analysis.kind === 'atwood'
          ? 'More: net force on each mass'
          : analysis.friction
            ? 'More: normal, friction and net force on each block'
            : 'More: normal and net force on each block';
      const mcHtml = renderMC();
      $('check-rows').innerHTML =
        (mcHtml ? '<div class="check-group"><h3>Questions</h3>' + mcHtml + '</div>' : '') +
        order.map((gname) => '<div class="check-group"><h3>' + esc(titleOf(gname)) + '</h3>' + qs.filter((q) => q.group === gname).map(row).join('') + '</div>').join('') +
        (lau.length ? '<div class="check-group"><h3>Flying off the table</h3>' + lau.map(row).join('') + '</div>' : '') +
        (more.length ? '<details class="check-group"><summary>' + moreTitle + '</summary>' + more.map(row).join('') + '</details>' : '');
    } else {
      document.querySelectorAll('.mcq').forEach((f) => {
        f.classList.remove('good', 'bad');
        f.querySelector('.mc-out').textContent = '';
      });
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
      const ok = isFinite(x) && Math.abs(x - ans) <= Math.max((q.tol || 0.02) * Math.abs(ans), 0.01);
      if (ok) right++;
      r.classList.add(ok ? 'good' : 'bad');
      out.textContent = ok ? '✓ correct' + (reveal ? ' (' + fmt(ans) + ')' : '') : reveal ? '✗ answer: ' + fmt(ans) : x > ans ? '✗ too high' : '✗ too low';
    });
    mcQuestions().forEach((m) => {
      const f = document.querySelector('.mcq[data-mc="' + m.key + '"]');
      if (!f) return;
      const out = f.querySelector('.mc-out');
      f.classList.remove('good', 'bad');
      const sel = f.querySelector('input:checked');
      if (!sel) {
        out.textContent = reveal ? 'Answer: ' + m.options[m.correct] + '. ' + m.why : '';
        return;
      }
      tried++;
      const ok = +sel.value === m.correct;
      if (ok) right++;
      f.classList.add(ok ? 'good' : 'bad');
      out.textContent = ok ? '✓ Correct. ' + m.why : reveal ? '✗ Answer: ' + m.options[m.correct] + '. ' + m.why : '✗ Not quite. ' + m.why;
    });
    $('score').textContent = tried ? right + ' of ' + tried + ' correct' : reveal ? '' : 'Enter at least one answer.';
    if (!reveal && mysteryOn()) {
      const m = document.querySelector('.qrow[data-key="mystery"] .qr');
      if (m && m.textContent.startsWith('✗')) m.textContent += '. Check your measured acceleration first.';
    }
  }

  /* ======================================================================
   * Worked solution
   * ==================================================================== */

  // A force vector: bold F with an arrow over it, plus an optional subscript.
  const vecF = (subText) => '<span class="vec">F</span>' + (subText ? '<sub>' + subText + '</sub>' : '');
  const SUMF = 'Σ' + vecF();
  const FNET = vecF('net');

  function alongTerms(b, withSlip) {
    const t = [];
    if (b.kind === 'hanging') t.push({ v: b.W, d: 'weight m' + sub(b.index + 1) + 'g' });
    else {
      if (Math.abs(b.Wpar) > 1e-12) t.push({ v: -b.Wpar, d: 'gravity along the slope, m' + sub(b.index + 1) + 'g sin θ' });
      if (Math.abs(b.Fpar) > 1e-12) t.push({ v: b.Fpar, d: 'applied force along the surface' });
    }
    if (withSlip && Math.abs(b.slip || 0) > 1e-12) t.push({ v: b.slip, d: 'kinetic friction between the stacked blocks' });
    return t;
  }

  function solutionHTML() {
    const an = analysis;
    if (!an.ok) return '<p>' + esc(an.error) + '</p>';
    if (xsFor(an)) return xsFor(an).solution(an);
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
      h += '<section><h3>5. Net force on each mass</h3><div class="eq">' + vecF('net,1') + ' = m₁a = ' + fmt(b1.m) + ' × ' + fmt(an.a) + ' = ' + fmt(b1.Fnet) + ' N\n' + vecF('net,2') + ' = m₂a = ' + fmt(b2.m) + ' × ' + fmt(an.a) + ' = ' + fmt(b2.Fnet) + ' N</div></section>';
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
        (b.kind !== 'hanging' ? fmt(b.N) : '—') + '</td>' + (fr ? '<td>' + (b.kind === 'block' ? fmt(b.mus * b.N) : '—') + '</td><td>' + (b.kind === 'block' ? fmt(b.muk * b.N) : '—') + '</td>' : '') + '</tr>';
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
    const stackLink = an.links.find((l) => l.type === 'stack');
    if (stackLink) {
      const base = an.bodies[stackLink.left];
      const top = an.bodies[stackLink.right];
      const tn = top.name.replace(' (on top)', '');
      notes += '<p>' + tn + ' rests on ' + base.name + ', so it gets its normal force N = ' + fmt(top.N) + ' N from ' + base.name + '. By Newton’s third law ' + tn + ' presses down on ' + base.name +
        ' just as hard, so the surface must hold up both: N on ' + base.name + ' = ' + fmt(base.Wperp - base.Fperp) + ' + ' + fmt(top.N) + ' = ' + fmt(base.N) + ' N.</p>';
    }
    h += '<section><h3>2. Forces on each object</h3>' + t + notes + '</section>';

    let step = 3;
    if (stackLink) {
      const l = stackLink;
      const tn = an.bodies[l.right].name.replace(' (on top)', '');
      const bn = an.bodies[l.left].name;
      let st = '<section><h3>' + step++ + '. Does ' + tn + ' slip on ' + bn + '?</h3>' +
        '<p>Suppose the two blocks move together. Then ' + tn + ' needs friction from ' + bn + ' to give it the same acceleration (a = ' + fmt(l.test.a) + ' m/s²). ' +
        'From ΣF = ma on ' + tn + ' alone, it needs |f| = ' + fmt(l.test.need) + ' N. The most static friction can give is μs·N = ' + trimNum(l.mus) + ' × ' + fmt(l.N) + ' = ' + fmt(l.test.max) + ' N.</p>';
      if (l.engaged) st += '<p>' + fmt(l.test.need) + ' N ≤ ' + fmt(l.test.max) + ' N, so static friction is enough: <b>the blocks move together</b>.</p>';
      else
        st += '<p>' + fmt(l.test.need) + ' N > ' + fmt(l.test.max) + ' N, so static friction is not enough: <b>' + tn + ' slips</b>. Kinetic friction acts instead, μk·N = ' + trimNum(l.muk) + ' × ' + fmt(l.N) + ' = ' + fmt(l.F) + ' N, pushing ' + tn + ' ' +
          (l.Fs > 0 ? dw.pos : dw.neg) + ' and ' + bn + ' ' + (l.Fs > 0 ? dw.neg : dw.pos) + '. Now each block has its own acceleration.</p>';
      h += st + '</section>';
    }
    const broken = an.links.map((l, j) => ({ l, j })).filter((x) => !x.l.engaged);
    if (an.groups.length > 1 && !broken.every(({ l }) => l.type === 'stack')) {
      h += '<section><h3>' + step++ + '. Which strings and contacts hold?</h3><p>' +
        (broken.length
          ? broken.map(({ l, j }) => (l.type === 'string' ? 'String ' + labels.links[j].name + ' goes slack (the objects it joins move toward each other).' : l.type === 'stack' ? an.bodies[l.right].name.replace(' (on top)', '') + ' slips on ' + an.bodies[l.left].name + '.' : an.bodies[l.left].name + ' and ' + an.bodies[l.right].name + ' move apart, so they stop touching.')).join(' ')
          : 'Some blocks are not connected to each other.') +
        ' Each connected part moves on its own, so solve each part separately.</p></section>';
    }

    an.groups.forEach((gr) => {
      const name = nameOf(gr.members, an.bodies);
      const terms = [];
      gr.members.forEach((i) => alongTerms(an.bodies[i], true).forEach((x) => terms.push(x)));
      let fTot = 0;
      gr.members.forEach((i) => (fTot += an.bodies[i].f));
      const many = gr.members.length > 1;
      const termStr = terms.length ? terms.map((x, k) => (k ? signed(x.v) : fmt(x.v))).join('') : '0';
      let s = '<section><h3>' + step++ + '. Acceleration' + (an.groups.length > 1 || many ? ' of ' + name : '') + '</h3>';
      if (many) s += '<p>Treat ' + name + ' as one system. Forces between them are internal and cancel in pairs (Newton’s third law), so only outside forces matter. Total mass M = ' + fmt(gr.M) + ' kg.</p>';
      const slipNote = gr.members.some((i) => Math.abs(an.bodies[i].slip || 0) > 1e-12) ? ' (includes the kinetic friction from the slipping block)' : '';
      const sumStr = (terms.length > 1 ? termStr + ' = ' : '') + fmt(gr.D) + ' N';
      if (fr && gr.cap + gr.kin > 0) {
        // Friction depends on whether it moves, so first add up everything except friction with the surface.
        s += '<div class="eq">Sum of outside forces, not counting friction with the surface:\n' + SUMF + ' = ' + sumStr + slipNote + '</div>';
        if (gr.state !== 'sliding') {
          s += '<p>Maximum static friction is Σ μs·N = ' + fmt(gr.cap) + ' N. Since |' + SUMF + '| = ' + fmt(Math.abs(gr.D)) + ' N is not larger, static friction holds and the system <b>stays at rest</b>: ' + FNET + ' = 0 and a = 0. Static friction supplies ' + fmt(Math.abs(gr.D)) + ' N' +
            (Math.abs(gr.D) > 1e-9 ? ', pointing ' + (gr.D > 0 ? dw.neg : dw.pos) : '') + '.</p>';
        } else {
          s += '<p>Maximum static friction is Σ μs·N = ' + fmt(gr.cap) + ' N. Since |' + SUMF + '| = ' + fmt(Math.abs(gr.D)) + ' N is larger, the system slides ' + (gr.D > 0 ? dw.pos : dw.neg) +
            '. Kinetic friction opposes the motion with Σ μk·N = ' + fmt(gr.kin) + ' N.</p>';
          s += '<div class="eq">' + FNET + ' = ' + SUMF + (fTot < 0 ? ' − ' : ' + ') + 'fₖ = ' + fmt(gr.D) + signed(fTot) + ' = ' + fmt(gr.D + fTot) + ' N\n' +
            'a = ' + FNET + ' / M = ' + fmt(gr.D + fTot) + ' / ' + fmt(gr.M) + ' = ' + fmt(gr.a) + ' m/s²</div>';
        }
      } else if (Math.abs(gr.D) < 1e-9) {
        s += '<div class="eq">' + FNET + ' = ' + sumStr + slipNote + '</div><p>No net force, so a = 0. Starting from rest, it stays at rest.</p>';
      } else {
        s += '<div class="eq">Net outside force:\n' + FNET + ' = ' + sumStr + slipNote + '\na = ' + FNET + ' / M = ' + fmt(gr.D) + ' / ' + fmt(gr.M) + ' = ' + fmt(gr.a) + ' m/s²</div>';
      }
      if (gr.state === 'sliding') s += '<p>' + (gr.a > 0 ? 'Positive, so the motion is ' + dw.pos + '.' : 'Negative, so the motion is ' + dw.neg + (dw.hang && gr.members.includes(an.bodies.length - 1) ? ' (the hanging block rises)' : '') + '.') + ' Magnitude |a| = ' + fmt(Math.abs(gr.a)) + ' m/s².</p>';
      h += s + '</section>';
    });

    const engagedLinks = an.links.map((l, j) => ({ l, j })).filter((x) => x.l.engaged || x.l.indeterminate);
    if (engagedLinks.length) {
      let s = '<section><h3>' + step++ + '. Tensions and contact forces</h3><p>Isolate the objects on one side of each string or contact and apply ΣF = ma to just that part.</p>';
      engagedLinks.forEach(({ l, j }) => {
        const nm = labels.links[j];
        const gr = an.groups.find((g) => g.members.includes(l.left));
        if (l.indeterminate) {
          s += '<p><b>' + (l.type === 'string' ? 'Tension ' + nm.name : 'Contact ' + nm.name) + ':</b> not uniquely determined. The blocks are held at rest and static friction on each side can take different shares of the load, so Newton’s laws alone do not fix this force.</p>';
          return;
        }
        // Isolate the simpler side: the hanging block if there is one, otherwise
        // the side with fewer forces on it (ties go to the right-hand side).
        const sideParts = (list) => {
          const out = [];
          let m = 0;
          for (const i of list) {
            const bd = an.bodies[i];
            m += bd.m;
            alongTerms(bd, true).forEach((x) => out.push(x.v));
            if (Math.abs(bd.f) > 1e-9) out.push(bd.f);
          }
          return { parts: out, M: m };
        };
        if (l.type === 'stack') {
          // isolate the top block: f + (its other forces) = m a
          const top = an.bodies[l.right];
          const tp = sideParts([l.right]);
          s += '<p><b>Static friction ' + nm.onRight + '</b>: isolate ' + top.name.replace(' (on top)', '') + '.</p>' +
            '<div class="eq">' + nm.onRight + tp.parts.map((v) => signed(v)).join('') + ' = (' + fmt(top.m) + ')(' + fmt(top.a) + ')\n' + nm.onRight + ' = ' + fmt(l.Fs) + ' N' +
            (Math.abs(l.Fs) > 1e-9 ? '  (points ' + (l.Fs > 0 ? dw.pos : dw.neg) + ')' : '') + '</div>';
          return;
        }
        const L = sideParts(l.sides.left);
        const R = sideParts(l.sides.right);
        const nL = l.sides.left.length;
        const nR = l.sides.right.length;
        const right =
          an.bodies[l.right].kind === 'hanging' ||
          R.parts.length < L.parts.length ||
          (R.parts.length === L.parts.length && nR <= nL);
        const sg = P.linkSigns(l.type)[right ? 'right' : 'left'];
        const lab = right ? nm.onRight : nm.onLeft;
        const { parts, M } = right ? R : L;
        const who = nameOf(right ? l.sides.right : l.sides.left, an.bodies).replace(' (hanging)', '');
        const lhs = (sg > 0 ? '' : '−') + lab + parts.map((v) => signed(v)).join('');
        // e.g. "T − 19.60 = (2.00)(3.27)" then "T = 13.07 N"
        const isolated = right ? l.sides.right : l.sides.left;
        const note = isolated.length > 1 ? ' Treat them as one part: any string or contact between them is internal and cancels.' : '';
        s += '<p><b>' + (l.type === 'string' ? 'Tension ' + nm.name : 'Contact ' + nm.name + ' and ' + nm.onLeft) + '</b>: isolate ' + who + '.' + note + '</p>' +
          '<div class="eq">' + lhs + ' = (' + fmt(M) + ')(' + fmt(gr.a) + ')\n' + lab + ' = ' + fmt(l.F) + ' N</div>';
      });
      broken.forEach(({ l, j }) => {
        if (l.type === 'stack') {
          s += '<p><b>Friction ' + labels.links[j].onRight + '</b> = μk·N = ' + fmt(l.F) + ' N (kinetic, because the top block slips).</p>';
          return;
        }
        s += '<p><b>' + (l.type === 'string' ? 'Tension ' + labels.links[j].name : 'Contact ' + labels.links[j].name) + '</b> = 0 N, because ' + (l.type === 'string' ? 'the string is slack.' : 'the blocks are not pressing on each other.') + '</p>';
      });
      h += s + '</section>';
    }

    let net = '<div class="eq">' + an.bodies.map((b) => FNET + ' on ' + b.name + ' = m a = ' + fmt(b.m) + ' × ' + fmt(b.a) + ' = ' + fmt(b.Fnet) + ' N').join('\n') + '</div>';
    if (fr) net += '<div class="eq">' + an.bodies.filter((b) => b.kind === 'block').map((b) => 'Friction on ' + b.name + ': ' + (b.fType === 'static' ? 'static, ' : b.fType === 'kinetic' ? 'kinetic, μk·N = ' : '') + fmt(Math.abs(b.f)) + ' N').join('\n') + '</div>';
    h += '<section><h3>' + step++ + '. Net force on each object</h3>' + net + '</section>';

    const fl = sim && sim.geom && sim.geom.flight;
    if (fl) {
      const L = cur().launch;
      const a = Math.abs(an.bodies[0].a);
      h += '<section><h3>' + step++ + '. Down the ramp</h3><p>Starting from rest and sliding d = ' + fmt(L.d) + ' m with |a| = ' + fmt(a) + ' m/s²:</p>' +
        '<div class="eq">v² = 2·a·d  →  v = √(2 × ' + fmt(a) + ' × ' + fmt(L.d) + ') = ' + fmt(fl.v0) + ' m/s\nt = v / a = ' + fmt(fl.v0) + ' / ' + fmt(a) + ' = ' + fmt(fl.tRamp) + ' s</div>' +
        (fl.phi > 0
          ? '<p>The small kicker ramp turns this velocity to φ = ' + trimNum(fl.phi) + '° above the horizontal without changing the speed (its size is ignored).</p></section>'
          : '<p>The smooth curve at the bottom turns this velocity horizontal without changing the speed.</p></section>');
      if (fl.phi > 0) {
        const vy0 = fl.vy0;
        h += '<section><h3>' + step++ + '. Split the launch velocity into parts</h3>' +
          '<div class="eq">vₓ = v cos φ = ' + fmt(fl.v0) + ' × cos ' + trimNum(fl.phi) + '° = ' + fmt(fl.vx) + ' m/s\nv_y = v sin φ = ' + fmt(fl.v0) + ' × sin ' + trimNum(fl.phi) + '° = ' + fmt(vy0) + ' m/s</div></section>';
        h += '<section><h3>' + step++ + '. Off the table as a projectile</h3><p>Once it leaves the table, the only force is its weight. Horizontally there is no force, so vₓ stays ' + fmt(fl.vx) +
          ' m/s. Vertically it starts upward at v_y = ' + fmt(vy0) + ' m/s and accelerates down at g. Take up as + and the table top as y = 0, so the floor is at y = −H.</p>' +
          '<div class="eq">Highest point:  v_y² = 2·g·Δy  →  Δy = ' + fmt(vy0) + '² / (2 × ' + fmt(an.g) + ') = ' + fmt(fl.hMax) + ' m above the table\n' +
          'Time in the air:  −H = v_y·t − ½·g·t²\n' +
          '   →  ½·g·t² − v_y·t − H = 0\n' +
          '   →  t = (v_y + √(v_y² + 2·g·H)) / g = (' + fmt(vy0) + ' + √(' + fmt(vy0) + '² + 2 × ' + fmt(an.g) + ' × ' + fmt(L.H) + ')) / ' + fmt(an.g) + ' = ' + fmt(fl.tAir) + ' s\n' +
          'Landing distance:  x = vₓ·t = ' + fmt(fl.vx) + ' × ' + fmt(fl.tAir) + ' = ' + fmt(fl.R) + ' m\n' +
          'Landing speed:  v = √(v² + 2·g·H) = √(' + fmt(fl.v0) + '² + 2 × ' + fmt(an.g) + ' × ' + fmt(L.H) + ') = ' + fmt(fl.vLand) + ' m/s</div>' +
          '<p>The landing speed only depends on the launch speed and the drop height, not on φ (energy is conserved in flight).</p></section>';
      } else {
        h += '<section><h3>' + step++ + '. Off the table as a projectile</h3><p>Once it leaves the table, the only force is its weight. Horizontally there is no force, so vₓ stays ' + fmt(fl.v0) +
          ' m/s. Vertically it starts with v_y = 0 and accelerates down at g.</p>' +
          '<div class="eq">Time to fall H = ' + fmt(L.H) + ' m:  H = ½·g·t²  →  t = √(2 × ' + fmt(L.H) + ' / ' + fmt(an.g) + ') = ' + fmt(fl.tAir) + ' s\n' +
          'Landing distance:  x = vₓ·t = ' + fmt(fl.v0) + ' × ' + fmt(fl.tAir) + ' = ' + fmt(fl.R) + ' m\n' +
          'Landing speed:  v = √(vₓ² + (g·t)²) = √(' + fmt(fl.v0) + '² + ' + fmt(an.g * fl.tAir) + '²) = ' + fmt(fl.vLand) + ' m/s</div></section>';
      }
    }
    return h;
  }

  function renderSolution() {
    const xsl = xsFor(analysis);
    const hidden = !!(xsl && xsl.hidesAnswers && analysis.ok && xsl.hidesAnswers(analysis));
    const locked = (mysteryOn() || hidden) && !ui.revealed;
    const open = ui.solutionOpen && !locked;
    $('btn-unlock').hidden = locked;
    $('solution-lock').querySelector('p').innerHTML = locked
      ? (hidden ? 'Some values are hidden in this problem.' : 'This is a mystery-mass problem.') + ' The worked solution opens when you choose <b>Show answers</b>.'
      : 'Make your predictions first. The worked solution opens after you check your answers or choose <b>Show answers</b>.';
    $('solution-lock').hidden = open;
    $('solution').hidden = !open;
    if (open) $('solution').innerHTML = solutionHTML();
  }

  /* ======================================================================
   * Extra scenarios: elevator and human body
   * Each module supplies its own controls, solving, scene, diagrams,
   * check rows and worked solution. The shared code calls into XS[kind].
   * ==================================================================== */

  const ELEVATOR_MOTIONS = [
    ['up-speed', 'Starting up: speeding up going up'],
    ['up-const', 'Going up at constant speed'],
    ['up-slow', 'Arriving at the top: slowing down going up'],
    ['down-speed', 'Starting down: speeding up going down'],
    ['down-const', 'Going down at constant speed'],
    ['down-slow', 'Arriving at the bottom: slowing down going down'],
    ['ride-up', 'A full ride up (all three stages)'],
    ['ride-down', 'A full ride down (all three stages)'],
    ['freefall', 'The cable breaks: free fall'],
  ];
  const BODY_MODES = [
    ['traction', 'Leg in traction (pulleys)'],
    ['jump', 'Jumping straight up'],
    ['land', 'Landing from a height'],
    ['sprint', 'Sprint start'],
  ];

  /* ---------- Human figures ----------
   * Side view, facing right. A pose is a set of joint positions in metres.
   * Angles for limbs are measured from straight down, positive = forward (+x). */
  const FIG = { shin: 0.42, thigh: 0.44, ankle: 0.06, torso: 0.55, neck: 0.07, head: 0.115, upper: 0.29, fore: 0.27, foot: 0.17 };
  const STAND_HIP = FIG.ankle + FIG.shin + FIG.thigh; // 0.92 m
  const vadd = (p, q) => ({ x: p.x + q.x, y: p.y + q.y });
  const polar = (len, ang) => ({ x: len * Math.sin(ang), y: -len * Math.cos(ang) });
  const clamp01 = (x) => Math.max(0, Math.min(1, x));
  const lerp = (a, b, f) => a + (b - a) * f;

  // Two-segment limb from joint H to end A: where does the middle joint go?
  // sign +1 bends forward (knees), -1 bends backward.
  function ik(H, A, L1, L2, sign) {
    const dx = A.x - H.x;
    const dy = A.y - H.y;
    const D = Math.max(1e-6, Math.hypot(dx, dy));
    const Dc = Math.min(D, L1 + L2 - 1e-6);
    const ux = dx / D;
    const uy = dy / D;
    const a = (L1 * L1 - L2 * L2 + Dc * Dc) / (2 * Dc);
    const h = Math.sqrt(Math.max(0, L1 * L1 - a * a));
    return { x: H.x + ux * a + sign * -uy * h, y: H.y + uy * a + sign * ux * h };
  }

  function armsFrom(shoulder, armA, bend = 0.45) {
    return [armA - 0.14, armA + 0.14].map((a) => {
      const elbow = vadd(shoulder, polar(FIG.upper, a));
      return { elbow, hand: vadd(elbow, polar(FIG.fore, a + bend)) };
    });
  }

  /* Standing, crouching or in the air.
   * o = { x: feet x, floor: floor height, yHip, contact, armA, lean, tuck } */
  function poseUpright(o) {
    const floor = o.floor || 0;
    const crouch = o.contact ? clamp01((floor + STAND_HIP - o.yHip) / 0.55) : 0;
    const hip = { x: o.x - 0.24 * crouch, y: o.yHip };
    const lean = (o.lean || 0) + 0.75 * crouch;
    const legs = [-0.04, 0.04].map((off) => {
      let ankle;
      let toe;
      if (o.contact) {
        ankle = { x: o.x + off, y: floor + FIG.ankle };
        toe = { x: ankle.x + FIG.foot, y: floor + 0.025 };
      } else {
        const tuck = o.tuck ?? 0.12;
        ankle = vadd(hip, { x: 0.06 + off, y: -(FIG.thigh + FIG.shin) * (1 - tuck) });
        toe = vadd(ankle, { x: 0.12, y: -0.11 });
      }
      return { knee: ik(hip, ankle, FIG.thigh, FIG.shin, 1), ankle, toe };
    });
    const shoulder = vadd(hip, { x: FIG.torso * Math.sin(lean), y: FIG.torso * Math.cos(lean) });
    const head = vadd(shoulder, { x: (FIG.neck + FIG.head) * Math.sin(lean * 0.55), y: (FIG.neck + FIG.head) * Math.cos(lean * 0.55) });
    return { hip, legs, shoulder, head, arms: armsFrom(shoulder, o.armA ?? 0.15), eye: { x: 0.055, y: 0.02 } };
  }

  /* Running stride. phase advances with distance; lean in radians. */
  function poseRun(xHip, phase, lean, floor = 0) {
    const rel = { x: 0, y: 0 };
    const legs = [0, Math.PI].map((off) => {
      const ph = phase + off;
      const th = 0.62 * Math.sin(ph); // thigh swing
      const kn = 0.3 + 0.95 * (0.5 - 0.5 * Math.sin(ph)); // knee bends most when the leg trails
      const knee = vadd(rel, polar(FIG.thigh, th));
      const ankle = vadd(knee, polar(FIG.shin, th - kn));
      const toe = vadd(ankle, polar(FIG.foot, th - kn + Math.PI / 2));
      return { knee, ankle, toe };
    });
    let minY = Infinity;
    legs.forEach((l) => (minY = Math.min(minY, l.ankle.y - FIG.ankle * 0.5, l.toe.y)));
    const dy = floor + 0.02 - minY;
    const shift = (p) => ({ x: p.x + xHip, y: p.y + dy });
    const hip = shift(rel);
    const L = legs.map((l) => ({ knee: shift(l.knee), ankle: shift(l.ankle), toe: shift(l.toe) }));
    const shoulder = vadd(hip, { x: FIG.torso * Math.sin(lean), y: FIG.torso * Math.cos(lean) });
    const head = vadd(shoulder, { x: (FIG.neck + FIG.head) * Math.sin(lean * 0.7), y: (FIG.neck + FIG.head) * Math.cos(lean * 0.7) });
    const arms = [0, Math.PI].map((off) => {
      const a = -0.85 * Math.sin(phase + off);
      const elbow = vadd(shoulder, polar(FIG.upper, a));
      return { elbow, hand: vadd(elbow, polar(FIG.fore, a + 1.5)) };
    });
    // the arm on the far side swings with the near leg
    return { hip, legs: L, shoulder, head, arms: [arms[1], arms[0]], eye: { x: 0.055, y: 0.02 } };
  }

  /* Lying on the back, head to the left, the near leg raised to the point `foot`. */
  function poseLying(y, foot) {
    const hip = { x: -1.3, y: y + 0.08 };
    const shoulder = { x: -2.05, y: y + 0.1 };
    const head = { x: -2.27, y: y + 0.13 };
    const ankleNear = { x: foot.x - 0.04, y: foot.y };
    const kneeNear = ik(hip, ankleNear, FIG.thigh, FIG.shin, 1);
    const ankleFar = { x: -0.42, y: y + 0.06 };
    const kneeFar = ik(hip, ankleFar, FIG.thigh, FIG.shin, 1);
    return {
      hip,
      shoulder,
      head,
      legs: [
        { knee: kneeFar, ankle: ankleFar, toe: { x: ankleFar.x + 0.03, y: ankleFar.y + FIG.foot } },
        { knee: kneeNear, ankle: ankleNear, toe: { x: ankleNear.x + 0.04, y: ankleNear.y + FIG.foot } },
      ],
      arms: [
        { elbow: { x: -1.8, y: y + 0.05 }, hand: { x: -1.55, y: y + 0.06 } },
        { elbow: { x: -1.8, y: y + 0.07 }, hand: { x: -1.56, y: y + 0.09 } },
      ],
      eye: { x: 0.01, y: 0.06 },
      lying: true,
    };
  }

  /* Draw a pose. o = { alpha, label, labelLeft } */
  function drawFigure(ctx, view, pose, o = {}) {
    const P = view.P;
    const k = view.k;
    ctx.save();
    if (o.alpha != null) ctx.globalAlpha = o.alpha;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    const edge = Math.max(1.5, k * 0.018);
    const seg = (a, b, w, color, dim) => {
      const [x1, y1] = P(a.x, a.y);
      const [x2, y2] = P(b.x, b.y);
      ctx.strokeStyle = C.scene;
      ctx.lineWidth = w * k + edge * 2;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      ctx.strokeStyle = color;
      ctx.lineWidth = w * k;
      ctx.stroke();
      if (dim) {
        // the far-side limb is shaded toward the background
        ctx.save();
        ctx.globalAlpha = (o.alpha ?? 1) * 0.38;
        ctx.strokeStyle = C.scene;
        ctx.stroke();
        ctx.restore();
      }
    };
    const leg = (l, dim) => {
      seg(pose.hip, l.knee, 0.14, C['fig-shorts'], dim);
      seg(l.knee, l.ankle, 0.1, C['fig-skin'], dim);
      seg(l.ankle, l.toe, 0.075, C['fig-shoe'], dim);
    };
    const arm = (a, dim) => {
      seg(pose.shoulder, a.elbow, 0.09, C['fig-shirt'], dim);
      seg(a.elbow, a.hand, 0.072, C['fig-skin'], dim);
    };
    arm(pose.arms[0], true);
    leg(pose.legs[0], true);
    // torso and neck
    const neckBase = vadd(pose.shoulder, { x: (pose.head.x - pose.shoulder.x) * 0.25, y: (pose.head.y - pose.shoulder.y) * 0.25 });
    seg(pose.shoulder, pose.head, 0.065, C['fig-skin']);
    seg(pose.hip, neckBase, 0.27, C['fig-shirt']);
    seg(pose.hip, pose.hip, 0.2, C['fig-shorts']);
    leg(pose.legs[1], false);
    // head with hair and an eye
    const [hx, hy] = P(pose.head.x, pose.head.y);
    const r = FIG.head * k;
    ctx.fillStyle = C['fig-skin'];
    ctx.strokeStyle = C.scene;
    ctx.lineWidth = edge;
    ctx.beginPath();
    ctx.arc(hx, hy, r, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = C['fig-hair'];
    ctx.beginPath();
    if (pose.lying) ctx.arc(hx, hy, r, Math.PI * 0.5, Math.PI * 1.25);
    else ctx.arc(hx, hy, r, Math.PI * 1.02, Math.PI * 1.85);
    ctx.closePath();
    ctx.fill();
    const [ex, ey] = P(pose.head.x + pose.eye.x, pose.head.y + pose.eye.y);
    ctx.fillStyle = C.scene;
    ctx.beginPath();
    ctx.arc(ex, ey, Math.max(1.2, k * 0.012), 0, 2 * Math.PI);
    ctx.fill();
    arm(pose.arms[1], false);
    if (o.label) {
      const size = Math.max(12, Math.min(16, k * 0.11));
      const at = vadd(pose.hip, { x: (pose.shoulder.x - pose.hip.x) * 0.5, y: (pose.shoulder.y - pose.hip.y) * 0.5 });
      const [lx, ly] = P(at.x + (o.labelLeft ? -0.3 : 0.3), at.y);
      ctx.globalAlpha = 1;
      ctx.fillStyle = C.ink;
      ctx.font = '700 ' + size + 'px ' + C.fontMono;
      ctx.textAlign = o.labelLeft ? 'right' : 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(o.label, lx, ly);
    }
    ctx.restore();
  }

  // A vertical dimension marker: a double-headed line from y0 to y1 at x, with a label.
  function dimV(ctx, view, x, y0, y1, text, color, side = 'left') {
    if (Math.abs(y1 - y0) < 1e-3) return;
    const [px, p0] = view.P(x, y0);
    const [, p1] = view.P(x, y1);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(px - 6, p0);
    ctx.lineTo(px + 6, p0);
    ctx.moveTo(px - 6, p1);
    ctx.lineTo(px + 6, p1);
    ctx.stroke();
    const head = Math.min(8, Math.abs(p1 - p0) / 3);
    const dir = Math.sign(p1 - p0);
    ctx.beginPath();
    ctx.moveTo(px, p0 + dir * head);
    ctx.lineTo(px, p1 - dir * head);
    ctx.stroke();
    for (const [py, d] of [[p0, dir], [p1, -dir]]) {
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px - 4, py + d * head);
      ctx.lineTo(px + 4, py + d * head);
      ctx.closePath();
      ctx.fill();
    }
    ctx.font = '700 15px ' + C.fontMono;
    ctx.textBaseline = 'middle';
    ctx.textAlign = side === 'left' ? 'right' : 'left';
    ctx.fillText(text, px + (side === 'left' ? -10 : 10), (p0 + p1) / 2);
    ctx.restore();
  }

  // Draw a pose inside a free-body diagram cell, centred on the hips.
  function drawFigureInCell(ctx, pose, cx, cy, scale = 78) {
    const view = { k: scale, P: (x, y) => [cx + (x - pose.hip.x) * scale, cy - (y - pose.hip.y) * scale] };
    drawFigure(ctx, view, pose, { alpha: 0.55 });
  }

  function labelText(ctx, text, x, y, color, align = 'center', size = 14, weight = 700, font) {
    ctx.save();
    ctx.fillStyle = color;
    ctx.font = weight + ' ' + size + 'px ' + (font || C.fontMono);
    ctx.textAlign = align;
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  /* ---------------- Elevator ---------------- */
  const ELEV = {
    defaults: () => ({ m: 60, motion: 'ride-up', A: 1.5, cable: false, M: 800, mystery: { on: false, target: 'person' } }),
    solve(c) {
      const an = P.solveElevator({ m: massOf('person', c.m), M: c.cable ? c.M : 0, g: state.g, motion: c.motion, A: c.A });
      an.bodies = [{ name: 'Person', m: an.m, kind: 'person', a: 0 }];
      if (c.cable) an.bodies.push({ name: 'Elevator car', m: c.M, kind: 'car', a: 0 });
      an.atRest = false;
      return an;
    },
    controls(c) {
      let h = '<div class="group"><h2>Person on a scale</h2>' +
        '<div class="blk"><span class="tag" style="background:var(--b1)">P</span>' +
        (isMystery('person') ? hiddenMass('Person’s mass') : numField('Person’s mass', 'm', c.m, 'kg', { min: 1, max: 300, step: 1 })) + '<span></span></div>' +
        '<p class="note">The scale reads the normal force it pushes up with.</p></div>';
      h += '<div class="group"><h2>Motion</h2>' + selectField('What the elevator does', 'sel-motion', 'motion', ELEVATOR_MOTIONS, c.motion, true);
      if (!/const|freefall/.test(c.motion)) h += numField('Size of the acceleration', 'A', c.A, 'm/s²', { min: 0.1, max: 6, step: 0.1 });
      if (/ride/.test(c.motion)) h += '<p class="note">The ride speeds up for 2.5 s, moves at constant speed for 3 s, then slows to a stop over 2.5 s.</p>';
      if (c.motion === 'freefall') h += '<p class="note">Starting at rest, the car and person both fall with acceleration g.</p>';
      h += '</div>';
      h += '<div class="group"><h2>Cable</h2>' + checkField('Also find the tension in the cable', 'cable', c.cable);
      if (c.cable) h += numField('Mass of the elevator car', 'M', c.M, 'kg', { min: 1, max: 10000, step: 10 });
      h += '</div>';
      return h;
    },
    randomize(c, r) {
      c.m = r(40, 100, 1);
      c.A = r(0.5, 3, 0.1);
      c.M = r(400, 1500, 50);
    },
    load(c, num) {
      const d = ELEV.defaults();
      return {
        m: num(c.m, d.m, 1, 300),
        motion: ELEVATOR_MOTIONS.some((x) => x[0] === c.motion) ? c.motion : d.motion,
        A: num(c.A, d.A, 0.1, 6),
        cable: !!c.cable,
        M: num(c.M, d.M, 1, 10000),
      };
    },
    // position, velocity, acceleration and scale reading at time t
    state(an, t) {
      let y = 0;
      for (const ph of an.phases) {
        if (t <= ph.t0 + ph.dur + 1e-12 || ph === an.phases[an.phases.length - 1]) {
          const tau = Math.min(Math.max(0, t - ph.t0), ph.dur);
          return { y: y + ph.v0 * tau + 0.5 * ph.a * tau * tau, v: ph.v0 + ph.a * tau, a: ph.a, N: ph.N, T: ph.T, ph };
        }
        y += ph.v0 * ph.dur + 0.5 * ph.a * ph.dur * ph.dur;
      }
      return null;
    },
    buildSim(an) {
      let lo = 0;
      let hi = 0;
      for (let k = 0; k <= 200; k++) {
        const y = ELEV.state(an, (an.tTotal * k) / 200).y;
        lo = Math.min(lo, y);
        hi = Math.max(hi, y);
      }
      const y0 = -lo + 3; // start a floor above the bottom so there is room below
      const top = hi - lo + 6;
      const last = an.phases[an.phases.length - 1];
      const reason = an.motion === 'freefall' ? 'The emergency brakes would stop the car here' : /slow|ride/.test(last.key) ? 'The elevator has stopped' : 'Time is up for this stage';
      return { kind: 'elevator', y0, top, tStop: an.tTotal, reason };
    },
    motion(an, t) {
      const st = ELEV.state(an, t);
      return { s: st.y, v: st.v, a: st.a };
    },
    graphNames: { x: 'Height–time', v: 'Velocity–time', a: 'Scale reading–time' },
    graphValue(an, which, t) {
      const st = ELEV.state(an, t);
      return which === 'x' ? st.y : which === 'v' ? st.v : st.N;
    },
    graphLabel: { x: 'height (m)', v: 'velocity (m/s)', a: 'scale reading (N)' },
    draw(ctx, W, H, t) {
      const an = analysis;
      const st = ELEV.state(an, t);
      const yc = sim.y0 + st.y + 1.3;
      const view = fitView({ x0: -2.8, x1: 3.2, y0: yc - 2.6, y1: yc + 2.6 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      // building: walls and floors
      ctx.save();
      const [bx0] = P(-2.7, 0);
      const [bx1] = P(2.7, 0);
      const [, byTop] = P(0, sim.top);
      const [, byBot] = P(0, 0);
      ctx.fillStyle = C.bench;
      ctx.globalAlpha = 0.35;
      ctx.fillRect(bx0, byTop, bx1 - bx0, byBot - byTop);
      ctx.globalAlpha = 1;
      const [sx0] = P(-1.35, 0);
      const [sx1] = P(1.35, 0);
      ctx.fillStyle = C.scene;
      ctx.fillRect(sx0, byTop, sx1 - sx0, byBot - byTop);
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx0, byTop);
      ctx.lineTo(sx0, byBot);
      ctx.moveTo(sx1, byTop);
      ctx.lineTo(sx1, byBot);
      ctx.stroke();
      for (let f = 0; f * 3 <= sim.top + 0.01; f++) {
        const [, fy] = P(0, f * 3);
        ctx.strokeStyle = C['bench-edge'];
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(bx0, fy);
        ctx.lineTo(sx0, fy);
        ctx.moveTo(sx1, fy);
        ctx.lineTo(bx1, fy);
        ctx.stroke();
        labelText(ctx, W < 520 ? f * 3 + ' m' : 'Floor ' + (f + 1) + ' · ' + f * 3 + ' m', bx0 + 6, fy - 12, C.muted, 'left', 13, 600, C.fontBody);
      }
      ctx.restore();

      // ghosts of the car
      for (const ts of strobeTimes(t)) {
        const g0 = sim.y0 + ELEV.state(an, ts).y;
        const [gx0, gy0] = P(-1.15, g0 + 2.6);
        const [gx1, gy1] = P(1.15, g0);
        ctx.save();
        ctx.globalAlpha = 0.25;
        ctx.strokeStyle = C.ink;
        ctx.strokeRect(gx0, gy0, gx1 - gx0, gy1 - gy0);
        ctx.restore();
      }

      // cable
      const yb = sim.y0 + st.y; // car floor
      const [cx, cyTop] = P(0, yb + 2.6);
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      if (an.motion === 'freefall') {
        ctx.moveTo(cx, cyTop);
        ctx.lineTo(cx + 6, cyTop - 30);
        ctx.lineTo(cx - 4, cyTop - 50);
      } else {
        ctx.moveTo(cx, cyTop);
        ctx.lineTo(cx, 0);
      }
      ctx.stroke();
      ctx.restore();

      // car
      const [x0, y0] = P(-1.15, yb + 2.6);
      const [x1, y1] = P(1.15, yb);
      ctx.save();
      ctx.fillStyle = C.panel;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
      ctx.restore();
      // scale on the floor
      const [qx0, qy0] = P(-0.75, yb + 0.12);
      const [qx1, qy1] = P(-0.05, yb);
      ctx.save();
      ctx.fillStyle = bodyColor(1);
      ctx.strokeStyle = C.ink;
      ctx.fillRect(qx0, qy0, qx1 - qx0, qy1 - qy0);
      ctx.strokeRect(qx0, qy0, qx1 - qx0, qy1 - qy0);
      ctx.restore();
      const weightless = st.a < -an.g + 1e-9;
      const rider = poseUpright({ x: -0.48, floor: yb + 0.12, yHip: yb + 0.12 + STAND_HIP, contact: true, armA: weightless ? 2.6 : 0.12 });
      drawFigure(ctx, view, rider, { label: W < 520 ? '' : massText(0, an.m) + ' kg', labelLeft: true });
      // scale display on the back wall
      const [dx0, dy0] = P(0.15, yb + 2.3);
      const [dx1, dy1] = P(1.05, yb + 1.45);
      ctx.save();
      ctx.fillStyle = C.scene;
      ctx.strokeStyle = C.accent;
      ctx.lineWidth = 2;
      ctx.fillRect(dx0, dy0, dx1 - dx0, dy1 - dy0);
      ctx.strokeRect(dx0, dy0, dx1 - dx0, dy1 - dy0);
      ctx.restore();
      const big = Math.max(13, Math.min(22, view.k * 0.2));
      labelText(ctx, 'SCALE', (dx0 + dx1) / 2, dy0 + big * 0.55, C.muted, 'center', Math.round(big * 0.55), 600, C.fontBody);
      labelText(ctx, Math.round(st.N) + ' N', (dx0 + dx1) / 2, (dy0 + dy1) / 2 + 2, C.accent, 'center', big, 700);
      labelText(ctx, fmt(st.N / an.g, 1) + ' kg', (dx0 + dx1) / 2, dy1 - big * 0.55, C.ink, 'center', Math.round(big * 0.7), 600);

      // velocity and acceleration arrows beside the car
      const ax = 1.75;
      const [ax0, ay0] = P(ax, yb + 1.3);
      const scaleA = view.k * 0.28;
      if (Math.abs(st.a) > 1e-9) {
        const len = Math.max(18, Math.min(110, Math.abs(st.a) * scaleA));
        arrow(ctx, ax0, ay0, ax0, ay0 - Math.sign(st.a) * len, C['f-a'], 3, 11);
        labelText(ctx, 'a', ax0 + 12, ay0 - Math.sign(st.a) * len * 0.6, C['f-a'], 'left', 15);
      } else labelText(ctx, 'a = 0', ax0 + 6, ay0, C['f-a'], 'left', 14);
      const [vx0, vy0] = P(ax + 0.6, yb + 1.3);
      if (Math.abs(st.v) > 1e-9) {
        const len = Math.max(18, Math.min(110, Math.abs(st.v) * view.k * 0.18));
        arrow(ctx, vx0, vy0, vx0, vy0 - Math.sign(st.v) * len, C['f-n'], 3, 11);
        labelText(ctx, 'v', vx0 + 12, vy0 - Math.sign(st.v) * len * 0.6, C['f-n'], 'left', 15);
      } else labelText(ctx, 'v = 0', vx0 + 6, vy0 + 18, C['f-n'], 'left', 14);
      // stage name at the top
      labelText(ctx, st.ph.label, 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    forceSets(an, t) {
      const st = ELEV.state(an, t || 0);
      const sets = [{
        body: an.bodies[0],
        idx: 0,
        tilt: 0,
        pose: poseUpright({ x: 0, yHip: STAND_HIP, contact: true, armA: st.a < -an.g + 1e-9 ? 2.6 : 0.12 }),
        arrows: [
          { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
          { dir: { x: 0, y: 1 }, mag: st.N, color: C['f-n'], label: 'N' },
        ],
      }];
      if (an.M > 0) {
        sets.push({
          body: an.bodies[1],
          idx: 1,
          tilt: 0,
          arrows: [
            { dir: { x: 0, y: 1 }, mag: st.T, color: C.l1, label: 'T' },
            { dir: { x: 0, y: -1 }, mag: an.M * an.g, color: C['f-w'], label: 'W_car' },
            { dir: { x: 0, y: -1 }, mag: st.N, color: C['f-n'], label: 'N′' },
          ],
        });
      }
      return sets;
    },
    fbdCaption(an, t) {
      return 'Showing: ' + ELEV.state(an, t || 0).ph.label.toLowerCase() + (an.phases.length > 1 ? '. Press Play to watch the forces change.' : '.');
    },
    legend(an, showV) {
      const chip = (color, text) => '<span><i style="background:' + color + '"></i>' + text + '</span>';
      let legend = chip('var(--f-w)', 'W: weight') + chip('var(--f-n)', 'N: the scale pushes up on the person');
      if (an.M > 0) legend += chip('var(--f-n)', 'N′: the person pushes down on the scale') + chip('var(--l1)', 'T: cable tension');
      const items = [
        '<span class="swatch" style="background:var(--f-n)"></span><b>N</b> (scale pushes up on the person) and the person pushing down on the scale' + (an.M > 0 ? ' (<b>N′</b>)' : '') +
          ' are a <b>third-law pair</b>. The scale shows the size of this force, which is why it can read more or less than your weight.',
        '<b>W and N are not a pair.</b> They both act on the person. They are equal only when the elevator is not accelerating.',
        '<b>Weight:</b> Earth pulls the person down and the person pulls Earth up just as hard.',
      ];
      if (showV) items.unshift('Your true weight is W = mg = ' + fmt(an.W) + ' N. It never changes; only the scale reading does.');
      return { legend, pairs: '<h3>Action–reaction pairs</h3><ul>' + items.map((x) => '<li>' + x + '</li>').join('') + '</ul>' };
    },
    quantities(an) {
      const q = [];
      const multi = an.phases.length > 1;
      an.phases.forEach((ph, k) => {
        const group = multi ? 'ph' + k : 'main';
        const title = multi ? ph.label : 'The person on the scale';
        q.push({ key: 'N' + k, group, groupTitle: title, label: 'Scale reading', sub: 'the normal force on the person', unit: 'N', value: ph.N });
        q.push({ key: 'kg' + k, group, groupTitle: title, label: 'What a scale marked in kg would show', sub: 'reading ÷ g', unit: 'kg', value: ph.kg });
        if (an.M > 0) q.push({ key: 'T' + k, group, groupTitle: title, label: 'Tension in the cable', unit: 'N', value: ph.T });
        q.push({ key: 'net' + k, group: 'more', label: 'Net force on the person' + (multi ? ' (' + ph.label.toLowerCase() + ')' : ''), unit: 'N', value: Math.abs(ph.Fnet) });
      });
      q.push({ key: 'W', group: 'more', label: 'The person’s true weight', unit: 'N', value: an.W });
      return q;
    },
    moreTitle: 'More: net force and true weight',
    solution(an) {
      const g = fmt(an.g);
      let h = '<section><h3>1. Choose a positive direction</h3><p>Take + as <b>up</b>. Accelerating upward (speeding up going up, or slowing down going down) means a is positive. Accelerating downward means a is negative.</p></section>';
      h += '<section><h3>2. Forces on the person</h3><p>Only two forces act on the person: the weight W = mg pulling down and the scale’s normal force N pushing up. The scale reads N.</p>' +
        '<div class="eq">ΣF = ma:   N − mg = ma\n   →   N = m(g + a)</div></section>';
      let step = 3;
      an.phases.forEach((ph) => {
        let s = '<section><h3>' + step++ + '. ' + ph.label + '</h3>';
        s += '<div class="eq">a = ' + fmt(ph.a) + ' m/s²\nN = ' + fmt(an.m) + ' × (' + g + signed(ph.a) + ') = ' + fmt(ph.N) + ' N\nA scale marked in kg shows N ÷ g = ' + fmt(ph.N) + ' / ' + g + ' = ' + fmt(ph.kg) + ' kg</div>';
        if (Math.abs(ph.a) < 1e-9) s += '<p>No acceleration means no net force (Newton’s first law), so the scale reads the true weight even though the elevator is moving.</p>';
        else if (ph.N < 1e-9) s += '<p>The person and the scale fall together, so the scale does not push at all. The person feels <b>weightless</b>, but their true weight is still ' + fmt(an.W) + ' N.</p>';
        else if (ph.a > 0) s += '<p>The scale reads <b>more</b> than the true weight (' + fmt(an.W) + ' N): it has to push harder than gravity to accelerate the person upward.</p>';
        else s += '<p>The scale reads <b>less</b> than the true weight (' + fmt(an.W) + ' N): gravity wins a little, so the person accelerates downward.</p>';
        if (an.M > 0) s += '<div class="eq">Cable, treating car + person as one system (mass ' + fmt(an.M + an.m) + ' kg):\nT − (M + m)g = (M + m)a\nT = ' + fmt(an.M + an.m) + ' × (' + g + signed(ph.a) + ') = ' + fmt(ph.T) + ' N</div>';
        h += s + '</section>';
      });
      return h;
    },
  };

  /* ---------------- Human body ---------------- */
  const HIP = 0.92; // hip height when standing (m), also the leg length
  const BODY = {
    defaults: () => ({
      mode: 'jump',
      m: 60,
      jump: { d: 0.4, h: 0.5 },
      land: { h: 1, d: 0.5 },
      sprint: { v: 8, d: 10 },
      traction: { m: 3, th1: 35, th2: 15 },
      mystery: { on: false, target: '' },
    }),
    isStatic: () => cur().mode === 'traction',
    solve(c) {
      if (c.mode === 'traction') {
        const an = P.solveTraction({ m: c.traction.m, th1: c.traction.th1, th2: c.traction.th2, g: state.g });
        an.bodies = [{ name: 'Hanging weight', m: c.traction.m, kind: 'hanging', a: 0 }];
        an.atRest = true;
        return an;
      }
      const p = c[c.mode];
      const an = P.solveBody({ mode: c.mode, m: c.m, g: state.g, d: p.d, h: p.h, v: p.v });
      an.bodies = [{ name: 'Person', m: c.m, kind: 'person', a: 0 }];
      an.atRest = false;
      return an;
    },
    controls(c) {
      let h = '<div class="group"><h2>Problem</h2>' + selectField('Type of problem', 'sel-body-mode', 'mode', BODY_MODES, c.mode, true) + '</div>';
      if (c.mode === 'traction') {
        h += '<div class="group"><h2>Traction setup</h2>' +
          numField('Hanging mass', 'traction.m', c.traction.m, 'kg', { min: 0.1, max: 30, step: 0.1 }) +
          numField('Upper rope angle θ₁ (above horizontal)', 'traction.th1', c.traction.th1, '°', { min: 0, max: 60, step: 1 }) +
          numField('Lower rope angle θ₂ (below horizontal)', 'traction.th2', c.traction.th2, '°', { min: 0, max: 60, step: 1 }) +
          '<p class="note">One rope runs from an anchor on the frame, around the pulley on the foot, over a pulley at the top, and down to the hanging weight. With massless, frictionless pulleys the tension is the same everywhere.</p></div>';
        return h;
      }
      h += '<div class="group"><h2>Person</h2>' + numField('Mass', 'm', c.m, 'kg', { min: 10, max: 200, step: 1 }) + '</div>';
      if (c.mode === 'jump') {
        h += '<div class="group"><h2>The jump</h2>' +
          numField('Push-off distance d (hips rise while feet push)', 'jump.d', c.jump.d, 'm', { min: 0.05, max: 0.7, step: 0.01 }) +
          numField('Jump height h (rise after leaving the floor)', 'jump.h', c.jump.h, 'm', { min: 0.05, max: 2, step: 0.01 }) +
          '<p class="note">The person starts crouched and pushes off with a constant force.</p></div>';
      } else if (c.mode === 'land') {
        h += '<div class="group"><h2>The landing</h2>' +
          numField('Drop height h', 'land.h', c.land.h, 'm', { min: 0.1, max: 5, step: 0.05 }) +
          numField('Stopping distance d (knees bend)', 'land.d', c.land.d, 'm', { min: 0.01, max: 0.7, step: 0.01 }) +
          '<p class="note">Try a stiff-legged landing with d = 0.02 m and compare the force.</p></div>';
      } else {
        h += '<div class="group"><h2>The start</h2>' +
          numField('Top speed v', 'sprint.v', c.sprint.v, 'm/s', { min: 1, max: 12, step: 0.1 }) +
          numField('Distance to reach it, d', 'sprint.d', c.sprint.d, 'm', { min: 1, max: 60, step: 0.5 }) +
          '<p class="note">The runner speeds up at a constant rate. Static friction from the track pushes the runner forward.</p></div>';
      }
      return h;
    },
    randomize(c, r) {
      c.m = r(45, 95, 1);
      c.jump = { d: r(0.25, 0.5, 0.01), h: r(0.2, 0.7, 0.01) };
      c.land = { h: r(0.4, 2, 0.05), d: r(0.1, 0.6, 0.01) };
      c.sprint = { v: r(5, 10, 0.1), d: r(5, 30, 0.5) };
      c.traction = { m: r(1, 6, 0.1), th1: r(10, 50, 1), th2: r(0, 40, 1) };
    },
    load(c, num) {
      const d = BODY.defaults();
      const j = c.jump || {};
      const l = c.land || {};
      const s = c.sprint || {};
      const t = c.traction || {};
      return {
        mode: BODY_MODES.some((x) => x[0] === c.mode) ? c.mode : d.mode,
        m: num(c.m, d.m, 10, 200),
        jump: { d: num(j.d, d.jump.d, 0.05, 0.7), h: num(j.h, d.jump.h, 0.05, 2) },
        land: { h: num(l.h, d.land.h, 0.1, 5), d: num(l.d, d.land.d, 0.01, 0.7) },
        sprint: { v: num(s.v, d.sprint.v, 1, 12), d: num(s.d, d.sprint.d, 1, 60) },
        traction: { m: num(t.m, d.traction.m, 0.1, 30), th1: num(t.th1, d.traction.th1, 0, 60), th2: num(t.th2, d.traction.th2, 0, 60) },
        mystery: d.mystery,
      };
    },
    // hip height (or position along the track), velocity, acceleration and ground force at time t
    state(an, t) {
      if (an.mode === 'jump') {
        if (t <= an.tPush) return { y: HIP - an.d + 0.5 * an.a * t * t, v: an.a * t, a: an.a, F: an.F, contact: true, phase: 'Pushing off' };
        const tau = t - an.tPush;
        return { y: HIP + an.v * tau - 0.5 * an.g * tau * tau, v: an.v - an.g * tau, a: -an.g, F: 0, contact: false, phase: tau < an.tUp ? 'Rising' : 'Falling back' };
      }
      if (an.mode === 'land') {
        if (t <= an.tFall) return { y: HIP + an.h - 0.5 * an.g * t * t, v: -an.g * t, a: -an.g, F: 0, contact: false, phase: 'Falling' };
        const tau = Math.min(t - an.tFall, an.tStop);
        return { y: HIP - (an.v * tau - 0.5 * an.a * tau * tau), v: -an.v + an.a * tau, a: an.a, F: an.F, contact: true, phase: 'Stopping' };
      }
      const tt = Math.min(t, an.t);
      return { y: 0.5 * an.a * tt * tt + (t > an.t ? an.v * (t - an.t) : 0), v: t <= an.t ? an.a * t : an.v, a: t <= an.t ? an.a : 0, F: t <= an.t ? an.f : 0, contact: true, phase: 'Speeding up' };
    },
    buildSim(an) {
      if (an.kind === 'traction') return { kind: 'traction', tStop: 0, reason: null, static: true };
      const tStop = an.mode === 'jump' ? an.tPush + 2 * an.tUp : an.mode === 'land' ? an.tFall + an.tStop : an.t;
      const reason = an.mode === 'jump' ? 'The person landed back on the floor' : an.mode === 'land' ? 'The person has stopped' : 'The runner reached top speed';
      return { kind: 'body', tStop, reason };
    },
    motion(an, t) {
      if (an.kind === 'traction') return { s: 0, v: 0, a: 0 };
      const st = BODY.state(an, t);
      const y0 = BODY.state(an, 0).y;
      return { s: st.y - y0, v: st.v, a: st.a };
    },
    graphNames: () => (cur().mode === 'sprint' ? { x: 'Position–time', v: 'Velocity–time', a: 'Ground force–time' } : { x: 'Height–time', v: 'Velocity–time', a: 'Floor force–time' }),
    graphValue(an, which, t) {
      const st = BODY.state(an, t);
      return which === 'x' ? st.y : which === 'v' ? st.v : st.F;
    },
    graphLabel: () =>
      cur().mode === 'sprint'
        ? { x: 'position (m)', v: 'velocity (m/s)', a: 'forward force from ground (N)' }
        : { x: 'hip height (m)', v: 'velocity (m/s)', a: 'force from the floor (N)' },
    // The pose that matches the physics at time t
    poseAt(an, t) {
      const st = BODY.state(an, t);
      if (an.mode === 'jump') {
        if (t <= an.tPush) return poseUpright({ x: 0.2, yHip: st.y, contact: true, armA: lerp(-0.9, 2.7, clamp01(t / an.tPush)) });
        const f = clamp01((t - an.tPush) / (2 * an.tUp));
        return poseUpright({ x: 0.2, yHip: st.y, contact: false, armA: lerp(2.7, 0.7, f), tuck: 0.08 });
      }
      if (an.mode === 'land') {
        if (t <= an.tFall) return poseUpright({ x: 0.2, yHip: st.y, contact: false, armA: 1.35, tuck: 0.02 });
        return poseUpright({ x: 0.2, yHip: st.y, contact: true, armA: lerp(1.35, 0.95, clamp01((t - an.tFall) / an.tStop)) });
      }
      const stride = 1.2 + 0.12 * an.v;
      const lean = 0.12 + 0.33 * (1 - Math.min(1, st.v / an.v));
      return poseRun(st.y, (st.y / stride) * 2 * Math.PI + 0.6, lean);
    },
    draw(ctx, W, H, t) {
      const an = analysis;
      if (an.kind === 'traction') return BODY.drawTraction(ctx, W, H);
      if (an.mode === 'sprint') return BODY.drawSprint(ctx, W, H, t);
      const st = BODY.state(an, t);
      const top = an.mode === 'jump' ? HIP + an.h + 0.95 : Math.max(an.h + HIP + 0.95, 2.2);
      const view = fitView({ x0: -1.9, x1: 1.9, y0: -0.15, y1: Math.max(2.2, top + 0.15) }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -1.9, 1.9);
      // meter stick
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.lineWidth = 1;
      const maxH = Math.floor(Math.max(2, top) * 10);
      for (let i = 0; i <= maxH; i++) {
        const [mx, my] = P(-1.55, i * 0.1);
        const len = i % 10 === 0 ? 10 : i % 5 === 0 ? 7 : 4;
        ctx.beginPath();
        ctx.moveTo(mx, my);
        ctx.lineTo(mx + len, my);
        ctx.stroke();
        if (i % 5 === 0 && view.k > 40) labelText(ctx, (i / 10).toFixed(1), mx - 6, my, C.muted, 'right', 12, 400);
      }
      ctx.restore();
      const guide = (y, text) => {
        const [gx0, gy] = P(text ? -0.9 : -0.35, y);
        const [gx1] = P(text ? 0.75 : 1.2, y);
        ctx.save();
        ctx.strokeStyle = C.muted;
        ctx.setLineDash([4, 5]);
        ctx.beginPath();
        ctx.moveTo(gx0, gy);
        ctx.lineTo(gx1, gy);
        ctx.stroke();
        ctx.restore();
        if (text) labelText(ctx, text, gx1 + 6, gy, C.muted, 'left', 13, 600, C.fontBody);
      };
      if (an.mode === 'jump') {
        guide(HIP - an.d, 'hips at the start');
        guide(HIP, 'hips at takeoff');
        guide(HIP + an.h, 'hips at the top');
        dimV(ctx, view, -0.75, HIP - an.d, HIP, 'd', C.accent, 'left');
        dimV(ctx, view, -0.75, HIP, HIP + an.h, 'h', C['f-a'], 'left');
      } else {
        // the ledge the person steps off
        const [lx0, ly0] = P(-1.3, an.h);
        const [lx1, ly1] = P(-0.4, 0);
        ctx.save();
        ctx.fillStyle = C.bench;
        ctx.strokeStyle = C['bench-edge'];
        ctx.fillRect(lx0, ly0, lx1 - lx0, ly1 - ly0);
        ctx.strokeRect(lx0, ly0, lx1 - lx0, ly1 - ly0);
        ctx.restore();
        guide(HIP, '');
        guide(HIP - an.d, '');
        guide(an.h, '');
        dimV(ctx, view, 1.45, 0, an.h, 'h', C['f-a'], 'right');
        dimV(ctx, view, -0.15, HIP - an.d, HIP, 'd', C.accent, 'left');
        labelText(ctx, 'Dashed lines: where the feet start, and the hip height when the feet touch and when stopped.', 12, 40, C.muted, 'left', 13, 400, C.fontBody);
      }
      for (const ts of strobeTimes(t)) {
        const g0 = BODY.state(an, ts);
        const [gx, gy] = P(0.2, g0.y);
        ctx.save();
        ctx.globalAlpha = 0.4;
        ctx.fillStyle = C.ink;
        ctx.beginPath();
        ctx.arc(gx, gy, 3, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
      }
      const pose = BODY.poseAt(an, t);
      drawFigure(ctx, view, pose, { label: W < 520 ? '' : trimNum(an.m) + ' kg' });
      // hip marker: the point whose motion the numbers describe
      const [hx, hy] = P(pose.hip.x, pose.hip.y);
      ctx.save();
      ctx.fillStyle = C.accent;
      ctx.strokeStyle = C.scene;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(hx, hy, 5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      // the floor's push while the feet are on the floor
      if (st.F > 1e-9) {
        const [fx, fy] = P(an.mode === 'land' ? 0.75 : 0.62, 0);
        const len = Math.min(150, 30 + (st.F / an.W) * 20);
        arrow(ctx, fx, fy, fx, fy - len, C['f-n'], 4.5, 14);
        labelText(ctx, 'F_floor', fx + 10, fy - len * 0.55, C['f-n'], 'left', 15);
      }
      labelText(ctx, st.phase, 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawSprint(ctx, W, H, t) {
      const an = analysis;
      const st = BODY.state(an, t);
      const view = fitView({ x0: st.y - 3.2, x1: st.y + 3.2, y0: -0.45, y1: 2.3 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      // running track
      const [tx0, ty0] = P(st.y - 5, 0);
      const [tx1, ty1] = P(st.y + 5, -0.3);
      ctx.save();
      ctx.fillStyle = C.track;
      ctx.fillRect(tx0, ty0, tx1 - tx0, ty1 - ty0);
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(tx0, ty0);
      ctx.lineTo(tx1, ty0);
      ctx.stroke();
      ctx.restore();
      // distance marks
      for (let x = Math.floor(st.y - 4); x <= st.y + 4; x++) {
        if (x < 0 || x > an.d + 3) continue;
        const [mx, my] = P(x, 0);
        ctx.save();
        ctx.strokeStyle = C.ink;
        ctx.globalAlpha = 0.7;
        ctx.beginPath();
        ctx.moveTo(mx, my);
        ctx.lineTo(mx, my + (x % 5 === 0 ? 14 : 7));
        ctx.stroke();
        ctx.restore();
        if (x % 5 === 0 || x === Math.round(an.d)) labelText(ctx, x + ' m', mx, my + 24, C.ink, 'center', 12, 600);
      }
      for (const [x, txt] of [[0, 'Start'], [an.d, 'Top speed']]) {
        const [lx, ly] = P(x, 0);
        const [, ly2] = P(x, 2.05);
        ctx.save();
        ctx.strokeStyle = C.accent;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(lx, ly);
        ctx.lineTo(lx, ly2);
        ctx.stroke();
        ctx.restore();
        labelText(ctx, txt, lx, ly2 - 10, C.accent, 'center', 13, 700, C.fontBody);
      }
      // starting block
      const [bx0, by0] = P(-0.42, 0);
      const [bx1, by1] = P(-0.12, 0.16);
      ctx.save();
      ctx.fillStyle = C['bench-edge'];
      ctx.beginPath();
      ctx.moveTo(bx0, by0);
      ctx.lineTo(bx1, by0);
      ctx.lineTo(bx1, by1);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      for (const ts of strobeTimes(t)) {
        const [gx, gy] = P(BODY.state(an, ts).y, HIP);
        ctx.save();
        ctx.globalAlpha = 0.4;
        ctx.fillStyle = C.ink;
        ctx.beginPath();
        ctx.arc(gx, gy, 3, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
      }
      const pose = BODY.poseAt(an, t);
      drawFigure(ctx, view, pose, { label: W < 520 ? '' : trimNum(an.m) + ' kg', labelLeft: true });
      if (st.F > 1e-9) {
        // friction acts at the foot on the ground
        const foot = pose.legs[0].toe.y < pose.legs[1].toe.y ? pose.legs[0] : pose.legs[1];
        const [fx, fy] = P(foot.ankle.x, 0);
        arrow(ctx, fx - 20, fy + 8, fx + 60, fy + 8, C['f-f'], 4.5, 14);
        labelText(ctx, 'f (track pushes the runner forward)', fx + 66, fy + 8, C['f-f'], 'left', 14);
      }
      labelText(ctx, t <= an.t && st.v < an.v - 1e-9 ? 'Speeding up' : 'At top speed', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawTraction(ctx, W, H) {
      const an = analysis;
      const a1 = an.th1 * DEG;
      const a2 = an.th2 * DEG;
      const Py = 1.7;
      const Px = 0.32;
      const Xf = Px + 1.7;
      const U = { x: Xf, y: Py + (Xf - Px) * Math.tan(a1) };
      const Lw = { x: Xf, y: Py - (Xf - Px) * Math.tan(a2) };
      const view = fitView({ x0: -2.85, x1: Xf + 0.95, y0: -0.15, y1: Math.max(U.y + 0.4, 2.4) }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -2.85, Xf + 0.95);
      const box = (x0, y0, x1, y1, fill, stroke, r = 0) => {
        const [a, b] = P(x0, y1);
        const [c2, d2] = P(x1, y0);
        ctx.save();
        ctx.fillStyle = fill;
        ctx.strokeStyle = stroke || C['bench-edge'];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(a, b, c2 - a, d2 - b, r * view.k);
        else ctx.rect(a, b, c2 - a, d2 - b);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      };
      // hospital bed: legs, frame, mattress and pillow
      box(-2.62, 0, -2.54, 1.3, C.bench);
      box(-0.1, 0, -0.02, 1.3, C.bench);
      box(-2.7, 1.25, 0.05, 1.35, C.bench);
      box(-2.68, 1.35, 0.02, 1.5, C.panel, C.muted, 0.05);
      box(-2.62, 1.5, -2.1, 1.62, C.panel, C.muted, 0.06);
      // traction frame with the top pulley and the rope's anchor
      const postTop = Math.max(U.y, 1.9) + 0.25;
      box(Xf + 0.06, 0, Xf + 0.16, postTop, C.bench);
      // patient
      const footAt = { x: 0.06, y: Py - 0.02 };
      const pose = poseLying(1.5, footAt);
      drawFigure(ctx, view, pose, {});
      // boot around the foot, with a short bar to the pulley
      const near = pose.legs[1];
      const [ax, ay] = P(near.ankle.x, near.ankle.y);
      const [tx, ty] = P(near.toe.x, near.toe.y);
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineCap = 'round';
      ctx.lineWidth = 0.16 * view.k;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      const [px, py] = P(Px, Py);
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo((ax + tx) / 2 + 0.06 * view.k, (ay + ty) / 2);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.restore();
      // the rope: anchor -> foot pulley -> top pulley -> down to the weight
      const [ux, uy] = P(U.x, U.y);
      const [lx, ly] = P(Lw.x, Lw.y);
      const rp = 0.08 * view.k;
      const [, wy] = P(0, Math.max(0.4, U.y - 1.05));
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(lx, ly);
      ctx.lineTo(px, py);
      ctx.lineTo(ux, uy);
      ctx.moveTo(ux + rp, uy);
      ctx.lineTo(ux + rp, wy);
      ctx.stroke();
      for (const [x, y] of [[px, py], [ux, uy]]) {
        ctx.fillStyle = C.panel;
        ctx.beginPath();
        ctx.arc(x, y, rp, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = C.ink;
        ctx.beginPath();
        ctx.arc(x, y, 2.5, 0, 2 * Math.PI);
        ctx.fill();
      }
      ctx.fillStyle = C['bench-edge'];
      ctx.fillRect(lx - 6, ly - 6, 12, 12);
      // hanging weight with a hook
      const wb = 0.3 * view.k;
      ctx.beginPath();
      ctx.arc(ux + rp, wy + 4, 4, 0, Math.PI);
      ctx.stroke();
      ctx.fillStyle = bodyColor(1);
      ctx.fillRect(ux + rp - wb / 2, wy + 8, wb, wb);
      ctx.strokeRect(ux + rp - wb / 2, wy + 8, wb, wb);
      ctx.restore();
      labelText(ctx, trimNum(an.m) + ' kg', ux + rp + wb / 2 + 8, wy + 8 + wb / 2, C.ink, 'left', 14);
      // angles at the foot pulley
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + 0.8 * view.k, py);
      ctx.stroke();
      ctx.setLineDash([]);
      const rr = 0.5 * view.k;
      ctx.beginPath();
      ctx.arc(px, py, rr, -a1, 0);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(px, py, rr * 0.8, 0, a2);
      ctx.stroke();
      ctx.restore();
      // short angle names near the pulley; the values go in the corner
      const la1 = a1 + 0.38;
      const la2 = a2 + 0.42;
      labelText(ctx, 'θ₁', px + rr * 0.78 * Math.cos(la1), py - rr * 0.78 * Math.sin(la1), C.ink, 'center', 15);
      labelText(ctx, 'θ₂', px + rr * 0.72 * Math.cos(la2), py + rr * 0.72 * Math.sin(la2), C.ink, 'center', 15);
      labelText(ctx, 'θ₁ = ' + trimNum(an.th1) + '° (upper rope)    θ₂ = ' + trimNum(an.th2) + '° (lower rope)', 12, 40, C.muted, 'left', 13, 600, C.fontBody);
      // force vectors on the foot pulley
      const sc = (1.05 * view.k) / Math.max(an.F, 1e-9);
      const tip = (fx, fy) => [px + fx * sc, py - fy * sc];
      const [t1x, t1y] = tip(an.T * Math.cos(a1), an.T * Math.sin(a1));
      const [t2x, t2y] = tip(an.T * Math.cos(a2), -an.T * Math.sin(a2));
      arrow(ctx, px, py, t1x, t1y, C.l1, 3, 11);
      arrow(ctx, px, py, t2x, t2y, C.l1, 3, 11);
      const out = (x, y, ang, sgn) => [x + 14 * Math.cos(ang) - sgn * 10 * Math.sin(ang), y - sgn * 14 * Math.sin(ang) * sgn - 10 * Math.cos(ang) * sgn];
      const [l1x, l1y] = out(t1x, t1y, a1, 1);
      const [l2x, l2y] = [t2x + 14 * Math.cos(a2) + 10 * Math.sin(a2), t2y + 14 * Math.sin(a2) + 10 * Math.cos(a2)];
      labelText(ctx, 'T', l1x, l1y, C.l1, 'center', 15);
      labelText(ctx, 'T', l2x, l2y, C.l1, 'center', 15);
      const [rx, ry] = tip(an.Fx, an.Fy);
      arrow(ctx, px, py, rx, ry, C['f-a'], 4.5, 14);
      labelText(ctx, 'F', rx + 10, ry, C['f-a'], 'left', 17);
      labelText(ctx, 'At rest: an equilibrium problem', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    forceSets(an) {
      if (an.kind === 'traction') {
        const d = (deg) => ({ x: Math.cos(deg * DEG), y: Math.sin(deg * DEG) });
        return [
          {
            body: { name: 'Foot and pulley', m: null, kind: 'hanging' },
            idx: 2,
            tilt: 0,
            arrows: [
              { dir: d(an.th1), mag: an.T, color: C.l1, label: 'T' },
              { dir: d(-an.th2), mag: an.T, color: C.l1, label: 'T' },
              { dir: d(an.angle + 180), mag: an.F, color: C['f-n'], label: 'F_leg' },
            ],
          },
          {
            body: an.bodies[0],
            idx: 1,
            tilt: 0,
            arrows: [
              { dir: { x: 0, y: 1 }, mag: an.T, color: C.l1, label: 'T' },
              { dir: { x: 0, y: -1 }, mag: an.m * an.g, color: C['f-w'], label: 'W' },
            ],
          },
        ];
      }
      const person = an.bodies[0];
      if (an.mode === 'sprint') {
        return [{
          body: { name: 'Runner, speeding up', m: an.m, kind: 'person' },
          idx: 0,
          tilt: 0,
          pose: poseRun(0, 1.4, 0.3),
          arrows: [
            { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
            { dir: { x: 0, y: 1 }, mag: an.N, color: C['f-n'], label: 'N' },
            { dir: { x: 1, y: 0 }, mag: an.f, color: C['f-f'], label: 'fₛ' },
          ],
        }];
      }
      const contact = { dir: { x: 0, y: 1 }, mag: an.F, color: C['f-n'], label: 'F_floor' };
      const W = { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' };
      const crouched = poseUpright({ x: 0, yHip: STAND_HIP - 0.3, contact: true, armA: an.mode === 'jump' ? 1.2 : 1.1 });
      const flying = poseUpright({ x: 0, yHip: STAND_HIP, contact: false, armA: an.mode === 'jump' ? 2.6 : 1.35, tuck: 0.06 });
      const a = [{ body: { name: an.mode === 'jump' ? 'Pushing off' : 'Falling', m: person.m, kind: 'person' }, idx: 0, tilt: 0, pose: an.mode === 'jump' ? crouched : flying, arrows: an.mode === 'jump' ? [W, contact] : [W] }];
      a.push({ body: { name: an.mode === 'jump' ? 'In the air' : 'Stopping', m: person.m, kind: 'person' }, idx: 0, tilt: 0, pose: an.mode === 'jump' ? flying : crouched, arrows: an.mode === 'jump' ? [W] : [W, contact] });
      return a;
    },
    fbdCaption(an) {
      if (an.kind === 'traction') return 'The foot and its pulley are at rest, so the leg must pull back with a force equal and opposite to the two rope tensions combined.';
      if (an.mode === 'sprint') return 'While the runner speeds up. Once at top speed, f drops to about zero.';
      return 'The same person at two moments. Only the weight acts while the person is off the floor.';
    },
    legend(an, showV) {
      const chip = (color, text) => '<span><i style="background:' + color + '"></i>' + text + '</span>';
      const v = (x) => (showV ? ' (' + fmt(x) + ' N)' : '');
      if (an.kind === 'traction') {
        return {
          legend: chip('var(--l1)', 'T: rope tension') + chip('var(--f-a)', 'F: traction force on the leg') + chip('var(--f-n)', 'F_leg: the leg pulls back on the pulley') + chip('var(--f-w)', 'W: weight'),
          pairs: '<h3>Action–reaction pairs</h3><ul>' +
            '<li>The pulley pulls the foot with the traction force F' + v(an.F) + ', and the foot pulls back on the pulley with <b>F_leg</b>, equal and opposite. That is a <b>third-law pair</b>.</li>' +
            '<li>The rope pulls on each pulley and each pulley pulls back on the rope. Because the rope and pulleys are massless and frictionless, the tension is T = mg everywhere' + v(an.T) + '.</li>' +
            '<li>Two rope segments pull on the foot pulley, so the traction force can be <b>bigger</b> than the hanging weight.</li></ul>',
        };
      }
      if (an.mode === 'sprint') {
        return {
          legend: chip('var(--f-w)', 'W: weight') + chip('var(--f-n)', 'N: normal force') + chip('var(--f-f)', 'fₛ: static friction from the track'),
          pairs: '<h3>Action–reaction pairs</h3><ul>' +
            '<li>The runner’s foot pushes <b>backward</b> on the track, and the track pushes the runner <b>forward</b> with friction fₛ' + v(an.f) + '. That forward push is the only horizontal force, so it is what accelerates the runner.</li>' +
            '<li>The friction is static because the foot does not slide while it pushes.</li>' +
            '<li>N and W act on the runner, so they are not a pair with each other. The track pushes up with N; the runner pushes down on the track just as hard.</li></ul>',
        };
      }
      return {
        legend: chip('var(--f-w)', 'W: weight') + chip('var(--f-n)', 'F_floor: the floor pushes up on the person'),
        pairs: '<h3>Action–reaction pairs</h3><ul>' +
          '<li>The floor pushes up on the person with F_floor' + v(an.F) + ', and the person pushes down on the floor just as hard. That is a <b>third-law pair</b>.</li>' +
          '<li>F_floor is bigger than W while the person ' + (an.mode === 'jump' ? 'speeds up upward' : 'slows to a stop') + '. The difference is the net force: F_floor − W = ma.</li>' +
          (an.mode === 'land' ? '<li>A smaller stopping distance d means a bigger deceleration and a bigger force on the legs. That is why bending your knees protects your joints.</li>' : '') +
          '<li>In the air, the only force is W, so the person accelerates downward at g.</li></ul>',
      };
    },
    quantities(an) {
      const q = [];
      const row = (key, label, unit, value, sub) => q.push({ key, group: 'main', label, unit, value, sub });
      if (an.kind === 'traction') {
        row('T', 'Tension in the rope', 'N', an.T);
        row('Fx', 'Horizontal part of the traction force', 'N', Math.abs(an.Fx));
        row('Fy', 'Vertical part of the traction force', 'N', Math.abs(an.Fy), 'up if θ₁ > θ₂, down if θ₂ > θ₁');
        row('F', 'Size of the traction force on the leg', 'N', an.F);
        row('ang', 'Direction of the traction force', '°', Math.abs(an.angle), 'angle from the horizontal');
        return q;
      }
      if (an.mode === 'sprint') {
        row('a', 'Acceleration of the runner', 'm/s²', an.a);
        row('t', 'Time to reach top speed', 's', an.t);
        row('f', 'Forward force from the track (friction)', 'N', an.f);
        row('Fg', 'Total force from the track', 'N', an.Fground, 'combine f and N');
        q.push({ key: 'ratio', group: 'more', label: 'Friction as a fraction of body weight', unit: '× W', value: an.ratio });
        q.push({ key: 'W', group: 'more', label: 'Weight of the runner', unit: 'N', value: an.W });
        return q;
      }
      if (an.mode === 'jump') {
        row('v', 'Takeoff speed', 'm/s', an.v);
        row('a', 'Acceleration while pushing off', 'm/s²', an.a);
        row('F', 'Force from the floor while pushing off', 'N', an.F);
        row('ratio', 'Floor force in body weights', '× W', an.ratio);
        q.push({ key: 't', group: 'more', label: 'Time spent pushing off', unit: 's', value: an.tPush });
      } else {
        row('v', 'Speed just as the feet touch', 'm/s', an.v);
        row('a', 'Deceleration while stopping', 'm/s²', an.a);
        row('F', 'Force from the floor while stopping', 'N', an.F);
        row('ratio', 'Floor force in body weights', '× W', an.ratio);
        q.push({ key: 't', group: 'more', label: 'Time to stop', unit: 's', value: an.tStop });
      }
      q.push({ key: 'W', group: 'more', label: 'Weight of the person', unit: 'N', value: an.W });
      q.push({ key: 'net', group: 'more', label: 'Net force while ' + (an.mode === 'jump' ? 'pushing off' : 'stopping'), unit: 'N', value: an.m * an.a });
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const g = fmt(an.g);
      if (an.kind === 'traction') {
        return '<section><h3>1. Tension</h3><p>The rope holds the hanging weight still, and massless, frictionless pulleys don’t change the tension, so it is the same in every segment.</p>' +
          '<div class="eq">T = mg = ' + fmt(an.m) + ' × ' + g + ' = ' + fmt(an.T) + ' N</div></section>' +
          '<section><h3>2. Add the two rope forces on the foot pulley</h3><p>Take x to the right (away from the body) and y up. One segment pulls at θ₁ = ' + trimNum(an.th1) + '° above the horizontal, the other at θ₂ = ' + trimNum(an.th2) + '° below it.</p>' +
          '<div class="eq">Fx = T cos θ₁ + T cos θ₂ = ' + fmt(an.T) + ' × (' + fmt(Math.cos(an.th1 * DEG), 3) + ' + ' + fmt(Math.cos(an.th2 * DEG), 3) + ') = ' + fmt(an.Fx) + ' N\n' +
          'Fy = T sin θ₁ − T sin θ₂ = ' + fmt(an.T) + ' × (' + fmt(Math.sin(an.th1 * DEG), 3) + ' − ' + fmt(Math.sin(an.th2 * DEG), 3) + ') = ' + fmt(an.Fy) + ' N</div></section>' +
          '<section><h3>3. Size and direction</h3><div class="eq">F = √(Fx² + Fy²) = √(' + fmt(an.Fx) + '² + ' + fmt(an.Fy) + '²) = ' + fmt(an.F) + ' N\n' +
          'angle = tan⁻¹(Fy / Fx) = ' + fmt(an.angle, 1) + '° ' + (an.angle >= 0 ? 'above' : 'below') + ' the horizontal</div>' +
          '<p>The leg is at rest, so the body (through the leg) pulls back on the pulley with an equal and opposite force: Newton’s first and third laws together.</p></section>';
      }
      if (an.mode === 'sprint') {
        return '<section><h3>1. Acceleration from kinematics</h3><p>Starting from rest and reaching v = ' + fmt(an.v) + ' m/s in d = ' + fmt(an.d) + ' m:</p>' +
          '<div class="eq">v² = 2ad   →   a = v² / (2d) = ' + fmt(an.v) + '² / (2 × ' + fmt(an.d) + ') = ' + fmt(an.a) + ' m/s²\nt = v / a = ' + fmt(an.t) + ' s</div></section>' +
          '<section><h3>2. Horizontal force from Newton’s second law</h3><p>The only horizontal force on the runner is static friction from the track.</p>' +
          '<div class="eq">f = ma = ' + fmt(an.m) + ' × ' + fmt(an.a) + ' = ' + fmt(an.f) + ' N   (' + fmt(an.ratio) + ' × body weight)</div></section>' +
          '<section><h3>3. Total force from the track</h3><p>Vertically nothing accelerates, so N = W = mg = ' + fmt(an.W) + ' N. The track’s total push combines both parts:</p>' +
          '<div class="eq">F = √(f² + N²) = √(' + fmt(an.f) + '² + ' + fmt(an.N) + '²) = ' + fmt(an.Fground) + ' N, at ' + fmt(an.groundAngle, 1) + '° above the horizontal</div></section>';
      }
      const jump = an.mode === 'jump';
      let h = '<section><h3>1. Speed from kinematics</h3>';
      h += jump
        ? '<p>After the feet leave the floor, only gravity acts. Rising h = ' + fmt(an.h) + ' m and stopping at the top means:</p><div class="eq">v² = 2gh   →   v = √(2 × ' + g + ' × ' + fmt(an.h) + ') = ' + fmt(an.v) + ' m/s</div></section>'
        : '<p>Falling from rest through h = ' + fmt(an.h) + ' m:</p><div class="eq">v² = 2gh   →   v = √(2 × ' + g + ' × ' + fmt(an.h) + ') = ' + fmt(an.v) + ' m/s</div></section>';
      h += '<section><h3>2. Acceleration while ' + (jump ? 'pushing off' : 'stopping') + '</h3><p>' + (jump ? 'The hips rise d = ' + fmt(an.d) + ' m while the legs push, going from rest to ' + fmt(an.v) + ' m/s:' : 'The knees bend through d = ' + fmt(an.d) + ' m, taking the speed from ' + fmt(an.v) + ' m/s to zero:') + '</p>' +
        '<div class="eq">a = v² / (2d) = ' + fmt(an.v) + '² / (2 × ' + fmt(an.d) + ') = ' + fmt(an.a) + ' m/s²   (upward)</div></section>';
      h += '<section><h3>3. Force from the floor</h3><p>Take + as up. Two forces act: the floor pushes up with F_floor and gravity pulls down with W.</p>' +
        '<div class="eq">F_floor − mg = ma\nF_floor = m(g + a) = ' + fmt(an.m) + ' × (' + g + ' + ' + fmt(an.a) + ') = ' + fmt(an.F) + ' N\n' +
        'In body weights: F_floor / W = ' + fmt(an.F) + ' / ' + fmt(an.W) + ' = ' + fmt(an.ratio) + '</div>' +
        (jump ? '' : '<p>Since a = gh/d, this is F_floor = mg(1 + h/d). Halving the stopping distance roughly doubles the force on the legs.</p>') + '</section>';
      return h;
    },
  };

  /* ---------------- shared helpers for the new tabs ---------------- */
  const chipH = (color, text) => '<span><i style="background:' + color + '"></i>' + text + '</span>';
  const pairsH = (items) => '<h3>Action–reaction pairs</h3><ul>' + items.map((x) => '<li>' + x + '</li>').join('') + '</ul>';
  const swatch = (v) => '<span class="swatch" style="background:var(' + v + ')"></span>';
  function boxW(ctx, view, x0, y0, x1, y1, fill, stroke, r = 0) {
    const [a, b] = view.P(x0, y1);
    const [c2, d2] = view.P(x1, y0);
    ctx.save();
    ctx.fillStyle = fill;
    ctx.strokeStyle = stroke || C['bench-edge'];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    if (ctx.roundRect && r) ctx.roundRect(a, b, c2 - a, d2 - b, r * view.k);
    else ctx.rect(a, b, c2 - a, d2 - b);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
  function wheel(ctx, view, x, y, r) {
    const [cx, cy] = view.P(x, y);
    ctx.save();
    ctx.fillStyle = C.ink;
    ctx.strokeStyle = C.scene;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r * view.k, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = C.muted;
    ctx.beginPath();
    ctx.arc(cx, cy, r * view.k * 0.35, 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();
  }
  // world-space force arrow from point p in direction (dx, dy), length in pixels
  function forceArrow(ctx, view, p, dx, dy, len, color, label, side = 'auto') {
    const [x1, y1] = view.P(p.x, p.y);
    const L = Math.hypot(dx, dy) || 1;
    const ux = dx / L;
    const uy = -dy / L;
    const x2 = x1 + ux * len;
    const y2 = y1 + uy * len;
    arrow(ctx, x1, y1, x2, y2, color, 4, 13);
    if (label) {
      const align = side === 'auto' ? (Math.abs(ux) > 0.4 ? (ux > 0 ? 'left' : 'right') : 'left') : side;
      labelText(ctx, label, x2 + (Math.abs(ux) > 0.4 ? ux * 8 : 10), y2 + (Math.abs(ux) > 0.4 ? -12 : uy * 6), color, align, 15);
    }
  }
  const staticPlay = 'Nothing moves in this problem. Use the diagrams and Predict & check.';

  /* ---------------- Equilibrium: traffic light and block on a wall ---------------- */
  const STATICS_MODES = [
    ['traffic', 'Hanging traffic light (two cables)'],
    ['wall', 'Block pressed against a wall'],
    ['tackle', 'Block and tackle (pulley system)'],
  ];
  const STATICS = {
    defaults: () => ({ mode: 'traffic', traffic: { m: 15, th1: 30, th2: 45 }, wall: { m: 2, F: 60, phi: 0, mus: 0.5, muk: 0.4 }, tackle: TACKLE.defaults(), mystery: { on: false, target: '' } }),
    solve(c) {
      if (c.mode === 'traffic') {
        const an = P.solveTrafficLight({ m: c.traffic.m, th1: c.traffic.th1, th2: c.traffic.th2, g: state.g });
        an.bodies = [{ name: 'Traffic light', m: c.traffic.m, kind: 'hanging', a: 0 }];
        an.atRest = true;
        return an;
      }
      const w = c.wall;
      const an = P.solveWall({ m: w.m, F: w.F, phi: w.phi, mus: w.mus, muk: w.muk, g: state.g });
      an.bodies = [{ name: 'Block', m: w.m, kind: 'block', a: an.a }];
      an.atRest = an.state === 'rest';
      return an;
    },
    controls(c) {
      let h = '<div class="group"><h2>Problem</h2>' + selectField('Type of problem', 'sel-statics-mode', 'mode', STATICS_MODES, c.mode, true) + '</div>';
      if (c.mode === 'traffic') {
        h += '<div class="group"><h2>Traffic light</h2>' +
          numField('Mass of the light', 'traffic.m', c.traffic.m, 'kg', { min: 1, max: 200, step: 0.5 }) +
          numField('Left cable angle θ₁ (above horizontal)', 'traffic.th1', c.traffic.th1, '°', { min: 3, max: 80, step: 1 }) +
          numField('Right cable angle θ₂ (above horizontal)', 'traffic.th2', c.traffic.th2, '°', { min: 3, max: 80, step: 1 }) +
          '<p class="note">The light hangs at rest from a knot where the two cables meet.</p></div>';
      } else {
        const w = c.wall;
        h += '<div class="group"><h2>Block and push</h2>' +
          numField('Mass of the block', 'wall.m', w.m, 'kg', { min: 0.1, max: 50, step: 0.1 }) +
          numField('Size of the push F', 'wall.F', w.F, 'N', { min: 0, max: 2000, step: 1 }) +
          numField('Push angle φ (above horizontal)', 'wall.phi', w.phi, '°', { min: 0, max: 80, step: 1 }) +
          '<p class="note">The push points into the wall. A positive angle also pushes the block upward.</p></div>' +
          '<div class="group"><h2>Friction with the wall</h2>' +
          numField('Static coefficient μs', 'wall.mus', w.mus, '', { min: 0, max: 2, step: 0.05 }) +
          numField('Kinetic coefficient μk', 'wall.muk', w.muk, '', { min: 0, max: 2, step: 0.05 }) +
          (w.muk > w.mus ? '<p class="warn">μk should not be larger than μs. The simulator uses μk = μs.</p>' : '') + '</div>';
      }
      return h;
    },
    randomize(c, r) {
      c.traffic = { m: r(5, 40, 0.5), th1: r(15, 60, 1), th2: r(15, 60, 1) };
      const mus = r(0.3, 0.8, 0.05);
      c.tackle = Object.assign(c.tackle || TACKLE.defaults(), { m: r(20, 300, 5), n: r(2, 6, 1), a: r(0.2, 2, 0.1) });
      c.wall = { m: r(0.5, 5, 0.1), F: r(10, 150, 1), phi: c.wall.phi > 0 ? r(10, 60, 5) : 0, mus, muk: Math.max(0.05, Math.round((mus - r(0.05, 0.2, 0.05)) * 100) / 100) };
    },
    load(c, num) {
      const d = STATICS.defaults();
      const t = c.traffic || {};
      const w = c.wall || {};
      return {
        mode: STATICS_MODES.some((x) => x[0] === c.mode) ? c.mode : d.mode,
        traffic: { m: num(t.m, d.traffic.m, 1, 200), th1: num(t.th1, d.traffic.th1, 3, 80), th2: num(t.th2, d.traffic.th2, 3, 80) },
        wall: { m: num(w.m, d.wall.m, 0.1, 50), F: num(w.F, d.wall.F, 0, 2000), phi: num(w.phi, 0, 0, 80), mus: num(w.mus, 0.5, 0, 2), muk: num(w.muk, 0.4, 0, 2) },
        tackle: (() => {
          const k = c.tackle || {};
          const dk = TACKLE.defaults();
          return {
            m: num(k.m, dk.m, 1, 2000),
            n: Math.round(num(k.n, dk.n, 1, 6)),
            motion: TACKLE_MOTIONS.some((x) => x[0] === k.motion) ? k.motion : dk.motion,
            a: num(k.a, dk.a, 0.05, 5),
          };
        })(),
        mystery: d.mystery,
      };
    },
    // wall: block height above its start; it starts 1.6 m up the wall
    wallY: (an, t) => 1.6 + 0.5 * an.a * t * t,
    buildSim(an) {
      if (an.kind === 'traffic' || an.state === 'rest') return { kind: an.kind, tStop: 0, reason: null, static: true, staticMessage: an.kind === 'traffic' ? 'This is an equilibrium problem: the light hangs at rest. Use the diagrams and Predict & check.' : 'Static friction holds the block in place: nothing moves.' };
      // slides down to the floor, or up 1.2 m
      const dist = an.a < 0 ? 1.6 : 1.2;
      const tStop = Math.sqrt((2 * dist) / Math.abs(an.a));
      return { kind: 'wall', tStop, reason: an.a < 0 ? 'The block slid down to the floor' : 'The block slid up the wall' };
    },
    motion(an, t) {
      if (an.kind === 'traffic' || an.state === 'rest') return { s: 0, v: 0, a: 0 };
      return { s: 0.5 * an.a * t * t, v: an.a * t, a: an.a };
    },
    noGraph: (an) => an.kind === 'traffic' || an.state === 'rest',
    graphNames: { x: 'Position–time', v: 'Velocity–time', a: 'Acceleration–time' },
    graphValue(an, which, t) {
      const m = STATICS.motion(an, t);
      return which === 'x' ? m.s : which === 'v' ? m.v : m.a;
    },
    graphLabel: { x: 'displacement (m, + up)', v: 'velocity (m/s)', a: 'acceleration (m/s²)' },
    draw(ctx, W, H, t) {
      const an = analysis;
      return an.kind === 'traffic' ? STATICS.drawTraffic(ctx, W, H) : STATICS.drawWall(ctx, W, H, t);
    },
    drawTraffic(ctx, W, H) {
      const an = analysis;
      const a1 = an.th1 * DEG;
      const a2 = an.th2 * DEG;
      const knot = { x: 0, y: 3.2 };
      const span = 2.4;
      const A = { x: -span, y: knot.y + span * Math.tan(a1) };
      const B = { x: span, y: knot.y + span * Math.tan(a2) };
      const top = Math.max(A.y, B.y) + 0.5;
      const view = fitView({ x0: -3.2, x1: 3.2, y0: -0.15, y1: Math.max(top, 4.6) }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -3.2, 3.2);
      // poles
      boxW(ctx, view, -span - 0.18, 0, -span - 0.02, A.y + 0.2, C.bench);
      boxW(ctx, view, span + 0.02, 0, span + 0.18, B.y + 0.2, C.bench);
      // cables
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2.2;
      const [ax, ay] = P(A.x, A.y);
      const [bx, by] = P(B.x, B.y);
      const [kx, ky] = P(knot.x, knot.y);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(kx, ky);
      ctx.lineTo(bx, by);
      ctx.moveTo(kx, ky);
      const [, hy] = P(0, knot.y - 0.25);
      ctx.lineTo(kx, hy);
      ctx.stroke();
      ctx.restore();
      // the light
      boxW(ctx, view, -0.2, knot.y - 1.3, 0.2, knot.y - 0.25, C.panel, C.ink, 0.05);
      [['#e5484d', 0], ['#f5b400', 1], ['#2fbf71', 2]].forEach(([col, i]) => {
        const [cx, cy] = P(0, knot.y - 0.42 - i * 0.3);
        ctx.save();
        ctx.fillStyle = col;
        ctx.globalAlpha = 0.9;
        ctx.beginPath();
        ctx.arc(cx, cy, 0.11 * view.k, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
      });
      labelText(ctx, trimNum(an.m) + ' kg', P(0.3, 0)[0], P(0, knot.y - 0.8)[1], C.ink, 'left', 15);
      // angle arcs at the knot
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(kx - 0.9 * view.k, ky);
      ctx.lineTo(kx + 0.9 * view.k, ky);
      ctx.stroke();
      ctx.setLineDash([]);
      const rr = 0.6 * view.k;
      ctx.beginPath();
      ctx.arc(kx, ky, rr, Math.PI, Math.PI + a1);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(kx, ky, rr * 0.85, -a2, 0);
      ctx.stroke();
      ctx.restore();
      labelText(ctx, 'θ₁', kx - (rr + 14) * Math.cos(a1 * 0.5), ky - (rr + 12) * Math.sin(a1 * 0.5), C.ink, 'center', 15);
      labelText(ctx, 'θ₂', kx + (rr * 0.85 + 14) * Math.cos(a2 * 0.5), ky - (rr * 0.85 + 12) * Math.sin(a2 * 0.5), C.ink, 'center', 15);
      labelText(ctx, 'θ₁ = ' + trimNum(an.th1) + '°    θ₂ = ' + trimNum(an.th2) + '°', 12, 40, C.muted, 'left', 13, 600, C.fontBody);
      // force vectors at the knot
      const sc = (1.3 * view.k) / Math.max(an.T1, an.T2, an.W);
      forceArrow(ctx, view, knot, -Math.cos(a1), Math.sin(a1), an.T1 * sc, C.l1, 'T₁');
      forceArrow(ctx, view, knot, Math.cos(a2), Math.sin(a2), an.T2 * sc, C.l2, 'T₂');
      labelText(ctx, 'At rest: an equilibrium problem', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawWall(ctx, W, H, t) {
      const an = analysis;
      const y = STATICS.wallY(an, t);
      const view = fitView({ x0: -1.4, x1: 2.6, y0: -0.15, y1: 3.4 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -1.4, 2.6);
      // the wall
      boxW(ctx, view, -0.5, 0, 0, 3.3, C.bench);
      // ruler on the wall
      ctx.save();
      ctx.strokeStyle = C.muted;
      for (let i = 0; i <= 32; i++) {
        const [mx, my] = P(0, i * 0.1);
        ctx.beginPath();
        ctx.moveTo(mx, my);
        ctx.lineTo(mx - (i % 10 === 0 ? 12 : i % 5 === 0 ? 8 : 4), my);
        ctx.stroke();
        if (i % 5 === 0 && view.k > 40) labelText(ctx, (i / 10).toFixed(1), mx - 16, my, C.muted, 'right', 12, 400);
      }
      ctx.restore();
      for (const ts of strobeTimes(t)) {
        const [gx, gy] = P(0.25, STATICS.wallY(an, ts));
        ctx.save();
        ctx.globalAlpha = 0.4;
        ctx.fillStyle = C.ink;
        ctx.beginPath();
        ctx.arc(gx, gy, 3, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
      }
      // block
      const bs = 0.5;
      boxW(ctx, view, 0, y - bs / 2, bs, y + bs / 2, bodyColor(0), C.ink);
      const [bx, by] = P(bs / 2, y);
      labelText(ctx, trimNum(an.m) + ' kg', bx, by, C.ink, 'center', 15);
      // the push: a rod pressing on the block's right face
      const p = an.phi * DEG;
      const dir = { x: -Math.cos(p), y: Math.sin(p) }; // direction of F
      const contact = { x: bs, y: y - 0.06 };
      const [cx, cy] = P(contact.x, contact.y);
      const len = 0.95 * view.k;
      const tail = [cx - dir.x * len, cy + dir.y * len];
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 0.08 * view.k;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(tail[0], tail[1]);
      ctx.lineTo(cx - dir.x * 10, cy + dir.y * 10);
      ctx.stroke();
      ctx.restore();
      arrow(ctx, tail[0] + dir.x * 0.2 * view.k, tail[1] - dir.y * 0.2 * view.k, cx - dir.x * 6, cy + dir.y * 6, C['f-a'], 4, 13);
      labelText(ctx, 'F = ' + trimNum(an.F) + ' N', tail[0] + 4, tail[1] - 16, C['f-a'], 'left', 15);
      if (an.phi > 0) labelText(ctx, 'φ = ' + trimNum(an.phi) + '°', tail[0] + 4, tail[1] + 18, C.ink, 'left', 14);
      const phase = an.state === 'rest' ? 'Held at rest' : an.state === 'down' ? 'Sliding down' : 'Sliding up';
      labelText(ctx, phase, 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    forceSets(an) {
      if (an.kind === 'traffic') {
        const a1 = an.th1 * DEG;
        const a2 = an.th2 * DEG;
        return [{
          body: { name: 'The knot (holding the light)', m: null, kind: 'hanging' },
          idx: 2,
          tilt: 0,
          arrows: [
            { dir: { x: -Math.cos(a1), y: Math.sin(a1) }, mag: an.T1, color: C.l1, label: 'T₁' },
            { dir: { x: Math.cos(a2), y: Math.sin(a2) }, mag: an.T2, color: C.l2, label: 'T₂' },
            { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
          ],
        }];
      }
      const p = an.phi * DEG;
      const arrows = [
        { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
        { dir: { x: 1, y: 0 }, mag: an.N, color: C['f-n'], label: 'N' },
        { dir: { x: -Math.cos(p), y: Math.sin(p) }, mag: an.F, color: C['f-a'], label: 'F' },
      ];
      if (Math.abs(an.f) > 1e-9) arrows.push({ dir: { x: 0, y: Math.sign(an.f) }, mag: Math.abs(an.f), color: C['f-f'], label: an.fType === 'static' ? 'fₛ' : 'fₖ' });
      return [{ body: an.bodies[0], idx: 0, tilt: 0, arrows }];
    },
    fbdCaption: (an) => (an.kind === 'traffic' ? 'The forces on the knot must add to zero. Think of the light and knot together as one object.' : 'The wall pushes out on the block (N) and friction from the wall acts along it, up or down.'),
    legend(an, showV) {
      const v = (x) => (showV ? ' (' + fmt(x) + ' N)' : '');
      if (an.kind === 'traffic') {
        return {
          legend: chipH('var(--l1)', 'T₁: left cable') + chipH('var(--l2)', 'T₂: right cable') + chipH('var(--f-w)', 'W: weight of the light'),
          pairs: pairsH([
            swatch('--l1') + 'Each cable pulls on the knot, and the knot pulls back on that cable just as hard' + v(an.T1) + '. The cable passes that pull on to its pole.',
            'The horizontal parts of T₁ and T₂ must be equal, because nothing else pulls sideways. The steeper cable carries more of the weight.',
            '<b>Weight:</b> Earth pulls the light down, and the light pulls Earth up just as hard.',
          ]),
        };
      }
      return {
        legend: chipH('var(--f-w)', 'W: weight') + chipH('var(--f-n)', 'N: the wall pushes out') + chipH('var(--f-a)', 'F: your push') + chipH('var(--f-f)', 'f: friction from the wall'),
        pairs: pairsH([
          '<b>N</b>' + v(an.N) + ': the wall pushes out on the block, and the block pushes into the wall just as hard.',
          '<b>Friction</b>' + v(Math.abs(an.f)) + ': the wall rubs on the block, and the block rubs on the wall in the opposite direction.',
          '<b>F</b>: you push on the block, and the block pushes back on your hand just as hard.',
          'Pressing harder increases N, and a bigger N means more friction is available. That is why a hard enough push can hold the block up.',
        ]),
      };
    },
    mcq(an) {
      if (an.kind !== 'wall') return [];
      return [{
        key: 'wallState',
        q: 'With this push, what does the block do?',
        options: ['It stays where it is', 'It slides down the wall', 'It slides up the wall'],
        correct: an.state === 'rest' ? 0 : an.state === 'down' ? 1 : 2,
        why: 'Compare the friction needed to hold the block (W − F sin φ = ' + fmt(an.W - an.Fpar) + ' N) with the most static friction available (μs N = ' + fmt(an.mus * an.N) + ' N).',
      }];
    },
    quantities(an) {
      const q = [];
      const row = (key, label, unit, value, sub, group = 'main') => q.push({ key, group, label, unit, value, sub });
      if (an.kind === 'traffic') {
        row('T1', 'Tension in the left cable, T₁', 'N', an.T1);
        row('T2', 'Tension in the right cable, T₂', 'N', an.T2);
        row('Tx', 'Horizontal part of each tension', 'N', an.T1x, 'the same for both cables');
        row('T1y', 'Vertical part of T₁', 'N', an.T1y, '', 'more');
        row('T2y', 'Vertical part of T₂', 'N', an.T2y, '', 'more');
        row('W', 'Weight of the light', 'N', an.W, '', 'more');
        return q;
      }
      row('N', 'Normal force from the wall', 'N', an.N);
      row('f', 'Friction force on the block', 'N', Math.abs(an.f), an.state === 'rest' ? 'static friction' : 'kinetic friction');
      if (an.state !== 'rest') row('a', 'Acceleration of the block', 'm/s²', Math.abs(an.a));
      if (isFinite(an.Fmin)) row('Fmin', 'Smallest push that holds the block up', 'N', an.Fmin, 'at this angle');
      if (an.phi > 0 && isFinite(an.Fmax)) row('Fmax', 'Largest push before it slides up', 'N', an.Fmax, 'at this angle');
      row('W', 'Weight of the block', 'N', an.W, '', 'more');
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const g = fmt(an.g);
      if (an.kind === 'traffic') {
        return '<section><h3>1. Forces on the knot</h3><p>The knot is at rest, so the forces on it add to zero (Newton’s first law). Three forces act: T₁ up and to the left at θ₁ = ' + trimNum(an.th1) + '°, T₂ up and to the right at θ₂ = ' + trimNum(an.th2) + '°, and the light’s weight W = mg = ' + fmt(an.m) + ' × ' + g + ' = ' + fmt(an.W) + ' N.</p></section>' +
          '<section><h3>2. Write ΣF = 0 for each direction</h3><div class="eq">x:  T₂ cos θ₂ − T₁ cos θ₁ = 0\ny:  T₁ sin θ₁ + T₂ sin θ₂ − W = 0</div></section>' +
          '<section><h3>3. Solve</h3><p>From the x equation, T₂ = T₁ cos θ₁ / cos θ₂. Put that into the y equation and simplify:</p>' +
          '<div class="eq">T₁ = W cos θ₂ / sin(θ₁ + θ₂) = ' + fmt(an.W) + ' × ' + fmt(Math.cos(an.th2 * DEG), 3) + ' / ' + fmt(Math.sin((an.th1 + an.th2) * DEG), 3) + ' = ' + fmt(an.T1) + ' N\n' +
          'T₂ = W cos θ₁ / sin(θ₁ + θ₂) = ' + fmt(an.W) + ' × ' + fmt(Math.cos(an.th1 * DEG), 3) + ' / ' + fmt(Math.sin((an.th1 + an.th2) * DEG), 3) + ' = ' + fmt(an.T2) + ' N</div></section>' +
          '<section><h3>4. Check</h3><div class="eq">Horizontal parts:  T₁ cos θ₁ = ' + fmt(an.T1x) + ' N,  T₂ cos θ₂ = ' + fmt(an.T2x) + ' N  (equal)\nVertical parts:  ' + fmt(an.T1y) + ' + ' + fmt(an.T2y) + ' = ' + fmt(an.T1y + an.T2y) + ' N = W</div>' +
          '<p>Shallow cables need large tensions: as both angles shrink toward zero, the tension grows without limit.</p></section>';
      }
      const p = an.phi;
      let h = '<section><h3>1. Forces on the block</h3><p>Take + as up and away from the wall. Four forces act: the weight W = mg = ' + fmt(an.W) + ' N, your push F = ' + trimNum(an.F) + ' N at φ = ' + trimNum(p) + '° above horizontal, the wall’s normal force N, and friction f along the wall.</p></section>';
      h += '<section><h3>2. Horizontal: find N</h3><p>The block does not move into or away from the wall.</p><div class="eq">N − F cos φ = 0   →   N = ' + trimNum(an.F) + ' × cos ' + trimNum(p) + '° = ' + fmt(an.N) + ' N</div></section>';
      h += '<section><h3>3. Vertical: is static friction enough?</h3><div class="eq">To hold it still:  f + F sin φ − W = 0   →   f needed = W − F sin φ = ' + fmt(an.W) + ' − ' + fmt(an.Fpar) + ' = ' + fmt(an.W - an.Fpar) + ' N\nMost static friction available:  μs N = ' + trimNum(an.mus) + ' × ' + fmt(an.N) + ' = ' + fmt(an.mus * an.N) + ' N</div>';
      if (an.state === 'rest') h += '<p>|f needed| ≤ μs N, so static friction holds: the block <b>stays put</b>, and friction is ' + fmt(Math.abs(an.f)) + ' N ' + (an.f >= 0 ? 'up' : 'down') + '.</p></section>';
      else
        h += '<p>|f needed| > μs N, so the block slides ' + (an.state === 'down' ? 'down' : 'up') + '. Kinetic friction μk N = ' + trimNum(an.muk) + ' × ' + fmt(an.N) + ' = ' + fmt(Math.abs(an.f)) + ' N acts ' + (an.state === 'down' ? 'up' : 'down') + '.</p>' +
          '<div class="eq">a = (F sin φ − W ' + (an.f >= 0 ? '+ ' : '− ') + 'fₖ) / m = (' + fmt(an.Fpar) + ' − ' + fmt(an.W) + signed(an.f) + ') / ' + fmt(an.m) + ' = ' + fmt(an.a) + ' m/s²</div></section>';
      h += '<section><h3>4. The smallest push that holds the block</h3><p>The block is about to slide down when static friction is at its maximum, pointing up:</p>' +
        '<div class="eq">F sin φ + μs F cos φ = W   →   F = W / (sin φ + μs cos φ) = ' + fmt(an.W) + ' / (' + fmt(Math.sin(p * DEG), 3) + ' + ' + trimNum(an.mus) + ' × ' + fmt(Math.cos(p * DEG), 3) + ') = ' + (isFinite(an.Fmin) ? fmt(an.Fmin) + ' N' : 'no push can hold it') + '</div>';
      if (p > 0)
        h += '<p>Push too hard at an upward angle and the block slides up instead. That starts when friction is at its maximum, pointing down:</p><div class="eq">F = W / (sin φ − μs cos φ) = ' + (isFinite(an.Fmax) ? fmt(an.Fmax) + ' N' : 'never (sin φ ≤ μs cos φ, so friction can always stop it)') + '</div>';
      return h + '</section>';
    },
  };

  /* ---------------- F = ma lab ---------------- */
  const LAB_D = 1; // distance between the photogates (m)
  function gauss() {
    let u = 0;
    let v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
  const LAB = {
    defaults: () => ({ type: 'fma', M: 1.2, f: 0.15, mg: 100, hidden: false, tilt: TILT.defaults(), mystery: { on: false, target: '' } }),
    hidesAnswers: (an) => an.hidden,
    // the true total mass and friction (hidden ones live only in this browser)
    params(c) {
      if (!c.hidden) return { M: c.M, f: c.f };
      if (!ui.labSecret) ui.labSecret = { M: Math.round((0.6 + Math.random() * 1.4) * 20) / 20, f: Math.round((0.05 + Math.random() * 0.35) * 100) / 100 };
      return ui.labSecret;
    },
    solve(c) {
      const p = LAB.params(c);
      const m = Math.min(c.mg / 1000, p.M - 0.05);
      const tr = P.labTrial({ M: p.M, m, f: p.f, g: state.g, d: LAB_D });
      const an = { kind: 'lab', ok: true, g: state.g, M: p.M, fr: p.f, m, ...tr, hidden: c.hidden };
      an.bodies = [{ name: 'Cart', m: p.M - m, kind: 'block', a: tr.a }, { name: 'Hanging mass', m, kind: 'hanging', a: tr.a }];
      an.atRest = tr.a <= 0;
      return an;
    },
    controls(c) {
      let h = '<div class="group"><h2>Lab</h2>' + selectField('Choose a lab', 'sel-lab-type', 'type', LAB_TYPES, c.type || 'fma', true) + '</div>';
      h += '<div class="group"><h2>The cart and track</h2>' + checkField('Hide the total mass and friction (find them from the graph)', 'hidden', c.hidden);
      if (c.hidden) {
        h += hiddenMass('Total mass M') + '<label class="field"><span>Friction f <span class="unit">(N)</span></span><input type="text" value="?" disabled aria-label="Friction is hidden"></label>' +
          '<div class="btn-row"><button type="button" class="btn btn-small" data-action="newlab">New hidden cart</button></div>';
      } else {
        h += numField('Total mass M (cart + hanger)', 'M', c.M, 'kg', { min: 0.3, max: 5, step: 0.05 }) +
          numField('Friction on the cart f', 'f', c.f, 'N', { min: 0, max: 3, step: 0.01 });
      }
      h += '<p class="note">To keep the total mass the same, mass is moved from the cart to the hanger. Friction is a constant drag on the cart.</p></div>';
      h += '<div class="group"><h2>This trial</h2>' + numField('Hanging mass m', 'mg', c.mg, 'g', { min: 5, max: 600, step: 5 }) +
        '<p class="note">Press Play. The timer runs from the first photogate to the second, ' + fmt(LAB_D) + ' m apart, and each run is added to the data table on the Data & graph tab.</p></div>';
      const n = (ui.labTrials || []).length;
      h += '<div class="group"><h2>Data</h2><p class="note">' + n + ' trial' + (n === 1 ? '' : 's') + ' recorded.</p><div class="btn-row"><button type="button" class="btn btn-small" data-action="labclear"' + (n ? '' : ' disabled') + '>Clear the data table</button></div></div>';
      return h;
    },
    randomize(c, r) {
      c.M = r(0.6, 2, 0.05);
      c.f = r(0.05, 0.4, 0.01);
      c.mg = r(40, 200, 10);
      ui.labSecret = null;
      ui.labTrials = [];
    },
    load(c, num) {
      const d = LAB.defaults();
      const tl = c.tilt || {};
      return {
        type: c.type === 'tilt' ? 'tilt' : 'fma',
        M: num(c.M, d.M, 0.3, 5),
        f: num(c.f, d.f, 0, 3),
        mg: num(c.mg, d.mg, 5, 600),
        hidden: !!c.hidden,
        tilt: { mus: num(tl.mus, 0.55, 0.05, 1.5), muk: num(tl.muk, 0.4, 0, 1.5), hidden: !!tl.hidden },
        mystery: d.mystery,
      };
    },
    // the cart starts with its front at gate 1 and runs until it reaches gate 2
    buildSim(an) {
      if (an.a <= 0) return { kind: 'lab', tStop: 0, static: true, staticMessage: 'The hanging weight (' + fmt(an.F) + ' N) is not enough to overcome friction, so the cart does not move. Add more hanging mass.' };
      return { kind: 'lab', tStop: an.t, reason: 'The cart reached the second photogate' };
    },
    motion(an, t) {
      return { s: 0.5 * an.a * t * t, v: an.a * t, a: an.a };
    },
    // record a finished run (with a little timing noise when the values are hidden)
    onRunEnd(an) {
      const noise = an.hidden ? 1 + 0.012 * gauss() : 1;
      const t = an.t * noise;
      ui.labTrials = ui.labTrials || [];
      ui.labTrials.push({ m: an.m, F: an.F, t, a: (2 * LAB_D) / (t * t) });
      renderControls();
      LAB.renderData();
      renderCheck();
    },
    renderData() {
      const el = $('lab-data');
      const rows = ui.labTrials || [];
      if (!rows.length) {
        el.innerHTML = '<p class="hint">No trials yet. Set a hanging mass and press Play. Try at least four different masses.</p>';
        return;
      }
      el.innerHTML = '<div class="table-wrap"><table class="data"><thead><tr><th>Trial</th><th>m (g)</th><th>F = mg (N)</th><th>t (s)</th><th>a = 2d/t² (m/s²)</th><th></th></tr></thead><tbody>' +
        rows.map((r, i) => '<tr><td>' + (i + 1) + '</td><td>' + fmt(r.m * 1000, 0) + '</td><td>' + fmt(r.F, 3) + '</td><td>' + fmt(r.t, 3) + '</td><td>' + fmt(r.a, 3) +
          '</td><td><button type="button" class="icon-btn" data-labdel="' + i + '" aria-label="Delete trial ' + (i + 1) + '">×</button></td></tr>').join('') +
        '</tbody></table></div>';
    },
    fit() {
      const rows = ui.labTrials || [];
      return P.fitLine(rows.map((r) => [r.a, r.F]));
    },
    graphNames: { x: 'Position–time', v: 'Velocity–time', a: 'Acceleration–time' },
    graphValue(an, which, t) {
      const m = LAB.motion(an, t);
      return which === 'x' ? m.s : which === 'v' ? m.v : m.a;
    },
    graphLabel: { x: 'distance (m)', v: 'velocity (m/s)', a: 'acceleration (m/s²)' },
    // the Data & graph tab: F vs a with a best-fit line
    drawGraph(ctx, W, H) {
      const rows = ui.labTrials || [];
      const L = 72;
      const R = 20;
      const T = 20;
      const B = 46;
      const amax = Math.max(1, ...rows.map((r) => r.a)) * 1.15;
      const Fmax = Math.max(0.5, ...rows.map((r) => r.F)) * 1.15;
      const X = (a) => L + ((W - L - R) * a) / amax;
      const Y = (F) => T + ((H - T - B) * (Fmax - F)) / Fmax;
      ctx.font = '13px ' + C.fontMono;
      ctx.strokeStyle = C.grid;
      ctx.fillStyle = C.muted;
      const xs = niceStep(amax, 8);
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (let a = 0; a <= amax + 1e-9; a += xs) {
        ctx.beginPath();
        ctx.moveTo(Math.round(X(a)) + 0.5, T);
        ctx.lineTo(Math.round(X(a)) + 0.5, H - B);
        ctx.stroke();
        ctx.fillText(+a.toFixed(3) + '', X(a), H - B + 6);
      }
      const ys = niceStep(Fmax, 6);
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let F = 0; F <= Fmax + 1e-9; F += ys) {
        ctx.beginPath();
        ctx.moveTo(L, Math.round(Y(F)) + 0.5);
        ctx.lineTo(W - R, Math.round(Y(F)) + 0.5);
        ctx.stroke();
        ctx.fillText(fmt(F, ys < 0.1 ? 2 : 1), L - 8, Y(F));
      }
      ctx.strokeStyle = C.muted;
      ctx.beginPath();
      ctx.moveTo(L, T);
      ctx.lineTo(L, H - B);
      ctx.lineTo(W - R, H - B);
      ctx.stroke();
      ctx.fillStyle = C.ink;
      ctx.font = '600 14px ' + C.fontBody;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'bottom';
      ctx.fillText('measured acceleration a (m/s²)', L + (W - L - R) / 2, H - 4);
      ctx.save();
      ctx.translate(16, T + (H - T - B) / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textBaseline = 'middle';
      ctx.fillText('hanging weight F = mg (N)', 0, 0);
      ctx.restore();
      const fit = LAB.fit();
      if (fit) {
        ctx.save();
        ctx.strokeStyle = C['f-a'];
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(X(0), Y(fit.intercept));
        ctx.lineTo(X(amax), Y(fit.intercept + fit.slope * amax));
        ctx.stroke();
        ctx.restore();
        labelText(ctx, 'best fit:  F = ' + fmt(fit.slope, 3) + ' a ' + (fit.intercept < 0 ? '− ' : '+ ') + fmt(Math.abs(fit.intercept), 3), L + 12, T + 12, C['f-a'], 'left', 15);
      }
      rows.forEach((r) => {
        ctx.fillStyle = C.accent;
        ctx.beginPath();
        ctx.arc(X(r.a), Y(r.F), 5.5, 0, 2 * Math.PI);
        ctx.fill();
      });
      if (!rows.length) labelText(ctx, 'Run trials to add points', L + (W - L - R) / 2, T + (H - T - B) / 2, C.muted, 'center', 15, 400, C.fontBody);
    },
    draw(ctx, W, H, t) {
      const an = analysis;
      const s = an.a > 0 ? 0.5 * an.a * Math.min(t, an.t) ** 2 : 0;
      const tableY = 1.55;
      const view = fitView({ x0: -0.5, x1: 2.85, y0: -0.12, y1: 2.2 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -0.5, 2.75);
      // table and track
      boxW(ctx, view, -0.35, tableY - 0.06, 2.3, tableY, C.bench);
      boxW(ctx, view, -0.25, 0, -0.17, tableY - 0.06, C.bench);
      boxW(ctx, view, 2.12, 0, 2.2, tableY - 0.06, C.bench);
      // photogates at 0.35 m and 1.35 m (the cart's front starts at gate 1)
      const g1 = 0.35;
      const g2 = g1 + LAB_D;
      for (const [gx, txt] of [[g1, 'Gate 1'], [g2, 'Gate 2']]) {
        const [x0, y0] = P(gx - 0.03, tableY + 0.32);
        const [x1, y1] = P(gx + 0.03, tableY);
        ctx.save();
        ctx.fillStyle = C['bench-edge'];
        ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
        ctx.fillStyle = C['f-w'];
        ctx.beginPath();
        ctx.arc((x0 + x1) / 2, y0 + 6, 3.5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
        labelText(ctx, txt, (x0 + x1) / 2, y0 - 10, C.muted, 'center', 12, 600, C.fontBody);
      }
      dimH(ctx, view, g1, g2, tableY + 0.42, fmt(LAB_D) + ' m', C.accent);
      // pulley at the table edge
      const rp = 0.06;
      const hookY = tableY + 0.09; // where the string ties to the cart
      const pc = { x: 2.4, y: hookY - rp }; // top of the pulley is level with the hook
      // clamp holding the pulley at the table edge
      boxW(ctx, view, 2.3, tableY - 0.12, 2.34, tableY + 0.02, C['bench-edge']);
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 3;
      const [ax1, ay1] = P(2.32, tableY - 0.02);
      const [ax2, ay2] = P(pc.x, pc.y);
      ctx.beginPath();
      ctx.moveTo(ax1, ay1);
      ctx.lineTo(ax2, ay2);
      ctx.stroke();
      ctx.restore();
      // cart
      const cw = 0.32;
      const front = g1 + s;
      const cx0 = front - cw;
      boxW(ctx, view, cx0, tableY + 0.04, front, tableY + 0.14, bodyColor(0), C.ink, 0.02);
      wheel(ctx, view, cx0 + 0.06, tableY + 0.035, 0.035);
      wheel(ctx, view, front - 0.06, tableY + 0.035, 0.035);
      // mass left on the cart, drawn as small blocks
      const onCart = Math.max(0, an.M - an.m - 0.3);
      const nCart = Math.min(6, Math.round(onCart / 0.15));
      for (let i = 0; i < nCart; i++) boxW(ctx, view, cx0 + 0.04 + i * 0.045, tableY + 0.14, cx0 + 0.075 + i * 0.045, tableY + 0.19, C.b5, C.ink);
      // string and hanger
      const drop = 0.25 + s;
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.6;
      const [sx, sy] = P(front, hookY);
      const [px, py] = P(pc.x, pc.y);
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(px, py - rp * view.k);
      ctx.arc(px, py, rp * view.k, -Math.PI / 2, 0);
      const [hx, hy] = P(pc.x + rp, pc.y - drop);
      ctx.lineTo(hx, hy);
      ctx.stroke();
      ctx.fillStyle = C.panel;
      ctx.beginPath();
      ctx.arc(px, py, rp * view.k, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      const nDisc = Math.max(1, Math.min(12, Math.round(an.m / 0.02)));
      for (let i = 0; i < nDisc; i++) {
        const yy = pc.y - drop - 0.02 - i * 0.025;
        boxW(ctx, view, pc.x + rp - 0.06, yy - 0.02, pc.x + rp + 0.06, yy, C.b2, C.ink);
      }
      labelText(ctx, fmt(an.m * 1000, 0) + ' g', P(pc.x + rp + 0.1, 0)[0], P(0, pc.y - drop - 0.1)[1], C.ink, 'left', 14);
      labelText(ctx, 'Cart + hanger = ' + (an.hidden ? '? kg' : fmt(an.M) + ' kg'), 12, 40, C.muted, 'left', 13, 600, C.fontBody);
      // timer between the gates
      const timer = an.a > 0 ? Math.min(t, an.t) : 0;
      labelText(ctx, 'Timer: ' + timer.toFixed(3) + ' s', 12, 18, C.ink, 'left', 16, 700);
    },
    forceSets(an) {
      const cart = an.M - an.m;
      return [
        {
          body: an.bodies[0],
          idx: 0,
          tilt: 0,
          arrows: [
            { dir: { x: 0, y: -1 }, mag: cart * an.g, color: C['f-w'], label: 'W' },
            { dir: { x: 0, y: 1 }, mag: cart * an.g, color: C['f-n'], label: 'N' },
            { dir: { x: 1, y: 0 }, mag: an.T, color: C.l1, label: 'T' },
            { dir: { x: -1, y: 0 }, mag: an.fr, color: C['f-f'], label: 'f' },
          ],
        },
        {
          body: an.bodies[1],
          idx: 1,
          tilt: 0,
          arrows: [
            { dir: { x: 0, y: 1 }, mag: an.T, color: C.l1, label: 'T' },
            { dir: { x: 0, y: -1 }, mag: an.F, color: C['f-w'], label: 'W = F' },
          ],
        },
      ];
    },
    fbdCaption: () => 'For the whole system (cart + hanger), the only unbalanced outside forces are the hanging weight F and friction f.',
    legend(an, showV) {
      return {
        legend: chipH('var(--f-w)', 'W: weight') + chipH('var(--f-n)', 'N: normal force') + chipH('var(--l1)', 'T: string tension') + chipH('var(--f-f)', 'f: friction'),
        pairs: pairsH([
          swatch('--l1') + 'The string pulls the cart forward and the hanger up with the same tension' + (showV ? ' (' + fmt(an.T) + ' N)' : '') + '. Each pull pairs with the object pulling back on the string.',
          'Tension is internal to the cart + hanger system, so it cancels when you treat them together: F − f = M a.',
        ]),
      };
    },
    quantities(an) {
      const q = [];
      const fit = LAB.fit();
      q.push({ key: 'M', group: 'main', groupTitle: 'From your graph', label: 'Total mass M (from the slope)', unit: 'kg', value: an.M, tol: 0.05, sub: fit ? 'your best-fit slope is ' + fmt(fit.slope, 3) : 'record at least two trials first' });
      q.push({ key: 'f', group: 'main', groupTitle: 'From your graph', label: 'Friction on the cart (from the intercept)', unit: 'N', value: an.fr, tol: 0.15, sub: fit ? 'your best-fit intercept is ' + fmt(fit.intercept, 3) : '' });
      if (!an.hidden) {
        q.push({ key: 'a', group: 'pred', groupTitle: 'Predict this trial', label: 'Acceleration with this hanging mass', unit: 'm/s²', value: an.a });
        q.push({ key: 't', group: 'pred', groupTitle: 'Predict this trial', label: 'Time between the photogates', unit: 's', value: an.t, sub: fmt(LAB_D) + ' m, starting from rest' });
        q.push({ key: 'T', group: 'more', label: 'Tension in the string', unit: 'N', value: an.T });
      }
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const fit = LAB.fit();
      let h = '<section><h3>1. Newton’s second law for the whole system</h3><p>Treat the cart and hanger as one object of mass M. Tension is internal, so it cancels. The outside forces along the motion are the hanging weight F = mg (forward) and friction f (backward).</p>' +
        '<div class="eq">F − f = M a   →   F = M·a + f</div><p>So a graph of F against a is a straight line. Its <b>slope is M</b> and its <b>intercept is f</b>. Moving mass from the cart to the hanger keeps M the same, which is what makes the graph a straight line.</p></section>';
      h += '<section><h3>2. Getting a from the photogates</h3><p>The cart starts from rest at the first gate and travels d = ' + fmt(LAB_D) + ' m in time t, so d = ½·a·t² and a = 2d / t².</p></section>';
      h += '<section><h3>3. Your results</h3>' + (fit
        ? '<div class="eq">Best fit:  F = ' + fmt(fit.slope, 3) + '·a ' + (fit.intercept < 0 ? '− ' : '+ ') + fmt(Math.abs(fit.intercept), 3) + '\nSlope  → M ≈ ' + fmt(fit.slope, 3) + ' kg   (true value ' + fmt(an.M, 3) + ' kg)\nIntercept → f ≈ ' + fmt(fit.intercept, 3) + ' N   (true value ' + fmt(an.fr, 3) + ' N)</div>'
        : '<p>Record at least two trials to fit a line.</p>') + '</section>';
      if (!an.hidden) h += '<section><h3>4. This trial</h3><div class="eq">a = (F − f) / M = (' + fmt(an.F, 3) + ' − ' + fmt(an.fr, 3) + ') / ' + fmt(an.M, 3) + ' = ' + fmt(an.a, 3) + ' m/s²\nt = √(2d / a) = ' + fmt(an.t, 3) + ' s\nT = (cart mass)·a + f = ' + fmt(an.M - an.m, 3) + ' × ' + fmt(an.a, 3) + ' + ' + fmt(an.fr, 3) + ' = ' + fmt(an.T, 3) + ' N</div></section>';
      return h;
    },
  };
  // horizontal dimension marker
  function dimH(ctx, view, x0, x1, y, text, color) {
    const [p0, py] = view.P(x0, y);
    const [p1] = view.P(x1, y);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(p0, py - 6);
    ctx.lineTo(p0, py + 6);
    ctx.moveTo(p1, py - 6);
    ctx.lineTo(p1, py + 6);
    ctx.moveTo(p0 + 7, py);
    ctx.lineTo(p1 - 7, py);
    ctx.stroke();
    for (const [px, d] of [[p0, 1], [p1, -1]]) {
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + d * 8, py - 4);
      ctx.lineTo(px + d * 8, py + 4);
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
    labelText(ctx, text, (p0 + p1) / 2, py - 12, color, 'center', 14);
  }

  /* ---------------- Third-law puzzles ---------------- */
  const PUZZLES = [
    ['horse', 'Horse and cart'],
    ['tug', 'Tug-of-war'],
    ['crash', 'Truck and car collision'],
    ['book', 'Book on a table'],
  ];
  // a simple side-view horse, facing right; phase animates the legs
  function drawHorse(ctx, view, x, phase) {
    const P = view.P;
    const k = view.k;
    const body = { x, y: 1.25 };
    ctx.save();
    ctx.lineCap = 'round';
    const limb = (a, b, w, color) => {
      const [x1, y1] = P(a.x, a.y);
      const [x2, y2] = P(b.x, b.y);
      ctx.strokeStyle = C.scene;
      ctx.lineWidth = w * k + 3;
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      ctx.strokeStyle = color;
      ctx.lineWidth = w * k;
      ctx.stroke();
    };
    const coat = C.horse;
    // legs: two far (shaded), two near
    const legsAt = [[-0.5, 0], [0.5, Math.PI], [-0.42, Math.PI], [0.58, 0]];
    legsAt.forEach(([dx, off], i) => {
      const sw = 0.18 * Math.sin(phase + off);
      const hip = { x: body.x + dx, y: body.y - 0.1 };
      const knee = { x: hip.x + sw * 0.6, y: 0.62 };
      const hoof = { x: hip.x + sw, y: 0.05 };
      limb(hip, knee, 0.11, coat);
      limb(knee, hoof, 0.08, coat);
      limb(hoof, { x: hoof.x + 0.05, y: 0.03 }, 0.08, C.ink);
      if (i < 2) {
        ctx.save();
        ctx.globalAlpha = 0.35;
        limb(hip, knee, 0.11, C.scene);
        limb(knee, hoof, 0.08, C.scene);
        ctx.restore();
      }
    });
    // body, neck, head, tail, mane
    const [bx, by] = P(body.x, body.y);
    ctx.fillStyle = coat;
    ctx.strokeStyle = C.scene;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.ellipse(bx, by, 0.78 * k, 0.3 * k, 0, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    limb({ x: body.x + 0.55, y: body.y + 0.1 }, { x: body.x + 0.9, y: body.y + 0.62 }, 0.24, coat);
    limb({ x: body.x + 0.9, y: body.y + 0.62 }, { x: body.x + 1.2, y: body.y + 0.42 }, 0.17, coat);
    limb({ x: body.x + 0.6, y: body.y + 0.25 }, { x: body.x + 0.86, y: body.y + 0.74 }, 0.06, C['fig-hair']);
    limb({ x: body.x - 0.76, y: body.y + 0.08 }, { x: body.x - 0.95, y: body.y - 0.45 }, 0.07, C['fig-hair']);
    const [ex, ey] = P(body.x + 0.98, body.y + 0.66);
    ctx.fillStyle = C.scene;
    ctx.beginPath();
    ctx.arc(ex, ey, 2.5, 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();
  }
  // a figure pulling on a rope; dir = +1 faces right, -1 faces left
  // dir = +1: faces right (toward the rope) and leans back to the left; -1 is the mirror image.
  // Returns where both hands grip, so the rope can be drawn through them.
  function pullerPose() {
    const pose = poseUpright({ x: 0, yHip: STAND_HIP - 0.18, contact: true, armA: 0.9, lean: -0.62 });
    // arms nearly straight, reaching forward and down to the rope
    pose.arms = [0.86, 0.96].map((a) => {
      const elbow = vadd(pose.shoulder, polar(FIG.upper, a));
      return { elbow, hand: vadd(elbow, polar(FIG.fore, a + 0.12)) };
    });
    // front foot braced forward, toward the rope
    const front = pose.legs[1];
    front.ankle.x += 0.22;
    front.toe.x += 0.22;
    front.knee = ik(pose.hip, front.ankle, FIG.thigh, FIG.shin, 1);
    return pose;
  }
  function puller(ctx, view, xFeet, dir) {
    const pose = pullerPose();
    const v = { k: view.k, P: (x, y) => view.P(xFeet + dir * x, y) };
    drawFigure(ctx, v, pose);
    return pose.arms.map((a) => ({ x: xFeet + dir * a.hand.x, y: a.hand.y }));
  }
  const PUZ = {
    defaults: () => ({
      p: 'horse',
      horse: { mh: 500, mc: 300, Fg: 1200, fc: 400 },
      tug: { mA: 300, mB: 280, FA: 2400, FB: 2100 },
      crash: { mt: 6000, mcar: 1200, F: 60 },
      book: { m: 1.5 },
      mystery: { on: false, target: '' },
    }),
    solve(c) {
      const g = state.g;
      const an = { kind: 'puzzle', ok: true, p: c.p, g };
      if (c.p === 'horse') Object.assign(an, c.horse, P.solveHorseCart(c.horse));
      else if (c.p === 'tug') Object.assign(an, c.tug, P.solveTug(c.tug));
      else if (c.p === 'crash') Object.assign(an, c.crash, P.solveCrash({ mt: c.crash.mt, mcar: c.crash.mcar, F: c.crash.F * 1000 }), { Fn: c.crash.F * 1000 });
      else Object.assign(an, c.book, { W: c.book.m * g });
      if (c.p === 'horse' && an.Fg <= an.fc) an.a = 0;
      an.bodies = [{ name: PUZZLES.find((x) => x[0] === c.p)[1], m: 1, kind: 'block', a: an.a || 0 }];
      an.atRest = !(an.a > 0 || (c.p === 'tug' && Math.abs(an.a) > 0));
      return an;
    },
    controls(c) {
      let h = '<div class="group"><h2>Puzzle</h2>' + selectField('Choose a puzzle', 'sel-puzzle', 'p', PUZZLES, c.p, true) + '</div>';
      if (c.p === 'horse') {
        h += '<div class="group"><h2>Horse and cart</h2>' +
          numField('Mass of the horse', 'horse.mh', c.horse.mh, 'kg', { min: 100, max: 1500, step: 10 }) +
          numField('Mass of the cart', 'horse.mc', c.horse.mc, 'kg', { min: 50, max: 2000, step: 10 }) +
          numField('Ground pushes the horse forward', 'horse.Fg', c.horse.Fg, 'N', { min: 0, max: 10000, step: 10 }) +
          numField('Friction on the cart', 'horse.fc', c.horse.fc, 'N', { min: 0, max: 10000, step: 10 }) +
          '<p class="note">The horse’s hooves push back on the ground; the ground pushes the horse forward.</p></div>';
      } else if (c.p === 'tug') {
        h += '<div class="group"><h2>Two teams</h2>' +
          numField('Team A (left) total mass', 'tug.mA', c.tug.mA, 'kg', { min: 50, max: 1000, step: 10 }) +
          numField('Team B (right) total mass', 'tug.mB', c.tug.mB, 'kg', { min: 50, max: 1000, step: 10 }) +
          numField('Ground pushes team A', 'tug.FA', c.tug.FA, 'N', { min: 0, max: 10000, step: 10 }) +
          numField('Ground pushes team B', 'tug.FB', c.tug.FB, 'N', { min: 0, max: 10000, step: 10 }) +
          '<p class="note">Each team digs in and pushes on the ground; the ground pushes back, away from the middle.</p></div>';
      } else if (c.p === 'crash') {
        h += '<div class="group"><h2>The collision</h2>' +
          numField('Mass of the truck', 'crash.mt', c.crash.mt, 'kg', { min: 500, max: 40000, step: 100 }) +
          numField('Mass of the car', 'crash.mcar', c.crash.mcar, 'kg', { min: 200, max: 5000, step: 50 }) +
          numField('Force of the crash (on the car)', 'crash.F', c.crash.F, 'kN', { min: 1, max: 1000, step: 1 }) + '</div>';
      } else {
        h += '<div class="group"><h2>The book</h2>' + numField('Mass of the book', 'book.m', c.book.m, 'kg', { min: 0.1, max: 20, step: 0.1 }) + '</div>';
      }
      return h;
    },
    randomize(c, r) {
      c.horse = { mh: r(300, 800, 10), mc: r(150, 600, 10), Fg: r(800, 2500, 50), fc: r(200, 700, 50) };
      c.tug = { mA: r(200, 450, 10), mB: r(200, 450, 10), FA: r(1500, 3000, 50), FB: r(1500, 3000, 50) };
      c.crash = { mt: r(3000, 15000, 500), mcar: r(800, 2000, 50), F: r(20, 200, 5) };
      c.book = { m: r(0.5, 4, 0.1) };
    },
    load(c, num) {
      const d = PUZ.defaults();
      const pick = (o, dd, spec) => {
        const out = {};
        for (const key of Object.keys(dd)) out[key] = num((o || {})[key], dd[key], spec[key][0], spec[key][1]);
        return out;
      };
      return {
        p: PUZZLES.some((x) => x[0] === c.p) ? c.p : d.p,
        horse: pick(c.horse, d.horse, { mh: [100, 1500], mc: [50, 2000], Fg: [0, 10000], fc: [0, 10000] }),
        tug: pick(c.tug, d.tug, { mA: [50, 1000], mB: [50, 1000], FA: [0, 10000], FB: [0, 10000] }),
        crash: pick(c.crash, d.crash, { mt: [500, 40000], mcar: [200, 5000], F: [1, 1000] }),
        book: pick(c.book, d.book, { m: [0.1, 20] }),
        mystery: d.mystery,
      };
    },
    buildSim(an) {
      if (an.p === 'crash' || an.p === 'book' || an.atRest) {
        const msg = an.p === 'crash' ? 'This shows one instant during the crash. Use the questions and the diagrams.' : an.p === 'book' ? 'The book sits at rest. Use the questions and the diagrams.' : an.p === 'tug' ? 'The teams are evenly matched: nothing moves.' : 'The ground’s push is not more than the friction on the cart, so nothing speeds up.';
        return { kind: 'puzzle', tStop: 0, static: true, staticMessage: msg };
      }
      return { kind: 'puzzle', tStop: 3, reason: 'Time is up' };
    },
    motion(an, t) {
      const a = an.a || 0;
      return { s: 0.5 * a * t * t, v: a * t, a };
    },
    noGraph: () => true,
    graphNames: { x: '', v: '', a: '' },
    graphValue: () => 0,
    graphLabel: { x: '', v: '', a: '' },
    draw(ctx, W, H, t) {
      const an = analysis;
      if (an.p === 'horse') return PUZ.drawHorseCart(ctx, W, H, t);
      if (an.p === 'tug') return PUZ.drawTug(ctx, W, H, t);
      if (an.p === 'crash') return PUZ.drawCrash(ctx, W, H);
      return PUZ.drawBook(ctx, W, H);
    },
    drawHorseCart(ctx, W, H, t) {
      const an = analysis;
      const s = 0.5 * an.a * t * t;
      const view = fitView({ x0: s - 4.7, x1: s + 3.3, y0: -0.4, y1: 2.6 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, s - 5, s + 5);
      for (let x = Math.floor(s - 5); x <= s + 5; x++) {
        const [mx, my] = P(x, 0);
        ctx.save();
        ctx.strokeStyle = C.muted;
        ctx.beginPath();
        ctx.moveTo(mx, my);
        ctx.lineTo(mx, my + 8);
        ctx.stroke();
        ctx.restore();
      }
      // cart
      const cartX = s - 2.6;
      boxW(ctx, view, cartX - 0.7, 0.55, cartX + 0.7, 1.15, C.b2, C.ink, 0.04);
      wheel(ctx, view, cartX - 0.4, 0.38, 0.35);
      wheel(ctx, view, cartX + 0.4, 0.38, 0.35);
      // shafts / rope from the cart to the horse
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 3;
      const [r1x, r1y] = P(cartX + 0.7, 1.0);
      const [r2x, r2y] = P(s - 0.75, 1.2);
      ctx.beginPath();
      ctx.moveTo(r1x, r1y);
      ctx.lineTo(r2x, r2y);
      ctx.stroke();
      ctx.restore();
      drawHorse(ctx, view, s, s * 5);
      // forces
      const sc = 70 / Math.max(an.Fg, an.T, an.fc, 1);
      forceArrow(ctx, view, { x: s + 0.5, y: 0.03 }, 1, 0, Math.max(24, an.Fg * sc), C['f-f'], 'ground pushes horse');
      forceArrow(ctx, view, { x: s - 0.78, y: 1.4 }, -1, 0, Math.max(24, an.T * sc), C.l1, 'T');
      forceArrow(ctx, view, { x: cartX + 0.7, y: 0.85 }, 1, 0, Math.max(24, an.T * sc), C.l1, 'T');
      forceArrow(ctx, view, { x: cartX - 0.7, y: 0.2 }, -1, 0, Math.max(24, an.fc * sc), C['f-f'], 'friction');
      labelText(ctx, an.a > 0 ? 'Speeding up' : 'Not speeding up', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawTug(ctx, W, H, t) {
      const an = analysis;
      const s = 0.5 * an.a * t * t; // + toward team A (left)
      const shift = -s;
      const view = fitView({ x0: -5.6, x1: 5.6, y0: -0.4, y1: 2.2 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -5.6, 5.6);
      // centre line and the rope's flag
      const [cx0, cy0] = P(0, 0);
      const [, cy1] = P(0, 0.4);
      ctx.save();
      ctx.strokeStyle = C.accent;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(cx0, cy0);
      ctx.lineTo(cx0, cy1);
      ctx.stroke();
      ctx.restore();
      // teams: two figures each
      // team A on the left faces right and pulls left; team B mirrors it
      const hands = [
        ...puller(ctx, view, -3.2 + shift, 1),
        ...puller(ctx, view, -2.1 + shift, 1),
        ...puller(ctx, view, 2.1 + shift, -1),
        ...puller(ctx, view, 3.2 + shift, -1),
      ];
      const ry = hands.reduce((sum, h) => sum + h.y, 0) / hands.length;
      const xs2 = hands.map((h) => h.x);
      ctx.save();
      ctx.strokeStyle = C['f-f'];
      ctx.lineWidth = 5;
      ctx.lineCap = 'round';
      const [l0x, ly] = P(Math.min(...xs2) - 0.35, ry);
      const [l1x] = P(Math.max(...xs2) + 0.35, ry);
      ctx.beginPath();
      ctx.moveTo(l0x, ly);
      ctx.lineTo(l1x, ly);
      ctx.stroke();
      // fists wrapped around the rope
      ctx.fillStyle = C['fig-skin'];
      ctx.strokeStyle = C.scene;
      ctx.lineWidth = 1.5;
      hands.forEach((h) => {
        const [hx] = P(h.x, 0);
        ctx.beginPath();
        ctx.arc(hx, ly, Math.max(4, 0.055 * view.k), 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
      });
      ctx.restore();
      const [fx, fy] = P(shift, ry);
      ctx.save();
      ctx.fillStyle = C['f-w'];
      ctx.beginPath();
      ctx.moveTo(fx, fy);
      ctx.lineTo(fx - 8, fy + 22);
      ctx.lineTo(fx + 8, fy + 22);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      labelText(ctx, 'Team A pulls ←', P(-2.65 + shift, 0)[0], P(0, 1.85)[1], C.ink, 'center', 15, 700, C.fontBody);
      labelText(ctx, '→ Team B pulls', P(2.65 + shift, 0)[0], P(0, 1.85)[1], C.ink, 'center', 15, 700, C.fontBody);
      const sc = 70 / Math.max(an.FA, an.FB, an.T, 1);
      forceArrow(ctx, view, { x: -2.95 + shift, y: 0.03 }, -1, 0, Math.max(24, an.FA * sc), C['f-n'], 'ground pushes A');
      forceArrow(ctx, view, { x: 2.95 + shift, y: 0.03 }, 1, 0, Math.max(24, an.FB * sc), C['f-n'], 'ground pushes B');
      labelText(ctx, an.winner === 'tie' ? 'Evenly matched' : 'Team ' + an.winner + ' is winning', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawCrash(ctx, W, H) {
      const an = analysis;
      const view = fitView({ x0: -4.4, x1: 3.4, y0: -0.4, y1: 3.4 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -4.4, 3.4);
      // truck from the left, car from the right, touching at x = 0
      boxW(ctx, view, -4.0, 0.45, -1.3, 2.7, C.b1, C.ink, 0.05);
      boxW(ctx, view, -1.3, 0.45, 0, 2.0, C.b1, C.ink, 0.05);
      boxW(ctx, view, -1.05, 1.35, -0.25, 1.85, C.scene, C.ink, 0.03);
      [-3.4, -2.2, -0.6].forEach((x) => wheel(ctx, view, x, 0.42, 0.4));
      boxW(ctx, view, 0, 0.35, 2.7, 1.05, C.b4, C.ink, 0.08);
      boxW(ctx, view, 0.6, 1.05, 2.1, 1.55, C.b4, C.ink, 0.08);
      [0.6, 2.1].forEach((x) => wheel(ctx, view, x, 0.33, 0.3));
      labelText(ctx, 'Truck ' + fmt(an.mt, 0) + ' kg', P(-2.6, 0)[0], P(0, 3.0)[1], C.ink, 'center', 15, 700, C.fontBody);
      labelText(ctx, 'Car ' + fmt(an.mcar, 0) + ' kg', P(1.35, 0)[0], P(0, 1.85)[1], C.ink, 'center', 15, 700, C.fontBody);
      // the equal and opposite crash forces at the contact point
      forceArrow(ctx, view, { x: 0.05, y: 0.75 }, 1, 0, 90, C.l2, 'truck on car');
      forceArrow(ctx, view, { x: -0.05, y: 1.25 }, -1, 0, 90, C.l2, 'car on truck');
      labelText(ctx, 'During the crash', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    drawBook(ctx, W, H) {
      const an = analysis;
      const view = fitView({ x0: -2.4, x1: 2.4, y0: -0.6, y1: 2.0 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      // Earth's surface as a gentle curve
      const [ex0, ey0] = P(-2.4, -0.25);
      const [ex1] = P(2.4, -0.25);
      const [, eyb] = P(0, -0.6);
      ctx.save();
      ctx.fillStyle = C.track;
      ctx.beginPath();
      ctx.moveTo(ex0, ey0);
      ctx.quadraticCurveTo((ex0 + ex1) / 2, ey0 - 18, ex1, ey0);
      ctx.lineTo(ex1, eyb);
      ctx.lineTo(ex0, eyb);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      labelText(ctx, 'Earth', P(-1.8, 0)[0], P(0, -0.45)[1], C.ink, 'left', 15, 700, C.fontBody);
      // table
      boxW(ctx, view, -1.1, 0.75, 1.1, 0.85, C.bench);
      boxW(ctx, view, -1.0, -0.2, -0.9, 0.75, C.bench);
      boxW(ctx, view, 0.9, -0.2, 1.0, 0.75, C.bench);
      // book
      boxW(ctx, view, -0.35, 0.85, 0.35, 1.02, C.b3, C.ink, 0.02);
      labelText(ctx, trimNum(an.m) + ' kg book', P(0.45, 0)[0], P(0, 0.94)[1], C.ink, 'left', 14);
      forceArrow(ctx, view, { x: -0.12, y: 0.94 }, 0, -1, 70, C['f-w'], 'W (Earth pulls book)');
      forceArrow(ctx, view, { x: 0.12, y: 1.02 }, 0, 1, 70, C['f-n'], 'N (table pushes book)');
      forceArrow(ctx, view, { x: 1.7, y: -0.25 }, 0, 1, 50, C['f-w'], 'book pulls Earth');
      labelText(ctx, 'At rest on the table', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
    },
    forceSets(an) {
      const set = (name, arrows, pose) => ({ body: { name, m: null, kind: pose ? 'person' : 'hanging' }, idx: 0, tilt: 0, arrows, pose });
      if (an.p === 'horse') {
        return [
          set('Horse', [
            { dir: { x: 1, y: 0 }, mag: an.Fg, color: C['f-f'], label: 'F_ground' },
            { dir: { x: -1, y: 0 }, mag: an.T, color: C.l1, label: 'T (cart)' },
          ]),
          set('Cart', [
            { dir: { x: 1, y: 0 }, mag: an.T, color: C.l1, label: 'T (horse)' },
            { dir: { x: -1, y: 0 }, mag: an.fc, color: C['f-f'], label: 'f' },
          ]),
        ];
      }
      if (an.p === 'tug') {
        return [
          set('Team A', [
            { dir: { x: -1, y: 0 }, mag: an.FA, color: C['f-n'], label: 'F_ground' },
            { dir: { x: 1, y: 0 }, mag: an.T, color: C['f-f'], label: 'T' },
          ]),
          set('Team B', [
            { dir: { x: -1, y: 0 }, mag: an.T, color: C['f-f'], label: 'T' },
            { dir: { x: 1, y: 0 }, mag: an.FB, color: C['f-n'], label: 'F_ground' },
          ]),
        ];
      }
      if (an.p === 'crash') {
        return [
          set('Truck', [{ dir: { x: -1, y: 0 }, mag: an.Fn, color: C.l2, label: 'F (car on truck)' }]),
          set('Car', [{ dir: { x: 1, y: 0 }, mag: an.Fn, color: C.l2, label: 'F (truck on car)' }]),
        ];
      }
      return [
        set('Book', [
          { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
          { dir: { x: 0, y: 1 }, mag: an.W, color: C['f-n'], label: 'N' },
        ]),
        set('Earth', [{ dir: { x: 0, y: 1 }, mag: an.W, color: C['f-w'], label: 'W′ (book pulls Earth)' }]),
      ];
    },
    fbdCaption: (an) =>
      ({
        horse: 'Only horizontal forces are shown. The two T arrows are a third-law pair: same size, but one acts on the horse and one on the cart.',
        tug: 'Only horizontal forces are shown. The rope pulls each team toward the middle with the same tension T.',
        crash: 'The two crash forces are a third-law pair: equal in size, opposite in direction, on different vehicles.',
        book: 'W and N both act on the book, so they are not a pair. W pairs with the book’s pull on Earth (W′).',
      })[an.p],
    legend(an, showV) {
      const v = (x) => (showV ? ' (' + fmt(x, x >= 1000 ? 0 : 2) + ' N)' : '');
      const L = {
        horse: [chipH('var(--f-f)', 'ground and friction forces'), chipH('var(--l1)', 'T: pull through the harness')],
        tug: [chipH('var(--f-n)', 'F_ground: the ground pushes a team'), chipH('var(--f-f)', 'T: rope tension')],
        crash: [chipH('var(--l2)', 'the crash forces: a third-law pair')],
        book: [chipH('var(--f-w)', 'W, W′: Earth and book pull on each other'), chipH('var(--f-n)', 'N: the table pushes up')],
      }[an.p].join('');
      const items = {
        horse: [
          swatch('--l1') + 'The horse pulls the cart forward and the cart pulls the horse back with equal force T' + v(an.T) + '. They act on <b>different objects</b>, so they never cancel each other.',
          'The horse speeds up because the ground pushes it forward' + v(an.Fg) + ' harder than the cart pulls it back. The cart speeds up because T is bigger than the friction on it' + v(an.fc) + '.',
          'The partner of the ground’s push is the horse’s hooves pushing back on the ground.',
        ],
        tug: [
          swatch('--f-f') + 'The rope pulls both teams with the <b>same</b> tension T' + v(an.T) + '. So pulling harder on the rope cannot win by itself.',
          'The team that pushes harder on the ground wins: the ground’s push is the only outside horizontal force on each team.',
          'Each team pushes on the ground and the ground pushes back on the team, away from the middle. Heavier teams with good shoes can push harder without slipping.',
        ],
        crash: [
          swatch('--l2') + 'The truck pushes on the car exactly as hard as the car pushes on the truck' + v(an.Fn) + ', even though the truck is heavier.',
          'The <b>accelerations</b> are different: a = F/m, so the lighter car gets a much bigger acceleration. That is why the car’s passengers are hurt more.',
        ],
        book: [
          swatch('--f-w') + 'Earth pulls the book down (W) and the book pulls Earth up (W′) just as hard' + v(an.W) + '. That is the third-law pair for weight.',
          swatch('--f-n') + 'The table pushes up on the book (N) and the book pushes down on the table just as hard. That is the pair for N.',
          'W and N are equal here only because the book is at rest (Newton’s first law), not because of the third law.',
        ],
      }[an.p];
      return { legend: L, pairs: pairsH(items) };
    },
    mcq(an) {
      const Q = {
        horse: [
          { key: 'h1', q: 'The cart pulls back on the horse just as hard as the horse pulls forward on the cart. So how can they speed up?', options: ['The horse pulls a little harder than the cart pulls back', 'Those two forces act on different objects. The horse speeds up because the ground pushes it forward harder than the cart pulls back', 'They can’t speed up; they can only keep moving at a constant speed', 'The cart’s pull back happens a moment later, so the horse gets ahead'], correct: 1, why: 'Third-law pairs act on different objects, so they never cancel. Look at each object’s own forces.' },
          { key: 'h2', q: 'What outside force pushes the horse-and-cart system forward?', options: ['The tension in the harness', 'The ground pushing forward on the horse’s hooves', 'The horse pushing on itself', 'The cart pushing the horse'], correct: 1, why: 'Tension is internal to the horse + cart system. The only forward outside force is the ground’s push on the hooves.' },
        ],
        tug: [
          { key: 't1', q: 'Which team wins a tug-of-war?', options: ['The team that pulls harder on the rope', 'The team that pushes harder on the ground', 'Neither: the rope pulls both teams equally, so nobody can win', 'Always the heavier team'], correct: 1, why: 'The rope pulls both teams with the same tension, so the difference must come from the ground.' },
          { key: 't2', q: 'Compare the rope’s pull on team A and on team B.', options: ['It is larger on the winning team', 'It is the same on both teams', 'It is larger on the losing team'], correct: 1, why: 'A light rope has the same tension all along it.' },
        ],
        crash: [
          { key: 'c1', q: 'During the crash, which vehicle pushes harder on the other?', options: ['The truck pushes harder on the car', 'The car pushes harder on the truck', 'They push on each other equally hard'], correct: 2, why: 'Newton’s third law: the forces in a pair are always equal in size.' },
          { key: 'c2', q: 'Which vehicle has the larger acceleration during the crash?', options: ['The truck', 'The car', 'They have the same acceleration'], correct: 1, why: 'a = F/m with the same F, so the smaller mass gets the bigger acceleration.' },
        ],
        book: [
          { key: 'b1', q: 'What is the third-law partner of the book’s weight (Earth pulling the book down)?', options: ['The normal force from the table pushing up on the book', 'The book pulling up on Earth', 'The table’s weight', 'The book pushing down on the table'], correct: 1, why: 'A pair involves the same two objects: Earth on book ↔ book on Earth.' },
          { key: 'b2', q: 'The table’s upward push on the book (N) pairs with:', options: ['The book’s weight', 'The book pushing down on the table', 'Earth pulling the table down'], correct: 1, why: 'Table on book ↔ book on table.' },
        ],
      };
      return Q[an.p];
    },
    quantities(an) {
      const q = [];
      const row = (key, label, unit, value, sub) => q.push({ key, group: 'main', groupTitle: 'Numbers', label, unit, value, sub });
      if (an.p === 'horse') {
        row('a', 'Acceleration of the horse and cart', 'm/s²', an.a);
        row('T', 'Tension in the harness (T)', 'N', an.T);
        row('netc', 'Net force on the cart', 'N', an.mc * an.a);
      } else if (an.p === 'tug') {
        row('a', 'Acceleration of the teams and rope', 'm/s²', Math.abs(an.a));
        row('T', 'Tension in the rope', 'N', an.T);
      } else if (an.p === 'crash') {
        row('ac', 'Car’s acceleration during the crash', 'm/s²', an.ac);
        row('at', 'Truck’s acceleration during the crash', 'm/s²', an.at);
        row('ratio', 'How many times bigger is the car’s acceleration?', '×', an.ac / an.at);
      } else {
        row('W', 'Weight of the book', 'N', an.W);
        row('N', 'Normal force on the book', 'N', an.W);
        row('Wp', 'The book’s pull on Earth', 'N', an.W);
      }
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      if (an.p === 'horse') {
        return '<section><h3>1. The key idea</h3><p>The horse pulls the cart (T forward on the cart) and the cart pulls the horse (T backward on the horse). These are a third-law pair, so they are equal, but they act on <b>different objects</b>. To find out whether something speeds up, add only the forces acting <b>on that object</b>.</p></section>' +
          '<section><h3>2. The whole system</h3><p>Treat horse + cart as one object of mass ' + fmt(an.mh + an.mc, 0) + ' kg. T is internal and cancels. The outside horizontal forces are the ground’s push F_ground = ' + fmt(an.Fg, 0) + ' N forward and friction on the cart f = ' + fmt(an.fc, 0) + ' N backward.</p>' +
          '<div class="eq">a = (F_ground − f) / (m_horse + m_cart) = (' + fmt(an.Fg, 0) + ' − ' + fmt(an.fc, 0) + ') / ' + fmt(an.mh + an.mc, 0) + ' = ' + fmt(an.a) + ' m/s²</div></section>' +
          '<section><h3>3. The cart alone</h3><div class="eq">T − f = m_cart·a   →   T = ' + fmt(an.mc, 0) + ' × ' + fmt(an.a) + ' + ' + fmt(an.fc, 0) + ' = ' + fmt(an.T) + ' N</div>' +
          '<p>Check the horse alone: F_ground − T = ' + fmt(an.Fg, 0) + ' − ' + fmt(an.T) + ' = ' + fmt(an.Fg - an.T) + ' N = m_horse·a = ' + fmt(an.mh * an.a) + ' N. The horse speeds up because the ground pushes it harder than the cart pulls back.</p></section>';
      }
      if (an.p === 'tug') {
        return '<section><h3>1. The key idea</h3><p>The rope pulls each team toward the middle with the same tension T. So the rope cannot decide the winner. The outside forces that differ are the ground’s pushes on the teams’ feet.</p></section>' +
          '<section><h3>2. The whole system</h3><p>Take + toward team A. Treat both teams and the rope as one object. Tension is internal.</p>' +
          '<div class="eq">a = (F_A − F_B) / (m_A + m_B) = (' + fmt(an.FA, 0) + ' − ' + fmt(an.FB, 0) + ') / ' + fmt(an.mA + an.mB, 0) + ' = ' + fmt(an.a) + ' m/s²</div></section>' +
          '<section><h3>3. The rope’s tension</h3><p>Team B alone: the rope pulls it toward A with T, and the ground pushes it away with F_B.</p>' +
          '<div class="eq">T − F_B = m_B·a   →   T = ' + fmt(an.FB, 0) + ' + ' + fmt(an.mB, 0) + ' × ' + fmt(an.a) + ' = ' + fmt(an.T) + ' N</div>' +
          '<p>' + (an.winner === 'tie' ? 'The ground pushes are equal, so nobody wins.' : 'Team ' + an.winner + ' wins because the ground pushes it harder.') + '</p></section>';
      }
      if (an.p === 'crash') {
        return '<section><h3>1. Forces</h3><p>By Newton’s third law the truck pushes on the car exactly as hard as the car pushes on the truck: ' + fmt(an.Fn, 0) + ' N each, in opposite directions.</p></section>' +
          '<section><h3>2. Accelerations</h3><div class="eq">Car:   a = F / m = ' + fmt(an.Fn, 0) + ' / ' + fmt(an.mcar, 0) + ' = ' + fmt(an.ac) + ' m/s²\nTruck: a = F / m = ' + fmt(an.Fn, 0) + ' / ' + fmt(an.mt, 0) + ' = ' + fmt(an.at) + ' m/s²\nRatio = ' + fmt(an.ac / an.at) + '  (the same as the mass ratio ' + fmt(an.mt, 0) + ' / ' + fmt(an.mcar, 0) + ')</div>' +
          '<p>Equal forces, unequal accelerations. The lighter car changes its velocity much more.</p></section>';
      }
      const aE = an.W / 5.97e24;
      return '<section><h3>1. Find the pairs</h3><p>A third-law pair always involves the same two objects, swapped: “A pushes B” pairs with “B pushes A”.</p>' +
        '<div class="eq">Earth pulls the book down (W = ' + fmt(an.W) + ' N)   ↔   the book pulls Earth up (W′ = ' + fmt(an.W) + ' N)\nThe table pushes the book up (N)   ↔   the book pushes the table down</div></section>' +
        '<section><h3>2. Why are W and N equal?</h3><p>Both act on the book, and the book is at rest, so they balance by <b>Newton’s first law</b>: N = W = ' + fmt(an.W) + ' N. If the table were in an accelerating elevator, N and W would differ, but each pair would still be equal.</p></section>' +
        '<section><h3>3. Does Earth move?</h3><p>The book pulls Earth up with ' + fmt(an.W) + ' N, giving Earth an acceleration of about ' + aE.toExponential(1) + ' m/s² (Earth’s mass is 5.97 × 10²⁴ kg). It’s real, just far too small to notice.</p></section>';
    },
  };

  /* ---------------- Accelerating car: an accelerometer ---------------- */
  const CAR_MOTIONS = [
    ['speed', 'Speeding up'],
    ['cruise', 'Constant speed'],
    ['brake', 'Braking'],
  ];
  const CAR = {
    defaults: () => ({ motion: 'speed', a: 3, m: 0.2, hidden: false, mystery: { on: false, target: '' } }),
    accel(c) {
      if (c.motion === 'cruise') return 0;
      if (c.hidden) {
        if (!ui.carSecret) ui.carSecret = Math.round((1 + Math.random() * 5) * 10) / 10;
        return c.motion === 'brake' ? -ui.carSecret : ui.carSecret;
      }
      return c.motion === 'brake' ? -c.a : c.a;
    },
    solve(c) {
      const a = CAR.accel(c);
      const r = P.solveCarAccel({ a, g: state.g, m: c.m });
      const v0 = c.motion === 'speed' ? 0 : c.motion === 'cruise' ? 15 : 20;
      const an = { kind: 'car', ok: true, ...r, motion: c.motion, hidden: c.hidden && c.motion !== 'cruise', v0 };
      an.bodies = [{ name: 'Ornament', m: c.m, kind: 'hanging', a }];
      an.atRest = false;
      return an;
    },
    hidesAnswers: (an) => an.hidden,
    controls(c) {
      let h = '<div class="group"><h2>The car</h2>' + selectField('What the car does', 'sel-car-motion', 'motion', CAR_MOTIONS, c.motion, true);
      if (c.motion !== 'cruise') {
        h += checkField('Hide the acceleration (measure the angle instead)', 'hidden', c.hidden);
        h += c.hidden
          ? '<label class="field"><span>Size of the acceleration <span class="unit">(m/s²)</span></span><input type="text" value="?" disabled aria-label="The acceleration is hidden"></label><div class="btn-row"><button type="button" class="btn btn-small" data-action="newcar">New hidden acceleration</button></div>'
          : numField('Size of the acceleration', 'a', c.a, 'm/s²', { min: 0.1, max: 12, step: 0.1 });
      }
      h += '</div><div class="group"><h2>The ornament</h2>' + numField('Mass of the ornament', 'm', c.m, 'kg', { min: 0.01, max: 5, step: 0.01 }) +
        '<p class="note">It hangs on a string from the rearview mirror. The angle shown is after it settles, when it moves along with the car.</p></div>';
      return h;
    },
    randomize(c, r) {
      c.a = r(1, 7, 0.1);
      c.m = r(0.05, 0.5, 0.01);
      ui.carSecret = null;
    },
    load(c, num) {
      const d = CAR.defaults();
      return { motion: CAR_MOTIONS.some((x) => x[0] === c.motion) ? c.motion : d.motion, a: num(c.a, d.a, 0.1, 12), m: num(c.m, d.m, 0.01, 5), hidden: !!c.hidden, mystery: d.mystery };
    },
    buildSim(an) {
      const tStop = an.motion === 'brake' ? Math.min(an.v0 / Math.abs(an.a), 5) : an.motion === 'cruise' ? 3 : 4;
      return { kind: 'car', tStop, reason: an.motion === 'brake' ? 'The car has stopped' : 'Time is up' };
    },
    motion(an, t) {
      return { s: an.v0 * t + 0.5 * an.a * t * t, v: an.v0 + an.a * t, a: an.a };
    },
    graphNames: { x: 'Position–time', v: 'Velocity–time', a: 'Acceleration–time' },
    graphValue(an, which, t) {
      const m = CAR.motion(an, t);
      return which === 'x' ? m.s : which === 'v' ? m.v : m.a;
    },
    graphLabel: { x: 'position of the car (m)', v: 'velocity (m/s)', a: 'acceleration (m/s²)' },
    draw(ctx, W, H, t) {
      const an = analysis;
      const mo = CAR.motion(an, t);
      const x = mo.s;
      const view = fitView({ x0: x - 3.4, x1: x + 3.4, y0: -0.35, y1: 2.65 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      // road
      const [rx0, ry0] = P(x - 6, 0);
      const [rx1, ry1] = P(x + 6, -0.3);
      ctx.save();
      ctx.fillStyle = C.bench;
      ctx.fillRect(rx0, ry0, rx1 - rx0, ry1 - ry0);
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 3;
      ctx.setLineDash([0.6 * view.k, 0.6 * view.k]);
      ctx.lineDashOffset = (x * view.k) % (1.2 * view.k);
      const [, dy] = P(0, -0.15);
      ctx.beginPath();
      ctx.moveTo(rx0, dy);
      ctx.lineTo(rx1, dy);
      ctx.stroke();
      ctx.restore();
      // car body (side view, facing right)
      const body = [[-2.1, 0.35], [2.2, 0.35], [2.2, 0.85], [1.15, 1.0], [0.6, 1.95], [-1.35, 1.95], [-1.95, 1.0], [-2.1, 0.95]];
      ctx.save();
      ctx.fillStyle = C.b1;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      body.forEach(([bx, by], i) => {
        const [px, py] = P(x + bx, by);
        i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      // big window so we can see inside
      const win = [[-1.2, 1.05], [1.0, 1.05], [0.52, 1.85], [-1.17, 1.85]];
      ctx.fillStyle = C.scene;
      ctx.beginPath();
      win.forEach(([bx, by], i) => {
        const [px, py] = P(x + bx, by);
        i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      wheel(ctx, view, x - 1.3, 0.33, 0.33);
      wheel(ctx, view, x + 1.4, 0.33, 0.33);
      // the mirror and the hanging ornament
      const pivot = { x: x + 0.05, y: 1.8 };
      const [pvx, pvy] = P(pivot.x, pivot.y);
      boxW(ctx, view, pivot.x - 0.12, pivot.y - 0.02, pivot.x + 0.12, pivot.y + 0.05, C['bench-edge'], C.ink, 0.02);
      const L = 0.55;
      const th = (an.theta * DEG) * (an.a > 0 ? -1 : an.a < 0 ? 1 : 0); // swings toward the back when speeding up
      const bob = { x: pivot.x + L * Math.sin(th), y: pivot.y - L * Math.cos(th) };
      // protractor
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.lineWidth = 1;
      const pr = 0.42 * view.k;
      ctx.beginPath();
      ctx.arc(pvx, pvy, pr, Math.PI * 0.25, Math.PI * 0.75);
      ctx.stroke();
      for (let d = -45; d <= 45; d += 5) {
        const ang = Math.PI / 2 + d * DEG;
        const len = d % 15 === 0 ? 9 : 5;
        ctx.beginPath();
        ctx.moveTo(pvx + pr * Math.cos(ang), pvy + pr * Math.sin(ang));
        ctx.lineTo(pvx + (pr - len) * Math.cos(ang), pvy + (pr - len) * Math.sin(ang));
        ctx.stroke();
      }
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(pvx, pvy);
      ctx.lineTo(pvx, pvy + 0.62 * view.k);
      ctx.stroke();
      ctx.restore();
      const [bx, by] = P(bob.x, bob.y);
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(pvx, pvy);
      ctx.lineTo(bx, by);
      ctx.stroke();
      ctx.fillStyle = C['f-a'];
      ctx.beginPath();
      ctx.arc(bx, by, Math.max(6, 0.065 * view.k), 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      if (Math.abs(an.a) > 1e-9) labelText(ctx, an.hidden ? 'θ = ?  (ticks every 5°)' : 'θ = ' + fmt(an.theta, 1) + '°', pvx + (an.a > 0 ? 14 : -14), pvy + 0.7 * view.k, C.ink, an.a > 0 ? 'left' : 'right', 15);
      // velocity and acceleration arrows above the car
      const [ax0, ay0] = P(x - 0.2, 2.25);
      if (Math.abs(an.a) > 1e-9) {
        const len = Math.min(120, 25 + Math.abs(an.a) * 10);
        arrow(ctx, ax0, ay0, ax0 + Math.sign(an.a) * len, ay0, C['f-a'], 4, 13);
        labelText(ctx, 'a', ax0 + Math.sign(an.a) * (len + 12), ay0, C['f-a'], 'center', 16);
      }
      if (mo.v > 1e-6) {
        const [vx0, vy0] = P(x - 0.2, 2.47);
        const len = Math.min(140, 15 + mo.v * 5);
        arrow(ctx, vx0, vy0, vx0 + len, vy0, C['f-n'], 4, 13);
        labelText(ctx, 'v', vx0 + len + 12, vy0, C['f-n'], 'center', 16);
      }
      labelText(ctx, an.motion === 'speed' ? 'Speeding up' : an.motion === 'brake' ? (mo.v > 1e-6 ? 'Braking' : 'Stopped') : 'Constant speed', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
      labelText(ctx, 'Speed: ' + fmt(Math.max(0, mo.v), 1) + ' m/s', 12, 40, C.muted, 'left', 13, 600, C.fontBody);
    },
    forceSets(an) {
      const th = an.theta * DEG;
      const s = an.a > 0 ? 1 : an.a < 0 ? -1 : 0; // the string pulls toward the front when speeding up
      return [{
        body: { name: 'Ornament', m: an.m, kind: 'hanging' },
        idx: 4,
        tilt: 0,
        arrows: [
          { dir: { x: s * Math.sin(th), y: Math.cos(th) }, mag: an.T, color: C.l1, label: 'T' },
          { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' },
        ],
      }];
    },
    fbdCaption: (an) => (Math.abs(an.a) < 1e-9 ? 'At constant speed the string hangs straight down: T and W balance.' : 'Only two forces act. The string’s tilt gives T a horizontal part, and that part is the net force that accelerates the ornament with the car.'),
    legend(an, showV) {
      return {
        legend: chipH('var(--l1)', 'T: the string pulls') + chipH('var(--f-w)', 'W: weight'),
        pairs: pairsH([
          swatch('--l1') + 'The string pulls the ornament, and the ornament pulls back on the string' + (showV && !an.hidden ? ' (' + fmt(an.T, 3) + ' N)' : '') + '. The string passes that pull to the mirror.',
          'Nothing pushes the ornament backward. It tends to keep its velocity (inertia), so it lags behind until the tilted string pulls it forward hard enough to keep up.',
          '<b>Weight:</b> Earth pulls the ornament down, and the ornament pulls Earth up just as hard.',
        ]),
      };
    },
    mcq(an) {
      return [{
        key: 'carHang',
        q: 'Which way does the ornament hang while the car is ' + (an.motion === 'speed' ? 'speeding up' : an.motion === 'brake' ? 'braking' : 'moving at constant speed') + '?',
        options: ['Straight down', 'Tilted toward the back of the car', 'Tilted toward the front of the car'],
        correct: an.motion === 'speed' ? 1 : an.motion === 'brake' ? 2 : 0,
        why: 'The string has to pull the ornament the same way the car accelerates, so it tilts toward the direction of the acceleration, leaving the ornament behind it.',
      }];
    },
    quantities(an) {
      const q = [];
      const row = (key, label, unit, value, sub, group = 'main', tol) => q.push({ key, group, label, unit, value, sub, tol });
      if (an.hidden) {
        row('a', 'Acceleration of the car (from the angle)', 'm/s²', Math.abs(an.a), 'read θ off the protractor', 'main', 0.06);
        row('T', 'Tension in the string', 'N', an.T, '', 'main', 0.04);
      } else {
        row('th', 'Angle of the string from vertical', '°', an.theta);
        row('T', 'Tension in the string', 'N', an.T);
        row('net', 'Net force on the ornament', 'N', an.Fnet);
      }
      row('W', 'Weight of the ornament', 'N', an.W, '', 'more');
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const g = fmt(an.g);
      if (Math.abs(an.a) < 1e-9)
        return '<section><h3>Constant speed</h3><p>With no acceleration there is no net force (Newton’s first law), so the string hangs straight down and T = W = mg = ' + fmt(an.W, 3) + ' N.</p></section>';
      const dir = an.a > 0 ? 'forward' : 'backward';
      let h = '<section><h3>1. Forces on the ornament</h3><p>Two forces act: the weight W = mg down and the string’s tension T along the string, which tilts by θ from vertical. The ornament accelerates ' + dir + ' with the car at a = ' + (an.hidden ? '?' : fmt(Math.abs(an.a))) + ' m/s².</p>' +
        '<div class="eq">Horizontal:  T sin θ = m a\nVertical:    T cos θ − m g = 0</div></section>';
      h += '<section><h3>2. Divide the two equations</h3><div class="eq">tan θ = a / g</div>';
      h += an.hidden
        ? '<p>So you can find the acceleration from the angle:</p><div class="eq">a = g tan θ = ' + g + ' × tan ' + fmt(an.theta, 1) + '° = ' + fmt(Math.abs(an.a)) + ' m/s²</div></section>'
        : '<div class="eq">θ = tan⁻¹(a / g) = tan⁻¹(' + fmt(Math.abs(an.a)) + ' / ' + g + ') = ' + fmt(an.theta, 1) + '°</div></section>';
      h += '<section><h3>3. Tension and net force</h3><div class="eq">T = m √(g² + a²) = ' + fmt(an.m) + ' × √(' + g + '² + ' + fmt(Math.abs(an.a)) + '²) = ' + fmt(an.T, 3) + ' N\nNet force = m a = ' + fmt(an.m) + ' × ' + fmt(Math.abs(an.a)) + ' = ' + fmt(an.Fnet, 3) + ' N, ' + dir + '</div>' +
        '<p>The angle does not depend on the ornament’s mass, which is what makes this a good accelerometer.</p></section>';
      return h;
    },
  };

  /* ---------------- Tilt-the-ramp friction lab ---------------- */
  const LAB_TYPES = [
    ['fma', 'Force vs acceleration (F = ma)'],
    ['tilt', 'Tilt the ramp (friction)'],
  ];
  const TILT_RATE = 4; // degrees per second while raising the ramp
  const TILT_D = 1; // slide distance timed (m)
  const TILT = {
    defaults: () => ({ mus: 0.55, muk: 0.4, hidden: false }),
    coeffs(t) {
      if (!t.hidden) return { mus: t.mus, muk: Math.min(t.muk, t.mus * 0.95) };
      if (!ui.tiltSecret) {
        const mus = Math.round((0.3 + Math.random() * 0.6) * 100) / 100;
        ui.tiltSecret = { mus, muk: Math.round(mus * (0.6 + Math.random() * 0.25) * 100) / 100 };
      }
      return ui.tiltSecret;
    },
    solve(c) {
      const t = c.tilt;
      const k = TILT.coeffs(t);
      const r = P.tiltLab({ mus: k.mus, muk: k.muk, g: state.g, d: TILT_D });
      // what the student measures (a little noise when the values are hidden)
      const key = (t.hidden ? 'h' : 'k') + k.mus + ',' + k.muk + ',' + state.g;
      if (!ui.tiltMeas || ui.tiltMeas.key !== key) {
        ui.tiltMeas = t.hidden
          ? { key, theta: Math.round((r.thetaC + 0.3 * gauss()) * 2) / 2, t: r.t * (1 + 0.01 * gauss()) }
          : { key, theta: r.thetaC, t: r.t };
      }
      const an = { kind: 'tilt', ok: true, g: state.g, ...k, ...r, hidden: t.hidden, meas: ui.tiltMeas };
      an.tTilt = r.thetaC / TILT_RATE;
      an.bodies = [{ name: 'Block', m: 1, kind: 'block', a: -r.a }];
      an.atRest = false;
      return an;
    },
    hidesAnswers: (an) => an.hidden,
    controls(c) {
      const t = c.tilt;
      let h = '<div class="group"><h2>Lab</h2>' + selectField('Choose a lab', 'sel-lab-type', 'type', LAB_TYPES, c.type, true) + '</div>';
      h += '<div class="group"><h2>The block and ramp</h2>' + checkField('Hide the friction coefficients (measure them)', 'tilt.hidden', t.hidden);
      if (t.hidden) {
        h += '<label class="field"><span>μs</span><input type="text" value="?" disabled aria-label="Hidden"></label><label class="field"><span>μk</span><input type="text" value="?" disabled aria-label="Hidden"></label>' +
          '<div class="btn-row"><button type="button" class="btn btn-small" data-action="newtilt">New hidden block</button></div>';
      } else {
        h += numField('Static coefficient μs', 'tilt.mus', t.mus, '', { min: 0.05, max: 1.5, step: 0.05 }) + numField('Kinetic coefficient μk', 'tilt.muk', t.muk, '', { min: 0, max: 1.5, step: 0.05 });
        if (t.muk >= t.mus) h += '<p class="warn">μk must be smaller than μs for the block to slide once it breaks loose. The simulator uses μk = 0.95 μs.</p>';
      }
      h += '<p class="note">Press Play. The ramp tilts up by ' + TILT_RATE + '° each second until the block breaks loose. Then the angle holds and a timer measures how long the block takes to slide ' + fmt(TILT_D) + ' m.</p></div>';
      return h;
    },
    randomize(c, r) {
      const mus = r(0.3, 0.9, 0.05);
      c.tilt.mus = mus;
      c.tilt.muk = Math.round(mus * r(0.6, 0.85, 0.05) * 100) / 100;
      ui.tiltSecret = null;
    },
    buildSim(an) {
      if (!(an.a > 0)) return { kind: 'tilt', tStop: an.tTilt, reason: 'The block broke loose but did not speed up' };
      return { kind: 'tilt', tStop: an.tTilt + an.t, reason: 'The block slid ' + fmt(TILT_D) + ' m' };
    },
    // the ramp angle and how far the block has slid at time t
    stateAt(an, t) {
      if (t <= an.tTilt) return { theta: TILT_RATE * t, s: 0, v: 0, sliding: false };
      const tau = t - an.tTilt;
      return { theta: an.thetaC, s: 0.5 * an.a * tau * tau, v: an.a * tau, sliding: true, tau };
    },
    motion(an, t) {
      const st = TILT.stateAt(an, t);
      return { s: -st.s, v: -st.v, a: st.sliding ? -an.a : 0 };
    },
    graphNames: { x: 'Position–time', v: 'Velocity–time', a: 'Acceleration–time' },
    graphValue(an, which, t) {
      const m = TILT.motion(an, t);
      return which === 'x' ? m.s : which === 'v' ? m.v : m.a;
    },
    graphLabel: { x: 'position along the ramp (m)', v: 'velocity (m/s)', a: 'acceleration (m/s²)' },
    draw(ctx, W, H, t) {
      const an = analysis;
      const st = TILT.stateAt(an, t);
      const th = st.theta * DEG;
      const Lr = 2.2;
      const hinge = { x: 0, y: 0.7 };
      const u = { x: Math.cos(th), y: Math.sin(th) };
      const n = { x: -Math.sin(th), y: Math.cos(th) };
      const view = fitView({ x0: -0.6, x1: 2.6, y0: -0.15, y1: 2.0 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -0.6, 2.6);
      // bench and hinge
      boxW(ctx, view, -0.3, 0.6, 2.4, 0.7, C.bench);
      boxW(ctx, view, -0.2, 0, -0.12, 0.6, C.bench);
      boxW(ctx, view, 2.22, 0, 2.3, 0.6, C.bench);
      // the jack lifting the far end
      const tip = { x: hinge.x + Lr * u.x, y: hinge.y + Lr * u.y };
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 5;
      const [jx, jy] = P(tip.x - 0.05, 0.7);
      const [jx2, jy2] = P(tip.x - 0.05, tip.y);
      ctx.beginPath();
      ctx.moveTo(jx, jy);
      ctx.lineTo(jx2, jy2);
      ctx.stroke();
      ctx.restore();
      // ramp board
      const at = (s, up) => ({ x: hinge.x + u.x * s + n.x * up, y: hinge.y + u.y * s + n.y * up });
      const corners = [at(0, -0.05), at(Lr, -0.05), at(Lr, 0), at(0, 0)].map((p) => P(p.x, p.y));
      ctx.save();
      ctx.fillStyle = C.b2;
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      corners.forEach(([px, py], i) => (i ? ctx.lineTo(px, py) : ctx.moveTo(px, py)));
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      const [hx, hy] = P(hinge.x, hinge.y);
      ctx.fillStyle = C.ink;
      ctx.beginPath();
      ctx.arc(hx, hy, 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      // photogate 1.00 m below where the block starts
      const s0 = 1.85;
      const sGate = s0 - TILT_D;
      const g0 = at(sGate, 0);
      const g1 = at(sGate, 0.32);
      const [gx0, gy0] = P(g0.x, g0.y);
      const [gx1, gy1] = P(g1.x, g1.y);
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 5;
      ctx.beginPath();
      ctx.moveTo(gx0, gy0);
      ctx.lineTo(gx1, gy1);
      ctx.stroke();
      ctx.fillStyle = C['f-w'];
      ctx.beginPath();
      ctx.arc(gx1, gy1, 3.5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      labelText(ctx, 'Gate', gx1, gy1 - 12, C.muted, 'center', 12, 600, C.fontBody);
      // block
      const bs = 0.24;
      const c0 = at(s0 - st.s, bs / 2);
      blockPath(ctx, view, c0, u, n, 0.34, bs);
      ctx.save();
      ctx.fillStyle = bodyColor(0);
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 1.5;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      // protractor at the hinge
      ctx.save();
      ctx.strokeStyle = C.muted;
      ctx.lineWidth = 1;
      const rr = 0.55 * view.k;
      ctx.beginPath();
      ctx.arc(hx, hy, rr, -Math.PI / 2, 0);
      ctx.stroke();
      for (let d = 0; d <= 90; d += 5) {
        const len = d % 10 === 0 ? 9 : 5;
        const a = -d * DEG;
        ctx.beginPath();
        ctx.moveTo(hx + rr * Math.cos(a), hy + rr * Math.sin(a));
        ctx.lineTo(hx + (rr - len) * Math.cos(a), hy + (rr - len) * Math.sin(a));
        ctx.stroke();
        if (d % 30 === 0) labelText(ctx, d + '°', hx + (rr + 14) * Math.cos(a), hy + (rr + 14) * Math.sin(a), C.muted, 'center', 11, 400);
      }
      ctx.restore();
      const shown = st.sliding ? an.meas.theta : st.theta;
      labelText(ctx, 'Ramp angle: ' + fmt(shown, 1) + '°', 12, 18, C.ink, 'left', 16, 700);
      const timer = st.sliding ? Math.min(st.tau, an.t) * (an.meas.t / an.t) : 0;
      labelText(ctx, 'Timer: ' + timer.toFixed(3) + ' s' + (st.sliding ? '' : '  (starts when the block slips)'), 12, 42, C.muted, 'left', 14, 600);
      labelText(ctx, st.sliding ? 'The block slipped and is sliding' : 'Raising the ramp…', 12, 64, C.ink, 'left', 14, 700, C.fontBody);
    },
    forceSets(an) {
      const th = an.thetaC * DEG;
      const u = { x: Math.cos(th), y: Math.sin(th) }; // up the slope
      const n = { x: -Math.sin(th), y: Math.cos(th) };
      const N = an.g * Math.cos(th);
      const set = (name, f, label) => ({
        body: { name, m: 1, kind: 'block' },
        idx: 0,
        tilt: th,
        arrows: [
          { dir: { x: 0, y: -1 }, mag: an.g, color: C['f-w'], label: 'W' },
          { dir: n, mag: N, color: C['f-n'], label: 'N' },
          { dir: u, mag: f, color: C['f-f'], label },
        ],
      });
      return [set('Just before it slips (per kg)', an.mus * N, 'fₛ,max'), set('Sliding (per kg)', an.muk * N, 'fₖ')];
    },
    fbdCaption: () => 'Shown for a 1 kg block at the slipping angle. Just before it slips, static friction is at its largest. Once it slides, kinetic friction is smaller, so the block speeds up.',
    legend() {
      return {
        legend: chipH('var(--f-w)', 'W: weight') + chipH('var(--f-n)', 'N: normal force') + chipH('var(--f-f)', 'f: friction'),
        pairs: pairsH([
          'The ramp pushes up on the block (N) and rubs on it (f); the block pushes into the ramp and rubs back on it just as hard.',
          'The mass cancels out of both formulas, so any block of the same material gives the same angle.',
        ]),
      };
    },
    quantities(an) {
      const q = [];
      q.push({ key: 'mus', group: 'main', groupTitle: 'From your measurements', label: 'Coefficient of static friction μs', unit: '', value: an.mus, tol: an.hidden ? 0.04 : 0.02, sub: 'from the angle where it slips' });
      q.push({ key: 'muk', group: 'main', groupTitle: 'From your measurements', label: 'Coefficient of kinetic friction μk', unit: '', value: an.muk, tol: an.hidden ? 0.07 : 0.02, sub: 'from the slide time' });
      if (!an.hidden) {
        q.push({ key: 'th', group: 'pred', groupTitle: 'Predict', label: 'Angle where the block slips', unit: '°', value: an.thetaC });
        q.push({ key: 'a', group: 'pred', groupTitle: 'Predict', label: 'Acceleration while sliding', unit: 'm/s²', value: an.a });
        q.push({ key: 't', group: 'pred', groupTitle: 'Predict', label: 'Time to slide ' + fmt(TILT_D) + ' m', unit: 's', value: an.t });
      }
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const g = fmt(an.g);
      const m = an.meas;
      const th = m.theta;
      const a = (2 * TILT_D) / (m.t * m.t);
      const mus = Math.tan(th * DEG);
      const muk = mus - a / (an.g * Math.cos(th * DEG));
      return '<section><h3>1. Static friction from the slipping angle</h3><p>Just before the block slips, it is at rest with static friction at its maximum, μs N. Along the slope and perpendicular to it:</p>' +
        '<div class="eq">μs N = mg sin θ   and   N = mg cos θ\n→   μs = tan θ = tan ' + fmt(th, 1) + '° = ' + fmt(mus, 3) + '</div></section>' +
        '<section><h3>2. Acceleration from the timer</h3><p>The block starts from rest and slides d = ' + fmt(TILT_D) + ' m in t = ' + fmt(m.t, 3) + ' s:</p><div class="eq">a = 2d / t² = 2 × ' + fmt(TILT_D) + ' / ' + fmt(m.t, 3) + '² = ' + fmt(a, 3) + ' m/s²</div></section>' +
        '<section><h3>3. Kinetic friction from Newton’s second law</h3><div class="eq">mg sin θ − μk mg cos θ = m a\n→   μk = tan θ − a / (g cos θ) = ' + fmt(mus, 3) + ' − ' + fmt(a, 3) + ' / (' + g + ' × ' + fmt(Math.cos(th * DEG), 3) + ') = ' + fmt(muk, 3) + '</div>' +
        (an.hidden ? '<p>The true values were μs = ' + fmt(an.mus) + ' and μk = ' + fmt(an.muk) + '. Small differences come from measurement error, just like in a real lab.</p>' : '') + '</section>';
    },
  };

  /* ---------------- Block and tackle ---------------- */
  const TACKLE_MOTIONS = [
    ['hold', 'Hold the load still'],
    ['steady', 'Lift at a constant speed'],
    ['accel', 'Lift while speeding up'],
  ];
  const TACKLE_RISE = 0.8; // the rope is pulled 0.8 m in the animation
  const TACKLE = {
    defaults: () => ({ m: 100, n: 4, motion: 'hold', a: 0.5 }),
    solve(c) {
      const t = c.tackle;
      const a = t.motion === 'accel' ? t.a : 0;
      const n = Math.max(1, Math.min(6, Math.round(t.n)));
      const r = P.solveTackle({ m: t.m, n, g: state.g, a });
      const an = { kind: 'tackle', ok: true, ...r, motion: t.motion };
      an.bodies = [{ name: 'Load', m: t.m, kind: 'hanging', a }];
      an.atRest = t.motion === 'hold';
      return an;
    },
    controls(c) {
      const t = c.tackle;
      let h = '<div class="group"><h2>Problem</h2>' + selectField('Type of problem', 'sel-statics-mode', 'mode', STATICS_MODES, c.mode, true) + '</div>';
      h += '<div class="group"><h2>Block and tackle</h2>' +
        numField('Mass of the load', 'tackle.m', t.m, 'kg', { min: 1, max: 2000, step: 1 }) +
        numField('Rope segments holding the lower block, n', 'tackle.n', t.n, '', { min: 1, max: 6, step: 1 }) +
        selectField('What happens', 'sel-tackle-motion', 'tackle.motion', TACKLE_MOTIONS, t.motion, true) +
        (t.motion === 'accel' ? numField('Upward acceleration of the load', 'tackle.a', t.a, 'm/s²', { min: 0.05, max: 5, step: 0.05 }) : '') +
        '<p class="note">One rope runs back and forth between the pulleys; you pull its free end down. Pulleys and rope are massless and frictionless.</p></div>';
      return h;
    },
    hidesAnswers: () => false,
    randomize(c, r) {
      STATICS.randomize(c, r);
    },
    buildSim(an) {
      if (an.motion === 'hold') return { kind: 'tackle', tStop: 0, static: true, staticMessage: 'The load hangs at rest. Use the diagrams and Predict & check, or choose a way to lift it.' };
      const rise = TACKLE_RISE / an.n;
      const tStop = an.motion === 'steady' ? rise / 0.15 : Math.sqrt((2 * rise) / an.a);
      return { kind: 'tackle', tStop, reason: 'You pulled ' + fmt(TACKLE_RISE) + ' m of rope and the load rose ' + fmt(rise, 3) + ' m' };
    },
    rise(an, t) {
      return an.motion === 'steady' ? 0.15 * t : an.motion === 'accel' ? 0.5 * an.a * t * t : 0;
    },
    motion(an, t) {
      const v = an.motion === 'steady' ? 0.15 : an.motion === 'accel' ? an.a * t : 0;
      return { s: TACKLE.rise(an, t), v, a: an.motion === 'accel' ? an.a : 0 };
    },
    noGraph: (an) => an.motion === 'hold',
    graphNames: { x: 'Height–time', v: 'Velocity–time', a: 'Acceleration–time' },
    graphValue(an, which, t) {
      const m = TACKLE.motion(an, t);
      return which === 'x' ? m.s : which === 'v' ? m.v : m.a;
    },
    graphLabel: { x: 'rise of the load (m)', v: 'velocity (m/s)', a: 'acceleration (m/s²)' },
    draw(ctx, W, H, t) {
      const an = analysis;
      const n = an.n;
      const rise = TACKLE.rise(an, t);
      const pulled = rise * n;
      const sp = 0.22;
      const r = sp / 2;
      const x0 = -0.2 - (n * sp) / 2;
      const xs = Array.from({ length: n + 1 }, (_, i) => x0 + i * sp);
      const yTop = 3.55;
      const yBot = 1.55 + rise;
      const view = fitView({ x0: -2.1, x1: 2.1, y0: -0.15, y1: 4.1 }, W, H);
      const P = view.P;
      drawGrid(ctx, W, H, view);
      drawFloor(ctx, view, -2.1, 2.1);
      // ceiling
      boxW(ctx, view, -1.6, 3.95, 1.6, 4.05, C.bench);
      const [chx, chy] = P((xs[0] + xs[n]) / 2, 3.95);
      const [, chy2] = P(0, yTop + 0.16);
      ctx.save();
      ctx.strokeStyle = C['bench-edge'];
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(chx, chy);
      ctx.lineTo(chx, chy2);
      ctx.stroke();
      ctx.restore();
      // upper and lower blocks (frames holding the pulley axles)
      boxW(ctx, view, xs[0] - 0.12, yTop + 0.12, xs[n] + 0.06, yTop + 0.2, C['bench-edge'], C.ink, 0.02);
      boxW(ctx, view, xs[0] - 0.12, yBot - 0.2, xs[n - 1] + 0.12, yBot - 0.12, C['bench-edge'], C.ink, 0.02);
      // load
      const lc = (xs[0] + xs[n - 1]) / 2;
      const [lx, ly0] = P(lc, yBot - 0.2);
      const [, ly1] = P(lc, yBot - 0.42);
      ctx.save();
      ctx.strokeStyle = C.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(lx, ly0);
      ctx.lineTo(lx, ly1);
      ctx.stroke();
      ctx.restore();
      boxW(ctx, view, lc - 0.35, yBot - 1.0, lc + 0.35, yBot - 0.42, bodyColor(1), C.ink, 0.03);
      const [tx, ty] = P(lc, yBot - 0.71);
      labelText(ctx, trimNum(an.m) + ' kg', tx, ty, C.ink, 'center', 15);
      // the rope: segment i goes up if (n - i) is odd
      const up = (i) => (n - i) % 2 === 1;
      ctx.save();
      ctx.strokeStyle = C['f-f'];
      ctx.lineWidth = 2.5;
      for (let i = 0; i < n; i++) {
        const [sx, sy0] = P(xs[i], yTop);
        const [, sy1] = P(xs[i], yBot);
        ctx.beginPath();
        ctx.moveTo(sx, sy0);
        ctx.lineTo(sx, sy1);
        ctx.stroke();
      }
      // free end down to the hands
      const handY = 2.05 - pulled;
      const [fx, fy0] = P(xs[n], yTop);
      const [, fy1] = P(xs[n], handY);
      ctx.beginPath();
      ctx.moveTo(fx, fy0);
      ctx.lineTo(fx, fy1);
      ctx.stroke();
      // the pulled rope piles up below the hands
      const [, fy2] = P(xs[n], 0.05);
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(fx, fy1);
      ctx.lineTo(fx, fy2);
      ctx.stroke();
      ctx.setLineDash([]);
      // pulleys at each turn
      for (let i = 0; i < n; i++) {
        const cx = (xs[i] + xs[i + 1]) / 2;
        const top = up(i);
        const [px, py] = P(cx, top ? yTop : yBot);
        ctx.strokeStyle = C['f-f'];
        ctx.beginPath();
        ctx.arc(px, py, r * view.k, top ? Math.PI : 0, top ? 2 * Math.PI : Math.PI);
        ctx.stroke();
        ctx.fillStyle = C.panel;
        ctx.strokeStyle = C.ink;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(px, py, r * view.k * 0.78, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = C.ink;
        ctx.beginPath();
        ctx.arc(px, py, 2.5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.lineWidth = 2.5;
      }
      // anchor for the start of the rope
      const [ax, ay] = P(xs[0], up(0) ? yBot - 0.12 : yTop + 0.12);
      ctx.fillStyle = C.ink;
      ctx.fillRect(ax - 5, ay - 5, 10, 10);
      ctx.restore();
      // the person pulling the free end down
      const pose = poseUpright({ x: 0, yHip: STAND_HIP, contact: true, armA: 2.6 });
      const feetX = xs[n] + 0.62;
      const target = { x: (xs[n] - feetX) / -1, y: handY };
      pose.arms = pose.arms.map(() => ({ elbow: ik(pose.shoulder, target, FIG.upper, FIG.fore, -1), hand: target }));
      drawFigure(ctx, { k: view.k, P: (x, y) => view.P(feetX - x, y) }, pose);
      // forces
      const sc = 90 / Math.max(an.W, 1);
      const pullLen = Math.max(26, an.T * sc);
      forceArrow(ctx, view, { x: xs[n], y: handY - 0.05 }, 0, -1, pullLen, C['f-a'], '');

      forceArrow(ctx, view, { x: lc - 0.47, y: yBot - 0.71 }, 0, -1, Math.max(26, an.W * sc), C['f-w'], 'W', 'right');
      labelText(ctx, 'n = ' + n + ' segments hold the lower block', 12, 18, C.ink, 'left', 15, 700, C.fontBody);
      labelText(ctx, 'Rope pulled: ' + fmt(pulled, 2) + ' m    Load raised: ' + fmt(rise, 3) + ' m', 12, 40, C.muted, 'left', 13, 600, C.fontBody);
      labelText(ctx, 'Purple arrow: your pull on the rope (= T)', 12, 60, C['f-a'], 'left', 13, 600, C.fontBody);
    },
    forceSets(an) {
      const Tarrows = Array.from({ length: an.n }, (_, i) => ({ dir: { x: 0, y: 1 }, mag: an.T, color: C['f-f'], label: i ? '' : an.n > 1 ? 'T (×' + an.n + ')' : 'T' }));
      return [
        { body: { name: 'Lower block + load', m: an.m, kind: 'hanging' }, idx: 1, tilt: 0, arrows: [...Tarrows, { dir: { x: 0, y: -1 }, mag: an.W, color: C['f-w'], label: 'W' }] },
        {
          body: { name: 'Upper block', m: null, kind: 'hanging' },
          idx: 2,
          tilt: 0,
          arrows: [{ dir: { x: 0, y: 1 }, mag: an.ceiling, color: C['f-n'], label: 'F_ceiling' }, ...Array.from({ length: an.n + 1 }, (_, i) => ({ dir: { x: 0, y: -1 }, mag: an.T, color: C['f-f'], label: i ? '' : 'T (×' + (an.n + 1) + ')' }))],
        },
      ];
    },
    fbdCaption: (an) => an.n + ' rope segments pull up on the lower block, each with the same tension T. The upper block is pulled down by those ' + an.n + ' segments plus the free end you pull.',
    legend(an, showV) {
      return {
        legend: chipH('var(--f-f)', 'T: rope tension (the same everywhere)') + chipH('var(--f-w)', 'W: weight of the load') + chipH('var(--f-n)', 'F_ceiling: the ceiling holds the upper block'),
        pairs: pairsH([
          'You pull down on the rope with T' + (showV ? ' (' + fmt(an.T) + ' N)' : '') + ', and the rope pulls up on your hands just as hard.',
          'The ceiling pulls up on the upper block, and the upper block pulls down on the ceiling with the same force' + (showV ? ' (' + fmt(an.ceiling) + ' N)' : '') + '.',
          'A block and tackle reduces the force you need, not the work: you pull the rope n times as far as the load rises.',
        ]),
      };
    },
    mcq(an) {
      return [{
        key: 'tackleMA',
        q: 'To raise the load by 1 m with this setup, how much rope do you pull?',
        options: an.n === 1 ? ['1 m', '2 m', '0.50 m'] : ['1 m', an.n + ' m', (1 / an.n).toFixed(2) + ' m'],
        correct: an.n === 1 ? 0 : 1,
        why: 'Each of the ' + an.n + ' segments must get 1 m shorter, so ' + an.n + ' m of rope comes out. Force × distance stays the same: T × ' + an.n + ' m = W × 1 m.',
      }];
    },
    quantities(an) {
      const q = [];
      const row = (key, label, unit, value, sub, group = 'main') => q.push({ key, group, label, unit, value, sub });
      row('T', 'Force you pull with (rope tension)', 'N', an.T);
      row('C', 'Force on the ceiling', 'N', an.ceiling, 'the upper block is held by the ceiling');
      row('d', 'Rope pulled to raise the load 1 m', 'm', an.n);
      row('W', 'Weight of the load', 'N', an.W, '', 'more');
      if (an.motion === 'accel') row('net', 'Net force on the load', 'N', an.m * an.a, '', 'more');
      return q;
    },
    moreTitle: 'More',
    solution(an) {
      const g = fmt(an.g);
      const aTxt = an.motion === 'accel' ? fmt(an.a) : '0';
      return '<section><h3>1. The same tension everywhere</h3><p>With a single massless rope over frictionless pulleys, the tension is the same in every segment: T, equal to your pull.</p></section>' +
        '<section><h3>2. Forces on the lower block and load</h3><p>' + an.n + ' segments pull up on the lower block, each with T. Take + as up' + (an.motion === 'accel' ? '; the load speeds up at a = ' + aTxt + ' m/s².' : '; the load is ' + (an.motion === 'hold' ? 'at rest' : 'moving at constant speed') + ', so a = 0.') + '</p>' +
        '<div class="eq">n T − m g = m a\nT = m (g + a) / n = ' + fmt(an.m) + ' × (' + g + ' + ' + aTxt + ') / ' + an.n + ' = ' + fmt(an.T) + ' N</div>' +
        '<p>That is 1/' + an.n + ' of the force you would need to lift ' + (an.motion === 'accel' ? 'and accelerate ' : '') + 'it directly.</p></section>' +
        '<section><h3>3. Force on the ceiling</h3><p>The upper block is pulled down by the ' + an.n + ' segments plus the free end you pull down, ' + (an.n + 1) + ' tensions in all.</p>' +
        '<div class="eq">F_ceiling = (n + 1) T = ' + (an.n + 1) + ' × ' + fmt(an.T) + ' = ' + fmt(an.ceiling) + ' N</div></section>' +
        '<section><h3>4. The trade-off</h3><p>To raise the load 1 m, each of the ' + an.n + ' segments shortens by 1 m, so you pull ' + an.n + ' m of rope. Your work, T × ' + an.n + ' m = ' + fmt(an.T * an.n) + ' J, matches the work done on the load' + (an.motion === 'accel' ? '' : ', W × 1 m = ' + fmt(an.W) + ' J') + '.</p></section>';
    },
  };

  const XS = { elevator: ELEV, body: BODY, traction: BODY, traffic: STATICS, wall: STATICS, tackle: TACKLE, lab: LAB, tilt: TILT, puzzle: PUZ, car: CAR };
  const xsFor = (an) => (an && XS[an.kind]) || null;
  const MODULES = { elevator: ELEV, body: BODY, statics: STATICS, lab: LAB, puzzles: PUZ, car: CAR };
  const scenarioModule = () => {
    const sc = state.scenario;
    if (sc === 'statics' && cur().mode === 'tackle') return TACKLE;
    if (sc === 'lab' && cur().type === 'tilt') return TILT;
    return MODULES[sc] || null;
  };
  const isFma = () => state.scenario === 'lab' && cur().type !== 'tilt';

  /* ======================================================================
   * Readouts, status and animation
   * ==================================================================== */

  // Mystery mode hides anything that gives the mass away; launch mode relabels the graphs.
  function syncModes() {
    const locked = mysteryOn() && !ui.revealed;
    const vals = $('chk-fbd-values');
    vals.disabled = locked;
    if (locked) {
      vals.checked = false;
      ui.showFbdValues = false;
    }
    vals.parentElement.title = locked ? 'Hidden in mystery-mass mode until you choose Show answers' : '';
    const launching = !!(sim && sim.geom && sim.geom.flight);
    const xs = xsFor(analysis);
    const xsNames = xs ? (typeof xs.graphNames === 'function' ? xs.graphNames() : xs.graphNames) : null;
    const names = xsNames || (launching ? { x: 'x–t (horizontal)', v: 'vₓ–t', a: 'v_y–t' } : { x: 'Position–time', v: 'Velocity–time', a: 'Acceleration–time' });
    const hideA = locked && !launching && !xs;
    document.querySelectorAll('[data-graph]').forEach((b) => {
      b.textContent = names[b.dataset.graph];
      b.hidden = b.dataset.graph === 'a' && hideA;
    });
    // an equilibrium problem has no motion to graph
    const isLab = isFma();
    const noGraph = !isLab && (!!(sim && sim.static) || !!(xs && xs.noGraph && analysis.ok && xs.noGraph(analysis)));
    $('tab-graph').hidden = noGraph;
    $('tab-graph').textContent = isLab ? 'Data & graph' : 'Motion graph';
    document.querySelector('#panel-graph .seg').hidden = isLab;
    $('lab-data').hidden = !isLab;
    if (isLab) LAB.renderData();
    if (noGraph && ui.tab === 'graph') showTab('check');
    $('fbd-caption').textContent = xs && xs.fbdCaption && analysis.ok ? xs.fbdCaption(analysis, anim.t) : '';
    $('fbd-caption').hidden = !$('fbd-caption').textContent;
    if (ui.graph === 'a' && hideA) {
      ui.graph = 'x';
      document.querySelectorAll('[data-graph]').forEach((x) => x.classList.toggle('on', x.dataset.graph === 'x'));
    }
    $('graph-hint').textContent = isLab
      ? 'Each run adds a point. A straight line through the points has slope M (the total mass) and intercept f (the friction).'
      : launching
      ? 'Press Play to draw the graph. On the ramp the block speeds up; in the air vₓ stays constant while v_y grows at g.'
      : state.scenario === 'elevator'
        ? 'Press Play to draw the graph. Watch the scale reading jump above or below the true weight whenever the elevator accelerates.'
        : state.scenario === 'body'
          ? 'Press Play to draw the graph. The force graph shows how big the push is and how briefly it lasts.'
          : 'Press Play to draw the graph for the tracked object. The slope of the velocity–time graph is the acceleration.';
  }

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
    if (xsFor(analysis)) return state.scenario === 'body' && cur().mode === 'sprint' ? '+ = forward' : '+ = up';
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
    if (m.air !== undefined) {
      $('ro-d-label').textContent = m.air ? 'Distance from edge' : 'Displacement';
      $('ro-v-label').textContent = m.air ? 'Speed' : 'Velocity';
      $('ro-d').textContent = fmt(m.air ? m.x : m.s) + ' m';
      $('ro-v').textContent = fmt(m.air ? m.speed : m.v) + ' m/s';
    } else {
      $('ro-d-label').textContent = 'Displacement';
      $('ro-v-label').textContent = 'Velocity';
      $('ro-d').textContent = fmt(m.s) + ' m';
      $('ro-v').textContent = fmt(m.v) + ' m/s';
    }
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
    if (sim && sim.static) {
      st.textContent = sim.staticMessage || 'This is an equilibrium problem: the leg and the weight stay at rest. Use the diagrams and Predict & check.';
      return;
    }
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
          if (slack.length) msg += ' ' + slack.map((l) => (l.type === 'string' ? 'A string went slack.' : l.type === 'stack' ? 'The top block slipped.' : 'Two blocks separated.')).filter((x, i, a) => a.indexOf(x) === i).join(' ');
        }
        st.textContent = msg + ' Press Reset to run it again.';
      }
      return;
    }
    st.textContent = 'Paused at t = ' + anim.t.toFixed(2) + ' s.';
  }

  function updatePlayButton() {
    const b = $('btn-play');
    b.disabled = !analysis.ok || !!(sim && sim.static);
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
      if (isFma() && analysis.ok && analysis.a > 0) LAB.onRunEnd(analysis);
    }
    drawScene();
    updateReadouts();
    drawGraph();
    // elevator forces change from stage to stage, so keep the diagrams live
    if (state.scenario === 'elevator' && ui.tab === 'fbd') {
      drawFBD();
      $('fbd-caption').textContent = ELEV.fbdCaption(analysis, anim.t);
    }
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
    state = defaults();
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
      else if (/mu[sk]$|^launch\.phi$/.test(e.target.dataset.path || '')) onControlInput(e);
    });
    controls.addEventListener('click', onControlClick);

    const setLook = (look) => {
      if (look === 'light') document.documentElement.setAttribute('data-look', 'light');
      else document.documentElement.removeAttribute('data-look');
      $('btn-look').textContent = look === 'light' ? 'Black background' : 'White background';
      try {
        localStorage.setItem('look', look);
      } catch (err) {
        /* storage may be blocked; the switch still works for this visit */
      }
    };
    let look = 'dark';
    try {
      look = localStorage.getItem('look') === 'light' ? 'light' : 'dark';
    } catch (err) {
      /* ignore */
    }
    setLook(look);
    $('btn-look').addEventListener('click', () => setLook(document.documentElement.getAttribute('data-look') === 'light' ? 'dark' : 'light'));

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
    $('check-rows').addEventListener('change', (e) => {
      if (e.target.type === 'radio') ui.pred['mc:' + e.target.name.slice(3)] = e.target.value;
    });
    $('lab-data').addEventListener('click', (e) => {
      const b = e.target.closest('[data-labdel]');
      if (!b) return;
      ui.labTrials.splice(+b.dataset.labdel, 1);
      LAB.renderData();
      renderControls();
      renderCheck();
      drawGraph();
    });
    $('btn-reveal').addEventListener('click', () => {
      ui.revealed = true;
      ui.solutionOpen = true;
      gradeAll(true);
      syncModes();
      renderSolution();
      renderLegendAndPairs();
      if (mysteryOn()) {
        drawScene();
        drawFBD();
      }
    });
    $('btn-clear').addEventListener('click', () => {
      ui.pred = {};
      ui.revealed = false;
      document.querySelectorAll('#check-rows input[type=number]').forEach((i) => (i.value = ''));
      document.querySelectorAll('#check-rows input[type=radio]').forEach((i) => (i.checked = false));
      ui.checkSig = '';
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
    new MutationObserver(recolor).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'data-look', 'class'] });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(recolor);

    renderControls();
    changed();
    showTab('check');
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
