/*
 * Newton's Laws Simulator: physics engine.
 * Plain functions, no DOM, so they run in the browser and in Node tests.
 *
 * Units are SI (kg, m, s, N).
 *
 * Surface systems ("chains") use one coordinate, s, along the surface:
 *   + s points to the right on a level surface, or up the incline,
 *   which is also toward the pulley when there is one.
 * A hanging block uses "downward" as its + direction, so when the string
 * is taut every object in the chain shares the same signed acceleration.
 */
(function (global) {
  'use strict';

  const DEG = Math.PI / 180;

  /* ---------- Atwood machine: two hanging masses ---------- */
  function solveAtwood({ m1, m2, g }) {
    const M = m1 + m2;
    const a = ((m2 - m1) * g) / M; // + means mass 2 falls and mass 1 rises
    const T = (2 * m1 * m2 * g) / M;
    return {
      kind: 'atwood',
      ok: true,
      g,
      a,
      T,
      M,
      atRest: Math.abs(a) < 1e-12,
      bodies: [
        { name: 'Mass 1', short: '1', m: m1, W: m1 * g, a, Fnet: m1 * a }, // + = up
        { name: 'Mass 2', short: '2', m: m2, W: m2 * g, a, Fnet: m2 * a }, // + = down
      ],
    };
  }

  /* ---------- Chain of blocks on a surface ---------- */

  /**
   * p = {
   *   theta: degrees (0 = level),
   *   g,
   *   friction: bool,                 // friction with the surface
   *   blocks: [{ m, mus, muk }],
   *   links: ['contact' | 'string' | 'none'],  // between neighbors; length blocks.length - 1
   *   longString: bool,               // a string from the first block to the last, over the ones between
   *   hanging: null | { m },          // hung over a pulley at the + end
   *   stack: null | { base, m, mus, muk }, // a block resting on top of blocks[base]; mu is block-on-block
   *   applied: null | { F, angle, block } // block: index, or 'top'; angle: degrees from +s, + tilts away from the surface
   * }
   */
  function buildChain(p) {
    const th = (p.theta || 0) * DEG;
    const g = p.g;
    const appliedOn = (key) => (p.applied && p.applied.block === key && p.applied.F ? p.applied : null);
    const makeBody = (kind, i, name, m, mus, muk, ap) => {
      const W = m * g;
      const onSurface = kind !== 'hanging';
      const Wpar = onSurface ? W * Math.sin(th) : 0;
      const Wperp = onSurface ? W * Math.cos(th) : 0;
      const Fpar = ap ? ap.F * Math.cos(ap.angle * DEG) : 0;
      const Fperp = ap ? ap.F * Math.sin(ap.angle * DEG) : 0;
      return {
        kind,
        index: i,
        name,
        short: String(i + 1),
        m,
        W,
        Wpar,
        Wperp,
        Fpar,
        Fperp,
        ext: kind === 'hanging' ? W : -Wpar + Fpar, // net force along s, before friction and links
        N: kind === 'hanging' ? 0 : Wperp - Fperp, // a base block gets the top block's push added below
        mus,
        muk,
      };
    };
    const bodies = p.blocks.map((b, i) =>
      makeBody('block', i, 'Block ' + (i + 1), b.m, p.friction ? b.mus : 0, p.friction ? b.muk : 0, appliedOn(i))
    );
    const nb = bodies.length;
    const links = [];
    p.links.forEach((type, j) => {
      if (type === 'string' || type === 'contact') links.push({ type, hanging: false, long: false, left: j, right: j + 1 });
    });
    if (p.longString && nb >= 3) {
      links.push({ type: 'string', hanging: false, long: true, left: 0, right: nb - 1 });
    }
    if (p.hanging) {
      const i = bodies.length;
      bodies.push(makeBody('hanging', i, 'Block ' + (i + 1) + ' (hanging)', p.hanging.m, 0, 0, null));
      links.push({ type: 'string', hanging: true, long: false, left: nb - 1, right: i });
    }
    if (p.stack) {
      const i = bodies.length;
      const base = Math.min(Math.max(0, p.stack.base), nb - 1);
      const top = makeBody('top', i, 'Block ' + (i + 1) + ' (on top)', p.stack.m, 0, 0, appliedOn('top'));
      top.base = base;
      bodies.push(top);
      // The top block presses on its base block with the normal force it feels.
      bodies[base].Ntop = top.N;
      bodies[base].N += Math.max(0, top.N);
      links.push({ type: 'stack', hanging: false, long: false, left: base, right: i, mus: p.stack.mus, muk: Math.min(p.stack.muk, p.stack.mus), N: Math.max(0, top.N) });
    }
    return { bodies, links, neighbors: p.links.slice(), theta: p.theta || 0, g, friction: !!p.friction };
  }

  // Sign of a link force on the body to its left / right.
  // A string pulls the two bodies together; a contact pushes them apart.
  // For a stacked pair, F is the friction on the top block (+ along s); the base feels -F.
  function linkSigns(type) {
    return type === 'string' ? { left: 1, right: -1 } : { left: -1, right: 1 };
  }

  // Bodies reachable from `start` through engaged links, without crossing link `cut`.
  function reach(links, engaged, start, cut) {
    const seen = new Set([start]);
    const stack = [start];
    while (stack.length) {
      const i = stack.pop();
      links.forEach((l, j) => {
        if (j === cut || !engaged(j)) return;
        const other = l.left === i ? l.right : l.right === i ? l.left : -1;
        if (other >= 0 && !seen.has(other)) {
          seen.add(other);
          stack.push(other);
        }
      });
    }
    return [...seen].sort((x, y) => x - y);
  }

  /*
   * Solve one guess of each link's state:
   *   'on'    string taut / blocks touching / stacked block sticking
   *   'off'   string slack / blocks apart
   *   1, -1   stacked block slipping forward (+s) or backward relative to its base
   */
  function tryPattern(model, states) {
    const { bodies, links } = model;
    const n = bodies.length;
    const engaged = (j) => states[j] === 'on';

    // Kinetic friction between slipping stacked blocks acts like an outside force on each part.
    const slip = new Array(n).fill(0);
    links.forEach((l, j) => {
      if (l.type !== 'stack' || engaged(j)) return;
      const d = states[j];
      slip[l.right] += -d * l.muk * l.N;
      slip[l.left] += d * l.muk * l.N;
    });
    const ext = bodies.map((b, i) => b.ext + slip[i]);

    const a = new Array(n).fill(0);
    const f = new Array(n).fill(0);
    const fType = new Array(n).fill('none');
    const F = new Array(links.length).fill(0);
    const sides = new Array(links.length).fill(null);
    const groups = [];
    let scale = 1;
    for (const b of bodies) scale = Math.max(scale, Math.abs(b.ext), b.W);

    const done = new Array(n).fill(false);
    for (let i0 = 0; i0 < n; i0++) {
      if (done[i0]) continue;
      const members = reach(links, engaged, i0, -1);
      members.forEach((i) => (done[i] = true));
      let M = 0;
      let D = 0;
      let cap = 0;
      let kin = 0;
      for (const i of members) {
        const b = bodies[i];
        M += b.m;
        D += ext[i];
        cap += b.mus * b.N;
        kin += b.muk * b.N;
      }
      const tol = 1e-9 * scale;
      let ag;
      let state;
      if (Math.abs(D) <= cap + tol) {
        // Stays at rest. Static friction is shared in proportion to each
        // block's maximum (one reasonable choice; the true split is not unique).
        ag = 0;
        state = Math.abs(D) <= tol ? 'balanced' : 'static';
        for (const i of members) {
          const b = bodies[i];
          f[i] = cap > 0 ? (-D * b.mus * b.N) / cap : 0;
          if (b.mus > 0 && b.N > 0) fType[i] = 'static';
        }
      } else {
        const dir = Math.sign(D);
        for (const i of members) {
          const b = bodies[i];
          f[i] = -dir * b.muk * b.N;
          if (b.muk > 0 && b.N > 0) fType[i] = 'kinetic';
        }
        ag = (D - dir * kin) / M;
        state = 'sliding';
      }
      for (const i of members) a[i] = ag;
      groups.push({ members, start: members[0], end: members[members.length - 1], M, D, cap, kin, a: ag, state });
    }

    // Internal link forces: cut the link and apply Newton's 2nd law to the
    // part on its left body's side.
    links.forEach((l, j) => {
      if (!engaged(j)) return;
      const left = reach(links, engaged, l.left, j);
      const right = reach(links, engaged, l.right, j);
      sides[j] = { left, right };
      let mSub = 0;
      let extSub = 0;
      for (const i of left) {
        mSub += bodies[i].m;
        extSub += ext[i] + f[i];
      }
      F[j] = (mSub * a[l.left] - extSub) / linkSigns(l.type).left;
    });

    let valid = true;
    const tolF = 1e-7 * scale;
    links.forEach((l, j) => {
      if (l.type === 'stack') {
        if (engaged(j)) {
          if (Math.abs(F[j]) > l.mus * l.N + tolF) valid = false;
        } else if (states[j] * (a[l.right] - a[l.left]) < -1e-9) valid = false;
        return;
      }
      if (engaged(j)) {
        if (F[j] < -tolF) valid = false;
      } else {
        // A slack string means the bodies are moving closer together;
        // a broken contact means they are moving apart.
        const rel = l.type === 'string' ? a[l.left] - a[l.right] : a[l.right] - a[l.left];
        if (rel < -1e-9) valid = false;
      }
    });
    return { valid, states, a, f, fType, F, sides, groups, slip };
  }

  // Every combination of link states, most-connected first.
  function allStates(links) {
    let out = [[]];
    links.forEach((l) => {
      const opts = l.type === 'stack' ? ['on', 1, -1] : ['on', 'off'];
      const next = [];
      out.forEach((s) => opts.forEach((o) => next.push(s.concat([o]))));
      out = next;
    });
    const on = (s) => s.filter((x) => x === 'on').length;
    return out.sort((x, y) => on(y) - on(x));
  }

  function solveChain(p) {
    const model = buildChain(p);
    const { bodies, links } = model;

    // The connections must not form a loop (that would make the forces indeterminate).
    const parent = bodies.map((_, i) => i);
    const find = (x) => (parent[x] === x ? x : (parent[x] = find(parent[x])));
    for (const l of links) {
      const ra = find(l.left);
      const rb = find(l.right);
      if (ra === rb) {
        return {
          kind: 'chain',
          ok: false,
          error:
            'These connections form a closed loop, so the forces cannot be worked out. With a string from Block 1 to Block ' +
            bodies.filter((b) => b.kind === 'block').length +
            ', set at least one pair of neighbors to Apart.',
          ...model,
        };
      }
      parent[ra] = rb;
    }

    for (const b of bodies) {
      if (b.kind !== 'hanging' && b.N < -1e-9) {
        return {
          kind: 'chain',
          ok: false,
          error:
            b.name +
            ' would lift off ' + (b.kind === 'top' ? 'the block below it' : 'the surface') +
            ': the applied force pulls away harder than the block presses down. Lower the force or its angle.',
          ...model,
        };
      }
      if (b.N < 0) b.N = 0;
    }

    let sol = null;
    const combos = allStates(links);
    for (const st of combos) {
      const r = tryPattern(model, st);
      if (r.valid) {
        sol = r;
        break;
      }
    }
    if (!sol) sol = tryPattern(model, combos[0]);

    bodies.forEach((b, i) => {
      b.a = sol.a[i];
      b.f = sol.f[i];
      b.fType = sol.fType[i];
      b.slip = sol.slip[i];
      b.fMaxStatic = b.mus * b.N;
      b.fKinetic = b.muk * b.N;
      b.Fnet = b.m * b.a;
    });

    links.forEach((l, j) => {
      l.engaged = sol.states[j] === 'on';
      l.sides = sol.sides[j];
      l.indeterminate = false;
      if (l.type === 'stack') {
        // Fs: friction on the top block along s (+/-). The base feels -Fs.
        l.Fs = l.engaged ? sol.F[j] : -sol.states[j] * l.muk * l.N;
        l.F = Math.abs(l.Fs);
        l.fType = l.engaged ? 'static' : 'kinetic';
        l.slipDir = l.engaged ? 0 : sol.states[j];
        if (!l.engaged) {
          // What friction would be needed for the top block to move with its base?
          const stick = sol.states.slice();
          stick[j] = 'on';
          const t = tryPattern(model, stick);
          l.test = { a: t.a[l.left], need: Math.abs(t.F[j]), max: l.mus * l.N };
        } else {
          l.test = { a: sol.a[l.left], need: l.F, max: l.mus * l.N };
        }
        return;
      }
      l.F = l.engaged ? Math.max(0, sol.F[j]) : 0;
      // Held at rest by static friction on both sides: Newton's laws alone
      // cannot pin down how the load is shared, so this force is not unique.
      if (l.engaged && l.sides) {
        const gr = sol.groups.find((g) => g.members.includes(l.left));
        if (gr.state === 'static') {
          const capOf = (list) => list.reduce((s2, i) => s2 + bodies[i].mus * bodies[i].N, 0);
          if (capOf(l.sides.left) > 0 && capOf(l.sides.right) > 0) l.indeterminate = true;
        }
      }
    });

    return {
      kind: 'chain',
      ok: true,
      ...model,
      groups: sol.groups,
      atRest: bodies.every((b) => Math.abs(b.a) < 1e-12),
    };
  }

  /*
   * A block slides from rest a distance d down a ramp with acceleration a (magnitude).
   * At the table edge a smooth lip (or a small kicker ramp) turns its velocity to
   * phi degrees above the horizontal without changing its speed, and it falls to a floor H below.
   * The lip's own size is ignored.
   */
  function launch({ a, d, H, g, phi = 0 }) {
    if (!(a > 0) || !(d > 0)) return null;
    const tRamp = Math.sqrt((2 * d) / a);
    const v0 = a * tRamp;
    const ang = phi * DEG;
    const vx = v0 * Math.cos(ang);
    const vy0 = v0 * Math.sin(ang);
    // H + vy0 t - g t^2 / 2 = 0
    const tAir = (vy0 + Math.sqrt(vy0 * vy0 + 2 * g * H)) / g;
    return {
      tRamp,
      v0,
      phi,
      vx,
      vy0,
      tAir,
      R: vx * tAir,
      hMax: (vy0 * vy0) / (2 * g), // highest point above the table
      tPeak: vy0 / g,
      vLand: Math.sqrt(v0 * v0 + 2 * g * H),
    };
  }

  /* ---------- Elevator: a person on a scale ---------- */
  // Motion choices. Each phase has an acceleration a (+ = up), a duration and a starting velocity.
  const RIDE = { speed: 2.5, cruise: 3 };
  function elevatorPhases(motion, A, g) {
    const t = RIDE.speed;
    switch (motion) {
      case 'up-speed':
        return [{ key: 'up-speed', label: 'Speeding up going up', a: A, dur: t, v0: 0 }];
      case 'up-const':
        return [{ key: 'up-const', label: 'Going up at constant speed', a: 0, dur: RIDE.cruise, v0: 2 }];
      case 'up-slow':
        return [{ key: 'up-slow', label: 'Slowing down going up', a: -A, dur: t, v0: A * t }];
      case 'down-speed':
        return [{ key: 'down-speed', label: 'Speeding up going down', a: -A, dur: t, v0: 0 }];
      case 'down-const':
        return [{ key: 'down-const', label: 'Going down at constant speed', a: 0, dur: RIDE.cruise, v0: -2 }];
      case 'down-slow':
        return [{ key: 'down-slow', label: 'Slowing down going down', a: A, dur: t, v0: -A * t }];
      case 'freefall':
        return [{ key: 'freefall', label: 'Falling freely (cable broken)', a: -g, dur: 1.5, v0: 0 }];
      case 'ride-up':
        return [
          { key: 'up-speed', label: 'Speeding up going up', a: A, dur: t, v0: 0 },
          { key: 'up-const', label: 'Constant speed going up', a: 0, dur: RIDE.cruise, v0: A * t },
          { key: 'up-slow', label: 'Slowing down going up', a: -A, dur: t, v0: A * t },
        ];
      case 'ride-down':
        return [
          { key: 'down-speed', label: 'Speeding up going down', a: -A, dur: t, v0: 0 },
          { key: 'down-const', label: 'Constant speed going down', a: 0, dur: RIDE.cruise, v0: -A * t },
          { key: 'down-slow', label: 'Slowing down going down', a: A, dur: t, v0: -A * t },
        ];
      default:
        return [];
    }
  }

  /** { m: person, M: car (0 = ignore the cable), g, motion, A: size of the acceleration } */
  function solveElevator({ m, M, g, motion, A }) {
    const phases = elevatorPhases(motion, A, g);
    for (const ph of phases) {
      if (ph.a < -g - 1e-9) {
        return {
          kind: 'elevator',
          ok: false,
          error: 'With a downward acceleration larger than g, the person would float off the scale. Use an acceleration of at most ' + g + ' m/s².',
          phases,
        };
      }
      ph.N = Math.max(0, m * (g + ph.a)); // scale reading
      ph.kg = ph.N / g; // what a scale marked in kg would show
      ph.Fnet = m * ph.a;
      ph.T = M > 0 ? (M + m) * (g + ph.a) : null;
    }
    let t0 = 0;
    for (const ph of phases) {
      ph.t0 = t0;
      t0 += ph.dur;
    }
    return { kind: 'elevator', ok: true, m, M, g, A, motion, phases, W: m * g, tTotal: t0 };
  }

  /* ---------- Leg traction ---------- */
  /** Two rope segments leave the foot pulley at th1 above and th2 below the horizontal; T = m g. */
  function solveTraction({ m, th1, th2, g }) {
    const T = m * g;
    const a1 = th1 * DEG;
    const a2 = th2 * DEG;
    const Fx = T * (Math.cos(a1) + Math.cos(a2));
    const Fy = T * (Math.sin(a1) - Math.sin(a2));
    return { kind: 'traction', ok: true, m, g, th1, th2, T, Fx, Fy, F: Math.hypot(Fx, Fy), angle: Math.atan2(Fy, Fx) / DEG };
  }

  /* ---------- Human body: jumping, landing, sprint start ---------- */
  function solveBody({ mode, m, g, d, h, v }) {
    const W = m * g;
    if (mode === 'jump') {
      // push off over distance d, then rise h after leaving the floor
      const v0 = Math.sqrt(2 * g * h);
      const a = (v0 * v0) / (2 * d);
      const F = m * (g + a);
      return { kind: 'body', ok: true, mode, m, g, d, h, W, v: v0, a, F, ratio: F / W, tPush: v0 / a, tUp: v0 / g };
    }
    if (mode === 'land') {
      // fall h onto the floor, then stop while the knees bend through d
      const v0 = Math.sqrt(2 * g * h);
      const a = (v0 * v0) / (2 * d);
      const F = m * (g + a);
      return { kind: 'body', ok: true, mode, m, g, d, h, W, v: v0, a, F, ratio: F / W, tFall: v0 / g, tStop: v0 / a };
    }
    // sprint: reach speed v over distance d
    const a = (v * v) / (2 * d);
    const f = m * a;
    return { kind: 'body', ok: true, mode: 'sprint', m, g, d, v, W, a, f, N: W, Fground: Math.hypot(f, W), groundAngle: Math.atan2(W, f) / DEG, ratio: f / W, t: v / a };
  }

  /* ---------- Equilibrium: a traffic light on two cables ---------- */
  /** Cable 1 rises to the left at th1, cable 2 to the right at th2 (degrees above horizontal). */
  function solveTrafficLight({ m, th1, th2, g }) {
    const W = m * g;
    const a1 = th1 * DEG;
    const a2 = th2 * DEG;
    const s = Math.sin(a1 + a2);
    if (!(s > 1e-9)) return { kind: 'traffic', ok: false, error: 'At least one cable must slant upward, or nothing can hold the light up.' };
    const T1 = (W * Math.cos(a2)) / s;
    const T2 = (W * Math.cos(a1)) / s;
    return {
      kind: 'traffic',
      ok: true,
      m,
      g,
      th1,
      th2,
      W,
      T1,
      T2,
      T1x: T1 * Math.cos(a1),
      T1y: T1 * Math.sin(a1),
      T2x: T2 * Math.cos(a2),
      T2y: T2 * Math.sin(a2),
    };
  }

  /* ---------- A block pressed against a wall ---------- */
  /** Push F at phi degrees above the horizontal, into the wall. + is up. */
  function solveWall({ m, F, phi, mus, muk, g }) {
    const W = m * g;
    const p = phi * DEG;
    const N = F * Math.cos(p);
    const up = F * Math.sin(p) - W; // vertical forces apart from friction
    const cap = mus * N;
    const denomDown = Math.sin(p) + mus * Math.cos(p);
    const denomUp = Math.sin(p) - mus * Math.cos(p);
    const Fmin = denomDown > 1e-12 ? W / denomDown : Infinity; // just stops it sliding down
    const Fmax = denomUp > 1e-12 ? W / denomUp : Infinity; // just starts it sliding up
    const base = { kind: 'wall', ok: true, m, F, phi, mus, muk: Math.min(muk, mus), g, W, N, Fpar: F * Math.sin(p), Fmin, Fmax };
    if (Math.abs(up) <= cap + 1e-9) return { ...base, f: -up, fType: up === 0 ? 'none' : 'static', a: 0, state: 'rest' };
    const dir = Math.sign(up);
    const fk = base.muk * N;
    return { ...base, f: -dir * fk, fType: fk > 0 ? 'kinetic' : 'none', a: (up - dir * fk) / m, state: dir > 0 ? 'up' : 'down' };
  }

  /* ---------- F = ma lab ---------- */
  /** Total mass M is split between the cart and the hanger (m); f is a constant friction force. */
  function labTrial({ M, m, f, g, d }) {
    const F = m * g;
    const a = Math.max(0, (F - f) / M);
    return { m, F, a, t: a > 0 ? Math.sqrt((2 * d) / a) : Infinity, T: (M - m) * a + f };
  }
  /** Least-squares line y = slope * x + intercept. */
  function fitLine(pts) {
    const n = pts.length;
    if (n < 2) return null;
    let sx = 0;
    let sy = 0;
    let sxx = 0;
    let sxy = 0;
    for (const [x, y] of pts) {
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
    }
    const den = n * sxx - sx * sx;
    if (Math.abs(den) < 1e-12) return null;
    const slope = (n * sxy - sx * sy) / den;
    return { slope, intercept: (sy - slope * sx) / n };
  }

  /* ---------- Third-law puzzles ---------- */
  /** Horse pulls a cart. Fg: the ground's forward push on the horse; fc: friction on the cart. */
  function solveHorseCart({ mh, mc, Fg, fc }) {
    const a = Math.max(0, (Fg - fc) / (mh + mc));
    return { a, T: mc * a + fc, net: (mh + mc) * a };
  }
  /** Tug-of-war. Each team pushes on the ground with FA, FB (team A on the left). + = toward A. */
  function solveTug({ mA, mB, FA, FB }) {
    const a = (FA - FB) / (mA + mB);
    return { a, T: FB + mB * a, winner: Math.abs(a) < 1e-12 ? 'tie' : a > 0 ? 'A' : 'B' };
  }
  /** Truck and car collide; the force between them has size F during the crash. */
  function solveCrash({ mt, mcar, F }) {
    return { at: F / mt, ac: F / mcar };
  }

  /* ---------- Accelerometer: an ornament hanging in an accelerating car ---------- */
  /** a: the car's acceleration (+ forward). The string tilts back by theta from vertical. */
  function solveCarAccel({ a, g, m }) {
    return { a, g, m, theta: Math.atan2(Math.abs(a), g) / DEG, T: m * Math.hypot(g, a), Fnet: m * Math.abs(a), W: m * g };
  }

  /* ---------- Tilt-the-ramp friction lab ---------- */
  /** The ramp is raised until the block slips (tan theta = mus), then it slides a distance d. */
  function tiltLab({ mus, muk, g, d }) {
    const th = Math.atan(mus);
    const a = g * (Math.sin(th) - muk * Math.cos(th));
    return { thetaC: th / DEG, a, t: a > 0 ? Math.sqrt((2 * d) / a) : Infinity };
  }

  /* ---------- Block and tackle ---------- */
  /** n rope segments hold the moving pulley block; a is the load's upward acceleration. */
  function solveTackle({ m, n, g, a = 0 }) {
    const T = (m * (g + a)) / n;
    return { m, n, g, a, W: m * g, T, ceiling: (n + 1) * T, MA: n };
  }

  /* ---------- Chaos: pendulums integrated numerically ---------- */
  /*
   * Each system gives the time derivative of its state vector.
   *   single: y = [theta, omega]          (angle from straight down, rad)
   *   double: y = [th1, w1, th2, w2]      (both angles from straight down)
   */
  const PENDULUMS = {
    single: {
      size: 2,
      rhs(p) {
        return (y, out) => {
          out[0] = y[1];
          out[1] = -(p.g / p.L) * Math.sin(y[0]) - (p.b || 0) * y[1];
        };
      },
      // energy per kilogram (the bob's mass cancels)
      energy(p, y) {
        return 0.5 * p.L * p.L * y[1] * y[1] - p.g * p.L * Math.cos(y[0]);
      },
    },
    double: {
      size: 4,
      rhs(p) {
        const { m1, m2, L1, L2, g } = p;
        const M = m1 + m2;
        return (y, out) => {
          const t1 = y[0];
          const w1 = y[1];
          const t2 = y[2];
          const w2 = y[3];
          const d = t2 - t1;
          const s = Math.sin(d);
          const c = Math.cos(d);
          const den1 = M * L1 - m2 * L1 * c * c;
          const den2 = (L2 / L1) * den1;
          out[0] = w1;
          out[1] = (m2 * L1 * w1 * w1 * s * c + m2 * g * Math.sin(t2) * c + m2 * L2 * w2 * w2 * s - M * g * Math.sin(t1)) / den1;
          out[2] = w2;
          out[3] = (-m2 * L2 * w2 * w2 * s * c + M * (g * Math.sin(t1) * c - L1 * w1 * w1 * s - g * Math.sin(t2))) / den2;
        };
      },
      energy(p, y) {
        const { m1, m2, L1, L2, g } = p;
        const [t1, w1, t2, w2] = y;
        const ke = 0.5 * m1 * L1 * L1 * w1 * w1 + 0.5 * m2 * (L1 * L1 * w1 * w1 + L2 * L2 * w2 * w2 + 2 * L1 * L2 * w1 * w2 * Math.cos(t1 - t2));
        const pe = -(m1 + m2) * g * L1 * Math.cos(t1) - m2 * g * L2 * Math.cos(t2);
        return ke + pe;
      },
    },
  };

  /* Driven, damped pendulum: y = [theta, omega, drive phase].
   * theta'' = -w0^2 sin(theta) - (w0/q) theta' + A w0^2 cos(phase), phase' = W w0,
   * where w0 = sqrt(g/L), q is the quality factor, A the drive strength and
   * W the drive frequency, both relative to the natural frequency w0. */
  PENDULUMS.driven = {
    size: 3,
    rhs(p) {
      const w0 = Math.sqrt(p.g / p.L);
      return (y, out) => {
        out[0] = y[1];
        out[1] = -w0 * w0 * Math.sin(y[0]) - (w0 / p.q) * y[1] + p.A * w0 * w0 * Math.cos(y[2]);
        out[2] = p.W * w0;
      };
    },
    energy: (p, y) => 0.5 * p.L * p.L * y[1] * y[1] - p.g * p.L * Math.cos(y[0]),
  };
  /* Lorenz equations: y = [x, y, z]. */
  PENDULUMS.lorenz = {
    size: 3,
    rhs(p) {
      return (y, out) => {
        out[0] = p.sigma * (y[1] - y[0]);
        out[1] = y[0] * (p.rho - y[2]) - y[1];
        out[2] = y[0] * y[1] - p.beta * y[2];
      };
    },
  };
  /* Magnetic pendulum (top view, small swings): y = [x, vx, y, vy].
   * A spring-like pull toward the centre, friction, and three magnets
   * below the plane at height d that attract with strength s. */
  const MAGNETS = [90, 210, 330].map((a) => ({ x: Math.cos((a * Math.PI) / 180), y: Math.sin((a * Math.PI) / 180) }));
  PENDULUMS.magnet = {
    size: 4,
    magnets: MAGNETS,
    rhs(p) {
      const d2 = p.d * p.d;
      return (y, out) => {
        let ax = -p.k * y[0] - p.b * y[1];
        let ay = -p.k * y[2] - p.b * y[3];
        for (const m of MAGNETS) {
          const dx = m.x - y[0];
          const dy = m.y - y[2];
          const r3 = Math.pow(dx * dx + dy * dy + d2, 1.5);
          ax += (p.s * dx) / r3;
          ay += (p.s * dy) / r3;
        }
        out[0] = y[1];
        out[1] = ax;
        out[2] = y[3];
        out[3] = ay;
      };
    },
  };

  /**
   * Fourth-order Runge-Kutta from y0 for time T with step dt.
   * Keeps every `every`-th step: { t: Float64Array, y: [Float64Array per component] }.
   */
  function integrate(f, y0, dt, T, every = 1) {
    const n = y0.length;
    const steps = Math.ceil(T / dt);
    const keep = Math.floor(steps / every) + 1;
    const t = new Float64Array(keep);
    const out = Array.from({ length: n }, () => new Float64Array(keep));
    let y = Float64Array.from(y0);
    const k1 = new Float64Array(n);
    const k2 = new Float64Array(n);
    const k3 = new Float64Array(n);
    const k4 = new Float64Array(n);
    const tmp = new Float64Array(n);
    let k = 0;
    const save = (step) => {
      t[k] = step * dt;
      for (let i = 0; i < n; i++) out[i][k] = y[i];
      k++;
    };
    save(0);
    for (let step = 1; step <= steps; step++) {
      f(y, k1);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + 0.5 * dt * k1[i];
      f(tmp, k2);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + 0.5 * dt * k2[i];
      f(tmp, k3);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + dt * k3[i];
      f(tmp, k4);
      for (let i = 0; i < n; i++) y[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
      if (step % every === 0 && k < keep) save(step);
    }
    return { t: t.subarray(0, k), y: out.map((a) => a.subarray(0, k)) };
  }

  /** Run a pendulum system. Returns samples every `sample` seconds plus the energy drift. */
  function simulatePendulum(sys, p, y0, T, dt = 0.001, sample = 0.01) {
    const S = PENDULUMS[sys];
    const every = Math.max(1, Math.round(sample / dt));
    const run = integrate(S.rhs(p), y0, dt, T, every);
    const last = run.y.map((a) => a[a.length - 1]);
    const E0 = S.energy ? S.energy(p, y0) : null;
    return { ...run, dtSample: dt * every, E0, Eend: S.energy ? S.energy(p, last) : null };
  }

  /* ---------- More chaos tools ---------- */
  function logisticOrbit(r, x0, n) {
    const out = new Float64Array(n + 1);
    out[0] = x0;
    for (let i = 1; i <= n; i++) out[i] = r * out[i - 1] * (1 - out[i - 1]);
    return out;
  }
  /** Values the logistic map settles onto (after a transient). */
  function logisticSettle(r, x0 = 0.2, trans = 1000, keep = 128) {
    let x = x0;
    for (let i = 0; i < trans; i++) x = r * x * (1 - x);
    const out = [];
    for (let i = 0; i < keep; i++) {
      x = r * x * (1 - x);
      out.push(x);
    }
    return out;
  }
  /** How many distinct values (within tol); a period, or Infinity for none found. */
  function countCycle(vals, tol = 1e-4) {
    const uniq = [];
    for (const v of vals) if (!uniq.some((u) => Math.abs(u - v) < tol)) uniq.push(v);
    return uniq.length > 32 ? Infinity : uniq.length;
  }

  /**
   * Bouncing ball on a plate that moves as A sin(w t).
   * p = { A (m), f (Hz), e (restitution), g }. Returns flight segments, impacts,
   * and ballAt(t) for drawing.
   */
  function bounceSim(p, T, start = { t: 0, y: 0.002, v: 0 }, maxImpacts = 1e6) {
    const w = 2 * Math.PI * p.f;
    const yp = (t) => p.A * Math.sin(w * t);
    const vp = (t) => p.A * w * Math.cos(w * t);
    const Gam = (p.A * w * w) / p.g;
    const segs = [];
    const impacts = [];
    let t = start.t;
    let y = Math.max(start.y, yp(t));
    let v = start.v;
    let stuck = false;
    const h = 1 / (p.f * 60);
    while (t < T && impacts.length < maxImpacts) {
      if (stuck) {
        segs.push({ t0: t, stuck: true });
        if (Gam <= 1) break; // the plate never accelerates downward faster than g
        // release where the plate's downward acceleration first exceeds g
        const ph = Math.asin(1 / Gam);
        let n = Math.ceil((w * t - ph) / (2 * Math.PI) - 1e-12);
        let tr = (ph + 2 * Math.PI * n) / w;
        if (tr <= t + 1e-12) tr += (2 * Math.PI) / w;
        t = tr;
        y = yp(t);
        v = vp(t);
        stuck = false;
        continue;
      }
      segs.push({ t0: t, y0: y, v0: v, stuck: false });
      // march forward until the ball is below the plate, then bisect
      const gap = (tt) => y + v * (tt - t) - 0.5 * p.g * (tt - t) * (tt - t) - yp(tt);
      let a = t;
      let b = t + h;
      let found = false;
      for (let k = 0; k < 1e6 && a < T + h; k++) {
        if (gap(b) < 0 && b > t + 1e-9) {
          found = true;
          break;
        }
        a = b;
        b += h;
      }
      if (!found) break;
      for (let k = 0; k < 50; k++) {
        const mid = (a + b) / 2;
        if (gap(mid) < 0) b = mid;
        else a = mid;
      }
      const ti = b;
      const vb = v - p.g * (ti - t);
      const vrel = vb - vp(ti);
      impacts.push({ t: ti, phase: (w * ti) % (2 * Math.PI), vrel: -vrel, v: vb });
      t = ti;
      y = yp(ti);
      v = vp(ti) - p.e * vrel;
      if (-vrel * p.e < 1e-4) stuck = true;
    }
    const ballAt = (tt) => {
      let lo = 0;
      let hi = segs.length - 1;
      while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (segs[mid].t0 <= tt) lo = mid;
        else hi = mid - 1;
      }
      const sg = segs[lo];
      if (!sg) return yp(tt);
      if (sg.stuck) return yp(tt);
      const tau = tt - sg.t0;
      return Math.max(yp(tt), sg.y0 + sg.v0 * tau - 0.5 * p.g * tau * tau);
    };
    return { segs, impacts, ballAt, plateAt: yp, Gam, end: { t, y, v } };
  }

  /** Magnitude spectrum of evenly spaced samples (Hann window), one-sided. */
  function spectrum(vals, dt) {
    let N = 1;
    while (N * 2 <= vals.length) N *= 2;
    const re = new Float64Array(N);
    const im = new Float64Array(N);
    let mean = 0;
    for (let i = 0; i < N; i++) mean += vals[i];
    mean /= N;
    for (let i = 0; i < N; i++) re[i] = (vals[i] - mean) * (0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (N - 1)));
    // iterative radix-2 FFT
    for (let i = 1, j = 0; i < N; i++) {
      let bit = N >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        [re[i], re[j]] = [re[j], re[i]];
        [im[i], im[j]] = [im[j], im[i]];
      }
    }
    for (let len = 2; len <= N; len <<= 1) {
      const ang = (-2 * Math.PI) / len;
      const wr = Math.cos(ang);
      const wi = Math.sin(ang);
      for (let i = 0; i < N; i += len) {
        let cr = 1;
        let ci = 0;
        for (let k = 0; k < len / 2; k++) {
          const ar = re[i + k];
          const ai = im[i + k];
          const br = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
          const bi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
          re[i + k] = ar + br;
          im[i + k] = ai + bi;
          re[i + k + len / 2] = ar - br;
          im[i + k + len / 2] = ai - bi;
          const ncr = cr * wr - ci * wi;
          ci = cr * wi + ci * wr;
          cr = ncr;
        }
      }
    }
    const half = N / 2;
    const freq = new Float64Array(half);
    const mag = new Float64Array(half);
    for (let k = 0; k < half; k++) {
      freq[k] = k / (N * dt);
      mag[k] = Math.hypot(re[k], im[k]) / N;
    }
    return { freq, mag };
  }

  // Single RK4 stepper reused by the long runs below
  function stepper(f, n) {
    const k1 = new Float64Array(n);
    const k2 = new Float64Array(n);
    const k3 = new Float64Array(n);
    const k4 = new Float64Array(n);
    const tmp = new Float64Array(n);
    return (y, dt) => {
      f(y, k1);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + 0.5 * dt * k1[i];
      f(tmp, k2);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + 0.5 * dt * k2[i];
      f(tmp, k3);
      for (let i = 0; i < n; i++) tmp[i] = y[i] + dt * k3[i];
      f(tmp, k4);
      for (let i = 0; i < n; i++) y[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    };
  }
  const wrapAngle = (a) => {
    const t = (a + Math.PI) % (2 * Math.PI);
    return (t < 0 ? t + 2 * Math.PI : t) - Math.PI;
  };

  /** Double pendulum Poincare section: (theta2, omega2) each time theta1 crosses 0 going up. */
  function poincareDouble(p, y0, T, dt = 0.002) {
    const step = stepper(PENDULUMS.double.rhs(p), 4);
    const y = Float64Array.from(y0);
    const pts = [];
    let prev = wrapAngle(y[0]);
    for (let t = 0; t < T; t += dt) {
      const before = Float64Array.from(y);
      step(y, dt);
      const now = wrapAngle(y[0]);
      if (prev < 0 && now >= 0 && Math.abs(now - prev) < 1 && y[1] > 0) {
        const f = -prev / (now - prev);
        pts.push([wrapAngle(before[2] + f * (y[2] - before[2])), before[3] + f * (y[3] - before[3])]);
      }
      prev = now;
    }
    return pts;
  }
  /** Driven pendulum strobe: (theta, omega) once per drive period after a transient. */
  function strobeDriven(p, y0, nTrans, nRec, perPeriod = 400) {
    const w0 = Math.sqrt(p.g / p.L);
    const Tdrive = (2 * Math.PI) / (p.W * w0);
    const dt = Tdrive / perPeriod;
    const step = stepper(PENDULUMS.driven.rhs(p), 3);
    const y = Float64Array.from(y0);
    const pts = [];
    for (let n = 0; n < nTrans + nRec; n++) {
      for (let k = 0; k < perPeriod; k++) step(y, dt);
      if (n >= nTrans) pts.push([wrapAngle(y[0]), y[1]]);
    }
    return { pts, state: Array.from(y) };
  }
  /** Lorenz map: successive maxima of z. */
  function lorenzMaxima(p, y0, T, dt = 0.002) {
    const step = stepper(PENDULUMS.lorenz.rhs(p), 3);
    const y = Float64Array.from(y0);
    const maxima = [];
    let z1 = y[2];
    let z0 = z1;
    for (let t = 0; t < T; t += dt) {
      step(y, dt);
      if (z1 > z0 && z1 >= y[2] && t > 5) maxima.push(z1);
      z0 = z1;
      z1 = y[2];
    }
    const pts = [];
    for (let i = 1; i < maxima.length; i++) pts.push([maxima[i - 1], maxima[i]]);
    return pts;
  }
  /** Time until either arm of the double pendulum flips over the top (Infinity if not by Tmax). */
  function flipTime(p, th1, th2, Tmax, dt = 0.01) {
    // energy check: too little energy and neither arm can ever flip
    const E = PENDULUMS.double.energy(p, [th1, 0, th2, 0]);
    const needLower = -(p.m1 + p.m2) * p.g * p.L1 + p.m2 * p.g * p.L2;
    const needUpper = (p.m1 + p.m2) * p.g * p.L1 - p.m2 * p.g * p.L2;
    if (E < Math.min(needLower, needUpper)) return Infinity;
    const step = stepper(PENDULUMS.double.rhs(p), 4);
    const y = Float64Array.from([th1, 0, th2, 0]);
    for (let t = 0; t < Tmax; t += dt) {
      step(y, dt);
      if (Math.abs(y[0]) > Math.PI || Math.abs(y[2]) > Math.PI) return t;
    }
    return Infinity;
  }
  /** Which magnet captures the magnetic pendulum (it stays close to one for 3 s), and when. */
  function magnetSettle(p, x, y, Tmax = 40, dt = 0.02) {
    const step = stepper(PENDULUMS.magnet.rhs(p), 4);
    const s = Float64Array.from([x, 0, y, 0]);
    let near = -1;
    let since = 0;
    for (let t = 0; t < Tmax; t += dt) {
      step(s, dt);
      let best = -1;
      MAGNETS.forEach((m, i) => {
        if (Math.hypot(m.x - s[0], m.y - s[2]) < 0.35) best = i;
      });
      if (best >= 0 && best === near) {
        if (t - since > 3) return { magnet: best, t: since };
      } else {
        near = best;
        since = t;
      }
    }
    // not captured yet: report the closest magnet
    let best = 0;
    let bd = Infinity;
    MAGNETS.forEach((m, i) => {
      const d = Math.hypot(m.x - s[0], m.y - s[2]);
      if (d < bd) {
        bd = d;
        best = i;
      }
    });
    return { magnet: best, t: Tmax };
  }

  const Physics = { DEG, solveAtwood, solveChain, buildChain, linkSigns, launch, solveElevator, solveTraction, solveBody, solveTrafficLight, solveWall, labTrial, fitLine, solveHorseCart, solveTug, solveCrash, solveCarAccel, tiltLab, solveTackle, PENDULUMS, integrate, simulatePendulum, logisticOrbit, logisticSettle, countCycle, bounceSim, spectrum, stepper, wrapAngle, poincareDouble, strobeDriven, lorenzMaxima, flipTime, magnetSettle };
  if (typeof module !== 'undefined' && module.exports) module.exports = Physics;
  else global.Physics = Physics;
})(typeof window !== 'undefined' ? window : globalThis);
