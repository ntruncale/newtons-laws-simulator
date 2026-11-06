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
   *   friction: bool,
   *   blocks: [{ m, mus, muk }],
   *   links: ['contact' | 'string'],  // length blocks.length - 1
   *   hanging: null | { m },          // hung over a pulley at the + end
   *   applied: null | { F, angle, block } // angle: degrees from +s, + tilts away from the surface
   * }
   */
  function buildChain(p) {
    const th = (p.theta || 0) * DEG;
    const g = p.g;
    const bodies = p.blocks.map((b, i) => {
      const W = b.m * g;
      const Wpar = W * Math.sin(th);
      const Wperp = W * Math.cos(th);
      let Fpar = 0;
      let Fperp = 0;
      if (p.applied && p.applied.block === i && p.applied.F) {
        Fpar = p.applied.F * Math.cos(p.applied.angle * DEG);
        Fperp = p.applied.F * Math.sin(p.applied.angle * DEG);
      }
      return {
        kind: 'block',
        index: i,
        name: 'Block ' + (i + 1),
        short: String(i + 1),
        m: b.m,
        W,
        Wpar,
        Wperp,
        Fpar,
        Fperp,
        ext: -Wpar + Fpar, // net force along s, before friction and links
        N: Wperp - Fperp,
        mus: p.friction ? b.mus : 0,
        muk: p.friction ? b.muk : 0,
      };
    });
    const links = p.links.map((type, j) => ({ type, hanging: false, left: j, right: j + 1 }));
    if (p.hanging) {
      const i = bodies.length;
      const W = p.hanging.m * g;
      bodies.push({
        kind: 'hanging',
        index: i,
        name: 'Block ' + (i + 1) + ' (hanging)',
        short: String(i + 1),
        m: p.hanging.m,
        W,
        Wpar: 0,
        Wperp: 0,
        Fpar: 0,
        Fperp: 0,
        ext: W,
        N: 0,
        mus: 0,
        muk: 0,
      });
      links.push({ type: 'string', hanging: true, left: i - 1, right: i });
    }
    return { bodies, links, theta: p.theta || 0, g, friction: !!p.friction };
  }

  // Sign of a link force on the body to its left / right.
  // A string pulls the two bodies together; a contact pushes them apart.
  function linkSigns(type) {
    return type === 'string' ? { left: 1, right: -1 } : { left: -1, right: 1 };
  }

  function popcount(x) {
    let c = 0;
    while (x) {
      c += x & 1;
      x >>= 1;
    }
    return c;
  }

  // Solve one guess of which links are "engaged" (taut string / touching blocks).
  function tryPattern(model, mask) {
    const { bodies, links } = model;
    const n = bodies.length;
    const ranges = [];
    let start = 0;
    for (let j = 0; j < links.length; j++) {
      if (!((mask >> j) & 1)) {
        ranges.push([start, j]);
        start = j + 1;
      }
    }
    ranges.push([start, n - 1]);

    const a = new Array(n).fill(0);
    const f = new Array(n).fill(0);
    const fType = new Array(n).fill('none');
    const F = new Array(links.length).fill(0);
    const groups = [];
    let scale = 1;
    for (const b of bodies) scale = Math.max(scale, Math.abs(b.ext), b.W);

    for (const [s, e] of ranges) {
      let M = 0;
      let D = 0;
      let cap = 0;
      let kin = 0;
      for (let i = s; i <= e; i++) {
        const b = bodies[i];
        M += b.m;
        D += b.ext;
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
        for (let i = s; i <= e; i++) {
          const b = bodies[i];
          f[i] = cap > 0 ? (-D * b.mus * b.N) / cap : 0;
          if (b.mus > 0 && b.N > 0) fType[i] = 'static';
        }
      } else {
        const dir = Math.sign(D);
        for (let i = s; i <= e; i++) {
          const b = bodies[i];
          f[i] = -dir * b.muk * b.N;
          if (b.muk > 0 && b.N > 0) fType[i] = 'kinetic';
        }
        ag = (D - dir * kin) / M;
        state = 'sliding';
      }
      for (let i = s; i <= e; i++) a[i] = ag;

      // Internal link forces: Newton's 2nd law on everything left of the link.
      let mSub = 0;
      let extSub = 0;
      for (let i = s; i < e; i++) {
        mSub += bodies[i].m;
        extSub += bodies[i].ext + f[i];
        F[i] = (mSub * ag - extSub) / linkSigns(links[i].type).left;
      }
      groups.push({ start: s, end: e, M, D, cap, kin, a: ag, state });
    }

    let valid = true;
    const tolF = 1e-7 * scale;
    for (let j = 0; j < links.length; j++) {
      const engaged = (mask >> j) & 1;
      if (engaged) {
        if (F[j] < -tolF) valid = false;
      } else {
        // A slack string means the bodies are moving closer together;
        // a broken contact means they are moving apart.
        const rel = links[j].type === 'string' ? a[j] - a[j + 1] : a[j + 1] - a[j];
        if (rel < -1e-9) valid = false;
      }
    }
    return { valid, mask, a, f, fType, F, groups };
  }

  function solveChain(p) {
    const model = buildChain(p);
    const { bodies, links } = model;

    for (const b of bodies) {
      if (b.kind === 'block' && b.N < -1e-9) {
        return {
          kind: 'chain',
          ok: false,
          error:
            b.name +
            ' would lift off the surface: the applied force pulls away from the surface harder than the block presses into it. Lower the force or its angle.',
          ...model,
        };
      }
      if (b.N < 0) b.N = 0;
    }

    const masks = [];
    for (let m = 0; m < 1 << links.length; m++) masks.push(m);
    masks.sort((x, y) => popcount(y) - popcount(x));
    let sol = null;
    for (const m of masks) {
      const r = tryPattern(model, m);
      if (r.valid) {
        sol = r;
        break;
      }
    }
    if (!sol) sol = tryPattern(model, (1 << links.length) - 1);

    bodies.forEach((b, i) => {
      b.a = sol.a[i];
      b.f = sol.f[i];
      b.fType = sol.fType[i];
      b.fMaxStatic = b.mus * b.N;
      b.fKinetic = b.muk * b.N;
      b.Fnet = b.m * b.a;
    });

    links.forEach((l, j) => {
      l.engaged = !!((sol.mask >> j) & 1);
      l.F = l.engaged ? Math.max(0, sol.F[j]) : 0;
      l.indeterminate = false;
    });

    // In a group held at rest by static friction, a link with friction on both
    // sides carries a force that Newton's laws alone cannot pin down.
    for (const gr of sol.groups) {
      if (gr.state !== 'static') continue;
      for (let j = gr.start; j < gr.end; j++) {
        let capL = 0;
        let capR = 0;
        for (let i = gr.start; i <= j; i++) capL += bodies[i].mus * bodies[i].N;
        for (let i = j + 1; i <= gr.end; i++) capR += bodies[i].mus * bodies[i].N;
        if (capL > 0 && capR > 0) links[j].indeterminate = true;
      }
    }

    return {
      kind: 'chain',
      ok: true,
      ...model,
      groups: sol.groups,
      atRest: bodies.every((b) => Math.abs(b.a) < 1e-12),
    };
  }

  const Physics = { DEG, solveAtwood, solveChain, buildChain, linkSigns };
  if (typeof module !== 'undefined' && module.exports) module.exports = Physics;
  else global.Physics = Physics;
})(typeof window !== 'undefined' ? window : globalThis);
