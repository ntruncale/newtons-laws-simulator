// Run with: node --test tests/
const test = require('node:test');
const assert = require('node:assert/strict');
const P = require('../js/physics.js');

const g = 9.8;
const close = (a, b, tol = 1e-6) => assert.ok(Math.abs(a - b) < tol, `expected ${b}, got ${a}`);
const block = (m, mus = 0, muk = 0) => ({ m, mus, muk });

test('Atwood machine', () => {
  const r = P.solveAtwood({ m1: 3, m2: 5, g });
  close(r.a, (2 * g) / 8);
  close(r.T, (2 * 3 * 5 * g) / 8);
});

test('modified Atwood, frictionless', () => {
  const r = P.solveChain({ theta: 0, g, friction: false, blocks: [block(4)], links: [], hanging: { m: 2 } });
  close(r.bodies[0].a, (2 * g) / 6);
  close(r.links[0].F, (4 * 2 * g) / 6);
});

test('modified Atwood, kinetic friction', () => {
  const r = P.solveChain({ theta: 0, g, friction: true, blocks: [block(4, 0.3, 0.2)], links: [], hanging: { m: 2 } });
  const a = (2 * g - 0.2 * 4 * g) / 6;
  close(r.bodies[0].a, a);
  close(r.links[0].F, 2 * (g - a));
});

test('modified Atwood held by static friction', () => {
  const r = P.solveChain({ theta: 0, g, friction: true, blocks: [block(10, 0.5, 0.4)], links: [], hanging: { m: 2 } });
  assert.ok(r.atRest);
  close(r.links[0].F, 2 * g);
  close(r.bodies[0].f, -2 * g);
  assert.equal(r.bodies[0].fType, 'static');
});

test('incline, frictionless', () => {
  const r = P.solveChain({ theta: 30, g, friction: false, blocks: [block(5)], links: [] });
  close(r.bodies[0].a, -g * 0.5);
  close(r.bodies[0].N, 5 * g * Math.cos(30 * P.DEG));
});

test('incline, static friction holds', () => {
  const r = P.solveChain({ theta: 20, g, friction: true, blocks: [block(5, 0.5, 0.4)], links: [] });
  assert.ok(r.atRest); // tan 20 deg = 0.36 < 0.5
});

test('incline, slides with kinetic friction', () => {
  const th = 40 * P.DEG;
  const r = P.solveChain({ theta: 40, g, friction: true, blocks: [block(5, 0.5, 0.3)], links: [] });
  close(r.bodies[0].a, -g * (Math.sin(th) - 0.3 * Math.cos(th)));
});

test('incline with hanging mass', () => {
  const th = 30 * P.DEG;
  const r = P.solveChain({ theta: 30, g, friction: false, blocks: [block(2)], links: [], hanging: { m: 3 } });
  const a = (3 * g - 2 * g * Math.sin(th)) / 5;
  close(r.bodies[0].a, a);
  close(r.links[0].F, 3 * (g - a));
});

test('two blocks pushed, contact force (3rd law pair)', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(2), block(3)], links: ['contact'],
    applied: { F: 20, angle: 0, block: 0 },
  });
  close(r.bodies[0].a, 4);
  close(r.links[0].F, 12);
  assert.ok(r.links[0].engaged);
});

test('pushed blocks with equal friction: same contact force as frictionless', () => {
  const r = P.solveChain({
    theta: 0, g, friction: true, blocks: [block(2, 0.2, 0.1), block(3, 0.2, 0.1)], links: ['contact'],
    applied: { F: 20, angle: 0, block: 0 },
  });
  close(r.links[0].F, 12);
});

test('pulling the back block away from a contact pair separates them', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(2), block(3)], links: ['contact'],
    applied: { F: 20, angle: 180, block: 0 },
  });
  assert.equal(r.links[0].engaged, false);
  close(r.links[0].F, 0);
  close(r.bodies[0].a, -10);
  close(r.bodies[1].a, 0);
});

test('three blocks pulled by strings', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1), block(2), block(3)], links: ['string', 'string'],
    applied: { F: 12, angle: 0, block: 2 },
  });
  close(r.bodies[0].a, 2);
  close(r.links[0].F, 2);
  close(r.links[1].F, 6);
});

test('pushing a string-linked chain from behind leaves the string slack', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1), block(2)], links: ['string'],
    applied: { F: 10, angle: 0, block: 0 },
  });
  assert.equal(r.links[0].engaged, false);
  close(r.bodies[0].a, 10);
});

test('angled pull changes the normal force', () => {
  const r = P.solveChain({
    theta: 0, g, friction: true, blocks: [block(5, 0.5, 0.3)], links: [],
    applied: { F: 40, angle: 30, block: 0 },
  });
  const N = 5 * g - 40 * 0.5;
  close(r.bodies[0].N, N);
  close(r.bodies[0].a, (40 * Math.cos(30 * P.DEG) - 0.3 * N) / 5);
});

test('lift-off is reported', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1)], links: [],
    applied: { F: 50, angle: 90, block: 0 },
  });
  assert.equal(r.ok, false);
});

test('static chain marks shared-friction tension as indeterminate', () => {
  const r = P.solveChain({
    theta: 0, g, friction: true, blocks: [block(5, 0.5, 0.4), block(5, 0.5, 0.4)], links: ['string'],
    applied: { F: 10, angle: 0, block: 1 },
  });
  assert.ok(r.atRest);
  assert.ok(r.links[0].indeterminate);
});

test('long string from Block 1 to Block 3, Block 2 pushes Block 3', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1), block(2), block(3)], links: ['none', 'contact'], longString: true,
    applied: { F: 12, angle: 0, block: 1 },
  });
  close(r.bodies[0].a, 2);
  close(r.bodies[1].a, 2);
  const contact = r.links.find((l) => l.type === 'contact');
  const str = r.links.find((l) => l.long);
  close(contact.F, 8); // pushes Blocks 3 and 1 together: (3 + 1)(2)
  close(str.F, 2); // pulls Block 1 only: (1)(2)
});

test('long string with friction', () => {
  const r = P.solveChain({
    theta: 0, g, friction: true, blocks: [block(1, 0.2, 0.1), block(2, 0.2, 0.1), block(3, 0.2, 0.1)],
    links: ['none', 'contact'], longString: true, applied: { F: 30, angle: 0, block: 1 },
  });
  const a = (30 - 0.1 * 6 * g) / 6;
  close(r.bodies[2].a, a);
  close(r.links.find((l) => l.long).F, 1 * a + 0.1 * 1 * g);
});

test('pushing Block 2 away from Block 3 leaves Blocks 1 and 3 at rest', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1), block(2), block(3)], links: ['none', 'contact'], longString: true,
    applied: { F: 12, angle: 180, block: 1 },
  });
  close(r.bodies[1].a, -6);
  close(r.bodies[0].a, 0);
  close(r.bodies[2].a, 0);
});

test('a closed loop of connections is reported', () => {
  const r = P.solveChain({
    theta: 0, g, friction: false, blocks: [block(1), block(2), block(3)], links: ['contact', 'contact'], longString: true,
  });
  assert.equal(r.ok, false);
});

const stack = (base, m, mus, muk) => ({ base, m, mus, muk });

test('stacked block rides along when static friction is enough', () => {
  // pull the bottom block (4 kg) with 10 N; top 1 kg, mu_s = 0.5
  const r = P.solveChain({ theta: 0, g, friction: false, blocks: [block(4)], links: [], stack: stack(0, 1, 0.5, 0.3), applied: { F: 10, angle: 0, block: 0 } });
  close(r.bodies[0].a, 2);
  close(r.bodies[1].a, 2);
  const l = r.links[0];
  assert.ok(l.engaged);
  close(l.Fs, 2); // friction on top = m_top a
  close(r.bodies[0].N, 5 * g); // table holds up both
});

test('stacked block slips when the pull is too large', () => {
  const r = P.solveChain({ theta: 0, g, friction: false, blocks: [block(4)], links: [], stack: stack(0, 1, 0.5, 0.3), applied: { F: 40, angle: 0, block: 0 } });
  const l = r.links[0];
  assert.equal(l.engaged, false);
  close(r.bodies[1].a, 0.3 * g); // top dragged by kinetic friction
  close(r.bodies[0].a, (40 - 0.3 * g) / 4);
  close(l.Fs, 0.3 * g);
  close(l.test.need, 8); // would need m_top * (40/5)
});

test('pulling the top block drags the bottom one; table friction included', () => {
  const r = P.solveChain({ theta: 0, g, friction: true, blocks: [block(4, 0.1, 0.1)], links: [], stack: stack(0, 2, 0.6, 0.4), applied: { F: 30, angle: 0, block: 'top' } });
  // together: a = (30 - 0.1*6g)/6 = 4.02; top needs 30 - 2a = 21.96 > 0.6*2g = 11.76 -> slips
  const l = r.links[0];
  assert.equal(l.engaged, false);
  close(r.bodies[1].a, (30 - 0.4 * 2 * g) / 2);
  close(r.bodies[0].a, (0.4 * 2 * g - 0.1 * 6 * g) / 4);
});

test('bottom block held by table friction while the top block slips', () => {
  const r = P.solveChain({ theta: 0, g, friction: true, blocks: [block(4, 0.5, 0.4)], links: [], stack: stack(0, 2, 0.3, 0.2), applied: { F: 30, angle: 0, block: 'top' } });
  close(r.bodies[0].a, 0);
  close(r.bodies[1].a, (30 - 0.2 * 2 * g) / 2);
});

test('stacked blocks slide down a frictionless incline together', () => {
  const r = P.solveChain({ theta: 30, g, friction: false, blocks: [block(3)], links: [], stack: stack(0, 1, 0.5, 0.4) });
  close(r.bodies[0].a, -g / 2);
  close(r.bodies[1].a, -g / 2);
  close(r.links[0].F, 0);
});

test('projectile launch from the ramp', () => {
  const L = P.launch({ a: 4, d: 2, H: 1.2, g });
  close(L.v0, 4);
  close(L.tAir, Math.sqrt(2.4 / g));
  close(L.R, 4 * Math.sqrt(2.4 / g));
});

test('elevator scale readings', () => {
  const up = P.solveElevator({ m: 60, M: 0, g, motion: 'up-speed', A: 2 });
  close(up.phases[0].N, 60 * (g + 2));
  const down = P.solveElevator({ m: 60, M: 0, g, motion: 'down-speed', A: 2 });
  close(down.phases[0].N, 60 * (g - 2));
  const c = P.solveElevator({ m: 60, M: 0, g, motion: 'up-const', A: 2 });
  close(c.phases[0].N, 60 * g);
  const ff = P.solveElevator({ m: 60, M: 0, g, motion: 'freefall', A: 2 });
  close(ff.phases[0].N, 0);
  const ride = P.solveElevator({ m: 60, M: 500, g, motion: 'ride-up', A: 1.5 });
  assert.equal(ride.phases.length, 3);
  close(ride.phases[2].N, 60 * (g - 1.5));
  close(ride.phases[0].T, 560 * (g + 1.5));
  // the ride ends at rest
  const last = ride.phases[2];
  close(last.v0 + last.a * last.dur, 0);
});

test('elevator rejects a downward acceleration bigger than g', () => {
  const r = P.solveElevator({ m: 60, M: 0, g, motion: 'down-speed', A: 12 });
  assert.equal(r.ok, false);
});

test('leg traction', () => {
  // OpenStax-style: 2.0 kg, both segments at 30 degrees above and below
  const r = P.solveTraction({ m: 2, th1: 30, th2: 30, g });
  close(r.T, 2 * g);
  close(r.Fx, 2 * 2 * g * Math.cos(30 * P.DEG));
  close(r.Fy, 0);
  const r2 = P.solveTraction({ m: 3, th1: 40, th2: 10, g });
  close(r2.Fy, 3 * g * (Math.sin(40 * P.DEG) - Math.sin(10 * P.DEG)));
});

test('jumping, landing and sprint start', () => {
  const j = P.solveBody({ mode: 'jump', m: 60, g, d: 0.4, h: 0.5 });
  close(j.v, Math.sqrt(2 * g * 0.5));
  close(j.a, (2 * g * 0.5) / 0.8);
  close(j.F, 60 * (g + j.a));
  const l = P.solveBody({ mode: 'land', m: 70, g, d: 0.5, h: 1 });
  close(l.F, 70 * g * (1 + 1 / 0.5)); // F = mg(1 + h/d)
  const s = P.solveBody({ mode: 'sprint', m: 70, g, d: 10, v: 8 });
  close(s.a, 3.2);
  close(s.f, 224);
});

test('angled launch off the table', () => {
  const L = P.launch({ a: 4, d: 2, H: 1.2, g, phi: 30 });
  close(L.v0, 4);
  close(L.vx, 4 * Math.cos(30 * P.DEG));
  close(L.vy0, 2);
  const t = (2 + Math.sqrt(4 + 2 * g * 1.2)) / g;
  close(L.tAir, t);
  close(L.R, 4 * Math.cos(30 * P.DEG) * t);
  close(L.hMax, 4 / (2 * g));
  close(L.vLand, Math.sqrt(16 + 2 * g * 1.2));
  // phi = 0 matches the horizontal launch
  const L0 = P.launch({ a: 4, d: 2, H: 1.2, g, phi: 0 });
  close(L0.tAir, Math.sqrt(2.4 / g));
});

test('traffic light on two cables', () => {
  // symmetric: each cable carries W / (2 sin theta)
  const r = P.solveTrafficLight({ m: 12, th1: 30, th2: 30, g });
  close(r.T1, (12 * g) / (2 * 0.5));
  close(r.T2, r.T1);
  const r2 = P.solveTrafficLight({ m: 10, th1: 37, th2: 53, g });
  close(r2.T1x, r2.T2x);
  close(r2.T1y + r2.T2y, 10 * g);
  assert.equal(P.solveTrafficLight({ m: 1, th1: 0, th2: 0, g }).ok, false);
});

test('block pressed against a wall', () => {
  // horizontal push: holds if mus F >= mg
  const hold = P.solveWall({ m: 2, F: 50, phi: 0, mus: 0.5, muk: 0.4, g });
  assert.equal(hold.state, 'rest');
  close(hold.f, 2 * g);
  close(hold.Fmin, (2 * g) / 0.5);
  assert.equal(hold.Fmax, Infinity);
  const slide = P.solveWall({ m: 2, F: 30, phi: 0, mus: 0.5, muk: 0.4, g });
  assert.equal(slide.state, 'down');
  close(slide.a, -(2 * g - 0.4 * 30) / 2);
  // angled push: min and max holding forces
  const ang = P.solveWall({ m: 2, F: 30, phi: 60, mus: 0.3, muk: 0.2, g });
  close(ang.Fmin, (2 * g) / (Math.sin(60 * P.DEG) + 0.3 * Math.cos(60 * P.DEG)));
  close(ang.Fmax, (2 * g) / (Math.sin(60 * P.DEG) - 0.3 * Math.cos(60 * P.DEG)));
});

test('F = ma lab: F vs a is a line with slope M and intercept f', () => {
  const pts = [0.05, 0.1, 0.15, 0.2].map((m) => {
    const tr = P.labTrial({ M: 1.2, m, f: 0.15, g, d: 1 });
    return [tr.a, tr.F];
  });
  const fit = P.fitLine(pts);
  close(fit.slope, 1.2);
  close(fit.intercept, 0.15);
});

test('third-law puzzles', () => {
  const hc = P.solveHorseCart({ mh: 500, mc: 300, Fg: 1200, fc: 400 });
  close(hc.a, 1);
  close(hc.T, 700); // the cart pulls back on the horse with 700 N, less than the ground's 1200 N push
  const tug = P.solveTug({ mA: 300, mB: 300, FA: 2400, FB: 1800 });
  assert.equal(tug.winner, 'A');
  close(tug.T, 1800 + 300 * 1);
  const c = P.solveCrash({ mt: 6000, mcar: 1200, F: 60000 });
  close(c.at, 10);
  close(c.ac, 50);
});

test('accelerometer in a car', () => {
  const r = P.solveCarAccel({ a: 3, g, m: 0.2 });
  close(r.theta, Math.atan(3 / g) / P.DEG);
  close(r.T, 0.2 * Math.hypot(g, 3));
  close(r.Fnet, 0.6);
});

test('tilt-the-ramp friction lab', () => {
  const r = P.tiltLab({ mus: 0.6, muk: 0.45, g, d: 1 });
  close(Math.tan(r.thetaC * P.DEG), 0.6);
  const th = Math.atan(0.6);
  close(r.a, g * (Math.sin(th) - 0.45 * Math.cos(th)));
  // recover muk from the measured slide, the way students do
  const a = 2 / (r.t * r.t);
  close(Math.tan(th) - a / (g * Math.cos(th)), 0.45);
});

test('block and tackle', () => {
  const r = P.solveTackle({ m: 120, n: 4, g });
  close(r.T, 120 * g / 4);
  close(r.ceiling, 5 * 120 * g / 4);
  const up = P.solveTackle({ m: 120, n: 3, g, a: 0.5 });
  close(up.T, 120 * (g + 0.5) / 3);
});
