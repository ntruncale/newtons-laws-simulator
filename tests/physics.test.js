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
