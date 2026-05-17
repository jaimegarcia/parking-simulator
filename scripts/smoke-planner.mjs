import { readFile } from 'node:fs/promises';
import vm from 'node:vm';

const script = await readFile(new URL('../app.js', import.meta.url), 'utf8');

function noop() {}
const ctx = {
  arc: noop,
  arcTo: noop,
  beginPath: noop,
  clearRect: noop,
  closePath: noop,
  fill: noop,
  fillRect: noop,
  fillText: noop,
  lineTo: noop,
  moveTo: noop,
  restore: noop,
  rotate: noop,
  save: noop,
  setLineDash: noop,
  stroke: noop,
  translate: noop
};

const elements = new Map();
function element(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      id,
      disabled: false,
      textContent: '',
      value: '',
      addEventListener: noop,
      getBoundingClientRect: () => ({ left: 0, top: 0, width: 620, height: 420 }),
      getContext: () => ctx
    });
  }
  return elements.get(id);
}

const sandbox = {
  alert: noop,
  cancelAnimationFrame: noop,
  console,
  document: { getElementById: element },
  matchMedia: () => ({ matches: false }),
  requestAnimationFrame: noop,
  setTimeout: fn => fn()
};

const instrumented = script.replace(
  'reset();\n})();',
  `globalThis.__parkingTest = {
    SPOTS,
    obstacles,
    updateObstacles,
    planPathAStar,
    planParkingPath,
    setTarget(idx) { targetIdx = idx; obstacles.delete(idx); updateObstacles(); },
    setManeuver(next) { maneuver = next; },
    setParkingAngle(deg) { parkingAngleDeg = deg; updateObstacles(); },
    clearObstacles() { obstacles.clear(); updateObstacles(); },
    setObstacles(ids) { obstacles.clear(); ids.forEach(id => { if (id !== targetIdx) obstacles.add(id); }); updateObstacles(); },
    planPathByManeuver,
    targetPose,
    spotPolygon
  };
  reset();
})();`
);

vm.createContext(sandbox);
vm.runInContext(instrumented, sandbox, { filename: 'index.html' });

const api = sandbox.__parkingTest;
const failures = [];
const overlapFailures = [];

function cross(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function lineIntersection(a, b, c, d) {
  const x1 = a.x, y1 = a.y, x2 = b.x, y2 = b.y;
  const x3 = c.x, y3 = c.y, x4 = d.x, y4 = d.y;
  const den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (Math.abs(den) < 1e-9) return b;
  return {
    x: ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den,
    y: ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
  };
}

function clipPolygon(subject, clip) {
  let output = subject;
  for (let i = 0; i < clip.length; i++) {
    const a = clip[i], b = clip[(i + 1) % clip.length];
    const input = output;
    output = [];
    if (!input.length) break;
    for (let j = 0; j < input.length; j++) {
      const curr = input[j], prev = input[(j + input.length - 1) % input.length];
      const currInside = cross(a, b, curr) >= -1e-7;
      const prevInside = cross(a, b, prev) >= -1e-7;
      if (currInside) {
        if (!prevInside) output.push(lineIntersection(prev, curr, a, b));
        output.push(curr);
      } else if (prevInside) {
        output.push(lineIntersection(prev, curr, a, b));
      }
    }
  }
  return output;
}

function polygonArea(poly) {
  let area = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    area += a.x * b.y - b.x * a.y;
  }
  return Math.abs(area) / 2;
}

for (const angle of [0, 30, 45]) {
  api.setParkingAngle(angle);
  for (const start of [0, 4]) {
    for (let idx = start; idx < start + 3; idx++) {
      const area = polygonArea(clipPolygon(api.spotPolygon(api.SPOTS[idx]), api.spotPolygon(api.SPOTS[idx + 1])));
      if (area > .01) overlapFailures.push({ angle, first: idx, second: idx + 1, area });
    }
  }
}

const scenarios = [
  { target: 5, maneuver: 'reverse', obstacles: [] },
  { target: 5, maneuver: 'reverse', obstacles: [0, 1, 3, 4, 6] },
  { target: 6, maneuver: 'reverse', obstacles: [1, 2, 4, 5, 7] },
  { target: 2, maneuver: 'reverse', obstacles: [0, 1, 3, 5, 6] },
  { target: 5, maneuver: 'forward', obstacles: [] },
  { target: 6, maneuver: 'forward', obstacles: [0, 2, 4, 5, 7] },
  { target: 5, maneuver: 'reverse', obstacles: [], angle: 30 },
  { target: 5, maneuver: 'reverse', obstacles: [0, 1, 2, 3, 4, 6, 7], angle: 32 },
  { target: 6, maneuver: 'forward', obstacles: [0, 2, 4, 5, 7], angle: 45 },
  { target: 2, maneuver: 'reverse', obstacles: [0, 1, 3, 5, 6], angle: 45 }
];

for (const scenario of scenarios) {
  api.setParkingAngle(scenario.angle || 0);
  api.setTarget(scenario.target);
  api.setManeuver(scenario.maneuver);
  api.setObstacles(scenario.obstacles);
  const path = api.planParkingPath(scenario.target, scenario.maneuver);
  const target = api.targetPose(api.SPOTS[scenario.target], scenario.maneuver);
  const last = path?.segs.at(-1);
  const finalError = last
    ? Math.hypot(target.x - last.x, target.y - last.y) + Math.abs(
      Math.atan2(Math.sin(target.theta - last.theta), Math.cos(target.theta - last.theta))
    ) * 25
    : Infinity;
  if (!path || path.segs.length < 2 || finalError > 18) {
    failures.push({ ...scenario, finalError });
  }
}

if (failures.length) {
  console.error('Planner failed scenarios:', JSON.stringify(failures, null, 2));
  process.exit(1);
}

if (overlapFailures.length) {
  console.error('Spot overlap failures:', JSON.stringify(overlapFailures, null, 2));
  process.exit(1);
}

console.log(`Planner smoke test passed: ${scenarios.length} scenarios`);
