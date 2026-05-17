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
    setTarget(idx) { targetIdx = idx; obstacles.delete(idx); updateObstacles(); },
    setManeuver(next) { maneuver = next; },
    clearObstacles() { obstacles.clear(); updateObstacles(); },
    setObstacles(ids) { obstacles.clear(); ids.forEach(id => { if (id !== targetIdx) obstacles.add(id); }); updateObstacles(); },
    planPathByManeuver,
    targetPose
  };
  reset();
})();`
);

vm.createContext(sandbox);
vm.runInContext(instrumented, sandbox, { filename: 'index.html' });

const api = sandbox.__parkingTest;
const failures = [];
const scenarios = [
  { target: 5, maneuver: 'reverse', obstacles: [] },
  { target: 5, maneuver: 'reverse', obstacles: [0, 1, 3, 4, 6] },
  { target: 6, maneuver: 'reverse', obstacles: [1, 2, 4, 5, 7] },
  { target: 2, maneuver: 'reverse', obstacles: [0, 1, 3, 5, 6] },
  { target: 5, maneuver: 'forward', obstacles: [] },
  { target: 6, maneuver: 'forward', obstacles: [0, 2, 4, 5, 7] }
];

for (const scenario of scenarios) {
  api.setTarget(scenario.target);
  api.setManeuver(scenario.maneuver);
  api.setObstacles(scenario.obstacles);
  const path = api.planPathAStar(scenario.target, scenario.maneuver);
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

console.log(`Planner smoke test passed: ${scenarios.length} scenarios`);
