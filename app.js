(function(){
const CV=document.getElementById('pc');
const ctx=CV.getContext('2d');
const CW=500, CH=265;
CV.width=CW; CV.height=CH;

const CAR_LEN=68, CAR_WID=28, WB=46, RA_BACK=12, RA_FRONT=RA_BACK+WB;
const SPOT_W=CAR_WID * 1.25, SPOT_H=CAR_LEN * 1.25, TOP_Y=10;
const BOT_Y=CH-15-SPOT_H, LANE_TOP=TOP_Y+SPOT_H, LANE_BOT=BOT_Y;
const LANE_H=LANE_BOT-LANE_TOP, LANE_MID=(LANE_TOP+LANE_BOT)/2;
const NCOLS=4, BASE_PITCH=SPOT_W;
const PARKED_CAR_LINE_GAP=12;

const SPOTS=[];
for(let i=0;i<4;i++) SPOTS.push({idx:i, col:i, cy:TOP_Y+SPOT_H/2, row:'top'});
for(let i=0;i<4;i++) SPOTS.push({idx:4+i, col:i, cy:BOT_Y+SPOT_H/2, row:'bot'});

const WW=5, WH=12;
const STEERING_WHEEL_RATIO=14;
const MAX_STEERING_WHEEL_TURN=540;
const METERS_PER_UNIT=4.7 / CAR_LEN;
const FEET_PER_METER=3.28084;

let car={x:0,y:0,heading:0,steerAngle:0};
let traj=[], path=null, dist=0, state='idle';
let raf=null, lastT=null;
let targetIdx=5, maneuver='reverse-direct';
let obstacles=new Set(), obsPolys=[];
let speedScale=100;
let parkingAngleDeg=0;

const MANEUVER_INFO = {
  'reverse-direct': {
    reverse: true,
    angled: false,
    cue: 'Mirror to boundary, swing left until the corner shows, then reverse right and straighten.'
  },
  'forward-direct': {
    reverse: false,
    angled: false,
    cue: 'Stay wide, mirror to adjacent headlight, turn right, then square up in the space.'
  },
  'reverse-angled': {
    reverse: true,
    angled: true,
    defaultAngle: 35,
    cue: 'Signal, pass the space, stop just beyond it, then reverse-turn into the angle and straighten.'
  },
  'forward-angled': {
    reverse: false,
    angled: true,
    defaultAngle: 35,
    cue: 'Stay wide, bumper to the near line, turn into the angled bay, then straighten ahead.'
  }
};

function normalizeManeuver(value) {
  if(value === 'reverse') return 'reverse-direct';
  if(value === 'forward') return 'forward-direct';
  return MANEUVER_INFO[value] ? value : 'reverse-direct';
}

function maneuverInfo(value = maneuver) {
  return MANEUVER_INFO[normalizeManeuver(value)];
}

function isReverseManeuver(value = maneuver) {
  return maneuverInfo(value).reverse;
}

function startPos(){ return {x:30, y:LANE_MID, heading:0}; }

function normAngle(theta) {
  return Math.atan2(Math.sin(theta), Math.cos(theta));
}

function parkingAngleRad() {
  return parkingAngleDeg * Math.PI / 180;
}

function spotPitch() {
  const cos = Math.cos(parkingAngleRad());
  return BASE_PITCH / Math.max(.7, cos);
}

function spotMouthLeftX(sp) {
  const pitch = spotPitch();
  const margin = (CW - NCOLS * pitch) / 2;
  return margin + sp.col * pitch;
}

function spotInwardTheta(sp) {
  const a = parkingAngleRad();
  return sp.row === 'top' ? -Math.PI / 2 - a : Math.PI / 2 + a;
}

function spotAislePoint(sp) {
  return {x: spotMouthLeftX(sp) + spotPitch() / 2, y: sp.row === 'top' ? LANE_TOP : LANE_BOT};
}

function pointFromSpotAisle(sp, distance) {
  const t = spotInwardTheta(sp);
  const a = spotAislePoint(sp);
  return {
    x: a.x + Math.cos(t) * distance,
    y: a.y + Math.sin(t) * distance
  };
}

function spotCenter(sp) {
  return pointFromSpotAisle(sp, SPOT_H / 2);
}

function spotLabelPoint(sp) {
  return pointFromSpotAisle(sp, SPOT_H * .48);
}

function spotPolygon(sp) {
  const t = spotInwardTheta(sp);
  const y = sp.row === 'top' ? LANE_TOP : LANE_BOT;
  const ix = Math.cos(t), iy = Math.sin(t);
  const left = {x: spotMouthLeftX(sp), y};
  const right = {x: left.x + spotPitch(), y};
  return [
    left,
    right,
    {x: right.x + ix * SPOT_H, y: right.y + iy * SPOT_H},
    {x: left.x + ix * SPOT_H, y: left.y + iy * SPOT_H}
  ];
}

function pointInPoly(point, poly) {
  let inside = false;
  for(let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const pi = poly[i], pj = poly[j];
    const crosses = (pi.y > point.y) !== (pj.y > point.y);
    if(crosses && point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x) inside = !inside;
  }
  return inside;
}

function getCarPoly(x, y, theta) {
  const cos = Math.cos(theta), sin = Math.sin(theta);
  const hw = CAR_WID / 2 + 1;
  return [
    {x: x + cos*RA_FRONT - sin*hw, y: y + sin*RA_FRONT + cos*hw},
    {x: x + cos*RA_FRONT + sin*hw, y: y + sin*RA_FRONT - cos*hw},
    {x: x - cos*RA_BACK + sin*hw, y: y - sin*RA_BACK - cos*hw},
    {x: x - cos*RA_BACK - sin*hw, y: y - sin*RA_BACK + cos*hw}
  ];
}

function polysIntersect(p1, p2) {
  for (let poly of [p1, p2]) {
    for (let i = 0; i < poly.length; i++) {
      let a = poly[i], b = poly[(i + 1) % poly.length];
      let normal = {x: b.y - a.y, y: a.x - b.x};
      let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;
      for (let p of p1) { let proj = p.x*normal.x + p.y*normal.y; minA=Math.min(minA,proj); maxA=Math.max(maxA,proj); }
      for (let p of p2) { let proj = p.x*normal.x + p.y*normal.y; minB=Math.min(minB,proj); maxB=Math.max(maxB,proj); }
      if (maxA < minB || maxB < minA) return false;
    }
  }
  return true;
}

function updateObstacles() {
  obsPolys = [];
  SPOTS.forEach(sp => {
    if(obstacles.has(sp.idx) && sp.idx !== targetIdx) {
      const h = spotInwardTheta(sp);
      const pos = pointFromSpotAisle(sp, RA_BACK + PARKED_CAR_LINE_GAP);
      obsPolys.push(getCarPoly(pos.x, pos.y, h));
    }
  });
}

class MinHeap {
  constructor() { this.data = []; }
  push(val) { this.data.push(val); this.up(this.data.length - 1); }
  pop() {
    if(this.data.length === 0) return null;
    let top = this.data[0], bot = this.data.pop();
    if(this.data.length > 0) { this.data[0] = bot; this.down(0); }
    return top;
  }
  up(i) {
    while(i > 0) {
      let p = Math.floor((i - 1) / 2);
      if(this.data[p].f <= this.data[i].f) break;
      [this.data[p], this.data[i]] = [this.data[i], this.data[p]]; i = p;
    }
  }
  down(i) {
    let l = this.data.length;
    while(true) {
      let min = i, left = 2*i + 1, right = 2*i + 2;
      if(left < l && this.data[left].f < this.data[min].f) min = left;
      if(right < l && this.data[right].f < this.data[min].f) min = right;
      if(min === i) break;
      [this.data[i], this.data[min]] = [this.data[min], this.data[i]]; i = min;
    }
  }
}

function arcDiff(a, b) {
  let d = b - a;
  while(d > Math.PI) d -= 2*Math.PI;
  while(d < -Math.PI) d += 2*Math.PI;
  return d;
}

function targetPose(sp, maneuver) {
  const reverse = isReverseManeuver(maneuver);
  const distance = reverse
    ? (RA_FRONT + SPOT_H - RA_BACK) / 2
    : (RA_BACK + SPOT_H - RA_FRONT) / 2;
  const pos = pointFromSpotAisle(sp, distance);
  const inward = spotInwardTheta(sp);
  return {
    x: pos.x,
    y: pos.y,
    theta: normAngle(reverse ? inward + Math.PI : inward)
  };
}

function planPathAStar(spotIdx, maneuver) {
  const sp = SPOTS[spotIdx];
  const reverse = isReverseManeuver(maneuver);
  const target = targetPose(sp, maneuver);
  const tX = target.x;
  const tY = target.y;
  const tTheta = target.theta;
  const s = startPos();
  
  let open = new MinHeap();
  let closed = new Set();
  open.push({x: s.x, y: s.y, theta: s.heading, g: 0, f: 0, gear: 1, steer: 0, parent: null});

  let best = null, nearest = null, nearestScore = Infinity, iters = 0;
  const STEP = reverse ? 7 : 4, MAX_STEER = 0.64;
  const STEERS = reverse
    ? [-MAX_STEER, -MAX_STEER * .48, 0, MAX_STEER * .48, MAX_STEER]
    : [-MAX_STEER, -MAX_STEER * .72, -MAX_STEER * .42, 0, MAX_STEER * .42, MAX_STEER * .72, MAX_STEER];
  const GOAL_DIST = reverse ? 15 : 10, GOAL_ANGLE = reverse ? 0.38 : 0.24;
  const MAX_ITERS = reverse ? 90000 : 180000;
  const pitch = spotPitch();
  const spotX = spotAislePoint(sp).x;
  let wpX = reverse ? spotX + pitch * .38 : spotX - pitch * .38;
  let wpY = sp.row === 'bot' ? LANE_TOP + 24 : LANE_BOT - 24;

  while(open.data.length > 0 && iters < MAX_ITERS) {
    iters++;
    let curr = open.pop();
    let dT = Math.hypot(tX - curr.x, tY - curr.y);
    let aD = Math.abs(arcDiff(curr.theta, tTheta));
    let score = dT + aD * 34;

    if(score < nearestScore) {
      nearestScore = score;
      nearest = curr;
    }

    if(dT < GOAL_DIST && aD < GOAL_ANGLE) { best = curr; break; }

    let key = `${Math.round(curr.x/4)},${Math.round(curr.y/4)},${Math.round(curr.theta/0.12)},${curr.gear}`;
    if(closed.has(key)) continue;
    closed.add(key);

    for(let gear of [1, -1]) {
      for(let steer of STEERS) {
        let nx = curr.x + gear * STEP * Math.cos(curr.theta);
        let ny = curr.y + gear * STEP * Math.sin(curr.theta);
        let ntheta = curr.theta + gear * STEP * Math.tan(steer) / WB;
        ntheta = Math.atan2(Math.sin(ntheta), Math.cos(ntheta));

        if(nx < 10 || nx > CW-10 || ny < TOP_Y || ny > BOT_Y+SPOT_H) continue;

        let poly = getCarPoly(nx, ny, ntheta);
        let coll = false;
        for(let op of obsPolys) { if(polysIntersect(poly, op)) { coll = true; break; } }
        if(coll) continue;

        let cost = curr.g + STEP;
        if(gear !== curr.gear) cost += reverse ? 90 : 36;
        if(steer !== 0) cost += Math.abs(steer) * 4;

        let f = cost;
        let ndT = Math.hypot(tX - nx, tY - ny);
        let nAD = Math.abs(arcDiff(ntheta, tTheta));

        if (reverse) {
          if (curr.gear === 1 && nx < wpX - 8) f += Math.hypot(wpX - nx, LANE_MID - ny) + Math.hypot(tX - wpX, tY - LANE_MID);
          else f += ndT * 1.15;
          if (ndT < 86) {
            f += nAD * 78;
            if (gear === 1) f += 420;
          }
        } else {
          if (curr.gear === 1 && nx < wpX - 8) f += Math.hypot(wpX - nx, wpY - ny) + Math.hypot(tX - wpX, tY - wpY);
          else f += ndT * 1.1;
          if (ndT < 78) {
            f += nAD * 78;
            if (gear === -1) f += 80; 
          }
        }
        open.push({x: nx, y: ny, theta: ntheta, g: cost, f: f, gear: gear, steer: steer, parent: curr});
      }
    }
  }

  if(!best && nearestScore < 42) best = nearest;
  if(!best) return planPathByManeuver(spotIdx, maneuver);

  let p = [];
  while(best) { p.push(best); best = best.parent; }
  p.reverse();

  const last = p[p.length - 1];
  const finalPoly = getCarPoly(tX, tY, tTheta);
  const finalBlocked = obsPolys.some(op => polysIntersect(finalPoly, op));
  if(!finalBlocked && Math.hypot(tX - last.x, tY - last.y) < 32) {
    p.push({x: tX, y: tY, theta: tTheta, g: last.g, f: last.f, gear: last.gear, steer: 0, parent: last, label: last.gear === 1 ? 'Forward' : 'Reverse'});
  }
  
  for(let i=1; i<p.length; i++) p[i].label = p[i].gear === 1 ? 'Forward' : 'Reverse';
  return finalizePath(p, STEP);
}

function bezierPoint(p0, p1, p2, p3, t) {
  const u = 1 - t;
  return {
    x: u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
    y: u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y
  };
}

function buildCurve(from, to, fromHeading, toHeading, gear, label, handleA, handleB, count) {
  const p0 = {x: from.x, y: from.y};
  const p3 = {x: to.x, y: to.y};
  const startMotion = gear === 1 ? fromHeading : fromHeading + Math.PI;
  const endMotion = gear === 1 ? toHeading : toHeading + Math.PI;
  const p1 = {x: p0.x + Math.cos(startMotion) * handleA, y: p0.y + Math.sin(startMotion) * handleA};
  const p2 = {x: p3.x - Math.cos(endMotion) * handleB, y: p3.y - Math.sin(endMotion) * handleB};
  const pts = [];

  for(let i = 0; i <= count; i++) {
    const t = i / count;
    const pt = bezierPoint(p0, p1, p2, p3, t);
    const nt = Math.min(1, (i + .5) / count);
    const look = bezierPoint(p0, p1, p2, p3, nt);
    const motionHeading = Math.atan2(look.y - pt.y, look.x - pt.x);
    pts.push({
      x: pt.x,
      y: pt.y,
      theta: i === count ? toHeading : (gear === 1 ? motionHeading : motionHeading + Math.PI),
      gear,
      steer: 0,
      label
    });
  }

  return pts;
}

function pathClear(segs) {
  for(const p of segs) {
    if(p.x < 8 || p.x > CW - 8 || p.y < TOP_Y || p.y > BOT_Y + SPOT_H) return false;
    const poly = getCarPoly(p.x, p.y, p.theta);
    for(const op of obsPolys) {
      if(polysIntersect(poly, op)) return false;
    }
  }
  return true;
}

function finalizePath(segs, fallbackStep) {
  const distances = [0];
  for(let i = 1; i < segs.length; i++) {
    const prev = segs[i - 1], curr = segs[i];
    distances[i] = distances[i - 1] + Math.hypot(curr.x - prev.x, curr.y - prev.y);
  }

  return {
    segs,
    totalLen: distances[distances.length - 1] || Math.max(1, segs.length - 1) * fallbackStep,
    step: fallbackStep,
    distances
  };
}

function appendCurve(segs, curve) {
  if(!segs.length) return curve.slice();
  return segs.concat(curve.slice(1));
}

function applyPhaseLabels(curve, phases) {
  const last = Math.max(1, curve.length - 1);
  curve.forEach((point, idx) => {
    const t = idx / last;
    const phase = phases.find(item => t <= item.until) || phases[phases.length - 1];
    point.label = phase.label;
  });
  return curve;
}

function easeCurveEndpointHeading(curve, count = 10) {
  if(curve.length < 3) return curve;
  const lastIdx = curve.length - 1;
  const firstIdx = Math.max(0, lastIdx - count);
  const fromTheta = curve[firstIdx].theta;
  const toTheta = curve[lastIdx].theta;
  for(let i = firstIdx + 1; i <= lastIdx; i++) {
    const t = (i - firstIdx) / (lastIdx - firstIdx);
    const eased = t * t * (3 - 2 * t);
    curve[i].theta = normAngle(fromTheta + arcDiff(fromTheta, toTheta) * eased);
  }
  return curve;
}

function targetDistanceFor(maneuver) {
  return isReverseManeuver(maneuver)
    ? (RA_FRONT + SPOT_H - RA_BACK) / 2
    : (RA_BACK + SPOT_H - RA_FRONT) / 2;
}

function laneFarFromSpotY(sp) {
  return sp.row === 'bot' ? LANE_TOP + 24 : LANE_BOT - 24;
}

function markSteering(segs) {
  for(let i = 1; i < segs.length; i++) {
    const prev = segs[i - 1];
    const curr = segs[i];
    curr.steer = Math.max(-0.64, Math.min(0.64, arcDiff(prev.theta, curr.theta) * 1.8));
  }
  for(let pass = 0; pass < 2; pass++) {
    const nextSteers = segs.map(seg => seg.steer || 0);
    for(let i = 1; i < segs.length - 1; i++) {
      if(segs[i - 1].gear !== segs[i].gear || segs[i + 1].gear !== segs[i].gear) continue;
      nextSteers[i] = (segs[i - 1].steer + segs[i].steer * 2 + segs[i + 1].steer) / 4;
    }
    for(let i = 1; i < segs.length - 1; i++) segs[i].steer = nextSteers[i];
  }
  segs.forEach(seg => {
    if(seg.label && seg.label.indexOf('Straight') === 0) seg.steer = 0;
  });
  return segs;
}

function tryVideoPath(segs) {
  markSteering(segs);
  return pathClear(segs) ? finalizePath(segs, 7) : null;
}

function appendKinematicSegment(segs, gear, steer, count, label, step = 3.5) {
  const start = segs[segs.length - 1];
  let x = start.x, y = start.y, theta = start.theta;
  for(let i = 0; i < count; i++) {
    x += gear * step * Math.cos(theta);
    y += gear * step * Math.sin(theta);
    theta = normAngle(theta + gear * step * Math.tan(steer) / WB);
    segs.push({x, y, theta, gear, steer, label});
  }
  return segs;
}

function appendKinematicTurnToHeading(segs, gear, steer, targetHeading, label, step = 3.5, maxCount = 90) {
  let prev = Math.abs(arcDiff(segs[segs.length - 1].theta, targetHeading));
  for(let i = 0; i < maxCount; i++) {
    appendKinematicSegment(segs, gear, steer, 1, label, step);
    const curr = Math.abs(arcDiff(segs[segs.length - 1].theta, targetHeading));
    if(curr < .035 || curr > prev + .01) break;
    prev = curr;
  }
  const last = segs[segs.length - 1];
  last.theta = targetHeading;
  return segs;
}

function appendStraightToPoint(segs, target, gear, label, step = 3.5) {
  const start = segs[segs.length - 1];
  const distance = Math.hypot(target.x - start.x, target.y - start.y);
  const count = Math.max(2, Math.ceil(distance / step));
  for(let i = 1; i <= count; i++) {
    const t = i / count;
    segs.push({
      x: start.x + (target.x - start.x) * t,
      y: start.y + (target.y - start.y) * t,
      theta: target.theta,
      gear,
      steer: 0,
      label
    });
  }
  return segs;
}

function signedDistanceAlongHeading(from, to, heading) {
  return (to.x - from.x) * Math.cos(heading) + (to.y - from.y) * Math.sin(heading);
}

function lateralDistanceToHeadingLine(point, linePoint, heading) {
  const dx = point.x - linePoint.x;
  const dy = point.y - linePoint.y;
  return Math.abs(dx * -Math.sin(heading) + dy * Math.cos(heading));
}

function reverseDirectStaysInAisle(segs, sp) {
  const minY = LANE_TOP + CAR_WID * .45;
  const maxY = LANE_BOT - CAR_WID * .45;
  for(const seg of segs) {
    if(seg.label === 'Straight reverse') continue;
    if(sp.row === 'bot' && seg.y < minY) return false;
    if(sp.row === 'top' && seg.y > maxY) return false;
  }
  return true;
}

function pathSmoothnessScore(pathData) {
  if(!pathData) return Infinity;
  let score = pathData.totalLen * .08;
  let prevTurn = 0;
  for(let i = 1; i < pathData.segs.length; i++) {
    const prev = pathData.segs[i - 1];
    const curr = pathData.segs[i];
    const turn = arcDiff(prev.theta, curr.theta);
    score += Math.abs(turn) * 18;
    if(Math.abs(turn) > .001 && Math.abs(prevTurn) > .001 && Math.sign(turn) !== Math.sign(prevTurn)) {
      score += curr.gear === -1 ? 52 : 16;
    }
    if(Math.abs(turn) > .16) score += 120;
    prevTurn = Math.abs(turn) > .001 ? turn : prevTurn;
  }
  return score;
}

function planReverseDirect(sp, target) {
  const start = startPos();
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const pitch = spotPitch();
  const spotRef = spotAislePoint(sp);
  const targetHeading = target.theta;
  const forwardXs = [spotRef.x - pitch * 1.15, spotRef.x - pitch * .92, spotRef.x - pitch * 1.38, spotRef.x - pitch * .68, spotRef.x - pitch * .44, spotRef.x - pitch * .22];
  const forwardYs = [LANE_MID, LANE_MID + rowDir * 8, LANE_MID + rowDir * 14];
  const forwardCounts = [4, 8, 12, 16, 20];
  const reverseSteers = [0.64, 0.58, 0.52, 0.46].map(steer => rowDir * steer);
  let best = null;
  let bestScore = Infinity;

  for(const forwardX of forwardXs) {
    for(const forwardY of forwardYs) {
      for(const forwardCount of forwardCounts) {
        for(const reverseSteer of reverseSteers) {
          const forward = {x: Math.max(34, Math.min(CW - 34, forwardX)), y: forwardY};
          let segs = buildCurve(start, forward, 0, 0, 1, 'Go forward', Math.max(12, forward.x - start.x), 18, 26);
          appendKinematicSegment(segs, 1, -rowDir * .62, forwardCount, 'Left-forward', 3.2);
          appendKinematicTurnToHeading(segs, -1, reverseSteer, targetHeading, 'Right-reverse align', 3.1, 80);
          const handoff = segs[segs.length - 1];
          const centerlineError = Math.abs(handoff.x - target.x);
          const reverseRemaining = -((target.x - handoff.x) * Math.cos(targetHeading) + (target.y - handoff.y) * Math.sin(targetHeading));
          if(centerlineError > 15 || reverseRemaining < 10 || !reverseDirectStaysInAisle(segs, sp)) continue;
          appendStraightToPoint(segs, target, -1, 'Straight reverse', 3.2);
          const path = tryVideoPath(segs);
          if(path) {
            let score = pathSmoothnessScore(path);
            const shift = path.segs.find(seg => seg.gear === -1);
            if(shift) {
              score += Math.abs(shift.x - (spotRef.x + pitch * 1.24)) * .4;
              score += Math.abs(shift.y - (LANE_MID - rowDir * 12)) * .25;
            }
            score += centerlineError * 8;
            score += Math.abs(reverseRemaining - 42) * .8;
            if(score < bestScore) {
              best = path;
              bestScore = score;
            }
          }
        }
      }
    }
  }
  return best;
}

function planReverseDirectFallback(sp, target) {
  const start = startPos();
  const targetHeading = target.theta;
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const pitch = spotPitch();
  const spotRef = spotAislePoint(sp);
  const offsets = [pitch * .7, pitch * .48, pitch * .92, pitch * .3, pitch * 1.12];
  const sideYs = [LANE_MID, LANE_MID - rowDir * 10, LANE_MID + rowDir * 10, LANE_MID - rowDir * 22];

  for(const offset of offsets) {
    for(const y of sideYs) {
      const stage = {x: Math.max(32, Math.min(CW - 32, target.x + offset)), y};
      const approachHandle = Math.max(12, Math.min(90, stage.x - start.x));
      const first = buildCurve(start, stage, 0, -rowDir * .34, 1, 'Left-forward', approachHandle, 32, 40);
      const second = buildCurve(stage, target, -rowDir * .34, targetHeading, -1, 'Right-reverse align', 68, 36, 58);
      const segs = appendCurve(first, second);
      const path = tryVideoPath(segs);
      if(path) return path;
    }
  }

  return null;
}

function planForwardDirect(sp, target) {
  const start = startPos();
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const pitch = spotPitch();
  const spotRef = spotAislePoint(sp);
  const targetHeading = target.theta;
  const wideYs = [laneFarFromSpotY(sp), LANE_MID - rowDir * 16, LANE_MID - rowDir * 24];
  const alignXs = [spotRef.x - pitch * 1.18, spotRef.x - pitch, spotRef.x - pitch * 1.42, spotRef.x - pitch * .78];

  for(const alignX of alignXs) {
    for(const y of wideYs) {
      const align = {x: Math.max(34, Math.min(CW - 34, alignX)), y};
      let segs = buildCurve(start, align, 0, 0, 1, 'Wide approach', Math.max(12, align.x - start.x), 24, 26);
      const turnIn = applyPhaseLabels(
        buildCurve(align, target, 0, targetHeading, 1, 'Full-right turn', 112, 46, 82),
        [
          {until: .68, label: 'Full-right turn'},
          {until: 1, label: 'Square up'}
        ]
      );
      segs = appendCurve(segs, turnIn);
      const path = tryVideoPath(segs);
      if(path) return path;
    }
  }
  return null;
}

function planReverseAngled(sp, target) {
  const start = startPos();
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const pitch = spotPitch();
  const spotRef = spotAislePoint(sp);
  const targetHeading = target.theta;
  const stopXs = [spotRef.x + pitch * 1.25, spotRef.x + pitch * 1.05, spotRef.x + pitch * 1.48, spotRef.x + pitch * .82];
  const stopYs = [LANE_MID, laneFarFromSpotY(sp), LANE_MID - rowDir * 8, LANE_MID + rowDir * 6];
  const reverseSteers = [0.64, 0.58, 0.52, 0.46].map(steer => rowDir * steer);
  const reverseSteps = [3.1, 2.8, 3.4];
  let best = null;
  let bestScore = Infinity;

  for(const stopX of stopXs) {
    for(const stopY of stopYs) {
      for(const reverseSteer of reverseSteers) {
        for(const reverseStep of reverseSteps) {
          const stop = {x: Math.max(34, Math.min(CW - 34, stopX)), y: stopY};
          let segs = buildCurve(start, stop, 0, 0, 1, 'Signal and pass space', Math.max(12, stop.x - start.x), 16, 34);
          const last = segs[segs.length - 1];
          segs.push({x: last.x, y: last.y, theta: last.theta, gear: 1, steer: 0, label: 'Stop just past space'});
          appendKinematicTurnToHeading(segs, -1, reverseSteer, targetHeading, 'Reverse into angle', reverseStep, 90);
          const handoff = segs[segs.length - 1];
          const centerlineError = lateralDistanceToHeadingLine(handoff, target, targetHeading);
          const reverseRemaining = -signedDistanceAlongHeading(handoff, target, targetHeading);
          if(centerlineError > 13 || reverseRemaining < 12 || reverseRemaining > 88) continue;
          appendStraightToPoint(segs, target, -1, 'Straight reverse', 3.2);
          const path = tryVideoPath(segs);
          if(path) {
            const shift = path.segs.find(seg => seg.gear === -1);
            let score = pathSmoothnessScore(path) + centerlineError * 12 + Math.abs(reverseRemaining - 38) * .9;
            if(shift) score += Math.abs(shift.x - (spotRef.x + pitch * 1.1)) * .25;
            if(score < bestScore) {
              best = path;
              bestScore = score;
            }
          }
        }
      }
    }
  }

  if(best) return best;

  const fallbackOffsets = [54, 64, 44, 74];
  for(const stopX of stopXs) {
    for(const stopY of stopYs) {
      for(const reverseOffset of fallbackOffsets) {
        const stop = {x: Math.max(34, Math.min(CW - 34, stopX)), y: stopY};
        const reverseStart = {
          x: target.x + Math.cos(targetHeading) * reverseOffset,
          y: target.y + Math.sin(targetHeading) * reverseOffset,
          theta: targetHeading
        };
        let segs = buildCurve(start, stop, 0, 0, 1, 'Signal and pass space', Math.max(12, stop.x - start.x), 16, 34);
        const last = segs[segs.length - 1];
        segs.push({x: last.x, y: last.y, theta: last.theta, gear: 1, steer: 0, label: 'Stop just past space'});
        segs = appendCurve(segs, easeCurveEndpointHeading(buildCurve(stop, reverseStart, 0, targetHeading, -1, 'Reverse into angle', 48, 24, 48), 24));
        segs = appendStraightToPoint(segs, target, -1, 'Straight reverse', 3.2);
        const path = tryVideoPath(segs);
        if(path) return path;
      }
    }
  }
  return null;
}

function planForwardAngled(sp, target) {
  const start = startPos();
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const pitch = spotPitch();
  const spotRef = spotAislePoint(sp);
  const targetHeading = target.theta;
  const wideYs = [laneFarFromSpotY(sp), LANE_MID - rowDir * 12, LANE_MID - rowDir * 22, LANE_MID + rowDir * 4];
  const alignXs = [spotRef.x - pitch * .82, spotRef.x - pitch, spotRef.x - pitch * .58, spotRef.x - pitch * 1.2, spotRef.x - pitch * .36, spotRef.x - pitch * 1.48];
  const approachHandles = [18, 24, 32];
  const turnHandlesA = [70, 86, 104, 122];
  const turnHandlesB = [26, 34, 46, 58];
  let best = null;
  let bestScore = Infinity;

  for(const alignX of alignXs) {
    for(const y of wideYs) {
      for(const approachHandleB of approachHandles) {
        for(const turnHandleA of turnHandlesA) {
          for(const turnHandleB of turnHandlesB) {
            const align = {x: Math.max(34, Math.min(CW - 34, alignX)), y};
            let segs = buildCurve(start, align, 0, 0, 1, 'Bumper to line', Math.max(12, align.x - start.x), approachHandleB, 28);
            const turnIn = applyPhaseLabels(
              buildCurve(align, target, 0, targetHeading, 1, 'Full-right turn', turnHandleA, turnHandleB, 78),
              [
                {until: .76, label: 'Full-right turn'},
                {until: 1, label: 'Straighten ahead'}
              ]
            );
            segs = appendCurve(segs, turnIn);
            const path = tryVideoPath(segs);
            if(path) {
              const score = pathSmoothnessScore(path)
                + Math.abs(align.x - (spotRef.x - pitch * .8)) * .35
                + Math.abs(align.y - laneFarFromSpotY(sp)) * .18
                + Math.abs(turnHandleA - 86) * .08
                + Math.abs(turnHandleB - 34) * .12;
              if(score < bestScore) {
                best = path;
                bestScore = score;
              }
            }
          }
        }
      }
    }
  }
  return best;
}

function planPathByManeuver(spotIdx, maneuver) {
  const sp = SPOTS[spotIdx];
  const normalized = normalizeManeuver(maneuver);
  const target = targetPose(sp, normalized);
  if(normalized === 'reverse-direct') return planReverseDirect(sp, target) || planReverseDirectFallback(sp, target);
  if(normalized === 'forward-direct') return planForwardDirect(sp, target);
  if(normalized === 'reverse-angled') return planReverseAngled(sp, target);
  if(normalized === 'forward-angled') return planForwardAngled(sp, target);
  return null;
}

function planParkingPath(spotIdx, maneuver) {
  return planPathByManeuver(spotIdx, maneuver) || planPathAStar(spotIdx, maneuver);
}

function pathSegmentAt(pathData, d) {
  const segs = pathData.segs;
  if(!pathData.distances) {
    const idx = Math.min(segs.length - 2, Math.max(0, Math.floor(d / pathData.step)));
    return {idx, t: (d % pathData.step) / pathData.step};
  }

  const distances = pathData.distances;
  let idx = 0;
  while(idx < distances.length - 2 && distances[idx + 1] < d) idx++;
  const span = distances[idx + 1] - distances[idx];
  return {idx, t: span > 0 ? (d - distances[idx]) / span : 0};
}

function pathSpeedFactor(pathData, d) {
  const {idx} = pathSegmentAt(pathData, d);
  const p1 = pathData.segs[idx], p2 = pathData.segs[idx + 1];
  const span = pathData.distances
    ? Math.max(.001, pathData.distances[idx + 1] - pathData.distances[idx])
    : pathData.step;
  const turnRate = Math.abs(arcDiff(p1.theta, p2.theta)) / span;
  return Math.max(.2, Math.min(1, 1 - turnRate * 7));
}

function walkDiscretePath(pathData, d) {
  let segs = pathData.segs, STEP = pathData.step;
  const asPose = p => ({
    x: p.x,
    y: p.y,
    heading: p.theta,
    steer: p.steer || 0,
    rev: p.gear === -1,
    label: p.label || (p.gear === -1 ? 'Reverse' : 'Forward')
  });

  if(d <= 0) return asPose(segs[0]);
  if(d >= pathData.totalLen) return asPose(segs[segs.length-1]);

  let {idx, t} = pathSegmentAt(pathData, d);
  if(idx >= segs.length - 1) return asPose(segs[segs.length-1]);

  let p1 = segs[idx], p2 = segs[idx+1];
  return {
    x: p1.x + (p2.x - p1.x) * t,
    y: p1.y + (p2.y - p1.y) * t,
    heading: p1.theta + arcDiff(p1.theta, p2.theta) * t,
    steer: p2.steer,
    rev: p2.gear === -1,
    label: p2.label
  };
}

function dk(){ return matchMedia('(prefers-color-scheme:dark)').matches; }
function col(){
  const d=dk();
  return{
    asp:d?'#18181c':'#28282e',lane:d?'#222228':'#303038',
    mark:d?'#555':'#888',sf:d?'rgba(255,255,255,0.05)':'rgba(0,0,0,0.06)',
    sl:d?'rgba(255,255,255,0.13)':'rgba(0,0,0,0.18)', tf:d?'rgba(55,138,221,0.15)':'rgba(24,95,165,0.1)',
    tl:d?'#378ADD':'#185FA5', tx:d?'#378ADD':'#185FA5', body:'#C84B20',roof:'#8B3010',
    glass:d?'rgba(150,195,255,0.4)':'rgba(130,175,245,0.55)', lit:'rgba(255,245,120,0.95)',
    tail:'rgba(210,30,30,0.9)', wr:'#1e1e1e', wf:'#2e2e2e', traj:d?'rgba(239,159,39,.8)':'rgba(186,117,23,.85)',
    pprev:d?'rgba(90,210,130,.4)':'rgba(10,120,60,.4)', ob:d?'#3C3489':'#534AB7', obr:d?'#26215C':'#3C3489',
    obg:d?'rgba(110,150,210,.3)':'rgba(90,130,190,.4)', txt:d?'rgba(255,255,255,0.5)':'rgba(0,0,0,0.45)',
    grid:d?'rgba(255,255,255,0.02)':'rgba(0,0,0,0.025)'
  };
}

function rr(x,y,w,h,r){
  const R=Math.min(r,w/2,h/2);
  ctx.beginPath(); ctx.moveTo(x+R,y); ctx.lineTo(x+w-R,y); ctx.arcTo(x+w,y,x+w,y+R,R);
  ctx.lineTo(x+w,y+h-R); ctx.arcTo(x+w,y+h,x+w-R,y+h,R); ctx.lineTo(x+R,y+h);
  ctx.arcTo(x,y+h,x,y+h-R,R); ctx.lineTo(x,y+R); ctx.arcTo(x,y,x+R,y,R); ctx.closePath();
}

function tracePoly(poly) {
  ctx.beginPath();
  ctx.moveTo(poly[0].x, poly[0].y);
  for(let i = 1; i < poly.length; i++) ctx.lineTo(poly[i].x, poly[i].y);
  ctx.closePath();
}

function lerpPoint(a, b, t) {
  return {x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t};
}

function drawLot(c){
  ctx.fillStyle=c.asp; ctx.fillRect(0,0,CW,CH);
  ctx.strokeStyle=c.grid; ctx.lineWidth=1;
  for(let x=0;x<CW;x+=20){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,CH);ctx.stroke();}
  for(let y=0;y<CH;y+=20){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(CW,y);ctx.stroke();}
  ctx.fillStyle=c.lane; ctx.fillRect(0,LANE_TOP,CW,LANE_H);
  ctx.strokeStyle=c.mark; ctx.lineWidth=1.5;
  [LANE_TOP,LANE_BOT].forEach(y=>{ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(CW,y);ctx.stroke();});
  
  SPOTS.forEach(sp=>{
    const poly=spotPolygon(sp), labelPoint=spotLabelPoint(sp), isTgt=sp.idx===targetIdx;
    ctx.fillStyle=isTgt?c.tf:c.sf; tracePoly(poly); ctx.fill();
    ctx.strokeStyle=isTgt?c.tl:c.sl; ctx.lineWidth=isTgt?1.5:.5;
    tracePoly(poly); ctx.stroke();
    ctx.fillStyle=isTgt?c.tl:c.txt; ctx.font=`${isTgt?'500 ':''}11px monospace`; 
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(String(sp.idx),labelPoint.x,labelPoint.y);
    if(isTgt){
      ctx.strokeStyle=c.tx; ctx.lineWidth=1.5; ctx.setLineDash([5,4]);
      const m=.18;
      const a=lerpPoint(poly[0],poly[2],m), b=lerpPoint(poly[2],poly[0],m);
      const c1=lerpPoint(poly[1],poly[3],m), d=lerpPoint(poly[3],poly[1],m);
      ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
      ctx.beginPath();ctx.moveTo(c1.x,c1.y);ctx.lineTo(d.x,d.y);ctx.stroke();
      ctx.setLineDash([]);
    }
  });
}

function drawPathPreview(c){
  if(!path) return;
  ctx.save(); ctx.strokeStyle=c.pprev; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(path.segs[0].x, path.segs[0].y);
  for(let i=1; i<path.segs.length; i++) ctx.lineTo(path.segs[i].x, path.segs[i].y);
  ctx.stroke(); ctx.restore();
  drawTurnRuler(path, c);
}

function firstTurnDistance(pathData) {
  const segs = pathData.segs;
  const distances = pathData.distances;
  const startHeading = segs[0].theta;
  for(let i = 1; i < segs.length; i++) {
    const headingChange = Math.abs(arcDiff(segs[i - 1].theta, segs[i].theta));
    const totalHeadingChange = Math.abs(arcDiff(startHeading, segs[i].theta));
    const steeringChange = Math.abs(segs[i].steer || 0);
    if(headingChange > .018 || totalHeadingChange > .055 || steeringChange > .08) {
      return distances ? distances[i] : i * pathData.step;
    }
  }
  return pathData.totalLen;
}

function reverseShiftDistance(pathData) {
  const segs = pathData.segs;
  const distances = pathData.distances;
  for(let i = 1; i < segs.length; i++) {
    if(segs[i - 1].gear === 1 && segs[i].gear === -1) {
      return distances ? distances[i] : i * pathData.step;
    }
  }
  return firstTurnDistance(pathData);
}

function pathPointAt(pathData, distance) {
  const pose = walkDiscretePath(pathData, distance);
  return {x: pose.x, y: pose.y, heading: pose.heading};
}

function vehiclePoint(pose, forwardDistance) {
  return {
    x: pose.x + Math.cos(pose.heading) * forwardDistance,
    y: pose.y + Math.sin(pose.heading) * forwardDistance
  };
}

function vehicleOffsetPoint(pose, forwardDistance, lateralDistance) {
  const fwdX = Math.cos(pose.heading), fwdY = Math.sin(pose.heading);
  const rightX = -fwdY, rightY = fwdX;
  return {
    x: pose.x + fwdX * forwardDistance + rightX * lateralDistance,
    y: pose.y + fwdY * forwardDistance + rightY * lateralDistance
  };
}

function drawTick(point, heading, size) {
  const nx = -Math.sin(heading), ny = Math.cos(heading);
  ctx.beginPath();
  ctx.moveTo(point.x - nx * size, point.y - ny * size);
  ctx.lineTo(point.x + nx * size, point.y + ny * size);
  ctx.stroke();
}

function drawTurnRuler(pathData, c) {
  const normalized = normalizeManeuver(maneuver);
  const isReverseParking = isReverseManeuver(normalized);
  const eventDistance = Math.min(isReverseParking ? reverseShiftDistance(pathData) : firstTurnDistance(pathData), pathData.totalLen);
  if(eventDistance < .5) return;

  const targetSpot = SPOTS[targetIdx];
  const spotRef = spotAislePoint(targetSpot);
  const eventPose = pathPointAt(pathData, eventDistance);
  const referencePoint = vehicleOffsetPoint(eventPose, RA_FRONT - WB * .42, -CAR_WID * .34);
  const dx = referencePoint.x - spotRef.x;
  const dy = referencePoint.y - spotRef.y;
  const absX = Math.abs(dx), absY = Math.abs(dy);
  if(absX < .5 && absY < .5) return;

  const tickStep = 1 / METERS_PER_UNIT;
  const corner = {x: referencePoint.x, y: spotRef.y};
  const axisHeading = (from, to) => Math.atan2(to.y - from.y, to.x - from.x);
  const pointOnAxis = (from, to, distance) => {
    const len = Math.hypot(to.x - from.x, to.y - from.y);
    const t = len > 0 ? distance / len : 0;
    return {x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t};
  };
  const labelBox = (lines, x, y) => {
    const padX = 7, padY = 5, lineH = 12;
    const labelW = Math.max(...lines.map(line => ctx.measureText(line).width)) + padX * 2;
    const labelH = lines.length * lineH + padY * 2;
    const labelX = Math.max(labelW / 2 + 4, Math.min(CW - labelW / 2 - 4, x));
    const labelY = Math.max(labelH / 2 + 4, Math.min(CH - labelH / 2 - 4, y));
    rr(labelX - labelW / 2, labelY - labelH / 2, labelW, labelH, 5);
    ctx.fillStyle = dk() ? 'rgba(12,24,38,.96)' : 'rgba(236,247,255,.97)';
    ctx.fill();
    ctx.strokeStyle = c.tx;
    ctx.lineWidth = .8;
    ctx.stroke();
    ctx.fillStyle = c.tx;
    lines.forEach((line, idx) => {
      ctx.fillText(line, labelX, labelY - (lines.length - 1) * lineH / 2 + idx * lineH);
    });
  };

  ctx.save();
  ctx.strokeStyle = c.tx;
  ctx.fillStyle = c.tx;
  ctx.lineWidth = 1.4;
  ctx.setLineDash([]);

  ctx.beginPath();
  ctx.moveTo(spotRef.x, spotRef.y);
  ctx.lineTo(corner.x, corner.y);
  ctx.lineTo(referencePoint.x, referencePoint.y);
  ctx.stroke();

  ctx.fillStyle = c.tx;
  ctx.beginPath();
  ctx.arc(referencePoint.x, referencePoint.y, 3, 0, Math.PI * 2);
  ctx.fill();

  if(absX >= .5) {
    const hHeading = axisHeading(spotRef, corner);
    drawTick(spotRef, hHeading, 5);
    for(let d = tickStep; d < absX; d += tickStep) {
      drawTick(pointOnAxis(spotRef, corner, d), hHeading, 3.5);
    }
    drawTick(corner, hHeading, absY >= .5 ? 4 : 5);
  }
  if(absY >= .5) {
    const vHeading = axisHeading(corner, referencePoint);
    drawTick(corner, vHeading, absX >= .5 ? 4 : 5);
    for(let d = tickStep; d < absY; d += tickStep) {
      drawTick(pointOnAxis(corner, referencePoint, d), vHeading, 3.5);
    }
    drawTick(referencePoint, vHeading, 5);
  }

  ctx.font = '500 10px monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const alongLabel = `${(absX * METERS_PER_UNIT).toFixed(1)} m ${dx >= 0 ? 'right' : 'left'} of center`;
  const outLabel = `${(absY * METERS_PER_UNIT).toFixed(1)} m into aisle`;
  const eventLabel = normalized === 'reverse-direct' ? 'shift reverse'
    : normalized === 'reverse-angled' ? 'straighten & reverse'
    : 'start turn';
  const pointLabel = normalized === 'forward-direct' ? 'right mirror at headlight'
    : normalized === 'forward-angled' ? 'front bumper at line'
    : 'side mirror at line';
  const lotMargin = (CW - NCOLS * spotPitch()) / 2;
  const labelAnchorY = targetSpot.row === 'bot' ? BOT_Y + SPOT_H * .5 : TOP_Y + SPOT_H * .5;
  const labelAnchorX = spotRef.x < CW / 2 ? CW - lotMargin / 2 : lotMargin / 2;
  labelBox([eventLabel, 'from spot centerline', pointLabel, alongLabel, outLabel], labelAnchorX, labelAnchorY);
  ctx.restore();
}

function drawTraj(c){
  if(traj.length<2) return;
  ctx.save(); ctx.strokeStyle=c.traj; ctx.lineWidth=1.5; ctx.setLineDash([4,4]);
  ctx.beginPath(); ctx.moveTo(traj[0].x,traj[0].y);
  traj.forEach(p=>ctx.lineTo(p.x,p.y)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle=c.traj;
  for(let i=0;i<traj.length;i+=5){ctx.beginPath();ctx.arc(traj[i].x,traj[i].y,1.5,0,Math.PI*2);ctx.fill();}
  ctx.restore();
}

function drawCar(rx,ry,heading,steer,bodyC,roofC,glassC,c){
  ctx.save(); ctx.translate(rx,ry); ctx.rotate(heading+Math.PI/2);
  ctx.fillStyle=dk()?'rgba(0,0,0,.35)':'rgba(0,0,0,.18)';
  rr(-CAR_WID/2+3,-RA_FRONT+3,CAR_WID,CAR_LEN,6); ctx.fill();
  ctx.fillStyle=bodyC; rr(-CAR_WID/2,-RA_FRONT,CAR_WID,CAR_LEN,6); ctx.fill();
  const rW=CAR_WID*.72, rStart=-(RA_FRONT-WB*.1), rLen=WB*.78;
  ctx.fillStyle=roofC; rr(-rW/2,rStart,rW,rLen,5); ctx.fill();
  ctx.fillStyle=glassC; rr(-rW/2+2,rStart+2,rW-4,rLen*.33,3); ctx.fill();
  ctx.fillStyle=dk()?'rgba(120,160,215,.25)':'rgba(100,145,200,.35)';
  rr(-rW/2+2,rStart+rLen*.62,rW-4,rLen*.33,2); ctx.fill();
  ctx.fillStyle=c.lit; ctx.fillRect(-CAR_WID/2+2,-RA_FRONT+2,8,5); ctx.fillRect(CAR_WID/2-10,-RA_FRONT+2,8,5);
  ctx.fillStyle=c.tail; ctx.fillRect(-CAR_WID/2+2,RA_BACK-6,8,4); ctx.fillRect(CAR_WID/2-10,RA_BACK-6,8,4);
  ctx.fillStyle=dk()?'rgba(255,255,255,.3)':'rgba(0,0,0,.25)';
  ctx.beginPath(); ctx.arc(0,0,2.5,0,Math.PI*2); ctx.fill();
  ctx.restore();
  
  const fwdX=Math.cos(heading), fwdY=Math.sin(heading), rtX=-fwdY, rtY=fwdX;
  function wpt(f,r){return{x:rx+fwdX*f+rtX*r,y:ry+fwdY*f+rtY*r};}
  const wOff=CAR_WID/2+2;
  const rl=wpt(0,-wOff), rr2=wpt(0,wOff), fl=wpt(WB,-wOff), fr=wpt(WB,wOff);
  function drawW(wx,wy,ang,frt){
    ctx.save(); ctx.translate(wx,wy); ctx.rotate(ang+Math.PI/2);
    ctx.fillStyle=frt?c.wf:c.wr; rr(-WW/2,-WH/2,WW,WH,1.5); ctx.fill();
    ctx.strokeStyle=dk()?'rgba(255,255,255,.07)':'rgba(255,255,255,.12)'; ctx.lineWidth=.5;
    for(let i=-4;i<=4;i+=2){ctx.beginPath();ctx.moveTo(-WW/2,i);ctx.lineTo(WW/2,i);ctx.stroke();}
    ctx.restore();
  }
  drawW(rl.x,rl.y,heading,false); drawW(rr2.x,rr2.y,heading,false);
  drawW(fl.x,fl.y,heading+steer,true); drawW(fr.x,fr.y,heading+steer,true);
}

function drawObstacles(c){
  SPOTS.forEach(sp=>{
    if(!obstacles.has(sp.idx)||sp.idx===targetIdx)return;
    const h=spotInwardTheta(sp);
    const pos=pointFromSpotAisle(sp,RA_BACK+PARKED_CAR_LINE_GAP);
    drawCar(pos.x,pos.y,h,0,c.ob,c.obr,c.obg,c);
  });
}

function drawOverlay(){
  if(state!=='done')return;
  ctx.save(); ctx.fillStyle=dk()?'rgba(5,55,22,.92)':'rgba(5,70,28,.92)';
  rr(CW/2-80,CH/2-22,160,44,8); ctx.fill();
  ctx.fillStyle='#fff'; ctx.font='500 13px monospace'; ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillText('PARKED  ✓',CW/2,CH/2); ctx.restore();
}

function draw(){
  const c=col(); ctx.clearRect(0,0,CW,CH);
  drawLot(c); drawPathPreview(c); drawObstacles(c); drawTraj(c);
  drawCar(car.x,car.y,car.heading,car.steerAngle,c.body,c.roof,c.glass,c);
  drawOverlay();
}

const SPD=100;
function setSpeedFromControl() {
  const speedRange = document.getElementById('speed-range');
  const next = speedRange ? parseInt(speedRange.value, 10) : speedScale;
  speedScale = Number.isFinite(next) ? next : speedScale;
  updateSpeedControl();
}

function updateSpeedControl() {
  const speedValue = document.getElementById('speed-value');
  if(speedValue) speedValue.textContent = (speedScale / 100).toFixed(2).replace(/0$/, '') + 'x';
}

function setAngleFromControl() {
  const angleRange = document.getElementById('angle-range');
  const next = angleRange ? parseInt(angleRange.value, 10) : parkingAngleDeg;
  parkingAngleDeg = Number.isFinite(next) ? Math.max(0, Math.min(45, next)) : parkingAngleDeg;
  updateAngleControl();
  if(state === 'idle' || state === 'done') {
    path = null;
    reset();
  }
}

function updateAngleControl() {
  const angleValue = document.getElementById('angle-value');
  const angleRange = document.getElementById('angle-range');
  if(angleValue) angleValue.textContent = `${parkingAngleDeg}°`;
  if(angleRange && String(angleRange.value) !== String(parkingAngleDeg)) angleRange.value = String(parkingAngleDeg);
}

function updateManeuverControl() {
  maneuver = normalizeManeuver(maneuver);
  const selMan = document.getElementById('sel-man');
  const cue = document.getElementById('maneuver-cue');
  const info = maneuverInfo(maneuver);
  if(selMan) selMan.value = maneuver;
  if(cue) cue.textContent = info.cue;
}

function updateDimensions() {
  const carDim = document.getElementById('dim-car');
  const spotDim = document.getElementById('dim-spot');
  const lotDim = document.getElementById('dim-lot');
  const fmt = (length, width) => {
    const lM = length * METERS_PER_UNIT;
    const wM = width * METERS_PER_UNIT;
    const lFt = lM * FEET_PER_METER;
    const wFt = wM * FEET_PER_METER;
    return `${lM.toFixed(1)} x ${wM.toFixed(1)} m / ${lFt.toFixed(1)} x ${wFt.toFixed(1)} ft`;
  };
  if(carDim) carDim.textContent = fmt(CAR_LEN, CAR_WID);
  if(spotDim) spotDim.textContent = fmt(SPOT_H, SPOT_W);
  if(lotDim) lotDim.textContent = fmt(CW, CH);
}

function updateActionControls() {
  const isActive = state === 'running' || state === 'paused' || state === 'planning';
  const isPaused = state === 'paused';
  const btnStart = document.getElementById('btn-start');
  const btnPause = document.getElementById('btn-pause');
  const angleRange = document.getElementById('angle-range');
  const selSpot = document.getElementById('sel-spot');
  const selMan = document.getElementById('sel-man');
  const btnObs = document.getElementById('btn-obs');

  if(btnStart) {
    btnStart.disabled = isActive;
    btnStart.textContent = state === 'planning' ? 'Planning...' : state === 'running' || state === 'paused' ? 'Parking...' : 'Start Auto-Park';
  }
  if(btnPause) {
    btnPause.disabled = !(state === 'running' || state === 'paused');
    btnPause.textContent = isPaused ? 'Resume Parking' : 'Pause Parking';
    if(btnPause.classList) btnPause.classList.toggle('is-paused', isPaused);
  }
  if(angleRange) angleRange.disabled = isActive || !maneuverInfo(maneuver).angled;
  if(selSpot) selSpot.disabled = isActive;
  if(selMan) selMan.disabled = isActive;
  if(btnObs) btnObs.disabled = isActive;
}

function reset(){
  cancelAnimationFrame(raf); raf=null;
  updateManeuverControl();
  const s=startPos();
  car={x:s.x,y:s.y,heading:s.heading,steerAngle:0};
  traj=[]; path=null; dist=0; state='idle'; lastT=null;
  updateActionControls();
  updateObstacles(); updateStats(null); draw();
}

function startAnim(){
  if(state === 'running' || state === 'paused' || state === 'planning') return;
  state = 'planning';
  updateActionControls();
  setTimeout(()=>{
    const s=startPos();
    car={x:s.x,y:s.y,heading:s.heading,steerAngle:0};
    traj=[]; dist=0; lastT=null;
    
    path = planParkingPath(targetIdx, maneuver);
    
    if(!path) {
      alert("No valid path found! The car is completely blocked by obstacles.");
      reset();
      return;
    }
    
    state='running';
    updateActionControls();
    raf=requestAnimationFrame(loop);
  }, 50);
}

function togglePause(){
  if(state === 'running') {
    state = 'paused';
    cancelAnimationFrame(raf); raf = null; lastT = null;
    updateActionControls();
    draw();
    return;
  }
  if(state === 'paused') {
    state = 'running';
    lastT = null;
    updateActionControls();
    raf = requestAnimationFrame(loop);
  }
}

function loop(ts){
  if(state !== 'running') return;
  if(!lastT) lastT=ts;
  const dt=Math.min((ts-lastT)/1000,.05); lastT=ts;
  dist+=SPD*(speedScale / 100)*pathSpeedFactor(path, dist)*dt;
  
  if(dist>=path.totalLen){
    dist=path.totalLen; state='done';
    updateActionControls();
  }
  
  const pose=walkDiscretePath(path,dist);
  if(pose){
    car.x=pose.x; car.y=pose.y; car.heading=pose.heading; 
    car.steerAngle += (pose.steer - car.steerAngle) * 0.2;
    traj.push({x:pose.x,y:pose.y});
    updateStats(pose);
  }
  draw();
  if(state==='running') raf=requestAnimationFrame(loop); else draw();
}

function updateStats(pose){
  const tireSteerDeg=pose?Math.round((car.steerAngle||0)*180/Math.PI):0;
  const wheelSteerDeg=Math.max(-MAX_STEERING_WHEEL_TURN, Math.min(MAX_STEERING_WHEEL_TURN, Math.round(tireSteerDeg * STEERING_WHEEL_RATIO)));
  const wheel=document.getElementById('steering-wheel');
  if(wheel && wheel.style) {
    wheel.style.setProperty('--steer-turn', `${wheelSteerDeg}deg`);
    if(wheel.classList) wheel.classList.toggle('is-active', state === 'running' || state === 'paused');
  }
  document.getElementById('s-dir').textContent=pose?(pose.rev?'REV':'FWD'):'—';
  document.getElementById('s-steer').textContent=wheelSteerDeg+'°';
  document.getElementById('s-head').textContent=Math.round(((car.heading||0)*180/Math.PI+360+90)%360)+'°';
  document.getElementById('s-seg').textContent=pose?pose.label:'—';
  document.getElementById('s-prog').textContent=path?Math.min(100, Math.round(dist/path.totalLen*100))+'%':'0%';
}

CV.addEventListener('mousedown', e => {
  if(state === 'running' || state === 'paused' || state === 'planning') return;
  const rect = CV.getBoundingClientRect();
  const scaleX = CV.width / rect.width, scaleY = CV.height / rect.height;
  const cx = (e.clientX - rect.left) * scaleX, cy = (e.clientY - rect.top) * scaleY;

  for(let sp of SPOTS) {
    if(sp.idx === targetIdx) continue;
    if(pointInPoly({x: cx, y: cy}, spotPolygon(sp))) {
      if(obstacles.has(sp.idx)) obstacles.delete(sp.idx); else obstacles.add(sp.idx);
      updateObstacles(); draw(); break;
    }
  }
});

document.getElementById('sel-spot').onchange=e=>{targetIdx=parseInt(e.target.value); obstacles.delete(targetIdx); if(state==='idle'||state==='done'){path=null;reset();}};
document.getElementById('sel-man').onchange=e=>{
  maneuver=normalizeManeuver(e.target.value);
  const info = maneuverInfo(maneuver);
  if(info.angled && parkingAngleDeg === 0) parkingAngleDeg = info.defaultAngle || 35;
  if(!info.angled) parkingAngleDeg = 0;
  const angleRange = document.getElementById('angle-range');
  if(angleRange) angleRange.value = String(parkingAngleDeg);
  updateAngleControl();
  updateManeuverControl();
  if(state==='idle'||state==='done'){path=null;reset();}
};
document.getElementById('angle-range').addEventListener('input', setAngleFromControl);
document.getElementById('angle-range').addEventListener('change', setAngleFromControl);
document.getElementById('speed-range').addEventListener('input', setSpeedFromControl);
document.getElementById('speed-range').addEventListener('change', setSpeedFromControl);
document.getElementById('btn-obs').onclick=()=>{
  obstacles.clear();
  SPOTS.forEach(sp=>{ if(sp.idx!==targetIdx&&Math.random()>.5) obstacles.add(sp.idx); });
  if(state==='idle'||state==='done'){path=null;reset();}
};
document.getElementById('btn-start').onclick=startAnim;
document.getElementById('btn-pause').onclick=togglePause;
document.getElementById('btn-reset').onclick=reset;

updateAngleControl();
updateManeuverControl();
updateDimensions();
setSpeedFromControl();
reset();
})();
