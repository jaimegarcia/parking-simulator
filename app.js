(function(){
const CV=document.getElementById('pc');
const ctx=CV.getContext('2d');
const CW=500, CH=265;
CV.width=CW; CV.height=CH;

const SPOT_W=72, SPOT_H=98, TOP_Y=10;
const BOT_Y=CH-15-SPOT_H, LANE_TOP=TOP_Y+SPOT_H, LANE_BOT=BOT_Y;
const LANE_H=LANE_BOT-LANE_TOP, LANE_MID=(LANE_TOP+LANE_BOT)/2;
const NCOLS=4, MARGIN=(CW - (NCOLS - 1) * SPOT_W) / 2, PITCH=SPOT_W;
const SPOT_CXS=Array.from({length:NCOLS},(_,i)=>MARGIN+i*PITCH);
const PARKED_CAR_LINE_GAP=12;

const SPOTS=[];
for(let i=0;i<4;i++) SPOTS.push({idx:i, cx:SPOT_CXS[i], cy:TOP_Y+SPOT_H/2, row:'top'});
for(let i=0;i<4;i++) SPOTS.push({idx:4+i, cx:SPOT_CXS[i], cy:BOT_Y+SPOT_H/2, row:'bot'});

const CAR_LEN=68, CAR_WID=28, WB=46, RA_BACK=12, RA_FRONT=RA_BACK+WB;
const WW=5, WH=12;

let car={x:0,y:0,heading:0,steerAngle:0};
let traj=[], path=null, dist=0, state='idle';
let raf=null, lastT=null;
let targetIdx=5, maneuver='reverse';
let obstacles=new Set(), obsPolys=[];
let speedScale=1;

function startPos(){ return {x:30, y:LANE_MID, heading:0}; }

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
      let h = sp.row === 'top' ? -Math.PI/2 : Math.PI/2;
      let ra_y = sp.row === 'top'
        ? sp.cy + SPOT_H/2 - RA_BACK - PARKED_CAR_LINE_GAP
        : sp.cy - SPOT_H/2 + RA_BACK + PARKED_CAR_LINE_GAP;
      obsPolys.push(getCarPoly(sp.cx, ra_y, h));
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
  const reverse = maneuver === 'reverse';
  if(sp.row === 'bot') {
    return {
      x: sp.cx,
      y: reverse ? BOT_Y + SPOT_H - RA_BACK - 8 : BOT_Y + RA_BACK + 8,
      theta: reverse ? -Math.PI/2 : Math.PI/2
    };
  }

  return {
    x: sp.cx,
    y: reverse ? TOP_Y + RA_BACK + 8 : TOP_Y + SPOT_H - RA_BACK - 8,
    theta: reverse ? Math.PI/2 : -Math.PI/2
  };
}

function planPathAStar(spotIdx, maneuver) {
  const sp = SPOTS[spotIdx];
  const target = targetPose(sp, maneuver);
  const tX = target.x;
  const tY = target.y;
  const tTheta = target.theta;
  const s = startPos();
  
  let open = new MinHeap();
  let closed = new Set();
  open.push({x: s.x, y: s.y, theta: s.heading, g: 0, f: 0, gear: 1, steer: 0, parent: null});

  let best = null, nearest = null, nearestScore = Infinity, iters = 0;
  const STEP = 7, MAX_STEER = 0.64;
  const STEERS = [-MAX_STEER, -MAX_STEER * .48, 0, MAX_STEER * .48, MAX_STEER];
  const GOAL_DIST = 15, GOAL_ANGLE = 0.38;
  const MAX_ITERS = 90000;
  let wpX = maneuver === 'reverse' ? sp.cx + PITCH * .38 : sp.cx - PITCH * .38;
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
        if(gear !== curr.gear) cost += 90;
        if(steer !== 0) cost += Math.abs(steer) * 4;

        let f = cost;
        let ndT = Math.hypot(tX - nx, tY - ny);
        let nAD = Math.abs(arcDiff(ntheta, tTheta));

        if (maneuver === 'reverse') {
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
            if (gear === -1) f += 420; 
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

function planPathByManeuver(spotIdx, maneuver) {
  const sp = SPOTS[spotIdx];
  const start = startPos();
  const target = targetPose(sp, maneuver);
  const targetHeading = target.theta;
  const rowDir = sp.row === 'bot' ? 1 : -1;
  const offsets = maneuver === 'reverse'
    ? [PITCH * .62, PITCH * .45, PITCH * .8, PITCH * .3, PITCH]
    : [-PITCH * 1.25, -PITCH * 1.05, -PITCH * 1.5, -PITCH * .85, -PITCH * .65];
  const sideYs = [LANE_MID, LANE_MID - rowDir * 12, LANE_MID + rowDir * 10, LANE_MID - rowDir * 24];

  for(const offset of offsets) {
    for(const y of sideYs) {
      const stage = {x: Math.max(32, Math.min(CW - 32, sp.cx + offset)), y};
      const approachHeading = 0;
      const approachHandle = Math.max(12, Math.min(50, stage.x - start.x));
      const first = buildCurve(start, stage, 0, approachHeading, 1, 'Forward', approachHandle, Math.min(36, approachHandle), 34);
      const finalGear = maneuver === 'reverse' ? -1 : 1;
      const label = maneuver === 'reverse' ? 'Reverse' : 'Forward';
      const finalStartHeading = approachHeading;
      const finalHandleA = maneuver === 'forward' ? 96 : 56;
      const finalHandleB = maneuver === 'forward' ? 34 : 40;
      const second = buildCurve(stage, target, finalStartHeading, targetHeading, finalGear, label, finalHandleA, finalHandleB, 52);
      const segs = first.concat(second.slice(1));

      for(let i = 1; i < segs.length; i++) {
        const prev = segs[i - 1];
        const curr = segs[i];
        curr.steer = Math.max(-0.64, Math.min(0.64, arcDiff(prev.theta, curr.theta) * 1.8));
      }

      if(pathClear(segs)) {
        return finalizePath(segs, 7);
      }
    }
  }

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

function drawLot(c){
  ctx.fillStyle=c.asp; ctx.fillRect(0,0,CW,CH);
  ctx.strokeStyle=c.grid; ctx.lineWidth=1;
  for(let x=0;x<CW;x+=20){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,CH);ctx.stroke();}
  for(let y=0;y<CH;y+=20){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(CW,y);ctx.stroke();}
  ctx.fillStyle=c.lane; ctx.fillRect(0,LANE_TOP,CW,LANE_H);
  ctx.strokeStyle=c.mark; ctx.lineWidth=1.5;
  [LANE_TOP,LANE_BOT].forEach(y=>{ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(CW,y);ctx.stroke();});
  
  SPOTS.forEach(sp=>{
    const sx=sp.cx-SPOT_W/2, sy=sp.row==='top'?TOP_Y:BOT_Y, isTgt=sp.idx===targetIdx;
    ctx.fillStyle=isTgt?c.tf:c.sf; rr(sx,sy,SPOT_W,SPOT_H,4); ctx.fill();
    ctx.strokeStyle=isTgt?c.tl:c.sl; ctx.lineWidth=isTgt?1.5:.5;
    rr(sx+.5,sy+.5,SPOT_W-1,SPOT_H-1,4); ctx.stroke();
    ctx.fillStyle=isTgt?c.tl:c.txt; ctx.font=`${isTgt?'500 ':''}11px monospace`; 
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(String(sp.idx),sp.cx,sp.cy+(sp.row==='top'?18:-18));
    if(isTgt){
      ctx.strokeStyle=c.tx; ctx.lineWidth=1.5; ctx.setLineDash([5,4]);
      const m=13;
      ctx.beginPath();ctx.moveTo(sx+m,sy+m);ctx.lineTo(sx+SPOT_W-m,sy+SPOT_H-m);ctx.stroke();
      ctx.beginPath();ctx.moveTo(sx+SPOT_W-m,sy+m);ctx.lineTo(sx+m,sy+SPOT_H-m);ctx.stroke();
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
    const h=sp.row==='top'?-Math.PI/2:Math.PI/2;
    const ra_y=sp.row==='top'
      ? sp.cy+SPOT_H/2-RA_BACK-PARKED_CAR_LINE_GAP
      : sp.cy-SPOT_H/2+RA_BACK+PARKED_CAR_LINE_GAP;
    drawCar(sp.cx,ra_y,h,0,c.ob,c.obr,c.obg,c);
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
function updateSpeedControl() {
  const speedValue = document.getElementById('speed-value');
  if(speedValue) speedValue.textContent = speedScale.toFixed(2).replace(/0$/, '') + 'x';
}

function reset(){
  cancelAnimationFrame(raf); raf=null;
  const s=startPos();
  car={x:s.x,y:s.y,heading:s.heading,steerAngle:0};
  traj=[]; path=null; dist=0; state='idle'; lastT=null;
  document.getElementById('btn-start').disabled=false;
  document.getElementById('btn-start').textContent="Start Auto-Park";
  updateObstacles(); updateStats(null); draw();
}

function startAnim(){
  if(state==='running')return;
  document.getElementById('btn-start').textContent="Planning...";
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
    document.getElementById('btn-start').textContent="Parking...";
    document.getElementById('btn-start').disabled=true;
    raf=requestAnimationFrame(loop);
  }, 50);
}

function loop(ts){
  if(!lastT) lastT=ts;
  const dt=Math.min((ts-lastT)/1000,.05); lastT=ts;
  dist+=SPD*speedScale*pathSpeedFactor(path, dist)*dt;
  
  if(dist>=path.totalLen){dist=path.totalLen; state='done'; document.getElementById('btn-start').disabled=false; document.getElementById('btn-start').textContent="Start Auto-Park";}
  
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
  document.getElementById('s-dir').textContent=pose?(pose.rev?'REV':'FWD'):'—';
  document.getElementById('s-steer').textContent=pose?Math.round((car.steerAngle||0)*180/Math.PI)+'°':'0°';
  document.getElementById('s-head').textContent=Math.round(((car.heading||0)*180/Math.PI+360+90)%360)+'°';
  document.getElementById('s-seg').textContent=pose?pose.label:'—';
  document.getElementById('s-prog').textContent=path?Math.min(100, Math.round(dist/path.totalLen*100))+'%':'0%';
}

CV.addEventListener('mousedown', e => {
  if(state === 'running') return;
  const rect = CV.getBoundingClientRect();
  const scaleX = CV.width / rect.width, scaleY = CV.height / rect.height;
  const cx = (e.clientX - rect.left) * scaleX, cy = (e.clientY - rect.top) * scaleY;

  for(let sp of SPOTS) {
    if(sp.idx === targetIdx) continue;
    let sx = sp.cx - SPOT_W/2, sy = sp.row === 'top' ? TOP_Y : BOT_Y;
    if(cx >= sx && cx <= sx + SPOT_W && cy >= sy && cy <= sy + SPOT_H) {
      if(obstacles.has(sp.idx)) obstacles.delete(sp.idx); else obstacles.add(sp.idx);
      updateObstacles(); draw(); break;
    }
  }
});

document.getElementById('sel-spot').onchange=e=>{targetIdx=parseInt(e.target.value); obstacles.delete(targetIdx); if(state!=='running'){path=null;reset();}};
document.getElementById('sel-man').onchange=e=>{maneuver=e.target.value; if(state!=='running'){path=null;reset();}};
document.getElementById('speed-range').oninput=e=>{speedScale=parseFloat(e.target.value); updateSpeedControl();};
document.getElementById('btn-obs').onclick=()=>{
  obstacles.clear();
  SPOTS.forEach(sp=>{ if(sp.idx!==targetIdx&&Math.random()>.5) obstacles.add(sp.idx); });
  if(state!=='running'){path=null;reset();}
};
document.getElementById('btn-start').onclick=startAnim;
document.getElementById('btn-reset').onclick=reset;

updateSpeedControl();
reset();
})();
