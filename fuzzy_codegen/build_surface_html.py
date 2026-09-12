#!/usr/bin/env python3
"""Assemble the interactive 3D control-surface artifact HTML from surface.json.
Inlines the grid data; all rendering is self-contained canvas JS (no CDN).

Usage:
    python3 build_surface_html.py [surface.json] [-o fuzzy_surface.html]
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
_ap = argparse.ArgumentParser(description="surface.json -> self-contained HTML.")
_ap.add_argument('json', nargs='?', default=os.path.join(HERE, 'surface.json'),
                 help="grid produced by gen_surface.py (default: surface.json)")
_ap.add_argument('-o', '--out', default=os.path.join(HERE, 'fuzzy_surface.html'),
                 help="output html (default: fuzzy_surface.html)")
_args = _ap.parse_args()

data = json.load(open(_args.json))
DATA = json.dumps(data, separators=(',', ':'))
# FIS name goes in the title so two .fis variants do not produce look-alike pages
NAME = data.get('name') or 'fuzzy'

HTML = r'''<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Mặt điều khiển fuzzy — @@NAME@@</title>
<style>
:root{
  --bg:#f5f6f8; --panel:#ffffff; --panel2:#eef0f3;
  --ink:#1b1f26; --muted:#7a828d; --faint:#d6dae0; --grid:#cfd4db;
  --accent:#c2410c; --line:rgba(0,0,0,.10);
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    --bg:#0d1015; --panel:#141a21; --panel2:#1b222b;
    --ink:#e7eaf0; --muted:#8b949e; --faint:#2a313b; --grid:#2f3742;
    --accent:#f97316; --line:rgba(255,255,255,.08);
  }
}
:root[data-theme="dark"]{
  --bg:#0d1015; --panel:#141a21; --panel2:#1b222b;
  --ink:#e7eaf0; --muted:#8b949e; --faint:#2a313b; --grid:#2f3742;
  --accent:#f97316; --line:rgba(255,255,255,.08);
}
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{
  background:var(--bg); color:var(--ink);
  font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  display:flex;flex-direction:column; overflow:hidden;
}
.bar{
  display:flex;align-items:baseline;gap:14px;padding:12px 18px;
  border-bottom:1px solid var(--faint); background:var(--panel);
}
.bar h1{font-size:16px;margin:0;font-weight:650;letter-spacing:.2px}
.bar .sub{color:var(--muted);font-variant-numeric:tabular-nums}
.bar .sp{flex:1}
.bar button{
  font:inherit;color:var(--ink);background:var(--panel2);border:1px solid var(--faint);
  padding:6px 12px;border-radius:8px;cursor:pointer;
}
.bar button:hover{border-color:var(--muted)}
.stage{position:relative;flex:1;min-height:0;background:var(--panel)}
canvas{display:block;width:100%;height:100%;cursor:grab;touch-action:none}
canvas:active{cursor:grabbing}
.overlay{position:absolute;pointer-events:none}
.legend{
  top:14px;right:16px;pointer-events:auto;
  background:color-mix(in srgb,var(--panel) 82%,transparent);
  border:1px solid var(--faint);border-radius:10px;padding:10px 11px 11px;
  backdrop-filter:blur(4px);
}
.legend .t{font-size:11px;color:var(--muted);margin-bottom:7px;letter-spacing:.3px;text-transform:uppercase}
.legend .bar-wrap{display:flex;gap:8px;align-items:stretch}
.legend .grad{
  width:14px;height:188px;border-radius:4px;
  background:linear-gradient(to top,
    rgb(40,90,160),rgb(150,190,225),rgb(232,233,236),rgb(245,175,110),rgb(200,70,35));
  border:1px solid var(--faint);
}
.legend .ticks{display:flex;flex-direction:column;justify-content:space-between;font-size:11px;color:var(--muted);font-variant-numeric:tabular-nums}
.legend .ticks span{line-height:1}
.hint{
  bottom:12px;left:50%;transform:translateX(-50%);
  color:var(--muted);font-size:12px;background:color-mix(in srgb,var(--panel) 80%,transparent);
  padding:5px 12px;border-radius:20px;border:1px solid var(--faint);
}
.cross{position:absolute;pointer-events:none;color:var(--accent);font-variant-numeric:tabular-nums;font-size:12px;background:color-mix(in srgb,var(--panel) 90%,transparent);border:1px solid var(--faint);padding:4px 8px;border-radius:6px;display:none;white-space:nowrap}
</style>
</head>
<body>
<div class="bar">
  <h1>Mặt điều khiển fuzzy — @@NAME@@</h1>
  <span class="sub" id="sub">z = f(e, ed)</span>
  <span class="sp"></span>
  <button id="reset">Đặt lại góc nhìn</button>
</div>
<div class="stage" id="stage">
  <canvas id="cv"></canvas>
  <div class="overlay legend">
    <div class="t">output (PWM)</div>
    <div class="bar-wrap">
      <div class="grad"></div>
      <div class="ticks" id="zticks"></div>
    </div>
  </div>
  <div class="overlay hint">Kéo để xoay · Cuộn để thu phóng · Di chuột để đọc giá trị</div>
  <div class="cross" id="cross"></div>
</div>
<script>
const D = @@DATA@@;
const X = D.axis, Y = D.axisY, G = D.grid;
const NX = X.length, NY = Y.length;
const ZMIN = D.zRange[0], ZMAX = D.zRange[1];
const ZC = (ZMIN+ZMAX)/2, ZH = ((ZMAX-ZMIN)/2)||1;
const X0=D.xRange[0],X1=D.xRange[1],Y0=D.yRange[0],Y1=D.yRange[1];
const nx=v=>(v-X0)/(X1-X0)*2-1;
const ny=v=>(v-Y0)/(Y1-Y0)*2-1;
const nz=v=>(v-ZC)/ZH;

// legend ticks
(function(){
  const wrap=document.getElementById('zticks');
  const ts=[ZMIN, ZMIN+(ZMAX-ZMIN)*.25, ZC, ZMIN+(ZMAX-ZMIN)*.75, ZMAX];
  wrap.innerHTML=ts.map(t=>'<span>'+t.toFixed(3)+'</span>').join('');
})();

const STOPS=[[-1,[40,90,160]],[-0.5,[150,190,225]],[0,[232,233,236]],[0.5,[245,175,110]],[1,[200,70,35]]];
function cmap(t){
  t=t<-1?-1:t>1?1:t;
  for(let i=0;i<STOPS.length-1;i++){const a=STOPS[i],b=STOPS[i+1];
    if(t>=a[0]&&t<=b[0]){const f=(t-a[0])/(b[0]-a[0]);
      const r=Math.round(a[1][0]+(b[1][0]-a[1][0])*f);
      const g=Math.round(a[1][1]+(b[1][1]-a[1][1])*f);
      const bl=Math.round(a[1][2]+(b[1][2]-a[1][2])*f);
      return 'rgb('+r+','+g+','+bl+')';}}
  return t>0?'rgb(200,70,35)':'rgb(40,90,160)';
}

const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
const stage=document.getElementById('stage');
let W,H,DPR,cx,cy,scale=1,azim=-0.95,elev=1.02,zoom=1;
let theme={ink:'#000',muted:'#888',grid:'#ccc',faint:'#ddd',line:'rgba(0,0,0,.1)',panel:'#fff'};

function readTheme(){
  const cs=getComputedStyle(document.body);
  theme.ink=cs.color;
  theme.panel=cs.backgroundColor;
  theme.grid=getComputedStyle(document.documentElement).getPropertyValue('--grid').trim()||'#ccc';
  theme.faint=getComputedStyle(document.documentElement).getPropertyValue('--faint').trim()||'#ddd';
  theme.muted=getComputedStyle(document.documentElement).getPropertyValue('--muted').trim()||'#888';
}
function resize(){
  DPR=window.devicePixelRatio||1;
  const r=stage.getBoundingClientRect();
  W=r.width;H=r.height;
  cv.width=Math.round(W*DPR);cv.height=Math.round(H*DPR);
  cv.style.width=W+'px';cv.style.height=H+'px';
  cx=W*0.5;cy=H*0.54;
  scale=Math.min(W,H)*0.30;
  draw();
}
function project(x,y,z){
  const cA=Math.cos(azim),sA=Math.sin(azim),cE=Math.cos(elev),sE=Math.sin(elev);
  const x1=x*cA-y*sA, y1=x*sA+y*cA, z1=z;
  const y2=y1*cE-z1*sE, z2=y1*sE+z1*cE;
  return {sx:cx+zoom*scale*x1, sy:cy-zoom*scale*z2, depth:y2};
}
function P(x,y,z){return project(nx(x),ny(y),nz(z));}

let prims=[];
function line(a,b,col,w,ty){prims.push({d:(a.depth+b.depth)/2,ty:ty||'line',a:a,b:b,col,col});}
function txt(p,s,off,al,col,wd){prims.push({d:p.depth,ty:'txt',p:p,s:s,off:off,al:al||'left',col:col,wd:wd});}

function build(){
  prims=[];
  // surface quads
  for(let j=0;j<NY-1;j++)for(let i=0;i<NX-1;i++){
    const z00=G[j][i],z10=G[j][i+1],z11=G[j+1][i+1],z01=G[j+1][i];
    const c=[P(X[i],Y[j],z00),P(X[i+1],Y[j],z10),P(X[i+1],Y[j+1],z11),P(X[i],Y[j+1],z01)];
    let d=0;for(const k of c)d+=k.depth;d/=4;
    const zavg=(z00+z10+z11+z01)/4;
    prims.push({d:d,ty:'quad',c:c,fill:cmap((zavg-ZC)/ZH)});
  }
  // cube corners (normalized)
  const C={};
  for(const a of [X0,X1])for(const b of [Y0,Y1])for(const cc of [ZMIN,ZMAX])C[''+a+','+b+','+cc]=project(nx(a),ny(b),nz(cc));
  const E=[[X0,Y0,ZMIN,X1,Y0,ZMIN],[X0,Y1,ZMIN,X1,Y1,ZMIN],[X0,Y0,ZMAX,X1,Y0,ZMAX],[X0,Y1,ZMAX,X1,Y1,ZMAX],
           [X0,Y0,ZMIN,X0,Y1,ZMIN],[X1,Y0,ZMIN,X1,Y1,ZMIN],[X0,Y0,ZMAX,X0,Y1,ZMAX],[X1,Y0,ZMAX,X1,Y1,ZMAX],
           [X0,Y0,ZMIN,X0,Y0,ZMAX],[X1,Y0,ZMIN,X1,Y0,ZMAX],[X0,Y1,ZMIN,X0,Y1,ZMAX],[X1,Y1,ZMIN,X1,Y1,ZMAX]];
  for(const e of E)line(C[''+e[0]+','+e[1]+','+e[2]],C[''+e[3]+','+e[4]+','+e[5]],theme.grid,1,'edge');
  // floor grid (z=zmin)
  const gx=[X0,X0+(X1-X0)*.25,X0+(X1-X0)*.5,X0+(X1-X0)*.75,X1];
  const gy=[Y0,Y0+(Y1-Y0)*.25,Y0+(Y1-Y0)*.5,Y0+(Y1-Y0)*.75,Y1];
  for(const t of gx)line(project(nx(t),ny(Y0),nz(ZMIN)),project(nx(t),ny(Y1),nz(ZMIN)),theme.faint,1,'edge');
  for(const t of gy)line(project(nx(X0),ny(t),nz(ZMIN)),project(nx(X1),ny(t),nz(ZMIN)),theme.faint,1,'edge');
  // 3 axes (strong)
  const ax=P(X0,Y0,ZMIN),bx=P(X1,Y0,ZMIN),ay=P(X0,Y1,ZMIN),az=P(X0,Y0,ZMAX);
  line(ax,bx,theme.ink,1.6,'axis');line(ax,ay,theme.ink,1.6,'axis');line(ax,az,theme.ink,1.6,'axis');
  // ticks + labels
  ctx.font='11px -apple-system,Segoe UI,Roboto,sans-serif';
  for(const t of gx){const p=P(t,Y0,ZMIN);txt(p,t.toFixed(1),[0,16],'center',theme.muted);}
  for(const t of gy){const p=P(X0,t,ZMIN);txt(p,t.toFixed(1),[-12,4],'right',theme.muted);}
  const zt=[ZMIN,ZMIN+(ZMAX-ZMIN)*.25,ZC,ZMIN+(ZMAX-ZMIN)*.75,ZMAX];
  for(const t of zt){const p=P(X0,Y0,t);txt(p,t.toFixed(2),[10,4],'left',theme.muted);}
  // axis titles
  txt(P((X0+X1)/2,Y0,ZMIN),D.xName+'  →',[0,34],'center',theme.ink,'bold');
  txt(P(X0,(Y0+Y1)/2,ZMIN),D.yName,[ -30,-4],'right',theme.ink,'bold');
  txt(P(X0,Y0,(ZMIN+ZMAX)/2),D.zName+' ↑',[14,-4],'left',theme.ink,'bold');
}

function draw(){
  readTheme();build();
  ctx.setTransform(DPR,0,0,DPR,0,0);
  ctx.clearRect(0,0,W,H);
  prims.sort((p,q)=>q.d-p.d);
  for(const o of prims){
    if(o.ty==='quad'){
      ctx.beginPath();ctx.moveTo(o.c[0].sx,o.c[0].sy);
      for(let k=1;k<4;k++)ctx.lineTo(o.c[k].sx,o.c[k].sy);ctx.closePath();
      ctx.fillStyle=o.fill;ctx.fill();
      ctx.lineWidth=0.5;ctx.strokeStyle='rgba(0,0,0,.10)';ctx.stroke();
    }else if(o.ty==='edge'){
      ctx.beginPath();ctx.moveTo(o.a.sx,o.a.sy);ctx.lineTo(o.b.sx,o.b.sy);
      ctx.lineWidth=1;ctx.strokeStyle=o.col;ctx.stroke();
    }else if(o.ty==='axis'){
      ctx.beginPath();ctx.moveTo(o.a.sx,o.a.sy);ctx.lineTo(o.b.sx,o.b.sy);
      ctx.lineWidth=1.6;ctx.strokeStyle=o.col;ctx.stroke();
    }else if(o.ty==='line'){
      ctx.beginPath();ctx.moveTo(o.a.sx,o.a.sy);ctx.lineTo(o.b.sx,o.b.sy);
      ctx.lineWidth=o.col.lineWidth||1;ctx.strokeStyle=o.col;ctx.stroke();
    }else if(o.ty==='txt'){
      ctx.font=(o.wd==='bold'?'bold ':'')+'12px -apple-system,Segoe UI,Roboto,sans-serif';
      ctx.fillStyle=o.col;ctx.textAlign=o.al;ctx.textBaseline='middle';
      ctx.fillText(o.s,o.p.sx+(o.off[0]),o.p.sy+(o.off[1]));
    }
  }
}

// interaction
let dragging=false,lx=0,ly=0,raf=0;
function sched(){if(raf)return;raf=requestAnimationFrame(()=>{raf=0;draw();});}
cv.addEventListener('pointerdown',e=>{dragging=true;lx=e.clientX;ly=e.clientY;cv.setPointerCapture(e.pointerId);});
cv.addEventListener('pointerup',e=>{dragging=false;});
cv.addEventListener('pointermove',e=>{
  if(dragging){azim-=(e.clientX-lx)*0.01;elev+=(e.clientY-ly)*0.008;
    elev=elev<0.08?0.08:elev>1.53?1.53:elev;lx=e.clientX;ly=e.clientY;sched();}
  else{crosshair(e);}
});
cv.addEventListener('wheel',e=>{e.preventDefault();zoom*=e.deltaY<0?1.08:0.925;zoom=zoom<0.4?0.4:zoom>3?3:zoom;sched();},{passive:false});
document.getElementById('reset').onclick=()=>{azim=-0.95;elev=1.02;zoom=1;sched();};

// crosshair readout: find nearest grid cell under cursor
const crossEl=document.getElementById('cross');
function crosshair(e){
  const r=cv.getBoundingClientRect();const mx=e.clientX-r.left,my=e.clientY-r.top;
  let best=null,bd=1e9;
  for(let j=0;j<NY;j++)for(let i=0;i<NX;i++){
    const p=P(X[i],Y[j],G[j][i]);const dx=p.sx-mx,dy=p.sy-my;const dd=dx*dx+dy*dy;
    if(dd<bd){bd=dd;best={i,j,p,z:G[j][i]};}
  }
  if(best&&bd<900){
    crossEl.style.display='block';
    crossEl.style.left=Math.max(8,Math.min(W-150,best.p.sx+12))+'px';
    crossEl.style.top=Math.max(8,best.p.sy-30)+'px';
    crossEl.innerHTML=D.xName+'='+X[best.i].toFixed(2)+' &nbsp;'+D.yName+'='+Y[best.j].toFixed(2)+'<br><b>z='+best.z.toFixed(3)+'</b>';
  }else crossEl.style.display='none';
}

// theme + resize observers
const mo=new MutationObserver(()=>sched());
mo.observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
if(window.matchMedia){const mq=window.matchMedia('(prefers-color-scheme: dark)');if(mq.addEventListener)mq.addEventListener('change',()=>sched());}
window.addEventListener('resize',resize);
readTheme();resize();
</script>
</body>
</html>
'''

html = HTML.replace('@@DATA@@', DATA).replace('@@NAME@@', NAME)
out = _args.out
with open(out, 'w') as f:
    f.write(html)
print('wrote', out, '(%d bytes)' % len(html))
