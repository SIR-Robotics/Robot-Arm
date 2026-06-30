// ─── web_ui.cpp — HTML/CSS/JS payload (kept here so main_new.cpp stays small)
#include "web_ui.h"

const char HTML_PAGE[] = R"raw(<!DOCTYPE html><html lang=en><head><meta charset=UTF-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no"><title>RoboArm</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
:root{--bg:#0b0f1a;--p1:#16213e;--p2:#0f1f3a;--ac:#e94560;--ac2:#27ae60;--ac3:#4fc3f7;--tx:#eef;--mu:#666}
body{font-family:Segoe UI,system-ui,sans-serif;background:radial-gradient(circle at 50% 0%,#1a2540 0,var(--bg) 60%);color:var(--tx);min-height:100vh;padding:10px;overscroll-behavior:none}
.hud{display:flex;align-items:center;gap:10px;background:linear-gradient(90deg,#16213e,#0f1f3a);border-radius:10px;padding:8px 12px;margin-bottom:10px;border:1px solid #2a3a5e;font-size:.78rem}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;box-shadow:0 0 0 0 #444;transition:background .3s,box-shadow .3s}
.dot.ok{background:var(--ac2);box-shadow:0 0 8px var(--ac2)}
.dot.err{background:var(--ac);box-shadow:0 0 8px var(--ac)}
.hud .tt{color:var(--ac);font-weight:700;letter-spacing:1px}
.hud .gap{flex:1}
.hud .pill{background:var(--p2);padding:3px 8px;border-radius:10px;color:var(--mu);font-size:.7rem}
.hud .pill b{color:var(--ac3);font-weight:700}
.jr{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin-bottom:10px}
.jc{background:var(--p2);border-radius:8px;padding:7px 4px;text-align:center;border:1px solid transparent;transition:border-color .15s,box-shadow .15s;cursor:pointer}
.jc:hover{border-color:#2a3a5e}
.jc.act{border-color:var(--ac);box-shadow:0 0 10px rgba(233,69,96,.3)}
.jc .l{font-size:.6rem;color:#789;letter-spacing:1px}
.jc .v{font-size:1.05rem;color:var(--ac);font-weight:800;font-family:Consolas,monospace}
.pad{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:10px}
.stk{background:linear-gradient(180deg,var(--p1),var(--p2));border-radius:14px;padding:14px;border:1px solid #2a3a5e;position:relative}
.stk .ttl{font-size:.62rem;color:var(--ac3);letter-spacing:2px;font-weight:700;margin-bottom:8px;text-transform:uppercase;text-align:center}
.stk .sub{font-size:.6rem;color:var(--mu);letter-spacing:1px;text-align:center;margin-bottom:8px}
.spad{width:100%;aspect-ratio:1/1;max-width:230px;margin:0 auto;background:radial-gradient(circle at 50% 50%,#0a1428 0,#040814 100%);border-radius:50%;position:relative;touch-action:none;border:2px solid #2a3a5e;box-shadow:inset 0 0 25px rgba(0,0,0,.6)}
.spad::before{content:"";position:absolute;left:50%;top:0;width:1px;height:100%;background:rgba(79,195,247,.08)}
.spad::after{content:"";position:absolute;top:50%;left:0;height:1px;width:100%;background:rgba(79,195,247,.08)}
.knob{width:34%;aspect-ratio:1/1;background:radial-gradient(circle at 35% 30%,#ff6b88,var(--ac));border-radius:50%;position:absolute;left:33%;top:33%;box-shadow:0 4px 14px rgba(233,69,96,.45),inset 0 -3px 8px rgba(0,0,0,.35);transition:background .15s,box-shadow .15s,transform .05s}
.spad.act .knob{background:radial-gradient(circle at 35% 30%,#5be095,var(--ac2));box-shadow:0 4px 18px rgba(39,174,96,.55),inset 0 -3px 8px rgba(0,0,0,.35)}
.tg{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:10px}
.tw{background:linear-gradient(180deg,var(--p1),var(--p2));border-radius:14px;padding:12px 14px;border:1px solid #2a3a5e}
.tw .ttl{font-size:.62rem;color:var(--ac3);letter-spacing:2px;font-weight:700;margin-bottom:8px;text-transform:uppercase;text-align:center}
.trk{width:60px;height:130px;margin:0 auto;background:radial-gradient(circle at 50% 50%,#0a1428,#040814);border-radius:30px;position:relative;touch-action:none;border:2px solid #2a3a5e;overflow:hidden;box-shadow:inset 0 0 15px rgba(0,0,0,.6)}
.tnb{width:46px;height:46px;background:radial-gradient(circle at 35% 30%,#ff6b88,var(--ac));border-radius:50%;position:absolute;left:5px;top:40px;box-shadow:0 3px 10px rgba(233,69,96,.4),inset 0 -3px 6px rgba(0,0,0,.35);transition:background .15s}
.trk.act .tnb{background:radial-gradient(circle at 35% 30%,#5be095,var(--ac2));box-shadow:0 3px 14px rgba(39,174,96,.5),inset 0 -3px 6px rgba(0,0,0,.35)}
.p{background:var(--p1);border-radius:12px;padding:12px;margin-bottom:10px;border:1px solid #2a3a5e}
.p h2{font-size:.62rem;color:var(--ac);margin-bottom:10px;text-transform:uppercase;letter-spacing:3px;font-weight:700}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:7px}
button{padding:11px 6px;border:0;border-radius:8px;background:linear-gradient(180deg,#1a3060,#0f3460);color:var(--tx);font-size:.78rem;cursor:pointer;font-weight:700;line-height:1.3;transition:transform .08s,background .15s,box-shadow .15s;border:1px solid #2a4070}
button:hover{background:linear-gradient(180deg,#3a1730,var(--ac));border-color:var(--ac)}
button:active{transform:scale(.96)}
button.on{background:linear-gradient(180deg,#3a1730,var(--ac));border-color:var(--ac);box-shadow:0 0 10px rgba(233,69,96,.4)}
button.red{background:linear-gradient(180deg,#3a0f0f,#5a1a1a);border-color:#5a2020}
button.red:hover{background:linear-gradient(180deg,#5a1a1a,#c0392b);border-color:#c0392b}
button.grn{background:linear-gradient(180deg,#0f3a1a,#1a5a2a);border-color:#205020}
button.grn:hover{background:linear-gradient(180deg,#1a5a2a,#27ae60);border-color:var(--ac2)}
button.amb{background:linear-gradient(180deg,#3a2a00,#5a4400);font-size:.88rem;letter-spacing:.5px;border-color:#5a4400}
button.amb:hover{background:linear-gradient(180deg,#5a4400,#e6a817);border-color:#e6a817}
button.amb.on{background:linear-gradient(180deg,#e6a817,#c08810);color:#111;border-color:#e6a817;box-shadow:0 0 10px rgba(230,168,23,.5)}
.bd{display:inline-block;background:var(--ac);color:#fff;border-radius:10px;font-size:.62rem;padding:1px 7px;margin-left:5px;vertical-align:middle;font-weight:700}
.pl{max-height:170px;overflow-y:auto;margin-top:10px}
.pi{display:flex;align-items:center;gap:7px;padding:6px 9px;border-radius:6px;margin-bottom:3px;background:var(--p2);border:1px solid transparent;transition:border-color .12s}
.pi:hover{border-color:#2a3a5e}
.pi.cur{border-color:var(--ac2);box-shadow:0 0 8px rgba(39,174,96,.3)}
.pnn{color:var(--ac);font-weight:800;font-size:.76rem;min-width:26px}
.pll{color:#dde;font-size:.78rem;cursor:pointer;flex:1;padding:2px 4px;border-radius:4px}
.pll:hover{background:#1a3060;color:var(--ac)}
.pa{color:#556;font-size:.64rem;cursor:pointer;font-family:Consolas,monospace}
.pa:hover{color:var(--ac3)}
.ri{background:var(--p2);color:var(--tx);border:1px solid var(--ac);border-radius:4px;padding:2px 5px;font-size:.78rem;width:120px;outline:none}
.ps{display:flex;flex-direction:column;align-items:center;gap:3px;padding:9px 6px}
.ps .pnm{font-size:.85rem;font-weight:800;color:var(--tx);padding:1px 6px;border-radius:4px}
.ps .pnm:hover{background:rgba(79,195,247,.15);color:var(--ac3)}
.ps .plc{font-size:.6rem;color:#789;letter-spacing:1px;text-transform:uppercase}
.ps.seq .plc{color:var(--ac2)}
#toast{position:fixed;left:50%;transform:translate(-50%,0);bottom:18px;background:rgba(20,28,50,.95);border:1px solid var(--ac);color:var(--tx);padding:9px 16px;border-radius:22px;font-size:.78rem;font-weight:600;opacity:0;pointer-events:none;transition:opacity .25s,transform .25s;box-shadow:0 4px 18px rgba(0,0,0,.5);z-index:99}
#toast.show{opacity:1;transform:translate(-50%,-6px)}
.kbd{font-size:.62rem;color:#456;margin-top:5px;text-align:center;letter-spacing:1px}
.kbd kbd{background:#0a1428;border:1px solid #2a3a5e;border-radius:3px;padding:1px 5px;margin:0 1px;font-family:Consolas,monospace;color:var(--ac3)}
@media(max-width:520px){.jr{grid-template-columns:repeat(3,1fr)}.jc .v{font-size:.95rem}.spad{max-width:180px}}
</style></head><body>
<div class=hud><span class=dot id=dt></span><span class=tt>ROBOARM</span><span class=gap></span><button id=ikBtn class=pill style="cursor:pointer;border:none;font-size:.7rem;padding:3px 8px;border-radius:10px;background:var(--p2);color:var(--mu)" onclick=toggleIK() title="Toggle IK control mode">JOINT</button><span class=pill id=fsm style="color:#fff;background:#444;font-weight:700">--</span><span class=pill id=ip>--</span><span class=pill>RSSI <b id=rs>--</b></span><span class=pill id=gpd style=display:none>&#127918; <b>GP</b></span></div>
<div class=jr id=jr></div>
<div class=pad>
 <div class=stk><div class=ttl id=sLt>LEFT STICK</div><div class=sub id=sLs>BASE / SHOULDER</div><div class=spad id=sL data-jx=0 data-jy=1><div class=knob></div></div><div class=kbd id=sLk><kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd></div></div>
 <div class=stk><div class=ttl id=sRt>RIGHT STICK</div><div class=sub id=sRs>ELBOW / WRIST P.</div><div class=spad id=sR data-jx=2 data-jy=3><div class=knob></div></div><div class=kbd id=sRk><kbd>I</kbd><kbd>J</kbd><kbd>K</kbd><kbd>L</kbd></div></div>
</div>
<div class=tg>
 <div class=tw><div class=ttl id=tRt>WRIST ROLL</div><div class=trk id=tR data-j=4><div class=tnb></div></div><div class=kbd><kbd>Q</kbd><kbd>E</kbd></div></div>
 <div class=tw><div class=ttl>GRIPPER</div><div class=trk id=tG data-j=5><div class=tnb></div></div><div class=kbd><kbd>Z</kbd><kbd>X</kbd></div></div>
</div>
<div class=p><h2>FK Position</h2><div id=fk style="font-family:Consolas,monospace;font-size:.82rem;color:var(--ac3);text-align:center;letter-spacing:.4px">--</div></div>
<div class=p><h2>Fine Jog</h2><div id=fjp></div></div>
<div class=p><h2>IK Move</h2>
 <div style="display:grid;grid-template-columns:repeat(4,1fr) 70px;gap:6px;align-items:center;margin-bottom:7px">
  <input id=mvx type=number placeholder=X step=1 class=ri style="width:auto">
  <input id=mvy type=number placeholder=Y step=1 class=ri style="width:auto">
  <input id=mvz type=number placeholder=Z step=1 class=ri style="width:auto">
  <input id=mvr type=number placeholder=Ry step=1 class=ri style="width:auto">
  <button class=grn onclick=mv()>GO</button>
 </div>
 <div class=kbd>Ry empty = free wrist pitch (wider reach)</div>
</div>
<div class=p><h2>Presets</h2><div class=g4 id=psg></div><div class=kbd style=margin-top:7px>Tap card to run &mdash; tap name to rename &mdash; long-press to save current pose</div></div>
<div class=p><h2>Recording <span class=bd id=rc>0</span></h2>
 <div class=g4 style=margin-bottom:7px>
  <button class=grn onclick="w('RC');tt('Recorded')">&#9679; REC</button>
  <button id=bp onclick="w('PY')">&#9654; Play</button>
  <button class=red onclick="w('ST')">&#9646;&#9646; Stop</button>
  <button class=red onclick="dc()">&#128465; Clear</button>
 </div>
 <button id=bc class=amb style=width:100%;margin-bottom:7px onclick="w('CY')">&#9654;&#9654; CYCLE LOOP &mdash; OFF</button>
 <div class=g2>
  <button onclick="w('SA');tt('Saved')">&#128190; Save</button>
  <button onclick="w('LD');tt('Loading')">&#128228; Load</button>
 </div>
 <div class=g2 style=margin-top:7px>
  <button onclick="eP()">&#11015; Export JSON</button>
  <button onclick="document.getElementById('iF').click()">&#11014; Import JSON</button>
 </div>
 <button onclick="sSP()" style=width:100%;margin-top:7px>&#128190; Save sequence to preset&hellip;</button>
 <input type=file id=iF accept=".json,application/json" style=display:none>
 <div class=pl id=pls></div>
 <div class=kbd style=margin-top:8px><kbd>SPACE</kbd>rec <kbd>P</kbd>play <kbd>O</kbd>stop <kbd>C</kbd>cycle <kbd>H</kbd>home</div>
</div>
<div id=toast></div>
<script>
const JD=[{n:'BASE',k:'B',mn:0,mx:180,hm:90},{n:'SHOULDER',k:'S',mn:30,mx:150,hm:90},{n:'ELBOW',k:'E',mn:0,mx:135,hm:90},{n:'WRIST P',k:'WP',mn:0,mx:180,hm:90},{n:'WRIST R',k:'WR',mn:0,mx:180,hm:90},{n:'GRIPPER',k:'G',mn:0,mx:90,hm:45}];
const FKD=['X','Y','Z','Ry','Rx','G'];
let ps=[],s,rT,lastA=[90,90,90,90,90,45],lastPlayIdx=-1,ikMode=false;
let ikD=[0,0,0,0,0],likD=[0,0,0,0,0];  // IK deltas + last-sent for dedup

function toggleIK(){
 ikMode=!ikMode;
 ikD=[0,0,0,0,0];likD=[0,0,0,0,0];
 w(ikMode?'IK:1':'IK:0');
 updateStickLabels();
}
function updateStickLabels(){
 const E=ikMode?'IK MODE':'JOINT MODE';
 document.getElementById('ikBtn').textContent=E;
 document.getElementById('ikBtn').style.color=ikMode?'#27ae60':'var(--mu)';
 document.getElementById('ikBtn').style.background=ikMode?'#0f3a1a':'var(--p2)';
 document.getElementById('sLs').textContent=ikMode?'X / Y (world)':'BASE / SHOULDER';
 document.getElementById('sRs').textContent=ikMode?'Z / Ry (pitch)':'ELBOW / WRIST P.';
 document.getElementById('sLk').innerHTML=ikMode?'<kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>':'<kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>';
 document.getElementById('sRk').innerHTML=ikMode?'<kbd>K</kbd><kbd>J</kbd><kbd>I</kbd><kbd>L</kbd>':'<kbd>I</kbd><kbd>J</kbd><kbd>K</kbd><kbd>L</kbd>';
}
function sendIkDeltas(){
 let ch=false;
 for(let i=0;i<5;i++){if(ikD[i]!==likD[i])ch=true;likD[i]=ikD[i]}
 if(ch)w('ID:'+ikD.join(':'));
}
let pst=[{n:'Home',l:1},{n:'Ready',l:1},{n:'Pick',l:1},{n:'Place',l:1}];

// Joint chips — click to manually set angle (server clamps to joint limits)
const jr=document.getElementById('jr');
JD.forEach((j,i)=>jr.insertAdjacentHTML('beforeend',`<div class=jc id=jc${i} title="Tap to set angle"><div class=l>${j.k}</div><div class=v id=v${i}>--</div></div>`));
JD.forEach((j,i)=>document.getElementById('jc'+i).addEventListener('click',()=>edJ(i)));
let edBusy=-1;
function edJ(i){
 if(edBusy===i)return;
 edBusy=i;
 const v=document.getElementById('v'+i),cur=lastA[i];
 v.innerHTML=`<input class=ri type=number value="${cur}" min=0 max=270 style="width:62px;font-size:.9rem;text-align:center">`;
 const I=v.querySelector('input');I.focus();I.select();
 const done=ok=>{
  if(ok){
   const a=parseInt(I.value);
   if(!isNaN(a))w('SV:'+i+':'+a);
  }
  v.textContent=lastA[i]+'°';
  edBusy=-1;
 };
 I.onblur=()=>done(true);
 I.onkeydown=ev=>{
  if(ev.key==='Enter')done(true);
  else if(ev.key==='Escape')done(false);
  ev.stopPropagation();
 };
}

// ── WebSocket
function cn(){
 s=new WebSocket(`ws://${location.hostname}/ws`);
 s.onopen=()=>{sd(1);tt('Connected');clearTimeout(rT)};
 s.onclose=()=>{sd(0);tt('Disconnected');rT=setTimeout(cn,2500)};
 s.onerror=()=>s.close();
 s.onmessage=(e)=>{const d=JSON.parse(e.data);if(d.t==='s')aS(d);else if(d.t==='p')aP(d);else if(d.t==='pl')aL(d)};
}
function w(x){if(s&&s.readyState===1)s.send(x)}
function sd(o){const d=document.getElementById('dt');d.classList.toggle('ok',!!o);d.classList.toggle('err',!o)}

// ── Toast
let tID=0;
function tt(m){const e=document.getElementById('toast');e.textContent=m;e.classList.add('show');clearTimeout(tID);tID=setTimeout(()=>e.classList.remove('show'),1500)}

// ── Apply status / poses
function aS(d){
 d.j.forEach((a,i)=>{
  if(edBusy!==i)document.getElementById('v'+i).textContent=a+'°';
  const c=document.getElementById('jc'+i);
  if(a!==lastA[i]){c.classList.add('act');clearTimeout(c._t);c._t=setTimeout(()=>c.classList.remove('act'),250)}
  lastA[i]=a;
 });
 document.getElementById('rc').textContent=d.l;
 document.getElementById('bp').classList.toggle('on',!!d.p);
 const cb=document.getElementById('bc');cb.classList.toggle('on',!!d.c);
 cb.innerHTML=d.c?'■ CYCLE LOOP &mdash; ON':'&#9654;&#9654; CYCLE LOOP &mdash; OFF';
 if(typeof d.r==='number')document.getElementById('rs').textContent=d.r+' dBm';
 if(typeof d.x==='number'){
  document.getElementById('fk').textContent=`X: ${d.x} mm   Y: ${d.y} mm   Z: ${d.z} mm   |   Rx: ${d.rx}°   Ry: ${d.ry}°   Rz: ${d.rz}°`;
 }
 // Refresh fine-jog angle readouts alongside the joint chips
 d.j.forEach((a,i)=>{const fv=document.getElementById('fv'+i);if(fv)fv.textContent=a+'°'});
 if(typeof d.b==='number'){
  const FSM=['INIT','HOMING','IDLE','BUSY','FAULT'];
  const COL=['#666','#3a78c9','#27ae60','#e6a817','#e94560'];
  const f=document.getElementById('fsm');
  f.textContent=FSM[d.b]||'?';f.style.background=COL[d.b]||'#444';
 }
if(d.c||d.p){
   if(d.i!==lastPlayIdx){lastPlayIdx=d.i;document.querySelectorAll('.pi').forEach((p,k)=>p.classList.toggle('cur',k===d.i-1))}
  } else if(lastPlayIdx!==-1){lastPlayIdx=-1;document.querySelectorAll('.pi').forEach(p=>p.classList.remove('cur'))}
  if(typeof d.ik==='number'&&!!d.ik!==ikMode){ikMode=!!d.ik;updateStickLabels()}
}
function aP(d){ps=d.i||[];document.getElementById('rc').textContent=d.c;rP()}
function eh(x){return x.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')}
function rP(){
 const e=document.getElementById('pls');e.innerHTML='';
 ps.forEach((p,i)=>{
  const d=document.createElement('div');d.className='pi';
  d.innerHTML=`<span class=pnn>#${i+1}</span><span class=pll id=pl${i}>${eh(p.n)}</span><span class=pa title="Go to FK pose">${p.a.map((v,k)=>FKD[k]+v).join(' ')}</span>`;
  d.querySelector('.pll').onclick=()=>sR(i);
  d.querySelector('.pa').onclick=()=>{w('GT:'+i);tt('Go #'+(i+1))};
  e.appendChild(d);
 });
}
function sR(i){
 const e=document.getElementById('pl'+i),o=ps[i].n;
 e.innerHTML=`<input class=ri type=text value="${eh(o)}" maxlength=19>`;
 const I=e.querySelector('input');I.focus();I.select();
 I.onblur=()=>{const v=I.value.trim().replace(/:/g,' ');if(v&&v!==o){w('RN:'+i+':'+v);tt('Renamed')}else e.textContent=o};
 I.onkeydown=ev=>{if(ev.key==='Enter')I.blur();if(ev.key==='Escape'){I.value=o;I.blur()}ev.stopPropagation()};
}

// ── XY thumbsticks
const sticks=[];
function bindStick(el){
 const jx=+el.dataset.jx,jy=+el.dataset.jy;
 const knob=el.querySelector('.knob');
 const st={el,knob,jx,jy,vx:0,vy:0,lvx:0,lvy:0,active:false,kbx:0,kby:0,gpx:0,gpy:0};
 function send(){w('JX:'+jx+':'+st.vx+':'+jy+':'+(st.vy))}
 st.send=send;
 function compose(){
  const cx=st.dx||st.kbx||st.gpx, cy=st.dy||st.kby||st.gpy;
  st.vx=Math.round(cx*100);st.vy=Math.round(cy*100);
  const r=el.getBoundingClientRect(),k=knob.getBoundingClientRect();
  const R=(r.width-k.width)/2;
  knob.style.left=`calc(33% + ${-cx*R}px)`;knob.style.top=`calc(33% + ${-cy*R}px)`;
  const on=Math.abs(st.vx)>4||Math.abs(st.vy)>4;
  el.classList.toggle('act',on);st.active=on;
 }
 st.compose=compose;
 function start(e){st.dragging=true;e.preventDefault()}
 function end(){if(!st.dragging)return;st.dragging=false;st.dx=0;st.dy=0;compose();send()}
 function move(e){
  if(!st.dragging)return;
  const r=el.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
  const px=e.touches?e.touches[0].clientX:e.clientX;
  const py=e.touches?e.touches[0].clientY:e.clientY;
  let dx=(px-cx)/(r.width/2),dy=(py-cy)/(r.height/2);
  const mag=Math.hypot(dx,dy);if(mag>1){dx/=mag;dy/=mag}
  // Negate so touch matches keyboard convention (and the flipped visual).
  st.dx=-dx;st.dy=-dy;compose();
 }
 st._move=move;st._end=end;
 el.addEventListener('mousedown',start);el.addEventListener('touchstart',start,{passive:false});
 sticks.push(st);
}
bindStick(document.getElementById('sL'));
bindStick(document.getElementById('sR'));
addEventListener('mousemove',e=>sticks.forEach(s=>s._move(e)));
addEventListener('touchmove',e=>sticks.forEach(s=>s._move(e)),{passive:false});
addEventListener('mouseup',()=>sticks.forEach(s=>s._end()));
addEventListener('touchend',()=>sticks.forEach(s=>s._end()));

// ── Triggers (vertical)
const trigs=[];
function bindTrig(el){
 const j=+el.dataset.j,nb=el.querySelector('.tnb');
 const tg={el,nb,j,v:0,lv:0,kb:0,gp:0,L:0};
 function send(){w('JG:'+j+':'+tg.v)}
 tg.send=send;
 function pos(){
  const cy=tg.dy!==undefined?tg.dy:(tg.kb||tg.gp);
  tg.v=Math.round(cy*100);
  nb.style.top=(40-cy*38)+'px';
  const on=Math.abs(tg.v)>4;el.classList.toggle('act',on);
 }
 tg.pos=pos;
 function start(e){tg.dragging=true;e.preventDefault()}
 function end(){if(!tg.dragging)return;tg.dragging=false;tg.dy=undefined;pos();send()}
 function move(e){
  if(!tg.dragging)return;
  const r=el.getBoundingClientRect(),cy=e.touches?e.touches[0].clientY:e.clientY;
  let y=cy-(r.top+r.height/2);y=Math.max(-46,Math.min(46,y));
  tg.dy=-y/46;pos();
 }
 tg._move=move;tg._end=end;
 el.addEventListener('mousedown',start);el.addEventListener('touchstart',start,{passive:false});
 trigs.push(tg);
}
bindTrig(document.getElementById('tR'));
bindTrig(document.getElementById('tG'));
addEventListener('mousemove',e=>trigs.forEach(t=>t._move(e)));
addEventListener('touchmove',e=>trigs.forEach(t=>t._move(e)),{passive:false});
addEventListener('mouseup',()=>trigs.forEach(t=>t._end()));
addEventListener('touchend',()=>trigs.forEach(t=>t._end()));

// ── Keyboard
const keys={};
addEventListener('keydown',e=>{
 if(document.activeElement.tagName==='INPUT')return;
 const k=e.key.toLowerCase();
 if(keys[k])return;
 keys[k]=true;
 if(k===' '){e.preventDefault();w('RC');tt('Recorded')}
 else if(k==='p'){w('PY');tt('Play')}
 else if(k==='o'){w('ST');tt('Stop')}
 else if(k==='c'){w('CY');tt('Cycle')}
 else if(k==='h'){w('PR:0');tt('Home')}
});
addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false});

// ── Browser Gamepad API
let lastGP=null;
function pollGP(){
 const gps=navigator.getGamepads?navigator.getGamepads():[];
 let gp=null;for(const g of gps)if(g){gp=g;break}
 document.getElementById('gpd').style.display=gp?'inline-block':'none';
 if(gp){
  sticks[0].gpx=Math.abs(gp.axes[0])>.12?gp.axes[0]:0;
  sticks[0].gpy=Math.abs(gp.axes[1])>.12?gp.axes[1]:0;
  sticks[1].gpx=Math.abs(gp.axes[2])>.12?gp.axes[2]:0;
  sticks[1].gpy=Math.abs(gp.axes[3])>.12?gp.axes[3]:0;
  trigs[0].gp=(gp.buttons[7]?.value||0)-(gp.buttons[6]?.value||0);
  trigs[1].gp=(gp.buttons[5]?.value||0)-(gp.buttons[4]?.value||0);
  if(gp.buttons[0]?.pressed&&!lastGP?.[0]){w('RC');tt('Rec')}
  if(gp.buttons[1]?.pressed&&!lastGP?.[1]){w('ST');tt('Stop')}
  if(gp.buttons[2]?.pressed&&!lastGP?.[2]){w('PY');tt('Play')}
  if(gp.buttons[3]?.pressed&&!lastGP?.[3]){w('CY');tt('Cycle')}
  lastGP=gp.buttons.map(b=>b.pressed);
 }
}

// ── Tick: compose keyboard + gamepad + drag, send only on change
function tick(){
 pollGP();
 const kx=(keys.a?1:0)-(keys.d?1:0), ky=(keys.w?1:0)-(keys.s?1:0);
 const rx=(keys.l?1:0)-(keys.j?1:0), ry=(keys.k?1:0)-(keys.i?1:0);
 const tk=(keys.e?1:0)-(keys.q?1:0), tgk=(keys.x?1:0)-(keys.z?1:0);
 if(ikMode){
  // IK mode: left=X/Y, right=Z/Ry, roll=Rx, gripper still direct
  const sx1=sticks[0], sx2=sticks[1], dx1=sx1.dx||0, dy1=sx1.dy||0, dx2=sx2.dx||0, dy2=sx2.dy||0;
  const gx1=sx1.gpx||0, gy1=sx1.gpy||0, gx2=sx2.gpx||0, gy2=sx2.gpy||0;
  const t0k=trigs[0].kb||0, t0g=trigs[0].gp||0;
  const t0d=trigs[0].dy!==undefined?trigs[0].dy:0;
  // Drag overrides keyboard; gamepad overrides both
  const lx=-(dx1!==0?dx1:(gx1!==0?gx1:kx));
  const ly=-(dy1!==0?dy1:(gy1!==0?-gy1:ky));
  const lz=(dx2!==0?dx2:(gx2!==0?gx2:rx));
  const lry=(dy2!==0?-dy2:(gy2!==0?gy2:ry));
  const lrx=t0d!==0?t0d:(t0g!==0?t0g:tk);
  ikD=[Math.round(lx*100),Math.round(ly*100),Math.round(lz*100),Math.round(lry*100),Math.round(lrx*100)];
  sendIkDeltas();
  // Gripper stays direct
  trigs[1].kb=tgk;
  const tv1=(trigs[1].dy!==undefined)?trigs[1].dy:(trigs[1].kb||trigs[1].gp);
  const tv=Math.round(tv1*100);
  if(tv!==trigs[1].lv){trigs[1].lv=tv;w('JG:'+trigs[1].j+':'+tv)}
 } else {
  // Joint mode (original)
  sticks[0].kbx=kx;sticks[0].kby=ky;
  sticks[1].kbx=rx;sticks[1].kby=ry;
  trigs[0].kb=tk;trigs[1].kb=tgk;
  sticks.forEach(s=>{
   s.compose();
   if(s.vx!==s.lvx||s.vy!==s.lvy){s.send();s.lvx=s.vx;s.lvy=s.vy}
  });
  trigs.forEach(t=>{
   if(!t.dragging)t.dy=undefined;
   t.pos();
   if(t.v!==t.lv){t.send();t.lv=t.v}
  });
 }
}
setInterval(tick,50);

function dc(){if(confirm('Clear all recorded poses?')){w('CL');tt('Cleared')}}

// ── Presets: dynamic cards, populated from the pl broadcast frame ──────────
function aL(d){if(d.i&&d.i.length){pst=d.i;rPS()}}
function rPS(){
 const e=document.getElementById('psg');e.innerHTML='';
 pst.forEach((p,i)=>{
  const btn=document.createElement('button');
  btn.className='ps'+(p.l>1?' seq':'');
  btn.dataset.pi=i;
  btn.innerHTML=`<span class=pnm>${eh(p.n)}</span><span class=plc>${p.l} pose${p.l===1?'':'s'}</span>`;
  e.appendChild(btn);
 });
 bindPresets();
}
function bindPresets(){
 document.querySelectorAll('.ps').forEach(btn=>{
  const i=+btn.dataset.pi;
  const nm=btn.querySelector('.pnm');
  let timer=null,longFired=false;
  const start=e=>{
   longFired=false;
   timer=setTimeout(()=>{
    longFired=true;
    if(confirm('Save current arm position as "'+pst[i].n+'" (overwrites the sequence)?')){
     w('SP:'+i);tt(pst[i].n+' saved');
    }
   },650);
  };
  const cancel=()=>clearTimeout(timer);
  btn.addEventListener('mousedown',start);
  btn.addEventListener('touchstart',start,{passive:true});
  btn.addEventListener('mouseup',cancel);
  btn.addEventListener('mouseleave',cancel);
  btn.addEventListener('touchend',cancel);
  btn.addEventListener('touchcancel',cancel);
  btn.onclick=ev=>{
   if(longFired){longFired=false;return}
   if(ev.target===nm)return;          // name click handled separately
   w('PR:'+i);tt('Run '+pst[i].n);
  };
  // Inline rename on name click
  nm.onclick=ev=>{
   ev.stopPropagation();
   const old=pst[i].n;
   nm.innerHTML=`<input class=ri type=text value="${eh(old)}" maxlength=19 style=width:90px;font-size:.78rem>`;
   const I=nm.querySelector('input');I.focus();I.select();
   const done=ok=>{
    if(ok){
     const v=I.value.trim().replace(/:/g,' ');
     if(v&&v!==old){w('PN:'+i+':'+v);tt('Renamed');return}
    }
    nm.textContent=old;
   };
   I.onblur=()=>done(true);
   I.onkeydown=k=>{if(k.key==='Enter')done(true);if(k.key==='Escape'){nm.textContent=old}k.stopPropagation()};
  };
 });
}
rPS();   // seed with defaults so something shows before first WS frame

// ── Save current recording as a preset sequence
function sSP(){
 if(!ps.length){tt('Recording is empty');return}
 const opts=pst.map((p,i)=>(i+1)+'. '+p.n+' ('+p.l+' pose'+(p.l===1?'':'s')+')').join('\n');
 const r=prompt('Save current recording ('+ps.length+' poses) to which preset?\n'+opts+'\n\nEnter 1-'+pst.length+':');
 if(r===null)return;
 const idx=parseInt(r)-1;
 if(isNaN(idx)||idx<0||idx>=pst.length){tt('Invalid');return}
 w('SQ:'+idx);tt('Saved to '+pst[idx].n);
}

// ── Export / Import poses ────────────────────────────────────────────────
function eP(){
 fetch('/poses.json').then(r=>r.blob()).then(b=>{
  const u=URL.createObjectURL(b),a=document.createElement('a');
  a.href=u;a.download='poses.json';document.body.appendChild(a);a.click();
  a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000);
  tt('Exported '+ps.length+' pose'+(ps.length===1?'':'s'));
 }).catch(()=>tt('Export failed'));
}
document.getElementById('iF').onchange=e=>{
 const f=e.target.files[0];if(!f)return;
 const r=new FileReader();
 r.onload=()=>{
  fetch('/poses.json',{method:'POST',headers:{'Content-Type':'application/json'},body:r.result})
   .then(rs=>rs.json()).then(j=>tt('Imported '+(j.n||0)+' pose'+(j.n===1?'':'s')))
   .catch(()=>tt('Import failed'));
 };
 r.readAsText(f);e.target.value='';
};

// ── Fine jog (per-joint ±1° / ±5° nudge buttons) ─────────────────────────
const fjpEl=document.getElementById('fjp');
JD.forEach((j,i)=>{
 fjpEl.insertAdjacentHTML('beforeend',`<div style="display:grid;grid-template-columns:70px 60px repeat(4,1fr);gap:4px;align-items:center;margin-bottom:4px">
  <span style="font-size:.7rem;color:#789;letter-spacing:1px">${j.k}</span>
  <span id=fv${i} style="text-align:right;font-family:Consolas,monospace;color:var(--ac);font-weight:800;font-size:.85rem">--</span>
  <button onclick="nj(${i},-5)" style="padding:6px 0">-5</button>
  <button onclick="nj(${i},-1)" style="padding:6px 0">-1</button>
  <button onclick="nj(${i},1)"  style="padding:6px 0">+1</button>
  <button onclick="nj(${i},5)"  style="padding:6px 0">+5</button>
 </div>`);
});
// Optimistic local update so rapid clicks accumulate before the server's
// next status broadcast (server will reconcile on its broadcast tick).
function nj(i,d){lastA[i]+=d;const a=lastA[i];w('SV:'+i+':'+a);document.getElementById('v'+i).textContent=a+'°';const fv=document.getElementById('fv'+i);if(fv)fv.textContent=a+'°';tt(JD[i].k+' '+(d>0?'+':'')+d+' → '+a+'°')}

// ── IK MOVE submit ───────────────────────────────────────────────────────
function mv(){
 const x=+document.getElementById('mvx').value||0;
 const y=+document.getElementById('mvy').value||0;
 const z=+document.getElementById('mvz').value||0;
 const rv=document.getElementById('mvr').value.trim();
 if(rv){w('MV:'+x+':'+y+':'+z+':'+rv);tt('IK '+x+','+y+','+z+' Ry='+rv)}
 else{w('MV:'+x+':'+y+':'+z);tt('IK '+x+','+y+','+z+' free Ry')}
}

cn();
</script></body></html>)raw";
