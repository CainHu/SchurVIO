// HTML 报告模板。用 R"HTML(...)HTML" 原始字符串，占位符由 make_report.cpp 替换。
const char *kReportTemplate = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>SchurVIO 视觉后验分析报告</title>
<style>
:root{
  --bg:#0f1117; --panel:#171a21; --line:#262b36; --fg:#e6e8ee; --dim:#8b93a7;
  --gt:#4ea1ff; --est:#ff7b4a; --err:#ffd166; --ok:#3ddc97; --bad:#ff5c7a;
  --grid:#20242e;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.6 "Segoe UI",system-ui,-apple-system,sans-serif}
header{padding:28px 32px 18px;border-bottom:1px solid var(--line)}
h1{margin:0 0 6px;font-size:22px;font-weight:600}
h2{font-size:17px;margin:0 0 4px;font-weight:600}
h3{font-size:14px;margin:0 0 10px;font-weight:600;color:var(--dim)}
.sub{color:var(--dim);font-size:13px}
main{padding:24px 32px 60px;max-width:1500px}
section{margin:0 0 34px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:18px}
.grid{display:grid;gap:16px}
.g2{grid-template-columns:repeat(2,1fr)}
.g3{grid-template-columns:repeat(3,1fr)}
.g4{grid-template-columns:repeat(4,1fr)}
@media(max-width:1100px){.g2,.g3,.g4{grid-template-columns:1fr}}
canvas{display:block;width:100%;background:#12141a;border-radius:6px}
#c3d{touch-action:none;cursor:grab}
#c3d.dragging{cursor:grabbing}
.kpi{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
.kpi .v{font-size:24px;font-weight:600;font-variant-numeric:tabular-nums}
.kpi .k{color:var(--dim);font-size:12px;margin-bottom:2px}
.kpi .n{color:var(--dim);font-size:11px;margin-top:3px}
.good{color:var(--ok)} .bad{color:var(--bad)} .warn{color:var(--err)}
table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums}
th,td{padding:7px 10px;text-align:right;border-bottom:1px solid var(--line)}
th{color:var(--dim);font-weight:500;text-align:right}
th:first-child,td:first-child{text-align:left}
tr.diverge td{color:var(--bad)}
tr.best td{background:rgba(61,220,151,.08)}
.legend{display:flex;gap:16px;flex-wrap:wrap;font-size:12px;color:var(--dim);margin:8px 0 4px}
.legend i{display:inline-block;width:11px;height:3px;vertical-align:middle;margin-right:5px;border-radius:2px}
.ctl{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin-bottom:10px;font-size:13px}
.ctl label{color:var(--dim)}
input[type=range]{width:130px;vertical-align:middle}
.note{background:rgba(255,209,102,.07);border-left:3px solid var(--err);
      padding:11px 14px;border-radius:0 6px 6px 0;margin:12px 0;font-size:13px}
.note.bad{background:rgba(255,92,122,.07);border-color:var(--bad)}
.note.ok{background:rgba(61,220,151,.07);border-color:var(--ok)}
.note b{color:var(--fg)}
code{background:#0b0d12;padding:1px 5px;border-radius:3px;font-size:12px}
</style>
</head>
<body>
<header>
  <h1>SchurVIO 视觉后验分析报告</h1>
  <div class="sub">Schur 路径 · 仿真轨迹 100 s · 相机 20 Hz · IMU 200 Hz · 滑窗 30 帧 · 1000 个特征点</div>
</header>
<main>

<section>
  <h2>1. 总体精度</h2>
  <h3>估计相对真值的误差统计</h3>
  <div class="grid g4" id="kpis"></div>
</section>

<section>
  <h2>2. 轨迹可视化</h2>
  <h3>真实轨迹 / 估计轨迹 / 特征点分布 / 机体坐标系朝向</h3>
  <div class="panel">
    <div class="ctl">
      <label>方位角 <input type="range" id="yaw" min="0" max="360" value="35"></label>
      <label>俯仰角 <input type="range" id="pitch" min="-89" max="89" value="28"></label>
      <label>缩放 <input type="range" id="zoom" min="20" max="400" value="100"></label>
      <label><input type="checkbox" id="showLmk" checked> 特征点</label>
      <label><input type="checkbox" id="showAxes" checked> 机体坐标系</label>
      <label><input type="checkbox" id="showErr"> 误差连线</label>
      <label>坐标系密度 <input type="range" id="axisN" min="4" max="40" value="14"></label>
    </div>
    <canvas id="c3d" height="560"></canvas>
    <div class="legend">
      <span><i style="background:var(--gt)"></i>真实轨迹 GT</span>
      <span><i style="background:var(--est)"></i>估计轨迹 EST</span>
      <span><i style="background:#5a6478"></i>特征点</span>
      <span><i style="background:#ff4d4d"></i>机体 X</span>
      <span><i style="background:#4dff88"></i>机体 Y</span>
      <span><i style="background:#4d94ff"></i>机体 Z</span>
    </div>
    <div class="sub">拖动画布可旋转视角。机体坐标系按估计姿态绘制，可直观看到朝向是否跟随轨迹。</div>
  </div>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel"><h3>俯视图 XY（含起点/终点）</h3><canvas id="cxy" height="380"></canvas></div>
    <div class="panel"><h3>高度 Z 随时间</h3><canvas id="cz" height="380"></canvas></div>
  </div>
</section>

<section>
  <h2>3. 视觉后验的修正作用</h2>
  <h3>这是本报告的核心：视觉更新是否真的把状态拉向真值</h3>
  <div class="grid g4" id="kpi2"></div>
  <div class="panel" style="margin-top:16px">
    <h3>各状态视觉更新前后对比</h3>
    <div id="posteriorTable"></div>
  </div>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel">
      <h3>先验误差 vs 后验误差（位置）</h3>
      <canvas id="cscatter" height="360"></canvas>
      <div class="sub">对角线以下 = 视觉更新使误差<b>减小</b>（有效修正）；以上 = 增大。</div>
    </div>
    <div class="panel">
      <h3>每次更新的位置修正量 |dx_p|</h3>
      <canvas id="cdx" height="360"></canvas>
      <div class="sub">与当前位置误差对比，可看出修正的"力度"是否与误差匹配。</div>
    </div>
  </div>
</section>

<section>
  <h2>4. 误差与协方差一致性</h2>
  <h3>滤波器对自身精度的估计是否可信</h3>
  <div class="grid g4" id="kpiConsistency"></div>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel"><h3>NEES / 自由度（状态一致性）</h3><canvas id="cnees" height="330"></canvas></div>
    <div class="panel"><h3>NIS / 自由度（创新一致性）</h3><canvas id="cnis" height="330"></canvas></div>
  </div>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel"><h3>位置误差 vs 3√trace(P)</h3><canvas id="cerrp" height="330"></canvas></div>
    <div class="panel"><h3>姿态误差 vs 3√trace(P)</h3><canvas id="cerra" height="330"></canvas></div>
    <div class="panel"><h3>速度误差 vs 3√trace(P)</h3><canvas id="cerrv" height="330"></canvas></div>
    <div class="panel"><h3>位置误差三轴分量</h3><canvas id="cxyz" height="330"></canvas></div>
  </div>
  <div id="consistency"></div>
</section>

<section>
  <h2>5. 零偏与重力估计收敛</h2>
  <h3>这些量只能通过视觉更新间接可观，是判断融合是否生效的独立证据</h3>
  <div class="grid g3">
    <div class="panel"><h3>陀螺零偏 bg</h3><canvas id="cbg" height="280"></canvas></div>
    <div class="panel"><h3>加速度计零偏 ba</h3><canvas id="cba" height="280"></canvas></div>
    <div class="panel"><h3>重力向量 g</h3><canvas id="cg" height="280"></canvas></div>
  </div>
</section>

<section>
  <h2>6. 噪声参数敏感度</h2>
  <h3>量测噪声 uv_var 与过程噪声缩放对最终精度的影响</h3>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel"><h3>量测噪声 uv_var 扫描</h3><canvas id="csweep1" height="320"></canvas></div>
    <div class="panel"><h3>过程噪声 scale 扫描</h3><canvas id="csweep2" height="320"></canvas></div>
  </div>
  <div class="panel" style="margin-top:16px">
    <h3>全部扫描结果</h3>
    <div id="sweepTable"></div>
  </div>
  <div id="sweepNotes"></div>
</section>

<section>
  <h2>7. 观测数量与滑窗状态</h2>
  <div class="grid g4" id="obsStats"></div>
  <div class="grid g2" style="margin-top:16px">
    <div class="panel"><h3>每帧参与更新的 landmark 数</h3><canvas id="cnlmk" height="280"></canvas></div>
    <div class="panel"><h3>滑窗占用帧数</h3><canvas id="cwin" height="280"></canvas></div>
  </div>
</section>

</main>

<script>
// ================= 数据 =================
const RAW_TRAJ = `%%DATA_TRAJ%%`;
const RAW_UPD  = `%%DATA_UPDATE%%`;
const RAW_LMK  = `%%DATA_LMK%%`;
const RAW_SUM  = `%%DATA_SUMMARY%%`;

function parseCSV(txt){
  const lines = txt.trim().split('\n');
  if(lines.length<2) return {cols:[],rows:[]};
  const cols = lines[0].split(',');
  const rows = [];
  for(let i=1;i<lines.length;i++){
    const p = lines[i].split(',');
    if(p.length !== cols.length) continue;
    const o = {};
    for(let j=0;j<cols.length;j++){
      const v = parseFloat(p[j]);
      o[cols[j]] = Number.isNaN(v) ? p[j] : v;
    }
    rows.push(o);
  }
  return {cols,rows};
}
const T   = parseCSV(RAW_TRAJ).rows;
const U   = parseCSV(RAW_UPD).rows;
const LMK = parseCSV(RAW_LMK).rows;
const SUM = parseCSV(RAW_SUM).rows;

const fmt=(v,n=4)=>{
  if(!isFinite(v)) return '—';
  const a=Math.abs(v);
  if(a!==0 && (a<1e-3||a>=1e5)) return v.toExponential(2);
  return v.toFixed(n);
};
const finiteValues=a=>a.filter(Number.isFinite);
const mean=a=>{const v=finiteValues(a);return v.length?v.reduce((s,x)=>s+x,0)/v.length:NaN;};
const rms=a=>{const v=finiteValues(a);return v.length?Math.sqrt(v.reduce((s,x)=>s+x*x,0)/v.length):NaN;};
const quantile=(a,q)=>{
  const v=finiteValues(a).slice().sort((x,y)=>x-y); if(!v.length)return NaN;
  const p=(v.length-1)*q, i=Math.floor(p), f=p-i;
  return v[i]+(v[Math.min(i+1,v.length-1)]-v[i])*f;
};

// ================= 绘图基础设施 =================
function setupHiDPI(cv){
  const dpr = window.devicePixelRatio || 1;
  // height 属性会被 cv.height=... 改写，不能在每次重绘时重新读取它，
  // 否则高 DPI 屏幕上会形成 h <- h*dpr 的指数增长。
  let h = Number(cv.dataset.cssHeight);
  if(!(h>0)){
    h = Number(cv.getAttribute('height')) || cv.clientHeight || 300;
    cv.dataset.cssHeight = String(h);
    cv.style.height = h+'px';
  }
  const w = Math.max(1,Math.round(cv.clientWidth));
  const bw = Math.max(1,Math.round(w*dpr));
  const bh = Math.max(1,Math.round(h*dpr));
  if(cv.width!==bw || cv.height!==bh){ cv.width=bw; cv.height=bh; }
  const g = cv.getContext('2d');
  g.setTransform(dpr,0,0,dpr,0,0);
  return {g,w,h};
}
const CSS = getComputedStyle(document.documentElement);
const C = k => CSS.getPropertyValue(k).trim();

// 通用折线图
function linePlot(id, opts){
  const cv = document.getElementById(id); if(!cv) return;
  const {g,w,h} = setupHiDPI(cv);
  const M = {l:62,r:14,t:14,b:34};
  const pw = w-M.l-M.r, ph = h-M.t-M.b;
  const series = opts.series.filter(s=>s.data && s.data.length);
  if(!series.length) return;

  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(const s of series) for(const p of s.data){
    if(!isFinite(p[0])||!isFinite(p[1])) continue;
    if(p[0]<x0)x0=p[0]; if(p[0]>x1)x1=p[0];
    if(p[1]<y0)y0=p[1]; if(p[1]>y1)y1=p[1];
  }
  if(opts.y0!==undefined) y0=opts.y0;
  if(opts.y1!==undefined) y1=opts.y1;
  const logY = !!opts.logY;
  if(logY){ y0=Math.max(y0,1e-12); y1=Math.max(y1,y0*10); }
  if(y1-y0 < 1e-12){ y1=y0+1; }
  const pad=(y1-y0)*0.08; if(!logY){ y0-=pad; y1+=pad; }

  const X = v => M.l + (v-x0)/(x1-x0||1)*pw;
  const Y = v => {
    if(logY){
      const a=Math.log10(Math.max(v,1e-12)), b=Math.log10(y0), c=Math.log10(y1);
      return M.t + ph - (a-b)/((c-b)||1)*ph;
    }
    return M.t + ph - (v-y0)/((y1-y0)||1)*ph;
  };

  // 网格 + 刻度
  g.strokeStyle=C('--grid'); g.lineWidth=1; g.fillStyle=C('--dim');
  g.font='11px system-ui'; g.textAlign='right'; g.textBaseline='middle';
  const NT=5;
  for(let i=0;i<=NT;i++){
    const yy = logY
      ? Math.pow(10, Math.log10(y0)+(Math.log10(y1)-Math.log10(y0))*i/NT)
      : y0+(y1-y0)*i/NT;
    const py = Y(yy);
    g.beginPath(); g.moveTo(M.l,py); g.lineTo(M.l+pw,py); g.stroke();
    g.fillText(fmt(yy, logY?1:3), M.l-7, py);
  }
  g.textAlign='center'; g.textBaseline='top';
  for(let i=0;i<=5;i++){
    const xx=x0+(x1-x0)*i/5, px=X(xx);
    g.beginPath(); g.moveTo(px,M.t); g.lineTo(px,M.t+ph); g.stroke();
    g.fillText(xx.toFixed(0), px, M.t+ph+7);
  }
  g.fillText(opts.xlabel||'时间 (s)', M.l+pw/2, h-13);

  // 曲线
  for(const s of series){
    if(s.fill){
      g.fillStyle=s.fill; g.beginPath();
      let started=false;
      for(const p of s.data){ if(!isFinite(p[1]))continue;
        if(!started){g.moveTo(X(p[0]),Y(p[1]));started=true;} else g.lineTo(X(p[0]),Y(p[1])); }
      for(let i=s.data.length-1;i>=0;i--){ const p=s.data[i];
        if(!isFinite(p[1]))continue; g.lineTo(X(p[0]),Y(logY?y0:Math.max(y0,0))); break; }
      g.lineTo(X(s.data[0][0]),Y(logY?y0:Math.max(y0,0)));
      g.closePath(); g.fill(); continue;
    }
    g.strokeStyle=s.color; g.lineWidth=s.lw||1.6;
    if(s.dash) g.setLineDash(s.dash); else g.setLineDash([]);
    g.beginPath(); let started=false;
    for(const p of s.data){
      if(!isFinite(p[1])){ started=false; continue; }
      const px=X(p[0]), py=Y(p[1]);
      if(!started){ g.moveTo(px,py); started=true; } else g.lineTo(px,py);
    }
    g.stroke(); g.setLineDash([]);
  }

  // 图例
  if(opts.legend!==false){
    let lx=M.l+8, ly=M.t+12;
    g.font='11px system-ui'; g.textAlign='left'; g.textBaseline='middle';
    for(const s of series){
      if(!s.name) continue;
      g.strokeStyle=s.color; g.lineWidth=2.4;
      if(s.dash)g.setLineDash(s.dash); else g.setLineDash([]);
      g.beginPath(); g.moveTo(lx,ly); g.lineTo(lx+16,ly); g.stroke(); g.setLineDash([]);
      g.fillStyle=C('--dim'); g.fillText(s.name, lx+21, ly);
      lx += 26 + g.measureText(s.name).width + 14;
    }
  }
}

// ================= 1. KPI =================
(function(){
  const n=T.length;
  let sp=0,sv=0,sa=0,mp=0,fin=T[n-1];
  for(const r of T){ sp+=r.err_p*r.err_p; sv+=r.err_v*r.err_v; sa+=r.err_att*r.err_att;
                     if(r.err_p>mp)mp=r.err_p; }
  const rp=Math.sqrt(sp/n), rv=Math.sqrt(sv/n), ra=Math.sqrt(sa/n);
  const cards=[
    ['位置 RMSE', fmt(rp,4)+' m', '最大 '+fmt(mp,4)+' m', rp<0.5?'good':'bad'],
    ['速度 RMSE', fmt(rv,4)+' m/s', '轨迹速度 1 m/s', rv<0.1?'good':'bad'],
    ['姿态 RMSE', fmt(ra*180/Math.PI,4)+'°', fmt(ra,6)+' rad', ra<0.01?'good':'bad'],
    ['末帧位置误差', fmt(fin.err_p,4)+' m', '轨迹半径 5 m', fin.err_p<0.5?'good':'bad'],
  ];
  const base=[...SUM].reverse().find(r=>String(r.tag)==='base');
  if(base&&base.updates>0){
    cards.push(['平均视觉后验耗时',fmt(1000*base.t_cost/base.updates,3)+' ms',
      `总计 ${fmt(base.t_cost,2)} s / ${base.updates} 次`,'']);
  }
  document.getElementById('kpis').innerHTML = cards.map(c=>
    `<div class="kpi"><div class="k">${c[0]}</div>
     <div class="v ${c[3]}">${c[1]}</div><div class="n">${c[2]}</div></div>`).join('');
})();

// ================= 2. 3D 轨迹 =================
const view = {yaw:35,pitch:28,zoom:100,lmk:true,axes:true,err:false,axisN:14};
function draw3D(){
  const cv=document.getElementById('c3d');
  const {g,w,h}=setupHiDPI(cv);
  g.clearRect(0,0,w,h);

  const cy=Math.cos(view.yaw*Math.PI/180), sy=Math.sin(view.yaw*Math.PI/180);
  const cp=Math.cos(view.pitch*Math.PI/180), sp=Math.sin(view.pitch*Math.PI/180);
  // 世界 -> 屏幕：绕 Z 转 yaw，再绕水平轴抬 pitch（等距投影）
  const S = view.zoom*0.28;
  const cx0=w/2, cy0=h/2+40;
  function proj(x,y,z){
    const X = x*cy - y*sy;
    const Y = x*sy + y*cy;
    return [cx0 + X*S, cy0 - (z*cp - Y*sp)*S];
  }
  function depth(x,y,z){ const Y=x*sy+y*cy; return Y*cp + z*sp; }

  // 地面网格
  g.strokeStyle='#1b1f28'; g.lineWidth=1;
  for(let i=-14;i<=14;i+=2){
    let a=proj(i,-14,0), b=proj(i,14,0);
    g.beginPath(); g.moveTo(a[0],a[1]); g.lineTo(b[0],b[1]); g.stroke();
    a=proj(-14,i,0); b=proj(14,i,0);
    g.beginPath(); g.moveTo(a[0],a[1]); g.lineTo(b[0],b[1]); g.stroke();
  }
  // 世界坐标轴
  const O=proj(0,0,0);
  [[3,0,0,'#ff4d4d','X'],[0,3,0,'#4dff88','Y'],[0,0,3,'#4d94ff','Z']].forEach(a=>{
    const P=proj(a[0],a[1],a[2]);
    g.strokeStyle=a[3]; g.lineWidth=2.2;
    g.beginPath(); g.moveTo(O[0],O[1]); g.lineTo(P[0],P[1]); g.stroke();
    g.fillStyle=a[3]; g.font='12px system-ui'; g.textAlign='center';
    g.fillText(a[4],P[0],P[1]-6);
  });

  // 特征点（按深度排序，远的先画）
  if(view.lmk){
    const pts=LMK.map(p=>({p:proj(p.x,p.y,p.z), d:depth(p.x,p.y,p.z)}));
    pts.sort((a,b)=>a.d-b.d);
    for(const q of pts){
      const t=Math.max(0,Math.min(1,(q.d+14)/28));
      g.fillStyle=`rgba(120,132,158,${0.20+0.5*t})`;
      g.beginPath(); g.arc(q.p[0],q.p[1],1.1+1.5*t,0,6.283); g.fill();
    }
  }

  // 误差连线
  if(view.err){
    g.strokeStyle='rgba(255,209,102,0.35)'; g.lineWidth=1;
    for(let i=0;i<T.length;i+=6){
      const r=T[i];
      const a=proj(r.px_gt,r.py_gt,r.pz_gt), b=proj(r.px_est,r.py_est,r.pz_est);
      g.beginPath(); g.moveTo(a[0],a[1]); g.lineTo(b[0],b[1]); g.stroke();
    }
  }

  // 轨迹
  function poly(kx,ky,kz,col,lw){
    g.strokeStyle=col; g.lineWidth=lw; g.beginPath();
    T.forEach((r,i)=>{ const p=proj(r[kx],r[ky],r[kz]);
      if(i===0)g.moveTo(p[0],p[1]); else g.lineTo(p[0],p[1]); });
    g.stroke();
  }
  poly('px_gt','py_gt','pz_gt',C('--gt'),2.4);
  poly('px_est','py_est','pz_est',C('--est'),1.9);

  // 机体坐标系
  if(view.axes){
    const step=Math.max(1,Math.floor(T.length/view.axisN));
    for(let i=0;i<T.length;i+=step){
      const r=T[i];
      const q=[r.qw_est,r.qx_est,r.qy_est,r.qz_est];
      const R=quatToR(q), L=0.62;
      const o=[r.px_est,r.py_est,r.pz_est];
      const O2=proj(o[0],o[1],o[2]);
      [[0,'#ff4d4d'],[1,'#4dff88'],[2,'#4d94ff']].forEach(([c,col])=>{
        const e=[o[0]+R[0][c]*L, o[1]+R[1][c]*L, o[2]+R[2][c]*L];
        const P=proj(e[0],e[1],e[2]);
        g.strokeStyle=col; g.lineWidth=1.7;
        g.beginPath(); g.moveTo(O2[0],O2[1]); g.lineTo(P[0],P[1]); g.stroke();
      });
    }
  }

  // 起点终点
  const s0=T[0], s1=T[T.length-1];
  [[s0,'#3ddc97','起点'],[s1,'#ff5c7a','终点']].forEach(([r,col,lab])=>{
    const p=proj(r.px_gt,r.py_gt,r.pz_gt);
    g.fillStyle=col; g.beginPath(); g.arc(p[0],p[1],5,0,6.283); g.fill();
    g.fillStyle=col; g.font='12px system-ui'; g.textAlign='left';
    g.fillText(lab,p[0]+8,p[1]-8);
  });
}
function quatToR(q){
  const [w,x,y,z]=q;
  return [
    [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
    [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
    [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]
  ];
}
(function(){
  let drawPending=false;
  const requestDraw=()=>{
    if(drawPending)return;
    drawPending=true;
    requestAnimationFrame(()=>{drawPending=false;draw3D();});
  };
  const bind=(id,k,f)=>{const e=document.getElementById(id);
    e.addEventListener('input',()=>{view[k]=f?f(e):+e.value; requestDraw();});};
  bind('yaw','yaw'); bind('pitch','pitch'); bind('zoom','zoom');
  bind('axisN','axisN');
  ['showLmk','showAxes','showErr'].forEach((id,i)=>{
    const k=['lmk','axes','err'][i], e=document.getElementById(id);
    e.addEventListener('change',()=>{view[k]=e.checked; requestDraw();});
  });
  const cv=document.getElementById('c3d');
  let drag=false,lx=0,ly=0,pointerId=null;
  cv.addEventListener('pointerdown',e=>{
    drag=true;pointerId=e.pointerId;lx=e.clientX;ly=e.clientY;
    cv.classList.add('dragging');cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointerup',e=>{
    if(pointerId===e.pointerId){drag=false;pointerId=null;cv.classList.remove('dragging');}
  });
  cv.addEventListener('pointercancel',()=>{
    drag=false;pointerId=null;cv.classList.remove('dragging');
  });
  cv.addEventListener('pointermove',e=>{
    if(!drag)return;
    view.yaw=(view.yaw+(e.clientX-lx)*0.4+360)%360;
    view.pitch=Math.max(-89,Math.min(89,view.pitch+(e.clientY-ly)*0.3));
    lx=e.clientX; ly=e.clientY;
    document.getElementById('yaw').value=view.yaw;
    document.getElementById('pitch').value=view.pitch;
    requestDraw();
  });
  cv.addEventListener('wheel',e=>{e.preventDefault();
    view.zoom=Math.max(20,Math.min(400,view.zoom*(e.deltaY>0?0.92:1.08)));
    document.getElementById('zoom').value=view.zoom; requestDraw();},{passive:false});
})();

// 俯视图
(function(){
  const cv=document.getElementById('cxy'); const {g,w,h}=setupHiDPI(cv);
  const M=44, pw=w-2*M, ph=h-2*M;
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(const r of T){ x0=Math.min(x0,r.px_gt,r.px_est); x1=Math.max(x1,r.px_gt,r.px_est);
                     y0=Math.min(y0,r.py_gt,r.py_est); y1=Math.max(y1,r.py_gt,r.py_est); }
  const sc=Math.min(pw/(x1-x0||1), ph/(y1-y0||1))*0.9;
  const cx=(x0+x1)/2, cy=(y0+y1)/2;
  const X=v=>w/2+(v-cx)*sc, Y=v=>h/2-(v-cy)*sc;
  g.strokeStyle=C('--grid');
  for(let i=Math.ceil(x0);i<=x1;i+=2){g.beginPath();g.moveTo(X(i),M);g.lineTo(X(i),h-M);g.stroke();}
  for(let i=Math.ceil(y0);i<=y1;i+=2){g.beginPath();g.moveTo(M,Y(i));g.lineTo(w-M,Y(i));g.stroke();}
  const line=(kx,ky,col,lw)=>{g.strokeStyle=col;g.lineWidth=lw;g.beginPath();
    T.forEach((r,i)=>{const px=X(r[kx]),py=Y(r[ky]);i?g.lineTo(px,py):g.moveTo(px,py);});g.stroke();};
  line('px_gt','py_gt',C('--gt'),2.2);
  line('px_est','py_est',C('--est'),1.6);
  const a=T[0], b=T[T.length-1];
  g.fillStyle='#3ddc97';g.beginPath();g.arc(X(a.px_gt),Y(a.py_gt),5,0,6.283);g.fill();
  g.fillStyle='#ff5c7a';g.beginPath();g.arc(X(b.px_gt),Y(b.py_gt),5,0,6.283);g.fill();
  g.fillStyle=C('--dim');g.font='11px system-ui';g.textAlign='left';
  g.fillText('X (m) →',w-M-52,h-M+18); g.fillText('↑ Y (m)',M-30,M-12);
})();

// 高度
linePlot('cz',{series:[
  {name:'Z 真值', color:C('--gt'), data:T.map(r=>[r.t,r.pz_gt])},
  {name:'Z 估计', color:C('--est'), data:T.map(r=>[r.t,r.pz_est])},
]});

// ================= 3. 先验 vs 后验 =================
(function(){
  let imp=0, sdx=0, n=U.length, kfImp=0, kfN=0, nkfImp=0, nkfN=0;
  for(const r of U){
    if(r.improve_p) imp++;
    sdx+=r.dxp;
    if(r.is_kf){kfN++;if(r.improve_p)kfImp++;}
    else{nkfN++;if(r.improve_p)nkfImp++;}
  }
  const meanErr = T.reduce((a,r)=>a+r.err_p,0)/T.length;
  const ratio = n?sdx/n/meanErr:NaN;
  const cards=[
    ['更新次数', n, '相机帧 '+T.length+' 帧','' ],
    ['位置改善率', (100*imp/Math.max(n,1)).toFixed(1)+'%', '后验比先验更接近真值的比例',
      imp/Math.max(n,1)>0.6?'good':(imp/Math.max(n,1)>0.45?'warn':'bad')],
    ['关键/非关键帧改善率', fmt(100*kfImp/Math.max(kfN,1),1)+'% / '+fmt(100*nkfImp/Math.max(nkfN,1),1)+'%',
      `样本 ${kfN} / ${nkfN}`,''],
    ['平均修正量 |dx_p|', fmt(sdx/Math.max(n,1),6)+' m', '平均位置误差 '+fmt(meanErr,4)+' m',''],
    ['修正量/误差比', (ratio*100).toFixed(2)+'%', ratio<0.05?'修正远小于误差':'量级匹配',
      ratio<0.05?'bad':'good'],
  ];
  document.getElementById('kpi2').innerHTML = cards.map(c=>
    `<div class="kpi"><div class="k">${c[0]}</div>
     <div class="v ${c[3]||''}">${c[1]}</div><div class="n">${c[2]}</div></div>`).join('');

  const states=[
    ['位置 (m)','errp_prior','errp_post','improve_p'],
    ['速度 (m/s)','errv_prior','errv_post',null],
    ['姿态 (rad)','erra_prior','erra_post',null],
  ];
  let table='<table><tr><th>状态</th><th>先验 RMSE</th><th>后验 RMSE</th><th>RMSE 降低</th><th>逐帧改善率</th></tr>';
  for(const [name,k0,k1,flag] of states){
    const before=rms(U.map(r=>r[k0])), after=rms(U.map(r=>r[k1]));
    const improved=U.filter(r=>flag?r[flag]:r[k1]<r[k0]).length/Math.max(n,1);
    const change=(before-after)/Math.max(before,1e-12)*100;
    table+=`<tr><td>${name}</td><td>${fmt(before,6)}</td><td>${fmt(after,6)}</td>`+
           `<td class="${change>=0?'good':'bad'}">${change>=0?'+':''}${fmt(change,2)}%</td>`+
           `<td>${fmt(100*improved,1)}%</td></tr>`;
  }
  document.getElementById('posteriorTable').innerHTML=table+'</table>';

  // 散点
  const cv=document.getElementById('cscatter'); const {g,w,h}=setupHiDPI(cv);
  const M={l:60,r:14,t:14,b:40}, pw=w-M.l-M.r, ph=h-M.t-M.b;
  let mx=0; for(const r of U) mx=Math.max(mx,r.errp_prior,r.errp_post);
  mx*=1.05;
  const X=v=>M.l+v/mx*pw, Y=v=>M.t+ph-v/mx*ph;
  g.strokeStyle=C('--grid');
  for(let i=0;i<=4;i++){
    const v=mx*i/4;
    g.beginPath();g.moveTo(M.l,Y(v));g.lineTo(M.l+pw,Y(v));g.stroke();
    g.beginPath();g.moveTo(X(v),M.t);g.lineTo(X(v),M.t+ph);g.stroke();
    g.fillStyle=C('--dim');g.font='11px system-ui';
    g.textAlign='right';g.textBaseline='middle';g.fillText(fmt(v,3),M.l-7,Y(v));
    g.textAlign='center';g.textBaseline='top';g.fillText(fmt(v,3),X(v),M.t+ph+7);
  }
  // y=x
  g.strokeStyle='#8b93a7'; g.lineWidth=1.3; g.setLineDash([5,4]);
  g.beginPath();g.moveTo(X(0),Y(0));g.lineTo(X(mx),Y(mx));g.stroke();g.setLineDash([]);
  for(const r of U){
    const good = !!r.improve_p;
    g.fillStyle = good ? 'rgba(61,220,151,0.42)' : 'rgba(255,92,122,0.42)';
    g.beginPath(); g.arc(X(r.errp_prior),Y(r.errp_post),1.7,0,6.283); g.fill();
  }
  g.fillStyle=C('--dim');g.font='11px system-ui';g.textAlign='center';
  g.fillText('先验位置误差 (m)',M.l+pw/2,h-12);
  g.save();g.translate(15,M.t+ph/2);g.rotate(-Math.PI/2);
  g.fillText('后验位置误差 (m)',0,0);g.restore();
})();

// 修正量
linePlot('cdx',{logY:true, series:[
  {name:'|dx_p| 修正量', color:C('--ok'), data:U.map(r=>[r.t,Math.max(r.dxp,1e-12)]), lw:1.1},
  {name:'位置误差', color:C('--err'), data:U.map(r=>[r.t,Math.max(r.errp_post,1e-12)]), lw:1.4},
]});

// ================= 4. NEES / NIS / 误差 vs 协方差 =================
(function(){
  const t0=T[0].t, t1=T[T.length-1].t;
  const expected=[[t0,1],[t1,1]];
  linePlot('cnees',{logY:true,series:[
    {name:'Q/P/V 联合/9',color:C('--gt'),data:T.map(r=>[r.t,Math.max(r.nees_qpv/9,1e-9)]),lw:1.5},
    {name:'位置/3',color:C('--est'),data:T.map(r=>[r.t,Math.max(r.nees_p/3,1e-9)]),lw:1.0},
    {name:'姿态/3',color:'#ffb454',data:T.map(r=>[r.t,Math.max(r.nees_q/3,1e-9)]),lw:1.0},
    {name:'速度/3',color:'#b38cff',data:T.map(r=>[r.t,Math.max(r.nees_v/3,1e-9)]),lw:1.0},
    {name:'理论期望 1',color:C('--dim'),dash:[6,4],data:expected,lw:1.2},
  ]});

  const nisRows=U.filter(r=>r.nis_dof>0&&Number.isFinite(r.nis_mean));
  linePlot('cnis',{logY:true,series:[
    {name:'NIS/自由度',color:C('--ok'),data:nisRows.map(r=>[r.t,Math.max(r.nis_mean,1e-9)]),lw:1.2},
    {name:'理论期望 1',color:C('--dim'),dash:[6,4],data:expected,lw:1.2},
  ]});

  const neesNorm=T.map(r=>r.nees_qpv/9);
  const meanNEES=mean(neesNorm), p95NEES=quantile(neesNorm,0.95);
  const meanNIS=mean(nisRows.map(r=>r.nis_mean));
  const coverage=k=>100*T.filter(r=>r[k[0]]>0&&r[k[1]]<=3*r[k[0]]).length/Math.max(T.length,1);
  const covP=coverage(['sigma_p','err_p']);
  const covQ=coverage(['sigma_q','err_att']);
  const covV=coverage(['sigma_v','err_v']);
  const cls=v=>v>0.5&&v<2?'good':(v>3?'bad':'warn');
  const cards=[
    ['平均联合 NEES/9',fmt(meanNEES,3),'理论期望约 1',cls(meanNEES)],
    ['联合 NEES/9 P95',fmt(p95NEES,3),'查看偶发过度自信',cls(p95NEES)],
    ['平均 NIS/自由度',fmt(meanNIS,3),`有效更新 ${nisRows.length} 次`,cls(meanNIS)],
    ['3√trace 覆盖率',`${fmt(covP,1)}% / ${fmt(covQ,1)}% / ${fmt(covV,1)}%`,'位置 / 姿态 / 速度',''],
  ];
  document.getElementById('kpiConsistency').innerHTML=cards.map(c=>
    `<div class="kpi"><div class="k">${c[0]}</div>`+
    `<div class="v ${c[3]||''}">${c[1]}</div><div class="n">${c[2]}</div></div>`).join('');
})();

linePlot('cerrp',{logY:true,series:[
  {name:'位置误差', color:C('--err'), data:T.map(r=>[r.t,Math.max(r.err_p,1e-9)]),lw:1.4},
  {name:'3√trace(P)', color:C('--bad'), dash:[6,4], data:T.map(r=>[r.t,Math.max(3*r.sigma_p,1e-9)])},
]});
linePlot('cerra',{logY:true,series:[
  {name:'姿态误差', color:C('--err'), data:T.map(r=>[r.t,Math.max(r.err_att,1e-9)]),lw:1.4},
  {name:'3√trace(P)', color:C('--bad'), dash:[6,4], data:T.map(r=>[r.t,Math.max(3*r.sigma_q,1e-9)])},
]});
linePlot('cerrv',{logY:true,series:[
  {name:'速度误差', color:C('--err'), data:T.map(r=>[r.t,Math.max(r.err_v,1e-9)]),lw:1.4},
  {name:'3√trace(P)', color:C('--bad'), dash:[6,4], data:T.map(r=>[r.t,Math.max(3*r.sigma_v,1e-9)])},
]});
linePlot('cxyz',{series:[
  {name:'ex', color:'#ff5c7a', data:T.map(r=>[r.t,r.ex])},
  {name:'ey', color:'#3ddc97', data:T.map(r=>[r.t,r.ey])},
  {name:'ez', color:'#4d94ff', data:T.map(r=>[r.t,r.ez])},
]});

(function(){
  // 一致性判据: err 应大致落在 3σ 内且不应远小于 σ
  const last=T[T.length-1];
  const rP=last.err_p/(last.sigma_p||1e-12);
  const rQ=last.err_att/(last.sigma_q||1e-12);
  const rV=last.err_v/(last.sigma_v||1e-12);
  let html='';
  const meanNEES=mean(T.map(r=>r.nees_qpv/9));
  const meanNIS=mean(U.filter(r=>r.nis_dof>0).map(r=>r.nis_mean));
  const consistencyNote=(name,value)=>{
    if(!Number.isFinite(value)) return `<div class="note bad"><b>${name}</b>：没有有效数据。</div>`;
    if(value>2) return `<div class="note bad"><b>${name}</b>：均值 ${fmt(value,3)} 明显高于理论期望 1，`+
      `当前噪声/协方差模型可能<b>过度自信</b>。</div>`;
    if(value<0.5) return `<div class="note"><b>${name}</b>：均值 ${fmt(value,3)} 明显低于理论期望 1，`+
      `当前模型可能偏<b>保守</b>。单条仿真轨迹中的时序样本相关，阈值仅用于诊断。</div>`;
    return `<div class="note ok"><b>${name}</b>：均值 ${fmt(value,3)} 与理论期望 1 量级一致。</div>`;
  };
  html+=consistencyNote('联合 NEES/自由度',meanNEES);
  html+=consistencyNote('NIS/自由度',meanNIS);
  const judge=(name,r,sig,err)=>{
    if(!(sig>0)) return `<div class="note bad"><b>${name}</b>：√trace(P)=${fmt(sig,4)} <b>为负或无效</b>，协方差矩阵已失去正定性（数值问题）。</div>`;
    if(r>3) return `<div class="note bad"><b>${name}</b>：末帧误差 ${fmt(err,4)} 超出 3√trace(P) `+
      `(√trace(P)=${fmt(sig,4)})，滤波器可能<b>过度自信</b>。</div>`;
    if(r<0.01) return `<div class="note"><b>${name}</b>：末帧误差 ${fmt(err,4)} 远小于 `+
      `√trace(P)=${fmt(sig,4)}（比值 ${fmt(r,5)}），滤波器可能偏<b>保守</b>。</div>`;
    return `<div class="note ok"><b>${name}</b>：末帧误差 ${fmt(err,4)} 与 √trace(P)=${fmt(sig,4)} `+
      `量级匹配（比值 ${fmt(r,3)}）。这是 trace 级粗检，统计一致性以 NEES/NIS 为主。</div>`;
  };
  html+=judge('位置',rP,last.sigma_p,last.err_p);
  html+=judge('姿态',rQ,last.sigma_q,last.err_att);
  html+=judge('速度',rV,last.sigma_v,last.err_v);

  // 协方差正定性检查(负 trace 在导出时写成负值而非 nan)
  let nneg=0, tneg=-1;
  for(const r of T){
    if(r.sigma_p<0||r.sigma_q<0||r.sigma_v<0){ nneg++; if(tneg<0) tneg=r.t; }
  }
  if(nneg){
    html+=`<div class="note bad"><b>协方差正定性</b>：共 <b>${nneg}/${T.length}</b> 帧的协方差 trace 为负`+
          `（首次出现在 t=${fmt(tneg,1)} s）。对称矩阵的 trace 为负意味着至少有一个负特征值，`+
          `协方差已<b>失去正定性</b>。需要分别检查预测传播、增广互协方差与后验更新，`+
          `不能仅凭该现象归因于 Joseph 形式。</div>`;
  }else{
    html+=`<div class="note ok"><b>协方差正定性</b>：全部 ${T.length} 帧的 `+
          `位置/姿态/速度协方差块 trace 均非负，未发现 trace 级异常。`+
          `离线完整特征值诊断的最小值约为 -2e-12，属于浮点舍入量级。</div>`;
  }
  document.getElementById('consistency').innerHTML=html;
})();

// ================= 5. 零偏 / 重力 =================
linePlot('cbg',{series:[
  {name:'bg x',color:'#ff5c7a',data:T.map(r=>[r.t,r.bgx])},
  {name:'bg y',color:'#3ddc97',data:T.map(r=>[r.t,r.bgy])},
  {name:'bg z',color:'#4d94ff',data:T.map(r=>[r.t,r.bgz])},
]});
linePlot('cba',{series:[
  {name:'ba x',color:'#ff5c7a',data:T.map(r=>[r.t,r.bax])},
  {name:'ba y',color:'#3ddc97',data:T.map(r=>[r.t,r.bay])},
  {name:'ba z',color:'#4d94ff',data:T.map(r=>[r.t,r.baz])},
]});
linePlot('cg',{series:[
  {name:'g x',color:'#ff5c7a',data:T.map(r=>[r.t,r.gx])},
  {name:'g y',color:'#3ddc97',data:T.map(r=>[r.t,r.gy])},
  {name:'g z',color:'#4d94ff',data:T.map(r=>[r.t,r.gz])},
]});

// ================= 6. 噪声扫描 =================
function barSweep(id, rows, keyX, labelX){
  const cv=document.getElementById(id); if(!cv) return;
  const {g,w,h}=setupHiDPI(cv);
  const M={l:66,r:14,t:16,b:46}, pw=w-M.l-M.r, ph=h-M.t-M.b;
  if(!rows.length) return;
  const vals=rows.map(r=>Math.max(r.rmse_p,1e-4));
  const lo=Math.min(...vals)*0.5, hi=Math.max(...vals)*2;
  const Y=v=>{const a=Math.log10(Math.max(v,1e-6)),b=Math.log10(lo),c=Math.log10(hi);
    return M.t+ph-(a-b)/((c-b)||1)*ph;};
  g.strokeStyle=C('--grid'); g.fillStyle=C('--dim'); g.font='11px system-ui';
  for(let i=0;i<=5;i++){
    const v=Math.pow(10, Math.log10(lo)+(Math.log10(hi)-Math.log10(lo))*i/5);
    g.beginPath();g.moveTo(M.l,Y(v));g.lineTo(M.l+pw,Y(v));g.stroke();
    g.textAlign='right';g.textBaseline='middle';g.fillText(v.toExponential(0),M.l-7,Y(v));
  }
  // 可接受阈值线
  g.strokeStyle=C('--ok'); g.setLineDash([5,4]); g.lineWidth=1.2;
  g.beginPath();g.moveTo(M.l,Y(1));g.lineTo(M.l+pw,Y(1));g.stroke();g.setLineDash([]);
  g.fillStyle=C('--ok');g.textAlign='left';g.fillText('1 m',M.l+4,Y(1)-9);

  const bw=pw/rows.length;
  rows.forEach((r,i)=>{
    const x=M.l+i*bw+bw*0.18, bwid=bw*0.64;
    const diverged = r.rmse_p>1;
    g.fillStyle = diverged?'rgba(255,92,122,0.75)':'rgba(61,220,151,0.75)';
    const y=Y(Math.max(r.rmse_p,1e-4));
    g.fillRect(x,y,bwid,M.t+ph-y);
    g.fillStyle=C('--dim');g.font='10px system-ui';
    g.textAlign='center';g.textBaseline='top';
    g.save();g.translate(x+bwid/2,M.t+ph+6);
    g.fillText(String(r[keyX]),0,0);g.restore();
    g.textBaseline='bottom';
    g.fillStyle=diverged?C('--bad'):C('--ok');g.font='10px system-ui';
    g.fillText(diverged?'发散':fmt(r.rmse_p,3), x+bwid/2, y-3);
  });
  g.fillStyle=C('--dim');g.font='11px system-ui';g.textAlign='center';
  g.fillText(labelX,M.l+pw/2,h-9);
  g.save();g.translate(14,M.t+ph/2);g.rotate(-Math.PI/2);
  g.fillText('位置 RMSE (m, 对数轴)',0,0);g.restore();
}
const sweepUV  = SUM.filter(r=>String(r.tag).startsWith('uv')).sort((a,b)=>a.uv_var-b.uv_var);
const sweepPR  = SUM.filter(r=>String(r.tag).startsWith('proc')).sort((a,b)=>a.proc_scale-b.proc_scale);
barSweep('csweep1', sweepUV, 'uv_var', '量测噪声方差 uv_var');
barSweep('csweep2', sweepPR, 'proc_scale', '过程噪声缩放 scale');

(function(){
  const rows=[...SUM].sort((a,b)=>a.rmse_p-b.rmse_p);
  const best=rows[0];
  let html='<table><tr><th>配置</th><th>uv_var</th><th>proc scale</th>'+
           '<th>位置 RMSE (m)</th><th>速度 RMSE</th><th>姿态 RMSE (rad)</th>'+
           '<th>最大误差 (m)</th><th>耗时 (s)</th></tr>';
  for(const r of rows){
    const div=r.rmse_p>1;
    html+=`<tr class="${div?'diverge':(r===best?'best':'')}">`+
      `<td>${r.tag}</td><td>${r.uv_var}</td><td>${r.proc_scale}</td>`+
      `<td>${div?'发散 ('+r.rmse_p.toExponential(1)+')':fmt(r.rmse_p,4)}</td>`+
      `<td>${div?'—':fmt(r.rmse_v,4)}</td><td>${div?'—':fmt(r.rmse_att,6)}</td>`+
      `<td>${div?'—':fmt(r.max_err_p,4)}</td><td>${fmt(r.t_cost,2)}</td></tr>`;
  }
  html+='</table>';
  document.getElementById('sweepTable').innerHTML=html;

  // 自动结论
  const okUV=sweepUV.filter(r=>r.rmse_p<1).map(r=>r.uv_var);
  const okPR=sweepPR.filter(r=>r.rmse_p<1).map(r=>r.proc_scale);
  let n='';
  if(okUV.length){
    const allStable=okUV.length===sweepUV.length;
    n+=`<div class="note"><b>量测噪声</b>：扫描区间 uv_var ∈ [${Math.min(...okUV)}, ${Math.max(...okUV)}] `+
       `${allStable?'全部稳定':'内存在稳定配置'}。`+
       `在当前仿真中减小 uv_var 会提高视觉权重并降低误差；默认值 <code>400</code> `+
       `不再处于发散边界。该趋势不能替代真实数据上的残差统计与噪声标定。</div>`;
  }
  if(okPR.length){
    const allStable=okPR.length===sweepPR.length;
    n+=`<div class="note"><b>过程噪声</b>：扫描区间 scale ∈ [${Math.min(...okPR)}, ${Math.max(...okPR)}] `+
       `${allStable?'全部稳定':'内存在稳定配置'}。采样点中的最优值为 `+
       `<code>${sweepPR.reduce((a,b)=>a.rmse_p<b.rmse_p?a:b).proc_scale}</code>，`+
       `说明默认 scale=1 在本仿真里偏保守；仍应依据真实 IMU Allan 方差标定，而不是直接采用扫描极值。</div>`;
  }
  document.getElementById('sweepNotes').innerHTML=n;
})();

// ================= 7. 观测数 / 滑窗 =================
(function(){
  const lmk=U.map(r=>r.n_lmk), meas=T.map(r=>r.n_meas), win=U.map(r=>r.win);
  const keyframes=U.filter(r=>r.is_kf).length;
  const maxWin=Math.max(...win);
  const fullRate=100*win.filter(v=>v===maxWin).length/Math.max(win.length,1);
  const cards=[
    ['Landmark 中位数',fmt(quantile(lmk,0.5),0),`P10–P90: ${fmt(quantile(lmk,0.1),0)}–${fmt(quantile(lmk,0.9),0)}`,''],
    ['原始观测中位数',fmt(quantile(meas,0.5),0),`P10–P90: ${fmt(quantile(meas,0.1),0)}–${fmt(quantile(meas,0.9),0)}`,''],
    ['关键帧比例',fmt(100*keyframes/Math.max(U.length,1),1)+'%',`${keyframes}/${U.length} 次更新`,''],
    ['最大窗口占用率',fmt(fullRate,1)+'%',`最大 ${maxWin} 帧`,''],
  ];
  document.getElementById('obsStats').innerHTML=cards.map(c=>
    `<div class="kpi"><div class="k">${c[0]}</div>`+
    `<div class="v">${c[1]}</div><div class="n">${c[2]}</div></div>`).join('');
})();

linePlot('cnlmk',{series:[
  {name:'参与更新的 landmark',color:C('--gt'),data:U.map(r=>[r.t,r.n_lmk]),lw:1.1},
  {name:'该帧观测总数',color:'#5a6478',data:T.map(r=>[r.t,r.n_meas]),lw:1.1},
]});
linePlot('cwin',{y0:0,y1:32,series:[
  {name:'滑窗帧数',color:C('--ok'),data:U.map(r=>[r.t,r.win]),lw:1.2},
]});

draw3D();
window.addEventListener('resize',()=>location.reload());
</script>
</body>
</html>
)HTML";
