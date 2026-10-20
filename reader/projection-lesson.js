(function(){
'use strict';
// These pure models are shared by the offline canvas and Node regression tests.
function workModel(degrees){
  const angle=degrees*Math.PI/180, parallel=10*Math.cos(angle);
  return {parallel, transverse:10*Math.sin(angle), work:2*parallel};
}
function fitModel(bx,by,t){
  const p=[2*t,t], residual=[bx-p[0],by-p[1]], optimal=(2*bx+by)/5;
  return {p,residual,optimal,foot:[2*optimal,optimal],
    normal:2*residual[0]+residual[1],distance:Math.hypot(...residual),
    cost:(residual[0]**2+residual[1]**2)/2};
}
if(typeof module!=='undefined'&&module.exports) module.exports={workModel,fitModel};
if(typeof window==='undefined')return;
const ink='#243b44', blue='#366bc2', green='#007d78', orange='#bb5b08', grid='#dce5e7';
const fmt=n=>(Math.abs(n)<0.00005?0:n).toFixed(3);
function setup(canvas){
  const w=Math.max(240,canvas.clientWidth),h=320,dpr=window.devicePixelRatio||1;
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);
  c.fillStyle='#ffffff';c.fillRect(0,0,w,h);c.font='13px sans-serif';
  return {c,w,h};
}
function line(c,a,b,color,width=2,dash=[]){
  c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);
  c.moveTo(...a);c.lineTo(...b);c.stroke();c.setLineDash([]);
}
function arrow(c,a,b,color){
  line(c,a,b,color,3);const dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);
  if(len<1)return;
  const ux=dx/len,uy=dy/len,size=Math.min(9,len/2);
  c.beginPath();c.fillStyle=color;c.moveTo(...b);
  c.lineTo(b[0]-size*ux+size*.5*uy,b[1]-size*uy-size*.5*ux);
  c.lineTo(b[0]-size*ux-size*.5*uy,b[1]-size*uy+size*.5*ux);c.closePath();c.fill();
}
function label(c,text,x,y,color=ink){c.fillStyle=color;c.fillText(text,x,y);}
function dot(c,p,color,r=5){c.beginPath();c.fillStyle=color;c.arc(...p,r,0,2*Math.PI);c.fill();}
function write(root,key,text){root.querySelector(`[data-projection-out="${key}"]`).textContent=text;}
function work(root){
  const slider=root.querySelector('[name=angle]');
  function draw(){
    const theta=Number(slider.value),m=workModel(theta),{c,w}=setup(root.querySelector('canvas'));
    const origin=[w/2,190],scale=Math.min(11,(w-60)/22),end=[origin[0]+scale*m.parallel,origin[1]-scale*m.transverse];
    label(c,'恒力 F = 10 N',14,24,blue);
    label(c,'位移 Δp = 2 m，始终向右',14,46,ink);
    line(c,[20,219],[w-20,219],grid);
    c.fillStyle='#e9f4f1';c.fillRect(origin[0]-16,194,32,24);
    line(c,end,[end[0],origin[1]],orange,1,[5,4]);
    arrow(c,origin,end,blue);
    arrow(c,[origin[0],origin[1]+6],[end[0],origin[1]+6],green);
    label(c,'F',Math.max(10,Math.min(w-24,end[0]+8)),end[1]-12,blue);
    arrow(c,[w*.2,264],[w*.8,264],ink);label(c,'Δp',w*.8+6,269);
    label(c,'绿色：沿位移方向的力分量',14,303,green);
    write(root,'angle',`${theta}°`);
    write(root,'work',`沿位移分量 = ${fmt(m.parallel)} N · 这股力的功 W = ${fmt(m.work)} J`);
    write(root,'work-note',Math.abs(m.work)<1e-8?'力与位移垂直，这股力做功为零':m.work>0?'力在位移方向上的分量为正，这股力做正功':'力在位移方向上的分量为负，这股力做负功');
  }
  slider.addEventListener('input',draw);
  root.querySelectorAll('[data-work-angle]').forEach(b=>b.addEventListener('click',()=>{slider.value=b.dataset.workAngle;draw();}));
  draw();return draw;
}
function fit(root){
  const controls=Object.fromEntries([...root.querySelectorAll('input')].map(x=>[x.name,x]));
  function draw(){
    const bx=+controls.bx.value,by=+controls.by.value,t=+controls.t.value,m=fitModel(bx,by,t);
    const {c,w,h}=setup(root.querySelector('canvas')),scale=Math.min((w-42)/13,(h-64)/9);
    const origin=[w/2,h/2+5],to=(x,y)=>[origin[0]+scale*x,origin[1]-scale*y];
    for(let i=-6;i<=6;i++)line(c,to(i,-4),to(i,4),grid,0.6);
    for(let i=-4;i<=4;i++)line(c,to(-6,i),to(6,i),grid,0.6);
    line(c,to(-6,0),to(6,0),'#91a1a6',1);line(c,to(0,-4),to(0,4),'#91a1a6',1);
    for(const i of [-4,-2,2,4]){label(c,String(i),to(i,0)[0]-4,origin[1]+15);label(c,String(i),origin[0]+4,to(0,i)[1]+4);}
    label(c,'y₁',to(6,0)[0]+3,origin[1]+4);label(c,'y₂',origin[0]+5,to(0,4)[1]-8);
    line(c,to(-6,-3),to(6,3),green,2);
    const b=to(bx,by),p=to(...m.p),foot=to(...m.foot);
    line(c,foot,b,'#8c969a',1,[5,4]);
    arrow(c,origin,b,blue);arrow(c,origin,p,green);arrow(c,p,b,orange);
    dot(c,foot,'#ffffff',6);c.strokeStyle=green;c.lineWidth=2;c.stroke();dot(c,p,green);dot(c,b,blue);
    label(c,m.distance<1e-8?'b = p':'b',Math.min(w-48,b[0]+8),b[1]-9,blue);
    if(m.distance>=1e-8)label(c,'p',p[0]+8,p[1]+18,green);
    label(c,'O',origin[0]-16,origin[1]+16);
    label(c,'轨道方向 a = (2, 1)',12,21,green);
    label(c,'蓝：目标  绿：输出  橙：残差',12,h-12);
    if(Math.abs(m.normal)<1e-8&&m.distance>1e-8){
      const u=[2/Math.sqrt(5),-1/Math.sqrt(5)],v=[(b[0]-p[0])/(m.distance*scale),(b[1]-p[1])/(m.distance*scale)],size=Math.min(10,m.distance*scale*.4);
      const p1=[p[0]+size*u[0],p[1]+size*u[1]],p2=[p1[0]+size*v[0],p1[1]+size*v[1]],p3=[p[0]+size*v[0],p[1]+size*v[1]];
      line(c,p1,p2,orange,1.5);line(c,p2,p3,orange,1.5);
    }
    for(const [key,input] of Object.entries(controls))write(root,key,Number(input.value).toFixed(1));
    write(root,'fit',`p = (${fmt(m.p[0])}, ${fmt(m.p[1])}) · 残差距离 = ${fmt(m.distance)} · 目标值 f = ${fmt(m.cost)}`);
    write(root,'normal',`aᵀr = ${fmt(m.normal)} · 最优 t* = ${fmt(m.optimal)}`);
    write(root,'fit-note',m.distance<1e-8?'目标位于轨道上，已经完全实现':Math.abs(m.normal)<1e-8?'已到垂足：残差与轨道正交，仍然可以有非零误差':'尚未到垂足：沿轨道调整输入还能缩短残差');
  }
  for(const slider of Object.values(controls))slider.addEventListener('input',draw);
  root.querySelector('[data-fit-best]').addEventListener('click',()=>{controls.t.value=(2*(+controls.bx.value)+(+controls.by.value))/5;draw();});
  root.querySelector('[data-fit-reset]').addEventListener('click',()=>{controls.bx.value=1;controls.by.value=2;controls.t.value=0;draw();});
  draw();return draw;
}
let active=[];
window.initProjectionLessonDemos=()=>{
  active=[...document.querySelectorAll('#page-content [data-projection-lab]')].map(root=>root.dataset.projectionLab==='work'?work(root):fit(root));
};
window.addEventListener('resize',()=>active.forEach(draw=>draw()));
})();
