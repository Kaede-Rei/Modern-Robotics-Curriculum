/* Interactive supplements for F02, scoped to its detail article and offline-safe */
(()=>{'use strict';
const rad=Math.PI/180;
const one=(root,selector)=>root.querySelector(selector);
const number=(value,places=2)=>Number(value.toFixed(places));
const signed=value=>value>0?'+'+value:value<0?'−'+Math.abs(value):'0';
const point=(x,y)=>`${number(x,2)} ${number(y,2)}`;
const line=(x1,y1,x2,y2)=>`M${point(x1,y1)}L${point(x2,y2)}`;
const set=(root,selector,attribute,value)=>{const node=one(root,selector);if(node)node.setAttribute(attribute,value)};
const write=(root,selector,value)=>{const node=one(root,selector);if(node)node.textContent=value};
const svgCircle=(x,y,r,cl)=>`<circle cx="${x}" cy="${y}" r="${r}" class="${cl}"/>`;
const arrowPath=(x,y,dx,dy)=>{
 const len=Math.hypot(dx,dy);if(len<1)return '';
 const ex=x+dx,ey=y+dy,a=9,b=4.5,ux=dx/len,uy=dy/len;
 return `${line(x,y,ex,ey)}M${point(ex-ux*a-uy*b,ey-uy*a+ux*b)}L${point(ex,ey)}L${point(ex-ux*a+uy*b,ey-uy*a-ux*b)}`;
};
function motors(root){
 const a=one(root,'[data-motor-input="v1"]'),b=one(root,'[data-motor-input="v2"]');
 function draw(){
  const v1=Number(a.value),v2=Number(b.value),v=v1+v2;
  write(root,'[data-motor-out="v1"]',signed(v1));
  write(root,'[data-motor-out="v2"]',signed(v2));
  write(root,'[data-motor-total]',signed(v));
  set(root,'[data-car]','transform',`translate(${170+v*18} 133)`);
  set(root,'[data-car-arrow]','d',arrowPath(170,96,v*18,0));
  const x=170+v1*31,y=113-v2*31;
  set(root,'[data-input-point]','cx',x);set(root,'[data-input-point]','cy',y);
  set(root,'[data-point-a]','d',line(170,113,x,113));
  set(root,'[data-point-b]','d',line(x,113,x,y));
  if(v===0 && (v1||v2))write(root,'[data-motor-message]','两个电机都在工作，但输入落在斜线上，合成速度为零');
  else if(v===0)write(root,'[data-motor-message]','两个输入都是零，小车静止，这不是非零的零空间运动');
  else write(root,'[data-motor-message]',`输入没有完全抵消，小车产生${v>0?'向右':'向左'}的合成速度`);
  root.classList.toggle('is-zero',v===0);
 }
 [a,b].forEach(slider=>slider.addEventListener('input',draw));
 root.querySelectorAll('[data-motor-preset]').forEach(button=>button.addEventListener('click',()=>{const preset=button.dataset.motorPreset;const vals=preset==='cancel'?[2,-2]:preset==='add'?[2,2]:[0,0];a.value=vals[0];b.value=vals[1];draw()}));
 draw();
}
function singularity(root){
 const slider=one(root,'[data-sing-input]'),scale=92,L1=1.25,L2=1.05,q1=15*rad,x0=250,y0=177;
 const cv=([x,y])=>[x0+scale*x,y0-scale*y];
 function draw(){
  const q2=Number(slider.value),angle=q1+q2*rad;
  const p0=[0,0],p1=[L1*Math.cos(q1),L1*Math.sin(q1)],p2=[p1[0]+L2*Math.cos(angle),p1[1]+L2*Math.sin(angle)];
  const [x1,y1]=cv(p1),[x2,y2]=cv(p2),[x0s,y0s]=cv(p0);
  set(root,'[data-2r-link1]','d',line(x0s,y0s,x1,y1));set(root,'[data-2r-link2]','d',line(x1,y1,x2,y2));
  one(root,'[data-2r-joints]').innerHTML=svgCircle(x0s,y0s,8,'rank-joint')+svgCircle(x1,y1,8,'rank-joint')+svgCircle(x2,y2,7,'rank-effector');
  // Jacobian columns: J1=[-y_TCP,x_TCP], J2=[-L2 sin(q1+q2),L2 cos(q1+q2)]
  const j1=[-p2[1],p2[0]],j2=[-L2*Math.sin(angle),L2*Math.cos(angle)];
  const norm=v=>Math.hypot(v[0],v[1]),vec=(v,len)=>[v[0]/norm(v)*len,-v[1]/norm(v)*len];
  const a=vec(j1,55),b=vec(j2,38);
  set(root,'[data-2r-vector1]','d',arrowPath(x2,y2,a[0],a[1]));
  set(root,'[data-2r-vector2]','d',arrowPath(x2,y2,b[0],b[1]));
  const isSingular=q2===0;
  const r=vec(p2,62);
  set(root,'[data-2r-lost]','d',isSingular?arrowPath(x2,y2,r[0],r[1]):'');
  const determinant=L1*L2*Math.sin(q2*rad);
  write(root,'[data-sing-out]',`${signed(q2)}°`);write(root,'[data-sing-rank]',isSingular?'1':'2');
  write(root,'[data-sing-nullity]',isSingular?'1':'0');write(root,'[data-sing-det]',isSingular?'0':number(determinant,3).toFixed(3));
  write(root,'[data-sing-message]',isSingular?'两列速度箭头平行，秩从 2 降到 1，沿手臂径向的末端瞬时运动无法独立产生':'两个速度箭头不共线，雅可比满秩，能够独立合成平面内任意方向的瞬时速度');
  root.classList.toggle('is-singular',isSingular);
 }
 slider.addEventListener('input',draw);
 root.querySelectorAll('[data-sing-preset]').forEach(button=>button.addEventListener('click',()=>{slider.value=button.dataset.singPreset;draw()}));
 draw();
}
function redundancy(root){
 const slider=one(root,'[data-red-input]'),L1=.95,L2=.95,L3=.85,goal=[1.55,.45],scale=108,x0=164,y0=201;
 const cv=([x,y])=>[x0+scale*x,y0-scale*y];
 let prev=null;
 function solve(q1){
  const a=q1*rad, p1=[L1*Math.cos(a),L1*Math.sin(a)];
  const wx=goal[0]-p1[0],wy=goal[1]-p1[1],d2=wx*wx+wy*wy;
  const c=Math.max(-1,Math.min(1,(d2-L2*L2-L3*L3)/(2*L2*L3)));
  const gamma=Math.acos(c),beta=Math.atan2(wy,wx)-Math.atan2(L3*Math.sin(gamma),L2+L3*Math.cos(gamma));
  const p2=[p1[0]+L2*Math.cos(beta),p1[1]+L2*Math.sin(beta)];
  const tcp=[p2[0]+L3*Math.cos(beta+gamma),p2[1]+L3*Math.sin(beta+gamma)];
  return {p:[[0,0],p1,p2,tcp],pose:(beta+gamma)/rad,positionError:Math.hypot(tcp[0]-goal[0],tcp[1]-goal[1])};
 }
 function draw(){
  const q=Number(slider.value),model=solve(q),p=model.p.map(cv),old=prev?.p.map(cv);
  if(old)set(root,'[data-3r-prev]','d',old.map((o,i)=>(i?'L':'M')+point(...o)).join(''));
  else set(root,'[data-3r-prev]','d','');
  ['link1','link2','link3'].forEach((name,i)=>set(root,`[data-3r-${name}]`,'d',line(...p[i],...p[i+1])));
  one(root,'[data-3r-joints]').innerHTML=p.map(([x,y],i)=>svgCircle(x,y,i===3?7:8,i===3?'rank-effector':'rank-joint')).join('');
  set(root,'[data-3r-target]','cx',p[3][0]);set(root,'[data-3r-target]','cy',p[3][1]);
  set(root,'[data-3r-target-label]','x',p[3][0]+18);set(root,'[data-3r-target-label]','y',p[3][1]-16);
  write(root,'[data-red-out]',`${signed(q)}°`);
  write(root,'[data-red-x]',`${goal[0].toFixed(2)} m`);
  write(root,'[data-red-y]',`${goal[1].toFixed(2)} m`);
  write(root,'[data-red-angle]',`${signed(number(model.pose,1))}°`);
  write(root,'[data-red-message]',`关节形状变化，但末端位置保持在目标点，数值位置误差 ${model.positionError.toExponential(1)} m，末端朝向不受约束`);
  prev=model;
 }
 slider.addEventListener('input',draw);
 root.querySelectorAll('[data-red-preset]').forEach(button=>button.addEventListener('click',()=>{slider.value=button.dataset.redPreset;draw()}));
 draw();
}
window.initRankLessonDemos=()=>{
 document.querySelectorAll('#page-content [data-rank-lab]').forEach(root=>{
  const kind=root.dataset.rankLab;
  if(kind==='motors')motors(root);
  if(kind==='singularity')singularity(root);
  if(kind==='redundancy')redundancy(root);
 });
};
})();
