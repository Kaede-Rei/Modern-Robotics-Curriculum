/* Self-contained educational sketches, deliberately separate from the numerical Python checks */
(()=>{'use strict';
const RAD=Math.PI/180;
let active=[];
const fmt=(v,n=2)=>Number(v).toFixed(n);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const clr=(el,key,fallback)=>getComputedStyle(el).getPropertyValue(key).trim()||fallback;
function canvasFor(card){
 const cvs=card.querySelector('canvas');const box=cvs.getBoundingClientRect();const w=Math.max(280,Math.round(box.width||620)),h=262;
 const dpr=Math.min(2,window.devicePixelRatio||1);cvs.width=Math.round(w*dpr);cvs.height=Math.round(h*dpr);const c=cvs.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);
 const theme={ink:clr(card,'--ink','#1c2b2d'),muted:clr(card,'--muted','#6c7777'),accent:clr(card,'--brand','#0e6356'),border:clr(card,'--border','#e3e8e4'),bg:clr(card,'--surface','#ffffff'),orange:'#d58a30',blue:'#3b92ce',red:'#d8655e'};
 c.fillStyle=theme.bg;c.fillRect(0,0,w,h);c.font='12px system-ui,sans-serif';c.textBaseline='middle';
 return {c,w,h,t:theme};
}
function line(c,points,color,width=2,dashes=[]){if(!points.length)return;c.beginPath();points.forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.setLineDash(dashes);c.strokeStyle=color;c.lineWidth=width;c.stroke();c.setLineDash([])}
function point(c,x,y,color,r=5){c.beginPath();c.arc(x,y,r,0,Math.PI*2);c.fillStyle=color;c.fill()}
function txt(c,s,x,y,color,align='left'){c.fillStyle=color;c.textAlign=align;c.fillText(s,x,y);c.textAlign='left'}
function graph(g,curves,{xmin=0,xmax=1,ymin=-1,ymax=1,xtitle='',ytitle='',marks=[]}={}){
 const {c,w,h,t}=g,L=47,R=14,T=18,B=42;const W=w-L-R,H=h-T-B;
 const sx=x=>L+(x-xmin)/(xmax-xmin)*W,sy=y=>T+H-(y-ymin)/(ymax-ymin)*H;
 for(let j=0;j<5;j++){const v=ymin+(ymax-ymin)*j/4,y=sy(v);line(c,[[L,y],[w-R,y]],t.border,1);txt(c,fmt(v,Math.abs(ymax-ymin)>30?0:1),L-7,y,t.muted,'right')}
 for(let j=0;j<5;j++){const v=xmin+(xmax-xmin)*j/4,x=sx(v);line(c,[[x,T],[x,T+H]],t.border,1);txt(c,fmt(v,Math.abs(xmax-xmin)>50?0:1),x,T+H+16,t.muted,'center')}
 curves.forEach(z=>line(c,z.points.map(p=>[sx(p[0]),sy(p[1])]),z.color||t.accent,z.width||2.5,z.dashes||[]));
 for(const z of marks) {if(z.x!=null && z.y!=null)point(c,sx(z.x),sy(z.y),z.color||t.orange,4)}
 txt(c,xtitle,w-R,T+H+36,t.muted,'right');txt(c,ytitle,L,T-7,t.muted);return {sx,sy};
}
function frame(g,v){const{c,w,h,t}=g,th=v.theta*RAD,tx=v.tx,p=[1,.45],out=[tx+Math.cos(th)*p[0]-Math.sin(th)*p[1],Math.sin(th)*p[0]+Math.cos(th)*p[1]],s=Math.min(58,w/7),ox=w/2-35,oy=h/2+32;
 const tr=([x,y])=>[ox+s*x,oy-s*y],axis=(o,angle,col,l=1.35)=>{const a=tr(o),xe=tr([o[0]+Math.cos(angle)*l,o[1]+Math.sin(angle)*l]),ye=tr([o[0]-Math.sin(angle)*l,o[1]+Math.cos(angle)*l]);line(c,[a,xe],col,3);line(c,[a,ye],col,2);point(c,...a,col,4)};
 for(let k=-3;k<=4;k++){line(c,[[0,oy-k*s],[w,oy-k*s]],t.border,.8);line(c,[[ox+k*s,0],[ox+k*s,h]],t.border,.8)}
 axis([0,0],0,t.muted);axis([tx,0],th,t.accent);line(c,[tr([0,0]),tr([tx,0])],t.orange,1.5,[4,4]);point(c,...tr(out),t.red,6);txt(c,'p',tr(out)[0]+9,tr(out)[1]-9,t.red);
 return `局部点 (1.00, 0.45) → 世界点 (${fmt(out[0])}, ${fmt(out[1])}) m`;
}
function fk(g,v){const{c,w,h,t}=g,a=v.q1*RAD,b=(v.q1+v.q2)*RAD,L1=1,L2=.7,ox=w/2,oy=h/2+25,s=Math.min(88,w/5.2),j=[L1*Math.cos(a),L1*Math.sin(a)],e=[j[0]+L2*Math.cos(b),j[1]+L2*Math.sin(b)],tr=p=>[ox+s*p[0],oy-s*p[1]];
 for(let i=-2;i<=2;i++){line(c,[[ox-2*s,oy-i*s],[ox+2*s,oy-i*s]],t.border,1);line(c,[[ox+i*s,5],[ox+i*s,h-4]],t.border,1)}
 line(c,[tr([0,0]),tr(j)],t.blue,7);line(c,[tr(j),tr(e)],t.orange,6);point(c,...tr([0,0]),t.ink,6);point(c,...tr(j),t.ink,6);point(c,...tr(e),t.red,7);txt(c,'TCP',tr(e)[0]+9,tr(e)[1]-9,t.red);return `末端 (${fmt(e[0])}, ${fmt(e[1])}) m · det(J) = ${fmt(.7*Math.sin(v.q2*RAD),3)} m²`;
}
function gravity(g,v){const{t,c,w}=g,q1=v.q1*RAD,q2=v.q2*RAD,m1=1,m2=.7,c1=.45,c2=.4,l1=1,GG=9.81;
 const tq=(a,b)=>[GG*((m1*c1+m2*l1)*Math.cos(a)+m2*c2*Math.cos(a+b)),GG*m2*c2*Math.cos(a+b)];
 const curves=[{color:t.blue,points:[]},{color:t.orange,points:[]}];for(let i=-180;i<=180;i+=2){const z=tq(q1,i*RAD);curves[0].points.push([i,z[0]]);curves[1].points.push([i,z[1]])}
 graph(g,curves,{xmin:-180,xmax:180,ymin:-17,ymax:17,xtitle:'q₂ / °',ytitle:'重力力矩 / N·m'});
 const z=tq(q1,q2),x=47+(v.q2+180)/360*(w-61);line(c,[[x,18],[x,220]],t.red,1,[4,4]);return `τg₁ = ${fmt(z[0])} N·m · τg₂ = ${fmt(z[1])} N·m · 固定 q₁ 时扫描 q₂`;
}
function pd(g,v){const{t}=g,dt=.008,points=[],kp=v.kp,kd=v.kd;let q=0,dq=0,max=0;for(let i=0;i<=1000;i++){const time=i*dt;points.push([time,q]);max=Math.max(max,q);const dd=kp*(1-q)-kd*dq;dq+=dt*dd;q+=dt*dq}
 graph(g,[{points,color:t.accent},{points:[[0,1],[8,1]],color:t.orange,dashes:[5,4]}],{xmin:0,xmax:8,ymin:Math.min(-.2,Math.min(...points.map(x=>x[1]))-.08),ymax:Math.max(1.3,max+.12),xtitle:'t / s',ytitle:'位置 q'});
 return `自然频率 ωn = ${fmt(Math.sqrt(kp))} rad/s · 阻尼比 ζ = ${fmt(kd/(2*Math.sqrt(kp)))} · 最大值 ${fmt(max)}`;
}
function admittance(g,v){const{t}=g,dt=.01,points=[],m=v.mass,d=v.damping,k=v.stiffness,f=6,ss=f/k;let x=0,vel=0,max=0;for(let i=0;i<=800;i++){points.push([i*dt,x]);max=Math.max(max,x);const acc=(f-d*vel-k*x)/m;vel+=dt*acc;x+=dt*vel}
 graph(g,[{points,color:t.accent},{points:[[0,ss],[8,ss]],color:t.orange,dashes:[6,4]}],{xmin:0,xmax:8,ymin:-.03,ymax:Math.max(.35,ss*1.8,max*1.18),xtitle:'t / s',ytitle:'位移 z / m'});
 return `理想静态位移 F/k = ${fmt(ss,3)} m · 阻尼比 ζ = ${fmt(d/(2*Math.sqrt(m*k)))} · 8 s 时 ${fmt(points.at(-1)[1],3)} m`;
}
function trajectory(g,v){const{t}=g,T=v.duration,s=v.time/100,cub=x=>3*x*x-2*x*x*x,quin=x=>10*x*x*x-15*x**4+6*x**5;
 const a=[],b=[];for(let i=0;i<=100;i++){let z=i/100;a.push([i*T,cub(z)]);b.push([i*T,quin(z)])}let min=0,max=T;
 graph(g,[{points:a,color:t.blue},{points:b,color:t.accent}],{xmin:min,xmax:max,ymin:-.04,ymax:1.08,xtitle:'t / s',ytitle:'归一化位置 s'});
 return `当前位置：三次 ${fmt(cub(s),3)} · 五次 ${fmt(quin(s),3)} · 五次峰值速度 = ${fmt(1.875/T,3)} /s`;
}
function gaussian(x,mu,varr){return Math.exp(-((x-mu)**2)/(2*varr))/Math.sqrt(2*Math.PI*varr)}
function kalman(g,v){const{t}=g,K=v.prior/(v.prior+v.noise),post=K*v.obs,P=(1-K)*v.prior,z=v.obs,cs=[{points:[],color:t.blue},{points:[],color:t.orange},{points:[],color:t.accent,width:3}];let ymax=.1;for(let x=-9;x<=9.001;x+=.09){const yy=[gaussian(x,0,v.prior),gaussian(x,z,v.noise),gaussian(x,post,P)];yy.forEach((y,j)=>{cs[j].points.push([x,y]);ymax=Math.max(ymax,y)})}
 graph(g,cs,{xmin:-9,xmax:9,ymin:0,ymax:ymax*1.14,xtitle:'状态 x',ytitle:'概率密度'});
 return `K = ${fmt(K,3)} · 后验均值 = ${fmt(post,3)} · 后验方差 = ${fmt(P,3)} · 蓝：先验  橙：测量  绿：后验`;
}
function camera(g,v){const{c,w,h,t}=g,fx=v.focal,X=v.px,Z=v.depth,u=fx*X/Z,cam=[80,h/2+18],planeX=w-110,scale=Math.min(1,(planeX-80)/(3.5)),object=[80+Z*scale,cam[1]-X*78];
 line(c,[[80,h/2+18],[w-30,h/2+18]],t.muted,1,[4,4]);line(c,[[planeX,40],[planeX,h-30]],t.blue,3);point(c,...cam,t.ink,6);line(c,[cam,object],t.orange,2);point(c,...object,t.red,7);txt(c,'相机',cam[0]-20,cam[1]+20,t.ink);txt(c,'成像平面',planeX-30,27,t.blue);
 const pp=clamp(cam[1]-u*.15,32,h-25);point(c,planeX,pp,t.accent,6);line(c,[[planeX-9,pp],[planeX+9,pp]],t.accent,1);txt(c,'u',planeX+14,pp,t.accent);txt(c,'P(X,Z)',object[0]+8,object[1]-13,t.red);
 return `u − cₓ = f·X/Z = ${fmt(u,1)} px · 已假设无畸变且只看水平坐标`;
}
function occupancy(g,v){const{c,w,h,t}=g,odds=v.hits*Math.log(.75/.25)+v.misses*Math.log(.3/.7),p=1/(1+Math.exp(-odds)),x=w/2-60,y=70,s=120;
 const shade=Math.round(255*(1-p)),hex=shade.toString(16).padStart(2,'0'),fill=`#${hex}${hex}${hex}`;
 c.fillStyle=fill;c.fillRect(x,y,s,s);c.strokeStyle=t.border;c.lineWidth=3;c.strokeRect(x,y,s,s);txt(c,'占据概率',x+60,y-24,t.ink,'center');txt(c,fmt(p*100,1)+'%',x+60,y+s+24,t.ink,'center');txt(c,'白：空闲   黑：占据',x+60,y+s+45,t.muted,'center');return `log-odds = ${fmt(odds,3)} · 后验占据概率 = ${fmt(p*100,1)}% · 未观测时维持 50%`;
}
function imitation(g,v){const{t}=g,w=v.weight/100,std=v.spread,mean=w*(-1.1)+(1-w)*1.1,curves=[{points:[],color:t.accent}];let ymax=.1;for(let x=-2.8;x<=2.8;x+=.035){const y=w*gaussian(x,-1.1,std*std)+(1-w)*gaussian(x,1.1,std*std);curves[0].points.push([x,y]);ymax=Math.max(ymax,y)}
 const o=graph(g,curves,{xmin:-2.8,xmax:2.8,ymin:0,ymax:ymax*1.15,xtitle:'动作 a',ytitle:'示范频率密度'});
 line(g.c,[[o.sx(mean),18],[o.sx(mean),220]],t.red,2,[5,4]);return `MSE 最优单点输出 = ${fmt(mean)} · 两个动作峰位于 -1.10 和 1.10 · 红线表示条件均值`;
}
function bellman(g,v){const{c,w,h,t}=g,immediate=v.reward,disc=v.gamma*v.future,total=immediate+disc,x0=50,s=Math.min(19,(w-95)/16),zero=x0+3*s,yp=70;
 line(c,[[zero,40],[zero,218]],t.muted,1.3);line(c,[[x0,220],[w-10,220]],t.border,1);
 const bar=(y,a,b,color,label)=>{const x1=zero+a*s,x2=zero+b*s;c.fillStyle=color;c.fillRect(Math.min(x1,x2),y,Math.max(2,Math.abs(x2-x1)),26);txt(c,label,x0,y+13,t.ink)};
 txt(c,'即时奖励',x0,33,t.muted);bar(58,0,immediate,t.blue,'');txt(c,'折扣后未来价值',x0,112,t.muted);bar(138,0,disc,t.orange,'');txt(c,'Q 目标',x0,191,t.muted);bar(215,0,total,t.accent,'');
 return `y = r + γ·maxQ = ${fmt(immediate)} + ${fmt(v.gamma)} × ${fmt(v.future)} = ${fmt(total)}`;
}
function deadline(g,v){const{c,w,t}=g,T=v.period,work=v.work,count=8,start=25,scale=(w-50)/(T*count+Math.max(0,work-T)*count+10),base=48;let ready=0,miss=0,maxdelay=0;
 for(let i=0;i<count;i++){const arrive=i*T,beg=Math.max(arrive,ready),end=beg+work,deadline=arrive+T,y=base+i*25;ready=end;const late=end>deadline+1e-8;if(late)miss++;maxdelay=Math.max(maxdelay,beg-arrive);
 line(c,[[start+arrive*scale,y-5],[start+arrive*scale,y+16]],t.muted,1);
 c.fillStyle=late?t.red:t.accent;c.fillRect(start+beg*scale,y,Math.max(2,work*scale),11);
 line(c,[[start+deadline*scale,y-4],[start+deadline*scale,y+14]],t.orange,2);
 txt(c,String(i+1),8,y+6,t.muted);
 }
 txt(c,'绿：按时完成  红：超时  橙：截止时刻',25,248,t.muted);
 return `8 个任务中 ${miss} 次截止时间超时 · 最大排队等待 ${fmt(maxdelay,1)} ms · 不代表真实硬件调度`;
}
const draw={frames:frame,fk,gravity,pd,admittance,trajectory,kalman,camera,occupancy,imitation,bellman,deadline};
window.initLearningLabs=()=>{active=[];document.querySelectorAll('#page-content [data-learning-lab]').forEach(card=>{const id=card.dataset.learningLab;if(!draw[id])return;const sliders=[...card.querySelectorAll('input[type=range]')],out=card.querySelector('[data-lab-summary]');
 const redraw=()=>{const values={};for(const s of sliders){values[s.name]=Number(s.value);const val=card.querySelector(`[data-value="${s.name}"]`);if(val)val.textContent=String(Number(s.value))+(s.dataset.unit?' '+s.dataset.unit:'')}
 const g=canvasFor(card);out.textContent=draw[id](g,values)};
 sliders.forEach(s=>s.addEventListener('input',redraw));active.push(redraw);redraw()})};
window.addEventListener('resize',()=>{active.forEach(fn=>fn())});
})();
