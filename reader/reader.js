(()=>{'use strict';
const data=JSON.parse(document.getElementById('book-payload').textContent);
const $=id=>document.getElementById(id), ids=data.parts.flatMap(p=>p.chapters.map(c=>c.id));
const chapters=Object.fromEntries(data.parts.flatMap(p=>p.chapters.map(c=>[c.id,c])));
const completed=ids.filter(id=>chapters[id].status!=='planned' && !!data.pages[id]);
const toc=$('page-toc'), main=$('main'), content=$('page-content');
let active='guide', activeSection='';
const storedTheme=(()=>{try{return localStorage.getItem('mrc-theme')}catch{return null}})();
if(storedTheme==='dark'||(!storedTheme && matchMedia('(prefers-color-scheme: dark)').matches)) document.documentElement.dataset.theme='dark';
$('theme-toggle').textContent=document.documentElement.dataset.theme==='dark'?'☀':'☾';
$('theme-toggle').onclick=()=>{const dark=document.documentElement.dataset.theme!=='dark';document.documentElement.dataset.theme=dark?'dark':'light';$('theme-toggle').textContent=dark?'☀':'☾';try{localStorage.setItem('mrc-theme',dark?'dark':'light')}catch{}};
$('progress-counter').textContent=`${completed.length} / ${ids.length} 已编写`;
$('progress-fill').style.width=`${100*completed.length/ids.length}%`;
const closeDrawer=()=>{$('sidebar').classList.remove('open');$('drawer-backdrop').hidden=true};
$('sidebar-toggle').onclick=()=>{const open=$('sidebar').classList.toggle('open');$('drawer-backdrop').hidden=!open};$('drawer-backdrop').onclick=closeDrawer;
function navButton(c){const b=document.createElement('button');b.className='chapter-item '+(c.status==='planned'?'planned':'available');b.dataset.target=c.id;b.innerHTML='<span class="chapter-code"></span><span class="chapter-name"></span><span class="status-dot"></span>';
b.querySelector('.chapter-code').textContent=c.id;b.querySelector('.chapter-name').textContent=c.title;b.title=c.status==='planned'?'规划中：尚无正式正文':'阅读 '+c.title;return b}
for(const p of data.parts){const section=document.createElement('section');section.className='part-group';const head=document.createElement('span');head.className='part-label';head.textContent=p.title;section.append(head);for(const c of p.chapters)section.append(navButton(c));$('chapters-nav').append(section)}
$('sidebar').addEventListener('click',e=>{const b=e.target.closest('[data-target]');if(!b)return;navigate(b.dataset.target);closeDrawer()});
function navigate(id,section){const target='#/'+encodeURIComponent(id)+(section?'/'+encodeURIComponent(section):'');if(location.hash===target)render();else location.hash=target}
function route(){const parts=decodeURIComponent((location.hash||'').replace(/^#\/?/,'')).split('/');const id=parts[0]||'guide';return {id:(id==='guide'||id==='notation'||chapters[id])?id:'guide',section:parts[1]||''}}
function htmlEscape(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function missingChapter(c){const prereqs=(c.prerequisites||[]).map(p=>`<button data-target="${htmlEscape(p)}">${htmlEscape(p)} · ${htmlEscape(chapters[p]?.title||p)}</button>`).join('');return `<div class="chapter-planned"><div class="chapter-overline"><span class="eyebrow">${htmlEscape(c.id)} · 学习内容</span><span class="mini-pill">规划中</span></div><h1>${htmlEscape(c.title)}</h1><div class="planned-note"><strong>本章尚未编写</strong><p>本页仅展示课程规划信息，不包含正式教学正文；该章节内容尚未加入当前教材</p></div><h2>计划学习范围</h2><p>${htmlEscape(c.scope||'待整理')}</p><h2>建议先修知识</h2><div class="prereq-chips">${prereqs||'<span>无需额外先修</span>'}</div></div>`}
function guidePage(){const cards=data.parts.map((p,i)=>{const built=p.chapters.filter(c=>c.status!=='planned').length;return `<div class="guide-row"><div><strong>${htmlEscape(p.title)}</strong><p>${p.chapters.length} 章 · 当前已编写 ${built} 章</p></div><button data-target="${htmlEscape(p.chapters[0].id)}">浏览章节 →</button></div>`}).join('');return `<div class="eyebrow">OPEN TEXTBOOK · OFFLINE FIRST</div><div class="guide-hero"><h1>把机器人学，讲清楚</h1><p>现代机器人学：建模、规划、感知、控制与智能</p><p style="color:var(--muted);font-size:13px">以数学建模为主线，从物理直觉理解公式，通过可复现实验验证重要结论</p></div><div class="guide-stat-grid"><div class="guide-stat"><strong>06</strong><small>知识篇章</small></div><div class="guide-stat"><strong>49</strong><small>完整课程规划</small></div><div class="guide-stat"><strong>${completed.length.toString().padStart(2,'0')}</strong><small>已编写的课程章节</small></div></div><h2>开始学习</h2><p>当前已编写 <strong>${completed.length} / ${ids.length}</strong> 章，章节状态由课程目录自动生成；尚未编写的章节明确显示“规划中”；推荐从 F01 开始，也可直接选择当前需要的理论章节；使用左侧目录或按 <kbd>/</kbd> 搜索已编写正文</p><div class="guide-list"><div class="guide-row" style="border-color:var(--brand)"><div><strong>F01 · 机器人学导论</strong><p>推荐从课程入口开始，沿自由度、坐标变换、运动链到正逆运动学顺序学习</p></div><button data-target="F01">从第一章开始 →</button></div><div class="guide-row"><div><strong>M03 · 微分运动学与雅可比矩阵</strong><p>交互样板：末端速度、奇异构型和双连杆仿真实验</p></div><button data-target="M03">进入交互样板 →</button></div>${cards}</div><h2>内容组织方式</h2><p>每一章以<strong>核心、进阶、实践</strong>三层组织：核心讲物理直觉与基本公式，进阶解释严格定义与边界，实践通过交互演示和 Python 实验校验结果</p><h2>离线阅读说明</h2><p>此文件不依赖网络、CDN 或本地服务器，正文、数学公式与基础交互演示都封装在一个 HTML 内；Python / C++ / ROS2 实验源码作为独立配套工程，不包含可在浏览器直接执行的机器人后端</p><blockquote><p>内容参考 Northwestern <em>Modern Robotics</em> 等公开资料，通过 Vibe Coding 整理，仅供个人学习使用</p></blockquote>`}
function headingOf(c){return `<div class="chapter-overline"><span class="eyebrow">${htmlEscape(c.id)} · ${htmlEscape(partFor(c.id))}</span><span class="mini-pill">核心 · 进阶 · 实践</span></div>`}
function partFor(id){return data.parts.find(p=>p.chapters.some(c=>c.id===id))?.title||''}
function render(){const r=route(),id=r.id;active=id;activeSection=r.section;
$('mobile-location').textContent=(chapters[id]?chapters[id].id+' · '+partFor(id):(id==='guide'?'阅读导览':'全书基础约定'));
if(id==='guide'){content.innerHTML=guidePage();document.title='阅读导览 · 现代机器人学'}
else if(id==='notation'){content.innerHTML='<div class="chapter-overline"><span class="eyebrow">REFERENCE · NOTATION</span><span class="mini-pill">全书约定</span></div>'+data.pages.notation.html;document.title='数学符号约定 · 现代机器人学'}
else{const c=chapters[id];content.innerHTML=c.status==='planned'||!data.pages[id]?missingChapter(c):headingOf(c)+data.pages[id].html;document.title=c.id+' '+c.title+' · 现代机器人学'}
content.querySelectorAll('[data-target]').forEach(b=>b.addEventListener('click',()=>navigate(b.dataset.target)));
document.querySelectorAll('[data-target].chapter-item,[data-target].nav-button').forEach(b=>{b.classList.toggle('active',b.dataset.target===id);if(b.dataset.target===id)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current')});
const n=ids.indexOf(id);let prev=null,next=null;
if(n>=0){prev=n>0?chapters[ids[n-1]]:null;next=n+1<ids.length?chapters[ids[n+1]]:null}
else if(id==='guide')next=chapters.F01;
setupPager('prev',prev);setupPager('next',next);
refreshToc();initDemos();
if(r.section){requestAnimationFrame(()=>{const el=content.querySelector('#'+CSS.escape(r.section));if(el)el.scrollIntoView({block:'start'})})}else window.scrollTo({top:0,behavior:'instant'});
}
function setupPager(which,c){const b=$(which);b.disabled=!c;b.onclick=c?()=>navigate(c.id):null;$(which+'-title').textContent=c?c.id+' '+c.title:'—'}
function refreshToc(){toc.replaceChildren();const headings=[...content.querySelectorAll('h2,h3')].filter(h=>h.id);if(!headings.length){toc.textContent='暂无章节小节';return}for(const h of headings){const a=document.createElement('a');a.className='toc-link '+(h.tagName==='H3'?'sub':'');a.href='#/'+encodeURIComponent(active)+'/'+encodeURIComponent(h.id);a.textContent=h.textContent;toc.append(a)}}
window.addEventListener('hashchange',render);document.addEventListener('click',e=>{const a=e.target.closest('a[href^="#/"]');if(a){const url=a.getAttribute('href');if(url===location.hash){e.preventDefault();render()}}});
// Search operates on embedded data only; planned chapter content is never indexed.
const dialog=$('search-dialog'),input=$('search-input');
function openSearch(){if(!dialog.open)dialog.showModal();input.value='';searchNow();input.focus()}
function closeSearch(){if(dialog.open)dialog.close()}
$('search-open').onclick=openSearch;$('search-close').onclick=closeSearch;
function searchable(){return [{id:'guide',title:'课程导览',text:'课程架构 离线阅读 使用说明'},{id:'notation',title:'数学符号约定',text:data.pages.notation.text},...ids.map(id=>({id,title:id+' '+chapters[id].title,text:data.pages[id]?.text||'',planned:chapters[id].status==='planned'}))]}
const entries=searchable();
function searchNow(){const q=input.value.trim().toLocaleLowerCase(),results=$('search-results');results.replaceChildren();if(!q){const hint=document.createElement('p');hint.className='search-empty';hint.textContent='输入关键词，检索已完成的课程内容';results.append(hint);return}
const matches=entries.map(e=>{const title=e.title.toLocaleLowerCase(),t=e.text.toLocaleLowerCase();let score=0;if(title.includes(q))score+=10;if(t.includes(q))score+=e.planned?0:3;return {...e,score,at:t.indexOf(q)}}).filter(e=>e.score>0).sort((a,b)=>b.score-a.score).slice(0,24);
if(!matches.length){const e=document.createElement('p');e.className='search-empty';e.textContent='未找到匹配结果，可换用同义词或章节编号';results.append(e);return}
for(const m of matches){const b=document.createElement('button');b.className='search-hit';const tag=document.createElement('span');tag.textContent=m.planned?'规划中 · 仅匹配标题':m.id==='notation'?'数学约定':'教材正文';const name=document.createElement('strong');name.textContent=m.title;const s=document.createElement('p');const t=m.text||'';const off=m.at<0?0:Math.max(0,m.at-33);s.textContent=(off?'…':'')+t.substring(off,off+116)+(off+116<t.length?'…':'');b.append(tag,name,s);b.onclick=()=>{closeSearch();navigate(m.id)};results.append(b)}}
input.addEventListener('input',searchNow);document.addEventListener('keydown',e=>{const t=e.target?.tagName?.toLowerCase();if(e.key==='/'&&!dialog.open&&!e.ctrlKey&&!e.metaKey&&t!=='input'&&t!=='textarea'){e.preventDefault();openSearch()}else if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='k'){e.preventDefault();openSearch()}});
function initDemos(){content.querySelectorAll('[data-arm2r-demo]').forEach(el=>{const q1=el.querySelector('[data-q1]'),q2=el.querySelector('[data-q2]'),svg=el.querySelector('svg');const item=k=>el.querySelector(`[data-${k}]`);const setLine=(selector,x1,y1,x2,y2)=>{const p=svg.querySelector(selector);for(const [k,v] of Object.entries({x1,y1,x2,y2}))p.setAttribute(k,Number(v).toFixed(3))};const setP=(selector,x,y)=>{const p=svg.querySelector(selector);p.setAttribute('cx',x.toFixed(3));p.setAttribute('cy',y.toFixed(3))};
const draw=()=>{const rad=Math.PI/180,a=+q1.value*rad,b=+q2.value*rad,L1=1,L2=.7,s=132,ox=300,oy=195;
const x1=L1*Math.cos(a),y1=L1*Math.sin(a),x=x1+L2*Math.cos(a+b),y=y1+L2*Math.sin(a+b);
const toX=v=>ox+s*v,toY=v=>oy-s*v;
setLine('.l1',ox,oy,toX(x1),toY(y1));setLine('.l2',toX(x1),toY(y1),toX(x),toY(y));setP('.joint-two',toX(x1),toY(y1));setP('.end',toX(x),toY(y));
const j11=-Math.sin(a)-.7*Math.sin(a+b),j12=-.7*Math.sin(a+b),j21=Math.cos(a)+.7*Math.cos(a+b),j22=.7*Math.cos(a+b);
const vx=j11*.8+j12*(-.5),vy=j21*.8+j22*(-.5),det=.7*Math.sin(b);const tr=j11*j11+j12*j12+j21*j21+j22*j22,disc=Math.sqrt(Math.max(0,tr*tr-4*det*det)),sigmaMin=Math.sqrt(Math.max(0,(tr-disc)/2));
const speedScale=55;setLine('.velocity',toX(x),toY(y),toX(x)+vx*speedScale,toY(y)-vy*speedScale);
item('q1-out').textContent=q1.value+'°';item('q2-out').textContent=q2.value+'°';item('xy-out').textContent=`(${x.toFixed(3)}, ${y.toFixed(3)})`;
item('v-out').textContent=`(${vx.toFixed(3)}, ${vy.toFixed(3)})`;item('det-out').textContent=det.toFixed(4);item('sigma-out').textContent=sigmaMin.toFixed(4);
item('warning').textContent=Math.abs(det)<1e-5?'⚠ 当前构型奇异：雅可比降秩，无法独立产生任意二维末端速度':sigmaMin<.08?'⚠ 接近奇异位形：一些运动方向的控制将变得敏感':'红色箭头表示指定关节角速度产生的末端瞬时速度';};q1.addEventListener('input',draw);q2.addEventListener('input',draw);draw()})}
render();
})();
