const {test} = require('node:test');
const assert = require('node:assert/strict');
const {workModel, fitModel} = require('../reader/projection-lesson.js');
const close = (a,b) => assert.ok(Math.abs(a-b)<1e-10, `${a} != ${b}`);
test('constant-force work has correct sign and units at canonical angles',()=>{
  close(workModel(0).work,20);
  close(workModel(60).work,10);
  close(workModel(90).work,0);
  close(workModel(180).work,-20);
});
test('projection coefficient divides by a dot a, not by length',()=>{
  const m=fitModel(1,2,0.8);
  close(m.optimal,0.8); close(m.p[0],1.6); close(m.p[1],0.8);
  close(m.residual[0],-0.6); close(m.residual[1],1.2);
  close(m.normal,0); close(m.cost,0.9);
});
test('moving away from foot adds exactly squared along-line error',()=>{
  const best=fitModel(1,2,0.8);
  for(const t of [-3,-1,0,1,3]) {
    const m=fitModel(1,2,t);
    close(m.cost, best.cost + 2.5*(t-best.optimal)**2);
  }
});
test('reachable and zero targets have zero residual at projection',()=>{
  for(const [x,y,t] of [[2,1,1],[-4,-2,-2],[0,0,0]]) {
    const m=fitModel(x,y,t);close(m.cost,0);close(m.normal,0);
  }
});
test('best residual orthogonality for every UI target, including negative projections',()=>{
  for(let x=-4;x<=4;x+=0.5) for(let y=-4;y<=4;y+=0.5) {
    const m=fitModel(x,y,(2*x+y)/5);
    close(m.normal,0);assert.ok(m.optimal>=-3&&m.optimal<=3);
  }
});
