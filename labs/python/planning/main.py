#!/usr/bin/env python3
"""Robot trajectory scaling and discrete A*/Dijkstra numerical checks (fixed grid model)."""
from __future__ import annotations
import argparse
import heapq
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
FIG=ROOT/'assets/figures'
OUT=Path(__file__).parent/'results'


def s3(u):return 3*u*u-2*u**3
def ds3(u):return 6*u-6*u*u
def dds3(u):return 6-12*u
def s5(u):return 10*u**3-15*u**4+6*u**5
def ds5(u):return 30*u*u*(1-u)**2
def dds5(u):return 60*u*(1-u)*(1-2*u)


def trajectory_check():
    u=np.linspace(0,1,20001)
    for fn in (s3,s5):
        assert abs(fn(0))<1e-12 and abs(fn(1)-1)<1e-12
        assert np.min(np.diff(fn(u))) >= -1e-12
    assert ds3(0)==0 and ds3(1)==0 and ds5(0)==0 and ds5(1)==0
    assert dds5(0)==0 and dds5(1)==0
    assert np.isclose(np.max(ds5(u)),15/8,atol=1e-9)
    peak_acc=10*np.sqrt(3)/3
    assert abs(max(abs(dds5(u)))-peak_acc)<1e-6
    dt=1e-5
    for uu in np.linspace(.01,.99,500):
        assert abs((s5(uu+dt)-s5(uu-dt))/(2*dt)-ds5(uu))<1e-7
    return u, {'quintic_max_velocity_scale':float(np.max(ds5(u))),
               'quintic_max_acceleration_scale':float(max(abs(dds5(u)))),
               'quintic_expected_acceleration_scale':float(peak_acc)}


def grid_example():
    grid=np.zeros((28,42),dtype=bool)
    grid[5:25,16]=True
    grid[4,16]=False
    grid[3:24,29]=True
    grid[24,29]=False
    grid[9:20,8:12]=True
    start=(24,3); goal=(3,39)
    assert not grid[start] and not grid[goal]
    return grid,start,goal


def search(grid,start,goal,heuristic=True):
    H,W=grid.shape
    pq=[(0.,0.,start)]; cost={start:0}; came={};seen=set()
    count=0
    while pq:
        _,g,u=heapq.heappop(pq)
        if u in seen:continue
        seen.add(u);count+=1
        if u==goal:
            path=[u]
            while path[-1]!=start:path.append(came[path[-1]])
            return path[::-1],g,count
        for dx,dy in ((0,1),(0,-1),(1,0),(-1,0)):
            v=(u[0]+dx,u[1]+dy)
            if not (0<=v[0]<H and 0<=v[1]<W) or grid[v]:continue
            ng=g+1
            if ng<cost.get(v,float('inf')):
                cost[v]=ng;came[v]=u
                h=(abs(v[0]-goal[0])+abs(v[1]-goal[1])) if heuristic else 0
                heapq.heappush(pq,(ng+h,ng,v))
    raise RuntimeError('grid unreachable')


def planning_check():
    u,curve=trajectory_check()
    grid,start,goal=grid_example()
    path,dist,expanded=search(grid,start,goal)
    dpath,ddist,_=search(grid,start,goal,False)
    assert dist==ddist and dist==len(path)-1
    assert path[0]==start and path[-1]==goal
    assert all(not grid[point] for point in path)
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1])==1 for a,b in zip(path,path[1:]))
    return u,grid,path,{'trajectory':curve,'astar':{'length':len(path)-1,'dijkstra_length':len(dpath)-1,
                'expanded':expanded,'start':start,'goal':goal}}


def plots(u,grid,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    FIG.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True)
    axes[0].plot(u,ds3(u),label='cubic velocity');axes[0].plot(u,ds5(u),label='quintic velocity');axes[0].legend();axes[0].set(ylabel='normalized velocity',title='Normalized polynomial time scaling')
    axes[1].plot(u,dds3(u),label='cubic acceleration');axes[1].plot(u,dds5(u),label='quintic acceleration');axes[1].legend();axes[1].set(xlabel='normalized time',ylabel='normalized acceleration');fig.tight_layout();fig.savefig(FIG/'time-scaling.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5));ax.imshow(grid,interpolation='nearest',cmap='Greys')
    ax.plot([p[1] for p in path],[p[0] for p in path],linewidth=2,label='A* path');ax.scatter([path[0][1],path[-1][1]],[path[0][0],path[-1][0]],marker='x');ax.set(xlabel='grid column',ylabel='grid row',title='A* route in a discrete 4-connected grid');ax.legend();fig.tight_layout();fig.savefig(FIG/'astar-grid.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');args=p.parse_args()
    u,grid,path,m=planning_check()
    OUT.mkdir(exist_ok=True);(OUT/'metrics.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not args.check:plots(u,grid,path)
    print('PASS planning demonstrations',json.dumps(m,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
