#!/usr/bin/env python3
"""reproducible synthetic perception/estimation checks (NOT a full SLAM system)."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FIG = ROOT / 'assets/figures'
OUT = Path(__file__).parent / 'results'


def project(points, K):
    p = np.asarray(points, dtype=float)
    if np.any(p[..., 2] <= 0):
        raise ValueError('camera points must have positive depth')
    z = p[..., 2]
    return np.stack((K[0, 0]*p[..., 0]/z + K[0, 2], K[1, 1]*p[..., 1]/z + K[1, 2]), axis=-1)


def unproject(uv, z, K):
    u = np.asarray(uv, dtype=float)
    return np.stack(((u[..., 0]-K[0, 2])*z/K[0, 0], (u[..., 1]-K[1, 2])*z/K[1, 1], z), axis=-1)


def geometry_check(rng):
    K = np.array([[525., 0., 320.], [0., 522., 240.], [0., 0., 1.]])
    xyz = np.column_stack((rng.uniform(-1, 1, 300), rng.uniform(-.8, .8, 300), rng.uniform(.7, 6., 300)))
    uv = project(xyz, K)
    rec = unproject(uv, xyz[:, 2], K)
    error = float(np.max(np.abs(xyz-rec)))
    assert error < 2e-14
    # Rigid registration: destination = R source + t. Proper rotation enforced.
    angle = .38
    axis = np.array([.3, -.4, .8]); axis /= np.linalg.norm(axis)
    S = np.array([[0.,-axis[2],axis[1]],[axis[2],0.,-axis[0]],[-axis[1],axis[0],0.]])
    R = np.eye(3) + np.sin(angle)*S + (1-np.cos(angle))*(S@S)
    t = np.array([.4, -.2, .15])
    tgt = xyz @ R.T + t
    src0 = xyz.mean(axis=0); dst0 = tgt.mean(axis=0)
    U,_,Vt = np.linalg.svd((xyz-src0).T @ (tgt-dst0))
    D=np.diag([1.,1.,np.linalg.det(Vt.T@U.T)])
    Rh=Vt.T@D@U.T
    th=dst0-Rh@src0
    assert np.linalg.det(Rh)>0.9999999999
    rot_err=float(np.linalg.norm(Rh-R)); trans_err=float(np.linalg.norm(th-t))
    assert rot_err<1e-12 and trans_err<1e-12
    return {'projection_roundtrip_max_m':error,'svd_registration_rotation_frobenius':rot_err,
            'svd_registration_translation_m':trans_err}, (xyz,tgt)


def ekf_check(rng):
    """SE(2) synthetic robot, known commanded twist + intermittent noisy 2D position fixes."""
    dt=.1;N=350
    x=np.array([0.,0.,.1]); truth=np.empty((N,3))
    est=np.zeros(3); dead=np.zeros(3);P=np.diag([.03,.03,.015])
    traj=np.empty((N,3)); dr=np.empty((N,3)); nis=[]; sym=[]
    I=np.eye(3);H=np.array([[1.,0.,0.],[0.,1.,0.]])
    Q=np.diag([.0015**2,.0015**2,(.003)**2])
    R=np.diag([.06**2,.06**2]);
    def step(state,v,w):
        heading=state[2]+0.5*w*dt
        return state+np.array([v*dt*np.cos(heading), v*dt*np.sin(heading), w*dt])
    for k in range(N):
        v = .65+.13*np.sin(k*.021); w=.15*np.sin(k*.017)+.055
        x = step(x,v,w);truth[k]=x
        # noisy wheel/gyro-like control estimates, correlated in time only through state
        vm=v+rng.normal(0,.045);wm=w+rng.normal(0,.022)
        dead=step(dead,vm,wm);dr[k]=dead
        th=est[2]+.5*wm*dt
        F=np.array([[1.,0.,-vm*dt*np.sin(th)],[0.,1.,vm*dt*np.cos(th)],[0.,0.,1.]])
        est=step(est,vm,wm)
        # velocity/gyro input noise mapped to state increment; not simply fixed Q
        L=np.array([[dt*np.cos(th),-.5*vm*dt**2*np.sin(th)],
                    [dt*np.sin(th), .5*vm*dt**2*np.cos(th)],
                    [0.,dt]])
        P=F@P@F.T+L@np.diag([.045**2,.022**2])@L.T+Q
        if k%5==0:
            z=x[:2]+rng.normal(0,.06,2)
            y=z-H@est;S=H@P@H.T+R
            G=np.linalg.solve(S,(P@H.T).T).T
            est=est+G@y
            P=(I-G@H)@P@(I-G@H).T+G@R@G.T # Joseph form
            nis.append(float(y@np.linalg.solve(S,y)))
        sym.append(float(np.max(np.abs(P-P.T))))
        assert np.linalg.eigvalsh(P).min() > -1e-12
        traj[k]=est
    re=float(np.sqrt(np.mean(np.sum((traj[:,:2]-truth[:,:2])**2,axis=1))))
    rd=float(np.sqrt(np.mean(np.sum((dr[:,:2]-truth[:,:2])**2,axis=1))))
    assert re<rd and re<.17
    assert max(sym)<1e-12
    return {'ekf_position_rmse_m':re,'dead_reckoning_rmse_m':rd,
            'position_fix_count':len(nis),'mean_nis_2d':float(np.mean(nis)),
            'covariance_symmetry_max':max(sym)}, (truth,traj,dr)


def occupancy_check():
    """Inverse binary sensor model log-odds update; simplified independent cells."""
    # Each beam updates free cells and one hit cell. Known prior occupancy 0.5 (log odds 0)
    L=np.zeros((18,28));inc_occ=np.log(.8/.2);inc_free=np.log(.3/.7)
    hit=(9,22)
    for _ in range(4):
        for c in range(4,22):L[9,c]+=inc_free
        L[hit]+=inc_occ
    # Missing observation retains prior
    assert L[0,0]==0
    p=1/(1+np.exp(-np.clip(L,-50,50)))
    assert p[hit]>.99 and p[9,10]<.05 and p[0,0]==.5
    return {'hit_posterior':float(p[hit]),'free_posterior':float(p[9,10]),
            'unobserved_posterior':float(p[0,0])},p


def pose_graph_check(rng):
    """Linear 1D pose graph with anchor and a verified loop constraint; NOT 2D SLAM."""
    N=35
    truth=np.linspace(0,17,N)
    odom=np.diff(truth)+rng.normal(.018,.075,N-1)
    raw=np.r_[0,np.cumsum(odom)]
    # Endpoint absolute return observation (known anchored pose) as a global constraint
    # Distinguish from relative loop closure to unknown start; here x0=0 is fixed.
    end_meas=truth[-1]+rng.normal(0,.025)
    A=np.zeros((N-1,N-1)); b=odom/.075
    for j in range(N-1):
        A[j,j]=1/.075
        if j>0:A[j,j-1]=-1/.075
    A2=np.vstack((A,np.zeros((1,N-1))))
    A2[-1,-1]=1/.025
    b2=np.r_[b,end_meas/.025]
    sol=np.r_[0,np.linalg.lstsq(A2,b2,rcond=None)[0]]
    raw_rmse=float(np.sqrt(np.mean((raw-truth)**2)))
    opt_rmse=float(np.sqrt(np.mean((sol-truth)**2)))
    assert opt_rmse<raw_rmse
    return {'dead_chain_rmse_m':raw_rmse,'constrained_graph_rmse_m':opt_rmse,
            'anchored_final_residual_m':float(abs(sol[-1]-end_meas))},(truth,raw,sol)


def scalar_fusion_check():
    # Independent Gaussian measurements of common constant scalar x
    z1,z2=2.3,1.8;v1,v2=.04,.09
    xhat=(z1/v1+z2/v2)/(1/v1+1/v2)
    vhat=1/(1/v1+1/v2)
    assert vhat<min(v1,v2)
    return {'fused_scalar':float(xhat),'fused_variance':float(vhat)}


def plots(trajectory,occ,poses):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    FIG.mkdir(parents=True,exist_ok=True)
    truth,est,dead=trajectory
    fig,ax=plt.subplots(figsize=(7,5))
    ax.plot(truth[:,0],truth[:,1],label='Ground truth');ax.plot(est[:,0],est[:,1],label='EKF');ax.plot(dead[:,0],dead[:,1],label='Dead reckoning')
    ax.set(xlabel='x [m]',ylabel='y [m]',title='Synthetic planar localization (EKF + position fixes)');ax.axis('equal');ax.legend();fig.tight_layout();fig.savefig(FIG/'ekf-trajectory.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,3.5));im=ax.imshow(occ,cmap='gray_r',vmin=0,vmax=1);ax.set(title='Binary occupancy log-odds updates (synthetic)',xlabel='Grid column',ylabel='Grid row');fig.colorbar(im,ax=ax,label='P(occupied)');fig.tight_layout();fig.savefig(FIG/'occupancy-grid.png',dpi=150);plt.close(fig)
    t,raw,opt=poses
    fig,ax=plt.subplots(figsize=(7,4));ax.plot(t,label='Truth');ax.plot(raw,label='Integrated odometry');ax.plot(opt,label='Anchored graph estimate');ax.set(xlabel='Pose index',ylabel='Position [m]',title='1D graph constraints reduce drift (synthetic)');ax.legend();fig.tight_layout();fig.savefig(FIG/'pose-graph.png',dpi=150);plt.close(fig)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--check',action='store_true');args=ap.parse_args()
    rng=np.random.default_rng(42)
    g,_=geometry_check(rng)
    k,traj=ekf_check(rng)
    o,occ=occupancy_check()
    pg,poses=pose_graph_check(rng)
    f=scalar_fusion_check()
    metrics={'model':'synthetic teaching examples; not real sensor or full SLAM evaluation','seed':42,
             'geometry':g,'state_estimation':k,'mapping':o,'pose_graph':pg,'fusion':f}
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not args.check:plots(traj,occ,poses)
    print('PASS perception/estimation checks')
    print(json.dumps(metrics,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
