#!/usr/bin/env python3
"""Standalone Robot validation: 2R DH/PoE/FK/IK, SO(3), SE(3) and Jacobian.

Run: python labs/python/kinematics/main.py --check
No ROS or network dependencies. All random tests use a fixed seed.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

L1, L2 = 1.0, 0.7


def hat(w: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(w, dtype=float)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_so3(w: np.ndarray, theta: float) -> np.ndarray:
    w = np.asarray(w, dtype=float)
    norm = np.linalg.norm(w)
    if norm < 1e-14:
        return np.eye(3)
    a = w / norm
    t = theta * norm
    A = hat(a)
    return np.eye(3) + np.sin(t) * A + (1-np.cos(t))*(A@A)


def exp_se3(s: np.ndarray, theta: float) -> np.ndarray:
    omega, v = np.asarray(s[:3], dtype=float), np.asarray(s[3:], dtype=float)
    norm = np.linalg.norm(omega)
    T = np.eye(4)
    if norm < 1e-14:
        T[:3, 3] = v * theta
        return T
    w = omega / norm
    t = theta * norm
    A = hat(w)
    T[:3, :3] = exp_so3(w, t)
    G = np.eye(3)*t + (1-np.cos(t))*A + (t-np.sin(t))*(A@A)
    T[:3, 3] = G @ (v / norm)
    return T


def trans_x(length: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = length
    return T


def rot_z(t: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = exp_so3(np.array([0,0,1.]), t)
    return T


def fk_dh(q: np.ndarray) -> np.ndarray:
    return rot_z(q[0]) @ trans_x(L1) @ rot_z(q[1]) @ trans_x(L2)


def fk_poe(q: np.ndarray) -> np.ndarray:
    S1 = np.array([0.,0.,1.,0.,0.,0.])
    S2 = np.array([0.,0.,1.,0.,-L1,0.])
    M = trans_x(L1 + L2)
    return exp_se3(S1, q[0]) @ exp_se3(S2, q[1]) @ M


def fk_xy(q: np.ndarray) -> np.ndarray:
    a, b = q[0], q[0]+q[1]
    return np.array([L1*np.cos(a)+L2*np.cos(b), L1*np.sin(a)+L2*np.sin(b)])


def jac(q: np.ndarray) -> np.ndarray:
    a, b = q[0], q[0]+q[1]
    return np.array([[-L1*np.sin(a)-L2*np.sin(b), -L2*np.sin(b)],
                     [ L1*np.cos(a)+L2*np.cos(b),  L2*np.cos(b)]])


def ik_analytic(p: np.ndarray, tol: float = 1e-12) -> list[np.ndarray]:
    x, y = map(float,p)
    D = (x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)
    if D > 1+tol or D < -1-tol:
        return []
    D = float(np.clip(D, -1.,1.))
    solutions = []
    for sgn in (1., -1.):
        q2 = np.arctan2(sgn*np.sqrt(max(0,1-D*D)),D)
        q1 = np.arctan2(y,x)-np.arctan2(L2*np.sin(q2), L1+L2*np.cos(q2))
        result = np.array([q1,q2])
        if not any(np.linalg.norm(np.angle(np.exp(1j*(result-old)))) < 1e-10 for old in solutions):
            solutions.append(result)
    return solutions


def ik_dls(p: np.ndarray, start: np.ndarray, *, damping: float = 0.04, alpha: float = 0.7,
           max_iter: int = 600, tol: float = 1e-9) -> tuple[np.ndarray, float, int]:
    q = np.array(start, dtype=float)
    for k in range(max_iter):
        e = np.asarray(p) - fk_xy(q)
        norm = float(np.linalg.norm(e))
        if norm < tol:
            return q, norm, k
        J = jac(q)
        lam = min(damping, max(1e-6, 0.3*np.sqrt(norm)))
        dq = np.linalg.solve(J.T @ J + lam*lam*np.eye(2), J.T @ e)
        if np.linalg.norm(dq) > 0.5: dq *= 0.5 / np.linalg.norm(dq)
        q = q + alpha * dq
    return q, float(np.linalg.norm(p - fk_xy(q))), max_iter


def create_figures(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    t = np.linspace(-np.pi,np.pi,400)
    fig, ax = plt.subplots(figsize=(6,4.6))
    for a in np.linspace(-np.pi,np.pi,90):
        p = np.array([L1*np.cos(a)+L2*np.cos(a+t),L1*np.sin(a)+L2*np.sin(a+t)])
        ax.plot(p[0],p[1],'.',ms=.4,alpha=.1)
    for r in (L1+L2,abs(L1-L2)):
        ax.plot(r*np.cos(t), r*np.sin(t), linewidth=1.8)
    ax.set(xlabel='x (m)',ylabel='y (m)',title='2R reachable positions (without joint limits)')
    ax.set_aspect('equal','box');ax.grid(True,alpha=.2)
    fig.tight_layout();fig.savefig(out/'arm2r-workspace.png',dpi=130);plt.close(fig)

    p=np.array([1.,.55]);sols=ik_analytic(p)
    fig,ax=plt.subplots(figsize=(5.8,4.2))
    for q in sols:
        j1=np.array([L1*np.cos(q[0]),L1*np.sin(q[0])]);xy=np.vstack(([0,0],j1,p))
        ax.plot(xy[:,0],xy[:,1],marker='o',lw=2,label=f'q2={np.rad2deg(q[1]):+.1f} deg')
    ax.plot(p[0],p[1],marker='x',ms=10,color='black',label='target')
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Two inverse-kinematics branches')
    ax.legend();ax.set_aspect('equal','box');ax.grid(True,alpha=.2)
    fig.tight_layout();fig.savefig(out/'arm2r-ik-branches.png',dpi=130);plt.close(fig)


def run_check(n: int = 240) -> dict:
    rng = np.random.default_rng(42)
    errors = {k:0.0 for k in ('poe_dh','fk_analytic','orthogonality','det_rotation','T_inverse','jacobian_fd','ik_analytic','ik_numerical')}
    successes=0
    for i in range(n):
        q=rng.uniform(-np.pi,np.pi,size=2)
        A=fk_dh(q);B=fk_poe(q)
        errors['poe_dh']=max(errors['poe_dh'],float(np.max(np.abs(A-B))))
        errors['fk_analytic']=max(errors['fk_analytic'],float(np.linalg.norm(A[:2,3]-fk_xy(q))))
        R=A[:3,:3]
        errors['orthogonality']=max(errors['orthogonality'],float(np.max(np.abs(R.T@R-np.eye(3)))))
        errors['det_rotation']=max(errors['det_rotation'],abs(float(np.linalg.det(R)-1)))
        errors['T_inverse']=max(errors['T_inverse'],float(np.max(np.abs(A@np.linalg.inv(A)-np.eye(4)))))
        h=1e-6
        finite=np.column_stack([(fk_xy(q+np.eye(2)[j]*h)-fk_xy(q-np.eye(2)[j]*h))/(2*h) for j in range(2)])
        errors['jacobian_fd']=max(errors['jacobian_fd'],float(np.max(np.abs(finite-jac(q)))))
        sols=ik_analytic(fk_xy(q))
        assert sols, 'unexpected missing analytic IK'
        errors['ik_analytic']=max(errors['ik_analytic'],min(float(np.linalg.norm(fk_xy(x)-fk_xy(q))) for x in sols))
        # Start near a known valid solution: numerical DLS must converge without relying on global IK.
        start=q+rng.uniform(-.25,.25,size=2)
        qn,err,_=ik_dls(fk_xy(q),start)
        errors['ik_numerical']=max(errors['ik_numerical'],err)
        if err<1e-7:successes+=1
    assert not ik_analytic(np.array([L1+L2+.2,0.])), 'out-of-reach target accepted'
    # Check both branches separately at a known internal target.
    branches=ik_analytic(np.array([1.,.55]))
    assert len(branches)==2
    assert np.sign(branches[0][1])!=np.sign(branches[1][1])
    assert np.linalg.matrix_rank(jac([0.,0.]))==1
    # Practical tolerances allow cross-platform variations, are set before results are judged.
    limits={'poe_dh':1e-11,'fk_analytic':1e-11,'orthogonality':1e-11,'det_rotation':1e-11,
            'T_inverse':1e-11,'jacobian_fd':3e-9,'ik_analytic':1e-10,'ik_numerical':1e-7}
    for name,limit in limits.items():
        assert errors[name] < limit, f'{name}: {errors[name]} >= {limit}'
    assert successes==n, f'{successes}/{n} numerical IK successful'
    return {'samples':n,'seed':42,'max_absolute_errors':errors,'tolerances':limits,
            'ik_numerical_successes':successes,'unreachable_target_rejected':True,
            'two_branches_confirmed':True,'singular_rank_confirmed':True}


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--out',default=str(Path(__file__).resolve().parent/'results'))
    args=parser.parse_args()
    output=Path(args.out);output.mkdir(parents=True,exist_ok=True)
    metrics=run_check()
    (output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not args.check:
        create_figures(Path(__file__).resolve().parents[3]/'assets/figures')
    print(json.dumps(metrics,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
