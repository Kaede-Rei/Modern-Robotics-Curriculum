#!/usr/bin/env python3
"""Robot teaching checks: linear response, computed torque, admittance, robust bound, MPC, ILC.

All demonstrations are deterministic simulations, not robot hardware validation.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'labs/python/dynamics'))
from main import P, gravity, mass_matrix, velocity_bias, friction, inverse_dynamics, forward_dynamics  # noqa:E402

OUT = Path(__file__).parent / 'results'
FIG = ROOT / 'assets/figures'


def rk4(fun, x, t, h):
    a = fun(t, x)
    b = fun(t + h / 2, x + h * a / 2)
    c = fun(t + h / 2, x + h * b / 2)
    d = fun(t + h, x + h * c)
    return x + h * (a + 2 * b + 2 * c + d) / 6


def response_demo():
    t = np.arange(0, 9.000001, .003)
    profiles = {}
    for zeta in [.3, 1.0, 1.6]:
        x = np.array([0., 0.])
        yy = []
        for tt in t:
            yy.append(float(x[0]))
            x = rk4(lambda _, v: np.array([v[1], 16 * (1 - v[0]) - 8 * zeta * v[1]]), x, tt, .003)
        profiles[str(zeta)] = yy
        assert np.isfinite(x).all() and abs(x[0] - 1) < .001
    assert max(profiles['0.3']) > 1.15
    assert max(profiles['1.6']) < 1.0001
    return t, profiles


def reference(t):
    q = np.array([.45 * np.sin(.7*t), -.5 + .25*np.cos(.8*t)])
    dq = np.array([.315*np.cos(.7*t), -.2*np.sin(.8*t)])
    ddq = np.array([-.2205*np.sin(.7*t), -.16*np.cos(.8*t)])
    return q, dq, ddq


def computed_torque_demo():
    dt = .004
    times = np.arange(0, 7.0001, dt)
    results = {}
    for perturbed in [False, True]:
        x = np.r_[reference(0)[0] + np.array([.2, -.15]), np.array([0., 0.])]
        e_hist = []
        for t in times:
            qd, dqd, ddqd = reference(t)
            e_hist.append(qd - x[:2])
            def f(tt, state):
                qt, dqt, ddqt = reference(tt)
                q, dq = state[:2], state[2:]
                acceleration_command = ddqt + 6.0 * (dqt-dq) + 16.0*(qt-q)
                tau = inverse_dynamics(q, dq, acceleration_command)
                if perturbed:
                    tau = tau + np.array([.6, -.4])   # persistent unknown external actuator torque
                return np.r_[dq, forward_dynamics(q, dq, tau)]
            x = rk4(f, x, t, dt)
        errs=np.asarray(e_hist)
        results['disturbed' if perturbed else 'nominal'] = {
            'time': times, 'err': errs,
            'final_position_error': float(np.linalg.norm(errs[-1])),
            'max_abs_position_error': float(np.max(abs(errs))),
        }
    assert results['nominal']['final_position_error'] < 3e-5
    assert results['disturbed']['final_position_error'] > results['nominal']['final_position_error'] + 1e-4
    qtest = np.array([.45,-.3])
    assert np.allclose(inverse_dynamics(qtest,np.zeros(2),np.zeros(2)),gravity(qtest),atol=1e-12)
    return results


def admittance_demo():
    m, d, k, F = 1.4, 7., 35., 10.
    h = .003
    ts=np.arange(0, 5.00001, h)
    x = np.zeros(2)
    ys=[]
    for t in ts:
        ys.append(x[0])
        def fun(tt, v):
            force = F if .35 <= tt < 1.7 else 0.
            return np.array([v[1], (force-d*v[1]-k*v[0])/m])
        x=rk4(fun, x, t, h)
    ys=np.array(ys)
    assert np.isfinite(ys).all() and np.max(ys) > .11
    assert abs(ys[-1]) < .003
    steady=np.array([0., 0.])
    for j in range(4000):
        steady=rk4(lambda tt,v: np.array([v[1],(F-d*v[1]-k*v[0])/m]),steady,j*.003,.003)
    assert abs(steady[0]-F/k) < 1e-7
    return ts,ys,{'static_displacement_m':float(steady[0]),'predicted_static_m':F/k}


def robust_bound():
    k, bound, e0 = 2., .4, .7
    h=.004
    e=e0
    max_violation=0.
    for j in range(1200):
        t=j*h
        d=.35*math.sin(3.1*t)
        e=rk4(lambda s,v: -k*v + .35*math.sin(3.1*s),e,t,h)
        theoretical=abs(e0)*math.exp(-k*(t+h)) + bound/k*(1-math.exp(-k*(t+h)))
        max_violation=max(max_violation,abs(e)-theoretical)
    assert max_violation<1e-9
    return {'max_bound_violation':float(max_violation)}


def mpc_demo():
    dt=.12
    controls=(-1.,0.,1.)
    horizon=4
    start=np.array([0.,0.])
    target=1.2
    states=[start.copy()]
    actions=[]
    for step in range(100):
        best_cost=float('inf'); best_action=None
        for seq in itertools.product(controls, repeat=horizon):
            x=states[-1].copy()
            cost=0.
            for u in seq:
                x=np.array([x[0]+dt*x[1]+.5*dt**2*u,x[1]+dt*u])
                cost+=(x[0]-target)**2 + .07*x[1]**2 + .035*u*u
            cost+=7*(x[0]-target)**2 + 2*x[1]**2
            if cost<best_cost:
                best_cost=cost;best_action=seq[0]
        u=best_action
        assert abs(u)<=1
        old=states[-1]
        states.append(np.array([old[0]+dt*old[1]+.5*dt**2*u,old[1]+dt*u]))
        actions.append(u)
    states=np.array(states)
    assert abs(states[-1,0]-target)<.11
    assert abs(states[-1,1])<.15
    return {'terminal_position_error': float(abs(states[-1,0]-target)),
            'terminal_speed_m_per_s':float(states[-1,1]),'max_applied_input':float(np.max(np.abs(actions))),
            'num_steps':len(actions)}


def ilc_demo():
    p,ell,d,r=.8,1.1,.15,1.
    u=0.
    errors=[]
    for _ in range(21):
        e=r-(p*u+d)
        errors.append(e)
        u+=ell*e
    errors=np.array(errors)
    rate=1-p*ell
    expected=errors[0]*rate**np.arange(len(errors))
    assert np.max(np.abs(errors-expected))<1e-14
    assert abs(errors[-1])<1e-14
    return errors, {'convergence_factor':rate,'last_error':float(errors[-1])}


def run():
    t,steps=response_demo()
    ct=computed_torque_demo()
    ta, xa, adm=admittance_demo()
    rb=robust_bound()
    mpc=mpc_demo()
    ie,ilc=ilc_demo()
    metrics={'scope':'deterministic teaching simulations; no hardware, no high-dimensional general MPC/adaptive controller',
             'second_order_final_error_max':float(max(abs(y[-1]-1) for y in steps.values())),
             'computed_torque':{k:{f:v for f,v in val.items() if f not in ('time','err')} for k,val in ct.items()},
             'admittance':adm, 'robust_bound':rb, 'finite_action_mpc':mpc, 'scalar_ilc':ilc}
    OUT.mkdir(exist_ok=True);(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return t, steps, ct, ta, xa, ie, metrics


def figures(t,steps,ct,ta,xa,ie):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    FIG.mkdir(parents=True,exist_ok=True)
    fig,ax=plt.subplots(figsize=(7.8,4.0));
    for z,y in steps.items():ax.plot(t,y,label='damping ratio '+z)
    ax.axhline(1,linestyle='--',linewidth=.8);ax.set(xlabel='time (s)',ylabel='position / target',title='Second-order closed-loop step response');ax.legend();fig.tight_layout();fig.savefig(FIG/'second-order-responses.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7.8,4));
    for label,r in ct.items():ax.plot(r['time'],np.linalg.norm(r['err'],axis=1),label=label)
    ax.set(xlabel='time (s)',ylabel='joint position error norm (rad)',title='2R computed-torque tracking');ax.legend();fig.tight_layout();fig.savefig(FIG/'tracking.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7.8,4));ax.plot(ta,xa,label='virtual admittance displacement');ax.axhline(10/35,linestyle='--',label='constant-force static response');ax.set(xlabel='time (s)',ylabel='displacement (m)',title='Admittance response to a finite force pulse');ax.legend();fig.tight_layout();fig.savefig(FIG/'admittance.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7.8,4));ax.semilogy(np.arange(len(ie)),np.abs(ie),marker='o');ax.set(xlabel='repetition index',ylabel='absolute error',title='Scalar iterative learning control');fig.tight_layout();fig.savefig(FIG/'ilc.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');args=parser.parse_args()
    t,steps,ct,ta,xa,ie,metrics=run()
    if not args.check:figures(t,steps,ct,ta,xa,ie)
    print('PASS control demonstrations',json.dumps(metrics,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
