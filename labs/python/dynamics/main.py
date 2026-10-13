#!/usr/bin/env python3
"""planar 2R mechanics checks (SI units; no ROS, hardware or network).

Tests are deterministic. Explicitly distinguish checks of equations from
claims about physical robot identification, rigid impacts or contact accuracy.
"""
from __future__ import annotations
import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class Arm:
    l1: float = 0.8
    l2: float = 0.6
    c1: float = 0.4
    c2: float = 0.28
    m1: float = 1.8
    m2: float = 1.1
    i1: float = 0.045   # planar central Izz, kg*m^2
    i2: float = 0.023
    g: float = 9.81
    b1: float = 0.07    # viscous friction, N*m*s/rad
    b2: float = 0.04


P = Arm()


def pos(q, p=P):
    a, b = q[0], q[0]+q[1]
    return np.array([p.l1*np.cos(a)+p.l2*np.cos(b),
                     p.l1*np.sin(a)+p.l2*np.sin(b)])


def jac(q, p=P):
    a, b = q[0], q[0]+q[1]
    return np.array([[-p.l1*np.sin(a)-p.l2*np.sin(b), -p.l2*np.sin(b)],
                     [p.l1*np.cos(a)+p.l2*np.cos(b), p.l2*np.cos(b)]])


def jdot_times_dq(q, dq, p=P):
    a,b = q[0], q[0]+q[1]
    v = dq[0]+dq[1]
    return np.array([-p.l1*np.cos(a)*dq[0]**2-p.l2*np.cos(b)*v*v,
                     -p.l1*np.sin(a)*dq[0]**2-p.l2*np.sin(b)*v*v])


def masses(p=P):
    a = p.i1 + p.i2 + p.m1*p.c1**2 + p.m2*(p.l1**2+p.c2**2)
    b = p.m2*p.l1*p.c2
    d = p.i2 + p.m2*p.c2**2
    e = (p.m1*p.c1+p.m2*p.l1)*p.g
    f = p.m2*p.c2*p.g
    return np.array([a,b,d,e,f,p.b1,p.b2])


def mass_matrix(q, p=P):
    a,b,d,*_ = masses(p)
    c = np.cos(q[1])
    return np.array([[a+2*b*c, d+b*c],[d+b*c,d]])


def velocity_bias(q, dq, p=P):
    b = masses(p)[1]
    h = b*np.sin(q[1])
    return np.array([-h*(2*dq[0]*dq[1]+dq[1]**2), h*dq[0]**2])


def coriolis_matrix(q, dq, p=P):
    h = masses(p)[1]*np.sin(q[1])
    return np.array([[-h*dq[1], -h*(dq[0]+dq[1])],
                     [ h*dq[0], 0.]])


def gravity(q, p=P):
    e,f = masses(p)[3:5]
    return np.array([e*np.cos(q[0])+f*np.cos(q[0]+q[1]),
                     f*np.cos(q[0]+q[1])])


def potential(q, p=P):
    e,f = masses(p)[3:5]
    return e*np.sin(q[0])+f*np.sin(q[0]+q[1])


def kinetic(q, dq, p=P):
    return .5*dq@mass_matrix(q,p)@dq


def kinetic_from_link_com(q, dq, p=P):
    """Independent physical-link energy formula, NOT based on mass_matrix()."""
    a, b = q[0], q[0]+q[1]
    v1 = p.c1*dq[0]*np.array([-np.sin(a),np.cos(a)])
    v2 = p.l1*dq[0]*np.array([-np.sin(a),np.cos(a)]) + p.c2*(dq[0]+dq[1])*np.array([-np.sin(b),np.cos(b)])
    return .5*(p.m1*(v1@v1) + p.m2*(v2@v2) + p.i1*dq[0]**2 + p.i2*(dq[0]+dq[1])**2)


def friction(dq, p=P):
    return np.array([p.b1*dq[0],p.b2*dq[1]])


def inverse_dynamics(q,dq,ddq,force=None,p=P):
    force=np.zeros(2) if force is None else np.asarray(force)
    return mass_matrix(q,p)@ddq+velocity_bias(q,dq,p)+gravity(q,p)+friction(dq,p)-jac(q,p).T@force


def forward_dynamics(q,dq,tau,force=None,p=P):
    force=np.zeros(2) if force is None else np.asarray(force)
    return np.linalg.solve(mass_matrix(q,p),tau-velocity_bias(q,dq,p)-gravity(q,p)-friction(dq,p)+jac(q,p).T@force)


def regressor(q,dq,ddq,p=P):
    """Regressor for base composites [a,b,d,e,f,b1,b2] (not individual masses/COM)."""
    v0,v1=dq; a0,a1=ddq; c=np.cos(q[1]);s=np.sin(q[1])
    return np.array([[a0, c*(2*a0+a1)-s*(2*v0*v1+v1*v1), a1,
                      np.cos(q[0]),np.cos(q.sum()),v0,0.],
                     [0.,c*a0+s*v0*v0,a0+a1,0.,np.cos(q.sum()),0.,v1]])


def excite(t):
    """Time-parametric multisine: analytic q/dq/ddq avoids noisy differentiation."""
    rates=np.array([.71, 1.39, 2.21, 1.07, 1.91, 2.89])
    amps=np.array([.4,.24,.11,.32,.17,.12])
    shifts=np.array([0.1,0.7,1.2,-0.6,.3,1.1])
    phase=rates*t+shifts
    q=np.array([np.dot(amps[:3],np.sin(phase[:3])),
                .6+np.dot(amps[3:],np.sin(phase[3:]))])
    dq=np.array([np.dot(amps[:3]*rates[:3],np.cos(phase[:3])),
                 np.dot(amps[3:]*rates[3:],np.cos(phase[3:]))])
    ddq=np.array([-np.dot(amps[:3]*rates[:3]**2,np.sin(phase[:3])),
                  -np.dot(amps[3:]*rates[3:]**2,np.sin(phase[3:]))])
    return q,dq,ddq


def run_check(samples=250):
    rng=np.random.default_rng(42)
    eps=1e-6
    errs=dict(gravity_gradient=0., mass_symmetry=0., coriolis_skew=0.,
              kinetic_hessian=0., kinetic_from_com=0., lagrange_residual=0., energy_power=0., id_fd_roundtrip=0.,
              statics_virtual_work=0., regressor=0., contact_acceleration=0.,
              contact_force_residual=0., impact_restitution=0.)
    min_eig=float('inf')
    for _ in range(samples):
        q=rng.uniform(-2.5,2.5,size=2)
        dq=rng.uniform(-1.5,1.5,size=2)
        ddq=rng.uniform(-2,2,size=2)
        F=rng.normal(size=2)
        M=mass_matrix(q)
        min_eig=min(min_eig,float(np.linalg.eigvalsh(M)[0]))
        errs['mass_symmetry']=max(errs['mass_symmetry'],float(np.max(abs(M-M.T))))
        gv=np.array([(potential(q+np.eye(2)[j]*eps)-potential(q-np.eye(2)[j]*eps))/(2*eps) for j in range(2)])
        errs['gravity_gradient']=max(errs['gravity_gradient'],float(np.max(abs(gv-gravity(q)))))
        # Finite differences of velocity derivative are the kinetic Hessian in dq.
        hv=np.column_stack([(mass_matrix(q)@(dq+np.eye(2)[j]*eps)-mass_matrix(q)@(dq-np.eye(2)[j]*eps))/(2*eps) for j in range(2)])
        errs['kinetic_hessian']=max(errs['kinetic_hessian'],float(np.max(abs(hv-M))))
        errs['kinetic_from_com']=max(errs['kinetic_from_com'],abs(kinetic_from_link_com(q,dq)-kinetic(q,dq)))
        # Independently form EL: d/dt(dK/d(dq)) - dK/dq + dU/dq.
        # K is evaluated from physical COM velocities, not the closed-form matrix.
        dK_dq = np.array([(kinetic_from_link_com(q+np.eye(2)[j]*eps,dq)-kinetic_from_link_com(q-np.eye(2)[j]*eps,dq))/(2*eps) for j in range(2)])
        grad_dq = lambda qq, vv: np.array([(kinetic_from_link_com(qq,vv+np.eye(2)[j]*eps)-kinetic_from_link_com(qq,vv-np.eye(2)[j]*eps))/(2*eps) for j in range(2)])
        # Two finite-difference directions (q and dq); avoids second differentiation of noisy data.
        hgrad=2e-4
        ddt_gradK=(grad_dq(q+dq*hgrad,dq+ddq*hgrad)-grad_dq(q-dq*hgrad,dq-ddq*hgrad))/(2*hgrad)
        lagrange=ddt_gradK-dK_dq+gravity(q)
        model=M@ddq+velocity_bias(q,dq)+gravity(q)
        errs['lagrange_residual']=max(errs['lagrange_residual'],float(np.max(abs(lagrange-model))))
        Mdot=sum((mass_matrix(q+np.eye(2)[j]*eps)-mass_matrix(q-np.eye(2)[j]*eps))/(2*eps)*dq[j] for j in range(2))
        skew=Mdot-2*coriolis_matrix(q,dq)
        errs['coriolis_skew']=max(errs['coriolis_skew'],float(np.max(abs(skew+skew.T))))
        lhs=dq@(M@ddq+velocity_bias(q,dq)+gravity(q))
        e_plus=kinetic(q+dq*eps,dq+ddq*eps)+potential(q+dq*eps)
        e_minus=kinetic(q-dq*eps,dq-ddq*eps)+potential(q-dq*eps)
        errs['energy_power']=max(errs['energy_power'],abs(float(lhs-(e_plus-e_minus)/(2*eps))))
        tau=inverse_dynamics(q,dq,ddq,F)
        recovered=forward_dynamics(q,dq,tau,F)
        errs['id_fd_roundtrip']=max(errs['id_fd_roundtrip'],float(np.max(abs(recovered-ddq))))
        dq_virtual=rng.normal(size=2)
        errs['statics_virtual_work']=max(errs['statics_virtual_work'],abs(float(F@(jac(q)@dq_virtual)-(jac(q).T@F)@dq_virtual)))
        yr=regressor(q,dq,ddq)@masses()
        errs['regressor']=max(errs['regressor'],float(np.max(abs(yr-inverse_dynamics(q,dq,ddq)))))
    assert min_eig>0, 'mass matrix lost positive definiteness'

    # Ground at y=0, zero contact-point velocity: solve a sticking normal constraint.
    q=np.array([.3, np.arcsin(-P.l1/P.l2*np.sin(.3))-.3])
    dq=np.zeros(2)
    assert abs(pos(q)[1])<1e-12
    J=jac(q)[1:2,:]
    jdd=jdot_times_dq(q,dq)[1:2]
    M=mass_matrix(q)
    h=velocity_bias(q,dq)+gravity(q)+friction(dq)
    # M qdd = tau - h + J^T lambda; J qdd = - Jdot qdot
    saddle=np.block([[M,-J.T],[J,np.zeros((1,1))]])
    solution=np.linalg.solve(saddle,np.hstack([-h,-jdd]))
    acc=solution[:2]; lam=float(solution[2])
    errs['contact_acceleration']=abs(float((J@acc+jdd).item()))
    errs['contact_force_residual']=float(np.max(abs(M@acc+h-J.T[:,0]*lam)))
    assert lam>=0, 'sticking contact would demand adhesive normal force'
    # Normal-only impulse with a separate pre-impact velocity; restitution e=0.2.
    vminus=-.5; restitution=.2
    eff=float((J@np.linalg.solve(M,J.T)).item())
    impulse=-(1+restitution)*vminus/eff
    vplus=vminus+eff*impulse
    errs['impact_restitution']=abs(vplus+restitution*vminus)
    assert impulse>0

    # OLS: estimate composite/base parameters from synthetic noisy torque observations.
    theta=masses()
    t_train=np.linspace(0,47.5,720,endpoint=False)
    t_test=np.linspace(48,67.5,250,endpoint=False)
    def matrices(t):
        ys=[];zs=[]
        for sec in t:
            q,dq,ddq=excite(sec)
            Y=regressor(q,dq,ddq)
            ys.append(Y);zs.append(Y@theta)
        return np.concatenate(ys),np.concatenate(zs)
    train_y,train_tau=matrices(t_train)
    test_y,test_tau=matrices(t_test)
    singular=np.linalg.svd(train_y,compute_uv=False)
    noisy_tau=train_tau+rng.normal(scale=.025,size=train_tau.shape)
    fit,residuals,rank,_=np.linalg.lstsq(train_y,noisy_tau,rcond=None)
    condition=float(singular[0]/singular[-1])
    param_rel=float(np.linalg.norm(fit-theta)/np.linalg.norm(theta))
    test_rmse=float(np.sqrt(np.mean((test_y@fit-test_tau)**2)))
    trivial_y=np.vstack([regressor(np.zeros(2),np.zeros(2),np.zeros(2)) for _ in range(15)])
    trivial_rank=int(np.linalg.matrix_rank(trivial_y))
    assert rank==7 and trivial_rank<7
    assert param_rel<.02 and test_rmse<.025
    limits={'gravity_gradient':1e-7,'mass_symmetry':1e-12,'coriolis_skew':1e-7,
            'kinetic_hessian':1e-8,'kinetic_from_com':1e-10,'lagrange_residual':3e-6,'energy_power':1e-7,'id_fd_roundtrip':1e-10,
            'statics_virtual_work':1e-12,'regressor':1e-11,
            'contact_acceleration':1e-10,'contact_force_residual':1e-10,
            'impact_restitution':1e-12}
    for k,limit in limits.items():
        assert errs[k]<limit, f'{k}: {errs[k]} >= {limit}'
    return dict(seed=42,random_configurations=samples,max_absolute_errors=errs,tolerances=limits,
                min_mass_eigenvalue=min_eig,contact_normal_force_N=lam,normal_impact_impulse_Ns=impulse,
                identification={'base_parameters':['a','b','d','e','f','b1','b2'],
                                'true':theta.tolist(),'fitted':fit.tolist(),
                                'excitation_rank':int(rank),'weak_excitation_rank':trivial_rank,
                                'regressor_condition_number':condition,
                                'train_rows':len(train_y),'test_rows':len(test_y),
                                'torque_noise_std_Nm':.025,
                                'relative_parameter_error':param_rel,
                                'noise_free_test_rmse_Nm':test_rmse},
                caveat='Synthetic fixed-base planar 2R, rigid ideal links, frictionless unilateral normal contact; not physical robot validation')


def generate_figures(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out.mkdir(parents=True,exist_ok=True)
    q2=np.linspace(-np.pi,np.pi,400)
    eig=np.array([np.linalg.eigvalsh(mass_matrix(np.array([0.,x]))) for x in q2])
    fig,ax=plt.subplots(figsize=(6.3,4.0))
    ax.plot(np.rad2deg(q2),eig[:,0],label='min eigenvalue')
    ax.plot(np.rad2deg(q2),eig[:,1],label='max eigenvalue')
    ax.set(xlabel='q2 (deg)',ylabel='eigenvalue (kg m²)',title='Planar 2R joint-space inertia')
    ax.legend();ax.grid(True,alpha=.25)
    fig.tight_layout();fig.savefig(out/'mass-matrix-eigenvalues.png',dpi=150);plt.close(fig)
    ts=np.linspace(0,15,250)
    grav=np.array([gravity(excite(t)[0]) for t in ts])
    fig,ax=plt.subplots(figsize=(6.3,4.0))
    ax.plot(ts,grav[:,0],label='joint 1');ax.plot(ts,grav[:,1],label='joint 2')
    ax.set(xlabel='time (s)',ylabel='holding torque (N m)',title='Gravity compensation along a trajectory')
    ax.legend();ax.grid(True,alpha=.25)
    fig.tight_layout();fig.savefig(out/'gravity-torques.png',dpi=150);plt.close(fig)
    times=np.linspace(0,47.5,360,endpoint=False)
    Y=np.concatenate([regressor(*excite(t)) for t in times])
    rank_s=np.linalg.svd(Y,compute_uv=False)
    fig,ax=plt.subplots(figsize=(6.3,4.0))
    ax.bar(np.arange(1,8),rank_s)
    ax.set(xlabel='regressor singular value index',ylabel='singular value',title='2R: multisine excitation regressor spectrum')
    ax.set_xticks(range(1,8));ax.grid(True,axis='y',alpha=.25)
    fig.tight_layout();fig.savefig(out/'regressor-singular-values.png',dpi=150);plt.close(fig)
    # Teaching schematic: same q with y_tip=0 used in the contact test.
    q=np.array([.3, np.arcsin(-P.l1/P.l2*np.sin(.3))-.3])
    base=np.array([0.,0.]);joint=np.array([P.l1*np.cos(q[0]),P.l1*np.sin(q[0])]);tip=pos(q)
    fig,ax=plt.subplots(figsize=(6.3,3.8))
    ax.axhline(0,color='#88949c',linewidth=2)
    ax.plot([base[0],joint[0],tip[0]],[base[1],joint[1],tip[1]],'-o',linewidth=4,markersize=7)
    ax.annotate('contact normal',xy=tip,xytext=(tip[0],tip[1]+.32),
                arrowprops={'arrowstyle':'<-','lw':2,'color':'#bd7030'},fontsize=10,ha='center')
    ax.annotate('base',xy=base,xytext=(-.10,.18),fontsize=10)
    ax.annotate('joint 2',xy=joint,xytext=(joint[0]-.25,joint[1]-.20),fontsize=10)
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Ideal unilateral normal contact (tip at y = 0)',ylim=(-.5,.65),xlim=(-.23,1.55))
    ax.set_aspect('equal',adjustable='box');ax.grid(True,alpha=.2)
    fig.tight_layout();fig.savefig(out/'single-contact-geometry.png',dpi=150);plt.close(fig)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--check',action='store_true',help='run checks only, do not regenerate figures')
    ap.add_argument('--out',default=str(Path(__file__).resolve().parent/'results'))
    args=ap.parse_args()
    m=run_check()
    output=Path(args.out);output.mkdir(parents=True,exist_ok=True)
    (output/'metrics.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not args.check:generate_figures(Path(__file__).resolve().parents[3]/'assets/figures')
    print(json.dumps(m,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
