#!/usr/bin/env python3
"""Reproducible Jacobian/finite-difference demonstration for a planar 2R arm.

Run from repository root:
  python labs/python/arm2r/main.py --check --out labs/python/arm2r/results
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

L1 = 1.0  # m
L2 = 0.7  # m


def fk(q: np.ndarray, l1=L1, l2=L2) -> np.ndarray:
    """Position of the end-effector origin in the fixed base frame, meters."""
    q1, q2 = np.asarray(q, dtype=float)
    return np.array([l1*np.cos(q1)+l2*np.cos(q1+q2),
                     l1*np.sin(q1)+l2*np.sin(q1+q2)])


def jacobian(q: np.ndarray, l1=L1, l2=L2) -> np.ndarray:
    """Geometric/position Jacobian J_p: joint angular velocities -> tip xy velocity."""
    q1, q2 = np.asarray(q, dtype=float)
    return np.array([[-l1*np.sin(q1)-l2*np.sin(q1+q2), -l2*np.sin(q1+q2)],
                     [l1*np.cos(q1)+l2*np.cos(q1+q2),  l2*np.cos(q1+q2)]])


def jacobian_fd(q: np.ndarray, h=1.0e-6) -> np.ndarray:
    """Central finite difference of the forward kinematics map."""
    j = np.empty((2, 2))
    for axis in range(2):
        dq = np.zeros(2)
        dq[axis] = h
        j[:, axis] = (fk(q+dq)-fk(q-dq))/(2*h)
    return j


def js_spatial(q: np.ndarray) -> np.ndarray:
    """Angular-first 6D space Jacobian in fixed {s}, planar 2R embedded in 3D."""
    q1 = float(q[0])
    j = np.zeros((6, 2))
    j[2, :] = 1.0
    j[3, 1] = L1*np.sin(q1)
    j[4, 1] = -L1*np.cos(q1)
    return j


def space_twist_to_tip_velocity(q: np.ndarray, dq: np.ndarray) -> np.ndarray:
    """Convert space twist linear component to physical velocity of tool origin."""
    v = js_spatial(q) @ dq
    p3 = np.array([*fk(q), 0.0])
    omega = v[:3]
    return (v[3:]+np.cross(omega, p3))[:2]


def analytic_accel(q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
    """Physical Cartesian acceleration via J ddq + Jdot dq for constant lengths."""
    a, b = q
    da, db = dq
    dd_a, dd_b = ddq
    w = da+db
    alpha = dd_a+dd_b
    return np.array([
        -L1*(np.cos(a)*da**2+np.sin(a)*dd_a) - L2*(np.cos(a+b)*w*w+np.sin(a+b)*alpha),
        -L1*(np.sin(a)*da**2-np.cos(a)*dd_a) - L2*(np.sin(a+b)*w*w-np.cos(a+b)*alpha),
    ])


def dls(j: np.ndarray, target: np.ndarray, damping: float) -> np.ndarray:
    """Regularized minimum-norm task-velocity inverse mapping."""
    return j.T @ np.linalg.solve(j@j.T + damping**2*np.eye(j.shape[0]), target)


def perform_checks() -> dict:
    rng = np.random.default_rng(42)
    errors, space_errors = [], []
    for _ in range(120):
        q = rng.uniform(-np.pi, np.pi, 2)
        dq = rng.normal(size=2)
        errors.append(float(np.max(np.abs(jacobian(q)-jacobian_fd(q)))))
        space_errors.append(float(np.max(np.abs(jacobian(q)@dq - space_twist_to_tip_velocity(q, dq)))))
    max_fd = max(errors)
    max_space = max(space_errors)
    example_q = np.array([0., np.pi/2])
    example_dq = np.array([.2, -.1])
    example_velocity = jacobian(example_q)@example_dq
    expected = np.array([-.07, .20])
    example_err = float(np.max(np.abs(example_velocity-expected)))
    det_errors, singular_sv = [], []
    for q2 in np.linspace(-np.pi, np.pi, 127):
        j = jacobian(np.array([.29, q2]))
        det_errors.append(abs(np.linalg.det(j)-L1*L2*np.sin(q2)))
    for q2 in [0., np.pi, -np.pi]:
        singular_sv.append(float(np.linalg.svd(jacobian(np.array([.38, q2])), compute_uv=False)[-1]))
    # Verify that Jdot*dq is not generally zero at constant angular velocity.
    qa=np.array([.3,.7]); dqa=np.array([.8,-.4]); ddqa=np.zeros(2)
    dt=1.e-5
    accel_num=(jacobian(qa+dqa*dt)@dqa-jacobian(qa-dqa*dt)@dqa)/(2*dt)
    accel_err=float(np.max(np.abs(analytic_accel(qa,dqa,ddqa)-accel_num)))
    # DLS remains finite at a singularity; avoid claiming it can track unattainable targets.
    dls_norm=float(np.linalg.norm(dls(jacobian(np.array([0.,0.])),np.array([1.,0.]),.05)))
    stats={
        'samples':120,
        'max_abs_analytic_vs_central_difference':max_fd,
        'max_abs_spatial_twist_vs_point_velocity':max_space,
        'max_abs_example_error':example_err,
        'max_abs_det_formula_error':float(max(det_errors)),
        'max_singular_sigma_min':max(singular_sv),
        'max_abs_acceleration_error':accel_err,
        'dls_norm_for_unachievable_direction':dls_norm,
        'example_velocity_m_per_s':example_velocity.tolist(),
        'passed':bool(max_fd<1e-8 and max_space<1e-12 and example_err<1e-12
                 and max(det_errors)<1e-12 and max(singular_sv)<1e-12
                 and accel_err<1e-8 and np.isfinite(dls_norm)),
    }
    return stats


def figures(out: Path) -> None:
    out.mkdir(parents=True,exist_ok=True)
    angles=np.linspace(-np.pi,np.pi,401)
    with (out/'singular_values.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['q2_rad','sigma_min_m','sigma_max_m','det_j_m2'])
        for a in angles:
            j=jacobian(np.array([np.pi/6,a]))
            sv=np.linalg.svd(j,compute_uv=False)
            w.writerow([f'{a:.12f}',f'{sv[-1]:.12f}',f'{sv[0]:.12f}',f'{np.linalg.det(j):.12f}'])
    fig, ax=plt.subplots(figsize=(8.4,4.7))
    ss=np.array([np.linalg.svd(jacobian(np.array([np.pi/6,a])),compute_uv=False) for a in angles])
    ax.plot(angles*180/np.pi,ss[:,0],label='Largest singular value')
    ax.plot(angles*180/np.pi,ss[:,1],label='Smallest singular value')
    ax.set(xlabel='Second joint angle (degrees)',ylabel='Singular value (m)',title='2R Jacobian singular values vs joint configuration')
    ax.axhline(0,linewidth=.7);ax.legend();ax.grid(True,alpha=.25)
    fig.tight_layout();fig.savefig(out/'singular_values.png',dpi=145);plt.close(fig)
    q=np.array([np.pi/6,np.pi/3]);dq=np.array([.8,-.5]);p=fk(q);v=jacobian(q)@dq
    shoulder=np.zeros(2);elbow=np.array([np.cos(q[0]),np.sin(q[0])])
    fig,ax=plt.subplots(figsize=(7,5))
    ax.plot([shoulder[0],elbow[0],p[0]],[shoulder[1],elbow[1],p[1]],'-o',lw=5,markersize=9)
    ax.quiver([p[0]],[p[1]],[v[0]],[v[1]],angles='xy',scale_units='xy',scale=1.0)
    ax.set(xlim=(-.3,1.85),ylim=(-.25,1.85),aspect='equal',xlabel='x (m)',ylabel='y (m)',title='2R end-effector instantaneous velocity')
    ax.grid(True,alpha=.2);fig.tight_layout();fig.savefig(out/'velocity_example.png',dpi=145);plt.close(fig)


def main()->int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=Path(__file__).parent/'results')
    parser.add_argument('--check',action='store_true',help='Exit nonzero if independent numeric checks fail')
    args=parser.parse_args()
    stats=perform_checks()
    figures(args.out)
    (args.out/'metrics.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(stats,ensure_ascii=False,indent=2))
    print('Saved plot, CSV, and metrics to',args.out)
    return 0 if (not args.check or stats['passed']) else 1


if __name__=='__main__':sys.exit(main())
