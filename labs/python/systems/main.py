#!/usr/bin/env python3
"""Robot systems engineering teaching experiment; synthetic numbers, NOT a real-time or safety certification test."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
RESULTS = Path(__file__).resolve().parent / 'results'
RNG = np.random.default_rng(42)


def periodic_task():
    """Synthetic single-thread non-preemptive periodic task scheduling."""
    n, T = 1200, .01
    release = np.arange(n) * T
    duration = RNG.uniform(.0025, .0055, size=n)
    overload = np.arange(41, n, 109)
    duration[overload] += .0095
    start = np.empty(n)
    end = np.empty(n)
    for k in range(n):
        start[k] = max(release[k], end[k - 1] if k else 0)
        end[k] = start[k] + duration[k]
    response = end - release
    miss = end > release + T
    assert np.all(end >= start)
    assert np.all(start >= release)
    assert int(np.count_nonzero(miss)) > 0
    assert np.max(response) > T
    return {'release': release, 'response': response, 'miss': miss,
            'deadline_miss_count': int(miss.sum()), 'deadline_miss_rate': float(miss.mean()),
            'response_p99_ms': float(np.percentile(response, 99)*1000),
            'response_max_ms': float(np.max(response)*1000), 'period_ms': T*1000}


def clock_sync():
    """Simulated timestamp clock offset/drift, fit on calibration anchors only."""
    true_t = np.arange(0, 16, 0.05)
    offset, drift = 0.035, 75e-6
    sensor_stamp = true_t * (1 + drift) + offset
    anchors = np.array([0, 4, 8, 12, 15.95])
    anchor_stamps = anchors * (1+drift) + offset
    coeff = np.polyfit(anchor_stamps, anchors, 1)
    corrected = np.polyval(coeff, sensor_stamp)
    err_uncorrected = sensor_stamp - true_t
    err_corrected = corrected - true_t
    assert np.max(np.abs(err_corrected)) < 1e-11
    assert np.max(np.abs(err_uncorrected)) > .035
    return {'t': true_t, 'uncorrected': err_uncorrected, 'corrected': err_corrected,
            'raw_clock_max_error_ms': float(np.max(np.abs(err_uncorrected))*1000),
            'corrected_clock_max_error_ms': float(np.max(np.abs(err_corrected))*1000),
            'offset_ms': offset*1000, 'drift_ppm': drift*1e6}


def latency_pipeline():
    """Synthetic stage latency with rare pathological delays; deliberately NOT a QoS guarantee."""
    n = 1200
    perception = RNG.uniform(.006, .012, n)
    plan = RNG.uniform(.004, .010, n)
    control = RNG.uniform(.0008, .0024, n)
    transport = RNG.uniform(.001, .003, n)
    perception[np.arange(23,n,151)] += .050
    chain = perception+plan+control+transport
    stale = chain > .050
    assert np.any(stale)
    assert abs(float(np.mean(chain))-float(np.mean(perception+plan+control+transport))) < 1e-12
    return {'total':chain, 'perception':perception, 'plan':plan, 'control':control,
            'transport':transport, 'latency_p99_ms':float(np.percentile(chain,99)*1000),
            'latency_max_ms':float(np.max(chain)*1000),
            'age_budget_ms':50., 'stale_count':int(stale.sum())}


class TeachingSupervisor:
    """Teaching-only state transitions: this is NOT a safety controller."""
    def __init__(self):
        self.state='INHIBITED'
        self.fault=False
        self.estop=False

    def update(self, *, activate=False, fault=False, estop=False, reset=False,
               operator_ack=False, guards_clear=False):
        if estop:
            self.estop=True
            self.state='ESTOP_LATCHED'
            return self.state
        if fault and not self.estop:
            self.fault=True
            self.state='FAULT_LATCHED'
            return self.state
        if self.estop:
            if reset and operator_ack and guards_clear and not fault and not estop:
                self.estop=False
                self.fault=False
                self.state='INHIBITED'
            return self.state
        if self.fault:
            if reset and operator_ack and guards_clear and not fault:
                self.fault=False
                self.state='INHIBITED'
            return self.state
        if self.state == 'INHIBITED':
            if reset and operator_ack and guards_clear:
                self.state='READY'
        elif self.state == 'READY' and activate and guards_clear:
            self.state='ACTIVE'
        elif self.state == 'ACTIVE' and not guards_clear:
            self.fault=True
            self.state='FAULT_LATCHED'
        return self.state


def supervisor_test():
    s=TeachingSupervisor()
    states=[s.state]
    events=[{'activate':True},{'reset':True,'operator_ack':True,'guards_clear':True},
            {'activate':True,'guards_clear':True},{'fault':True},
            {},{'activate':True},{'reset':True,'guards_clear':True},
            {'reset':True,'operator_ack':True,'guards_clear':True},
            {'reset':True,'operator_ack':True,'guards_clear':True},
            {'activate':True,'guards_clear':True},{'estop':True},
            {'activate':True,'guards_clear':True},
            {'reset':True,'operator_ack':True,'guards_clear':True},
            {'reset':True,'operator_ack':True,'guards_clear':True},
            {'activate':True,'guards_clear':True}]
    for event in events: states.append(s.update(**event))
    expected=['INHIBITED','INHIBITED','READY','ACTIVE','FAULT_LATCHED',
              'FAULT_LATCHED','FAULT_LATCHED','FAULT_LATCHED','INHIBITED','READY',
              'ACTIVE','ESTOP_LATCHED','ESTOP_LATCHED','INHIBITED','READY','ACTIVE']
    assert states == expected, (states, expected)
    return {'states':states, 'no_spontaneous_fault_reset':True, 'estop_remains_latched_without_ack':True}


def figures(sch, sync, pipe):
    assets=ROOT/'assets'/'figures';assets.mkdir(parents=True,exist_ok=True)
    fig, ax=plt.subplots(figsize=(9,3.6))
    ax.plot(sch['release'][:320],sch['response'][:320]*1000,lw=1)
    ax.axhline(sch['period_ms'],ls='--',label='deadline')
    ax.set(xlabel='Synthetic release time (s)',ylabel='Response time (ms)',title='Periodic loop: backlog and deadline misses')
    ax.legend(loc='upper right');fig.tight_layout();fig.savefig(assets/'deadline-backlog.png',dpi=145);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,3.5))
    ax.plot(sync['t'],sync['uncorrected']*1000,label='Raw sensor timestamp')
    ax.plot(sync['t'],sync['corrected']*1000,label='Fitted clock-to-reference mapping')
    ax.set(xlabel='Reference time (s)',ylabel='Timestamp error (ms)',title='Clock offset/drift demonstration')
    ax.legend(loc='best');fig.tight_layout();fig.savefig(assets/'clock-offset.png',dpi=145);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,3.5))
    ax.plot(pipe['total'][:300]*1000,label='End-to-end age',lw=1.0)
    ax.axhline(pipe['age_budget_ms'],ls='--',label='Illustrative age threshold')
    ax.set(xlabel='Synthetic message index',ylabel='Age (ms)',title='End-to-end latency and delayed messages')
    ax.legend();fig.tight_layout();fig.savefig(assets/'latency-chain.png',dpi=145);plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--check',action='store_true',help='Run mathematical/invariant checks and write JSON only')
    opt=parser.parse_args()
    sch=periodic_task(); sync=clock_sync(); pipe=latency_pipeline(); guard=supervisor_test()
    metrics={'model':'Synthetic teaching simulation; not a hardware real-time/safety validation',
             'periodic':{k:v for k,v in sch.items() if not isinstance(v,np.ndarray)},
             'clock':{k:v for k,v in sync.items() if not isinstance(v,np.ndarray)},
             'pipeline':{k:v for k,v in pipe.items() if not isinstance(v,np.ndarray)},
             'teaching_supervisor':guard}
    RESULTS.mkdir(parents=True,exist_ok=True)
    (RESULTS/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    if not opt.check: figures(sch,sync,pipe)
    print('PASS synthetic engineering checks')
    print(json.dumps({k:v for k,v in metrics.items() if k!='teaching_supervisor'},indent=2,ensure_ascii=False))

if __name__=='__main__':main()
