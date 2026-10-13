#!/usr/bin/env python3
"""Robot small, deterministic teaching experiments. Not full robot learning systems."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).parent/'results'
FIG=ROOT/'assets/figures'
SEED=42


def design(x):
    x=np.asarray(x,dtype=float)
    return np.stack((x,x*np.abs(x),np.ones_like(x)),axis=-1)


def expert(x):
    return -.52*np.tanh(np.asarray(x,dtype=float))


def solve_policy(x,y):
    return np.linalg.lstsq(design(x),y,rcond=None)[0]


def rollout(weights,starts,horizon=12):
    x=np.asarray(starts).copy()
    states=[]
    for _ in range(horizon):
        states.append(x.copy())
        u=np.clip(design(x)@weights,-.65,.65)
        x=np.clip(x+u,-5,5)
    return np.concatenate(states)


def imitation(rng):
    # Expert demonstrations concentrated near nominal tracking states (little support outside).
    nominal=rng.uniform(-.28,.28,150)
    w0=solve_policy(nominal,expert(nominal))
    # Data-aggregation-inspired loop: learner on-policy states get expert labels; NOT a full DAgger trainer.
    xs=list(nominal);ys=list(expert(nominal))
    for _ in range(4):
        w=solve_policy(np.asarray(xs),np.asarray(ys))
        visited=rollout(w,np.r_[rng.uniform(-2.5,2.5,36),[-3.,3.]])
        xs.extend(visited.tolist()); ys.extend(expert(visited).tolist())
    w1=solve_policy(np.asarray(xs),np.asarray(ys))
    test=np.linspace(-3,3,501)
    rmse0=float(np.sqrt(np.mean((design(test)@w0-expert(test))**2)))
    rmse1=float(np.sqrt(np.mean((design(test)@w1-expert(test))**2)))
    assert rmse1<rmse0*.6, (rmse0,rmse1)
    return {'bc_ood_action_rmse':rmse0,'aggregated_ood_action_rmse':rmse1,
            'initial_demonstrations':len(nominal),'aggregated_examples':len(xs)}, (test,w0,w1)


def reinforcement(rng):
    # 7 position states [0..6], state 6 goal; 2 deterministic actions left/right.
    n=7;goal=6;alpha=.24;gamma=.97;Q=np.zeros((n,2));episodes=3200
    for ep in range(episodes):
        s=int(rng.integers(0,6));epsilon=max(.04,.75*(1-ep/episodes))
        for _ in range(40):
            a=int(rng.integers(0,2)) if rng.random()<epsilon else int(np.argmax(Q[s]))
            sp=int(np.clip(s+(1 if a==1 else -1),0,goal))
            reward=10.0 if sp==goal else -1.0
            td=reward + (0 if sp==goal else gamma*np.max(Q[sp]))-Q[s,a]
            Q[s,a]+=alpha*td;s=sp
            if s==goal:break
    # greedy path test from each nonterminal state.
    routes=[]
    for s0 in range(6):
        s=s0;route=[s]
        for _ in range(16):
            a=int(np.argmax(Q[s]));s=int(np.clip(s+(1 if a==1 else -1),0,goal));route.append(s)
            if s==goal:break
        assert route[-1]==goal,route
        assert len(route)-1==goal-s0,(route,s0)
        routes.append(route)
    return {'episodes':episodes,'discount_gamma':gamma,'greedy_start_zero_steps':len(routes[0])-1,
            'greedy_right_actions':[int(np.argmax(Q[s])) for s in range(6)],'q_state0_right':float(Q[0,1])}, Q


def log_gaussian(x,mu,var):
    return -.5*(np.log(2*np.pi*var)+(x-mu)**2/var)


def denoise_mixture(xt,mus=np.array([-1.5,1.5]),priorvar=.13**2,noisevar=.55**2):
    # Exact Bayes E[X0|Xt] for known 2-Gaussian mixture; pedagogical score/denoising concept.
    v=priorvar+noisevar
    l=np.stack([log_gaussian(xt,mu,v) for mu in mus],axis=0)
    p=np.exp(l-np.max(l,axis=0,keepdims=True));p/=p.sum(axis=0,keepdims=True)
    means=np.stack([(priorvar*xt+noisevar*mu)/(priorvar+noisevar) for mu in mus],axis=0)
    return np.sum(p*means,axis=0)


def generative(rng):
    n=8000;label=rng.integers(0,2,n);x=np.where(label==0,-1.5,1.5)+rng.normal(0,.13,n)
    noisy=x+rng.normal(0,.55,n)
    pred=denoise_mixture(noisy)
    rmse=float(np.sqrt(np.mean((pred-x)**2)))
    noisy_rmse=float(np.sqrt(np.mean((noisy-x)**2)))
    assert rmse<.25 and rmse<noisy_rmse*.45,(rmse,noisy_rmse)
    # Mean is around 0, even though expert demonstrations are rarely close to 0.
    middle_fraction=float(np.mean(np.abs(x)<.45))
    assert middle_fraction<.005
    return {'noisy_action_rmse':noisy_rmse,'bayes_denoised_action_rmse':rmse,
            'low_density_middle_fraction':middle_fraction,'demonstrations':n},(x,noisy,pred)


def world_model():
    # One-step fitted b on source dynamics; compare to target with b changed.
    x=np.linspace(-1,1,51);u=np.linspace(-1,1,51);b_source=1.4;b_target=.55
    # x_next = x + b u (fully observed linear, no stochastic process noise)
    estimated_b=float(np.dot(u,(x+b_source*u)-x)/np.dot(u,u))
    source_err=float(np.sqrt(np.mean(((x+estimated_b*u)-(x+b_source*u))**2)))
    target_err=float(np.sqrt(np.mean(((x+estimated_b*u)-(x+b_target*u))**2)))
    # Prove robust static linear feedback contraction for bounded b in [0.55,1.4]
    klo=1/b_source;krob=2/(b_source+b_target)
    bs=np.linspace(b_target,b_source,200)
    worst_source=float(np.max(np.abs(1-bs*klo)))
    worst_rob=float(np.max(np.abs(1-bs*krob)))
    assert abs(source_err)<1e-12 and target_err>.1 and worst_rob<worst_source
    return {'fit_b_source':estimated_b,'source_one_step_rmse':source_err,
            'shifted_target_one_step_rmse':target_err,'source_feedback_gain':klo,
            'robust_feedback_gain':krob,'source_gain_worst_contraction':worst_source,
            'robust_gain_worst_contraction':worst_rob},(bs,klo,krob)


def plot(imitation_result,gen_result,world_result,Q):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    FIG.mkdir(parents=True,exist_ok=True)
    x,w0,w1=imitation_result
    fig,ax=plt.subplots(figsize=(7.3,4.1))
    ax.plot(x,expert(x),label='Expert action',lw=2)
    ax.plot(x,design(x)@w0,label='BC, narrow demonstration states')
    ax.plot(x,design(x)@w1,label='Aggregated on-policy labels')
    ax.axvspan(-.28,.28,alpha=.12,label='Initial data support')
    ax.set(xlabel='State x',ylabel='Action u',title='Imitation learning: covariate shift')
    ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(FIG/'imitation-shift.png',dpi=135);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7.3,4.1))
    x,noisy,pred=gen_result
    ax.hist(x,bins=65,density=True,alpha=.48,label='Bimodal expert actions')
    ax.axvline(np.mean(x),color='black',ls='--',label='Single MSE mean')
    ax.set(xlabel='Action value',ylabel='Density',title='Multimodal actions: the average may be unrepresentative')
    ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(FIG/'multimodal-actions.png',dpi=135);plt.close(fig)
    bs,ks,kr=world_result
    fig,ax=plt.subplots(figsize=(7.3,4.1))
    ax.plot(bs,np.abs(1-bs*ks),label='Source-tuned feedback')
    ax.plot(bs,np.abs(1-bs*kr),label='Worst-case tuned feedback')
    ax.set(xlabel='Actuator gain b',ylabel='One-step error contraction',title='Known gain interval: nominal vs robust')
    ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(FIG/'sim2real-gain.png',dpi=135);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7.0,3.7))
    ax.imshow(Q[:6],aspect='auto',cmap='viridis');ax.set_xticks([0,1],['left','right']);ax.set_yticks(range(6))
    ax.set(xlabel='Action',ylabel='State',title='Tabular Q-learning after training')
    fig.tight_layout();fig.savefig(FIG/'tabular-rl.png',dpi=135);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');args=parser.parse_args()
    rng=np.random.default_rng(SEED)
    imi,imi_plot=imitation(rng)
    rl,Q=reinforcement(rng)
    gen,gen_plot=generative(rng)
    world,world_plot=world_model()
    metrics={'seed':SEED,'imitation':imi,'tabular_rl':rl,'mixture_denoising':gen,'model_shift_and_robust_gain':world,
             'experiment_scope':'synthetic illustrative checks; not DAgger, Diffusion Policy, VLA, Dreamer or real robot reproduction'}
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    if not args.check:plot(imi_plot,gen_plot,world_plot,Q)
    print('PASS synthetic robot learning checks | '+json.dumps(metrics,ensure_ascii=False))

if __name__=='__main__':main()
