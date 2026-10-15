# 最优控制与 MPC：如何在约束下选择未来动作

> **问题：** 电机不能输出任意大力矩；为什么 LQR 能优化一条反馈律，但不能直接保证每一步都满足力矩和避障硬约束？MPC 如何在每个采样时刻重算

**先修：** F04、C02；**层次：** 有限时域优化 → 动态规划/PMP → MPC 可行性与稳定条件


## 1. 把“最优”写成明确目标

考虑离散模型 $\mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k)$；给定预测长度 $N$，决策变量是 $\mathbf u_{0:N-1}$（亦可含预测状态），一类优化问题为

$$
\min\ \sum_{i=0}^{N-1}\ell(\mathbf x_i,\mathbf u_i)+V_f(\mathbf x_N)
$$

$$
\text{s.t.}\quad\mathbf x_{i+1}=f_d(\mathbf x_i,\mathbf u_i),\quad
\mathbf x_i\in\mathcal X,\quad\mathbf u_i\in\mathcal U,\quad\mathbf x_0=\hat{\mathbf x}_k
$$

这里 $\mathcal X,\mathcal U$ 是状态、输入约束；$V_f$ 为终端代价；若省略预测模型、采样步长或约束单位，所谓“优化控制效果更好”没有可复现意义

## 2. 动态规划与最优性原理

定义剩余 $j$ 步最优代价 $V_j(\mathbf x)$，Bellman 递推为

$$
V_{j+1}(\mathbf x)=\min_{\mathbf u\in\mathcal U}\left[\ell(\mathbf x,\mathbf u)+V_j(f_d(\mathbf x,\mathbf u))\right]
$$

这是最优控制的基本结构，但一般连续高维状态空间求解存在维度灾难；连续时间约束下，Pontryagin 极小值原理通过 Hamiltonian 与协态给出必要条件；它通常是**必要条件**，并非自动说明候选轨迹是全局最优

## 3. LQR 为什么是一个重要特例

对线性 $\mathbf x_{k+1}=\mathbf A\mathbf x_k+\mathbf B\mathbf u_k$ 和二次代价 $\ell=\mathbf x^T\mathbf Q\mathbf x+\mathbf u^T\mathbf R\mathbf u$，在适当可稳定化条件下，有解析 Riccati 递推/代数方程；无约束的 LQR 对输入幅值不施加显式硬限值；对最优指令事后裁剪一般会破坏原本的 LQR 最优性证明

## 4. 核心：滚动时域 MPC

在时刻 $k$：估计状态 → 解长度 $N$ 的受约束优化 → 只应用第一步 $\mathbf u_0^*$ → 下个采样时刻根据新状态重新优化；**重算未来不等于下一时刻依然可行**；若发生约束不可行，需要有清晰的约束软化、备用安全控制器或故障处理策略

### 4.1 稳定性保证到底缺哪一块

仅凭 $N$ 足够长或 $Q,R\succ0$ 不自动保证有约束 MPC 稳定；经典充分条件通常涉及终端集合 $\mathcal X_f$、终端控制律 $\kappa_f$ 及终端代价满足

$$
V_f(f_d(\mathbf x,\kappa_f(\mathbf x)))-V_f(\mathbf x)
\le-\ell(\mathbf x,\kappa_f(\mathbf x)),\quad \mathbf x\in\mathcal X_f
$$

以及终端集合在相应控制下保持约束与正向不变等条件；合适条件下可推导递归可行性与闭环稳定；实际系统的估计误差、计算超时和模型失配还需考虑鲁棒 MPC 或安全降级

<figure class="concept-diagram web-source-figure" data-asset="mpc">
  <img src="../../assets/figures/web-png/mpc.png" alt="模型预测控制滚动优化示意图" loading="lazy"/>
  <figcaption>竖线左侧是过去，右侧是当前时刻预测的未来；每次优化一串输入，只执行第一步，再用新测量重新预测，因此预测轨迹不等于最终实测轨迹 · 来源：<a href="https://commons.wikimedia.org/wiki/File:Model_predictive_control.png" target="_blank" rel="noopener noreferrer">Tsesea</a> · <a href="https://creativecommons.org/licenses/by-sa/4.0" target="_blank" rel="noopener noreferrer">CC BY-SA 4.0</a></figcaption>
</figure>

## 5. 工程里最容易漏掉的问题

- 控制周期必须覆盖优化求解时间分布，而非只看平均求解时间
- 刚体动力学可能需要离散化、约束线性化和求解容错
- 末端避障约束通常非凸，即使局部求解器收敛也不保证全局无碰撞
- 所有预测、执行与记录都必须使用相同采样单位；数值模型 $\mathbf A,\mathbf B$ 不等于连续系统 $\mathbf A_c,\mathbf B_c$

## 6. 实践：受限双积分器

`python labs/python/control/main.py --check` 实施一个极小的穷举输入选择示例：固定步长双积分器在有界输入集合内滚动选择最低预测代价的第一步，检查全部真实执行输入满足幅值约束；该算例**只展示有限候选动作 MPC 的滚动决策机制**，不是通用快速非线性 MPC 求解器，也不据此宣称闭环稳定性定理已验证

### 自测

1. 对 $\mathbf x_{k+1}=\mathbf A\mathbf x_k+\mathbf B\mathbf u_k$ 写出两步预测式
2. 为什么只施加 $\mathbf u_0^*$，而不是把整条优化轨迹开环执行到底
3. 区分“求解器返回解”“满足硬约束”“递归可行”“渐近稳定”

**资料：** [MIT Underactuated Robotics, Trajectory Optimization](https://underactuated.mit.edu/trajectories.html)；[MIT LQR](https://underactuated.mit.edu/lqr.html)；Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed.；Boyd & Vandenberghe, *Convex Optimization*