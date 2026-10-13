# 经典控制与状态反馈：从 PID 到 LQR

> **问题：** 为什么伺服驱动器大多有电流、速度和位置环？什么时候 PID 足够，什么时候需要线性状态反馈或 LQR

**先修：** C01；**核心：** PID、状态反馈；**进阶：** 可控性、LQR 与观测器；**实践：** 控制饱和和积分抗饱和检查

## 1. PID 是误差反馈，不是万能模型

全书采用 $e=r-y$；连续 PID 写为

$$
\nu(t)=K_pe(t)+K_i\int_0^t e(\xi)\,d\xi+K_d\dot e(t)
$$

这里 $\nu$ 是尚未受饱和限制的*理想控制器输出*；真实指令 $u=\operatorname{sat}(\nu)$ 可能受电流、速度、力矩约束；在目标发生阶跃时，直接对误差求导可能带来微分冲击（derivative kick）；实践中常改为对测量 $y$ 求导，并给微分环节加低通滤波

- 比例 P：立即针对当前误差纠偏，但一般不能消除所有恒值扰动残差
- 积分 I：积累误差，在闭环稳定及执行器能够提供足够控制量等条件下消除恒定跟踪偏差
- 微分 D：提供与误差变化率相关的阻尼趋势，但容易受噪声干扰

### 1.1 饱和时为什么会积分饱和

若 $u_{\max}$ 已限制输出，积分器仍积累 $e$，恢复控制范围后内部积分状态可能很大，导致显著超调；一个常见的回算抗饱和形式

$$
\dot z=e+K_{aw}(u-\nu),\qquad
\nu=K_pe+K_i z-K_d\dot y
$$

$K_{aw}>0$ 的大小需要与积分、饱和和控制周期共同整定；这不是在任何系统上都能自动保证稳定的“插入式补丁”

## 2. 线性系统的状态反馈

对 $\dot{\mathbf x}=\mathbf A\mathbf x+\mathbf B\mathbf u$，若状态可测且希望在平衡点附近工作，设 $\mathbf u=-\mathbf K\mathbf x$，闭环为 $\dot{\mathbf x}=(\mathbf A-\mathbf B\mathbf K)\mathbf x$；目标不是“计算任意增益”，而是在执行器约束和模型精度允许范围内选择合适的闭环特征值

可控性矩阵

$$
\mathcal C=[\mathbf B\ \mathbf A\mathbf B\ \cdots\ \mathbf A^{n-1}\mathbf B]
$$

当 $\operatorname{rank}(\mathcal C)=n$ 时线性系统可控；一般而言，连续状态反馈能够稳定系统至少要求 $(\mathbf A,\mathbf B)$ **可稳定化**；不可控且不稳定的模态不能靠任意状态反馈“调过去”

## 3. 进阶：LQR 到底最优什么

考虑连续时间无限时域线性二次型

$$
J=\int_0^\infty (\mathbf x^T\mathbf Q\mathbf x+\mathbf u^T\mathbf R\mathbf u)\,dt,
\quad \mathbf Q\succeq0,\ \mathbf R\succ0
$$

满足合适可稳定化、可检测性条件（例如 $(\mathbf A,\mathbf B)$ 可稳定化、$(\mathbf Q^{1/2},\mathbf A)$ 可检测）时，稳定化的半正定 Riccati 解 $\mathbf P$ 满足

$$
\mathbf A^T\mathbf P+\mathbf P\mathbf A-\mathbf P\mathbf B\mathbf R^{-1}\mathbf B^T\mathbf P+\mathbf Q=0
$$

最优反馈为 $\mathbf u=-\mathbf R^{-1}\mathbf B^T\mathbf P\mathbf x$；$\mathbf Q$ 和 $\mathbf R$ 并不代表机械臂固有属性，它们表达设计者对偏差与输入代价的权衡；LQR 不会自动处理输入硬饱和，也不保证模型失配和非线性范围外稳定

### 3.1 跟踪目标不能直接照搬原点镇定公式

稳态目标 $\mathbf x_r,\mathbf u_r$ 应满足 $\mathbf A\mathbf x_r+\mathbf B\mathbf u_r=0$（常值目标），使用 $\mathbf u=\mathbf u_r-\mathbf K(\mathbf x-\mathbf x_r)$；对随时间变化的轨迹还需要适当的动态前馈或时变设计；单纯将状态 $\mathbf x$ 替换成跟踪误差而忽略参考加速度，会产生残差

## 4. 状态不可全部测得怎么办

构造 Luenberger 观测器

$$
\dot{\hat{\mathbf x}}=\mathbf A\hat{\mathbf x}+\mathbf B\mathbf u+\mathbf L(\mathbf y-\mathbf C\hat{\mathbf x})
$$

观测误差 $\tilde{\mathbf x}=\mathbf x-\hat{\mathbf x}$ 满足 $\dot{\tilde{\mathbf x}}=(\mathbf A-\mathbf L\mathbf C)\tilde{\mathbf x}$（无噪声、无模型失配情况下）；线性时间不变系统的分离原理不能直接无条件推广到非线性、时延和强饱和的机器人系统；更一般的传感器融合见 P03

## 5. 实践：伺服环的物理限制

位置环通常输出目标速度，速度环输出目标电流/力矩，电流环输出驱动器电压或 PWM 调制量；带宽通常需要分层考虑，前提是明确驱动接口实际执行的是**电流、力矩、速度还是位置目标**；不能把“发送了力矩指令”直接当作驱动器无滞后精确施加力矩

实验 `labs/python/control/main.py --check` 对二维 2R 的关节 PD/重力补偿做数值验证；比较测量扰动、扭矩饱和影响时应固定参考轨迹与采样时间

### 自测

1. 对常值目标，为什么比例控制可能留下稳态差
2. LQR 与 MPC 在输入硬约束方面有什么区别
3. 若速度来自有限差分编码器，如何避免 D 项把量化噪声变成高频扭矩

**资料：** Åström & Murray, [*Feedback Systems*](https://fbswiki.org/)；[MIT Underactuated Robotics: LQR](https://underactuated.mit.edu/lqr.html)；Spong et al., *Robot Modeling and Control*
