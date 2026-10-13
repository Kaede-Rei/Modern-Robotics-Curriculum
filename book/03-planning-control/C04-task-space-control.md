# 任务空间控制：为什么关节到位不等于末端按指定方向运动

> **问题：** 六轴臂的末端被要求沿工具坐标系 $z$ 方向缓慢移动，怎样把这个任务变成关节速度或力矩？七轴臂多出来的自由度该如何利用

**先修：** M03、M04、M06、C03；**路线：** 雅可比速度控制 → 冗余零空间 → 操作空间动力学

## 1. 先确定你控制的“任务”是什么

任务空间可以是末端点位置 $\mathbf p\in\mathbb R^3$、完整 6D 位姿，或更复杂的抓取/视觉误差；写出 $\mathbf x=f(\mathbf q)$ 及其导数 $\dot{\mathbf x}=\mathbf J_x(\mathbf q)\dot{\mathbf q}$ 前，必须说明 $\mathbf x$ 的坐标系、单位和姿态误差定义；**不要把角度三个数直接当成全局可微的欧氏空间位置变量**

## 2. 核心：速度层的任务空间控制

对只包含平移的局部任务，给出速度命令

$$
\dot{\mathbf x}_{\mathrm{cmd}}=\dot{\mathbf x}_d+\mathbf K_x(\mathbf x_d-\mathbf x)
$$

若 $\mathbf J_x$ 在目标任务方向满行秩，并且关节可被理想速度控制，可取

$$
\dot{\mathbf q}=\mathbf J_x^\dagger\dot{\mathbf x}_{\mathrm{cmd}}
$$

当 $\mathbf J_x\mathbf J_x^\dagger=\mathbf I$ 时有 $\dot{\mathbf e}_x=-\mathbf K_x\mathbf e_x$；如果目标不可达、雅可比降秩、关节限位或速度饱和，则不能沿用这个理想误差结论

### 2.1 多余自由度去哪里

若 $n>m$ 且任务雅可比满行秩，可以加入任务的一阶零空间动作

$$
\dot{\mathbf q}=\mathbf J_x^\dagger\dot{\mathbf x}_{\mathrm{cmd}}+(\mathbf I-\mathbf J_x^\dagger\mathbf J_x)\mathbf z
$$

$\mathbf J_x(\mathbf I-\mathbf J_x^\dagger\mathbf J_x)=0$，因此第二项在瞬时一阶意义下不改变主任务速度；$\mathbf z$ 可用于关节限位避让、奇异性回避等，但不能保证有限步的轨迹避障，也不能越过速度/力矩约束

### 2.2 接近奇异时为什么不盲目求伪逆

对 SVD $\mathbf J=\mathbf U\mathbf\Sigma\mathbf V^T$，很小的奇异值会被伪逆放大；阻尼最小二乘可选

$$
\mathbf J_\lambda^\#=\mathbf J^T(\mathbf J\mathbf J^T+\lambda^2\mathbf I)^{-1}
$$

它减少噪声和目标速度放大，但 $\mathbf J\mathbf J_\lambda^\#\ne\mathbf I$，因此会引入任务跟踪偏差；**阻尼不是凭空补出不存在的控制方向**

## 3. 进阶：从速度到任务空间力矩

只考虑满行秩、固定基、无其他硬约束、$\mathbf M$ 正定的机械臂，机器人动力学

$$
\mathbf M\ddot{\mathbf q}+\mathbf h+\mathbf g=\boldsymbol\tau,
\quad \ddot{\mathbf x}=\mathbf J\ddot{\mathbf q}+\dot{\mathbf J}\dot{\mathbf q}
$$

定义动态一致性操作空间惯量

$$
\boldsymbol\Lambda=(\mathbf J\mathbf M^{-1}\mathbf J^T)^{-1}
$$

矩阵只有在任务雅可比满行秩时可逆；若选择 $\boldsymbol\tau=\mathbf J^T\mathbf F$，则

$$
\boldsymbol\Lambda\ddot{\mathbf x}=\mathbf F-\boldsymbol\eta,
\quad \boldsymbol\eta=\boldsymbol\Lambda\mathbf J\mathbf M^{-1}(\mathbf h+\mathbf g)-\boldsymbol\Lambda\dot{\mathbf J}\dot{\mathbf q}
$$

因此可将期望任务加速度乘以 $\boldsymbol\Lambda$ 并补偿 $\boldsymbol\eta$ 来设计操作空间控制；需要关心**力矩是否可实现**、与关节空间零任务的冲突，以及环境接触约束

### 3.1 动态一致零空间不能随意用欧氏伪逆替代

一种常见的质量加权广义逆为 $\bar{\mathbf J}=\mathbf M^{-1}\mathbf J^T\boldsymbol\Lambda$；速度层的 $\mathbf I-\mathbf J^\dagger\mathbf J$ 与动力学层的动态一致投影分别优化不同度量，不能一概称作“同一个零空间投影”

## 4. 姿态控制与坐标系约定

完整任务位姿 $\mathbf T_d,\mathbf T$ 的误差需要在 $SE(3)$ 上定义，例如本体误差 $\operatorname{Log}(\mathbf T^{-1}\mathbf T_d)^\vee$；姿态旋转误差与其时间导数之间通常有左/右 Jacobian 因子，不能在有限姿态误差下无条件把“姿态误差导数=期望角速度−实际角速度”使用

## 5. 实践

可先用 `labs/python/arm2r/main.py --check` 校验二维末端 $\dot{\mathbf p}=\mathbf J_p\dot{\mathbf q}$、奇异值和速度分量；完整 6D 操作空间控制属于进阶扩展，这一二维验证不代表通用六维控制器的完整实现

### 自测

1. 证明 $\mathbf J(\mathbf I-\mathbf J^\dagger\mathbf J)=0$ 在 Moore–Penrose 伪逆下成立
2. 什么条件下 $\boldsymbol\Lambda$ 有定义？接近奇异时会发生什么
3. 为什么不能将空间 twist 的线分量直接称为末端点平移速度

**资料：** Khatib, [*A Unified Approach for Motion and Force Control of Robot Manipulators*](https://doi.org/10.1109/JRA.1987.1087068), 1987；[Modern Robotics Ch. 8.6](https://modernrobotics.northwestern.edu/nu-gm-book-resource/)；Nakamura, *Advanced Robotics*, 1991
