# 线性代数：读懂机器人矩阵的几何含义

> **本章问题：** 为什么姿态用矩阵、奇异性看秩、逆运动学使用伪逆、动力学要求惯性矩阵正定

**学习结果：** 不只会计算矩阵，还能解释列空间、特征值、SVD、条件数及其机器人含义

## 核心 1. 向量、基与坐标

三维几何向量 $\mathbf v$ 本身与表达它的坐标列向量不是同一个概念；给定两个正交坐标系 $\{A\},\{B\}$，它们描述同一个向量的坐标满足

$$
{}^A\mathbf v={}^A\mathbf R_B{}^B\mathbf v
$$

其中 ${}^A\mathbf R_B\in\mathbb R^{3\times3}$；机器人学大量错误来自忽略上标所指的表达系，而不是单纯的矩阵乘法算错

矩阵 $\mathbf A\in\mathbb R^{m\times n}$ 可理解为将 $n$ 维输入映射为 $m$ 维输出：$\mathbf y=\mathbf A\mathbf x$；第 $i$ 列表示对应单位输入的输出；雅可比也是这样的局部线性映射

## 核心 2. 秩、零空间与几何约束

秩 $\operatorname{rank}(\mathbf A)$ 是矩阵列空间的维度；若 $\mathbf A\mathbf x=\mathbf 0$ 且 $\mathbf x\ne0$，则 $\mathbf x$ 属于零空间，意味着输入变化并没有改变映射的输出

秩–零度定理为

$$
\operatorname{rank}(\mathbf A)+\dim\ker(\mathbf A)=n
$$

对六轴机械臂，如果一个六维任务雅可比的秩下降，部分末端瞬时运动方向可能无法独立实现；如果七轴机械臂的雅可比满行秩，则通常存在至少一个非零关节速度方向，不改变当前瞬时末端任务速度

**不要混淆：** 满行秩并不保证关节速度不触及限位，也不保证有限运动没有碰撞

## 核心 3. 内积、范数与正交投影

向量内积 $\mathbf a^T\mathbf b$ 与夹角、功和能量直接相关；常用 $2$-范数为 $\|\mathbf x\|_2=\sqrt{\mathbf x^T\mathbf x}$

对 $\mathbf A\in\mathbb R^{m\times n}$、向量 $\mathbf b$，最小二乘问题

$$
\min_{\mathbf x}\frac12\|\mathbf A\mathbf x-\mathbf b\|_2^2
$$

在可微情形的一阶驻点条件为

$$
\mathbf A^T(\mathbf A\mathbf x-\mathbf b)=\mathbf0
$$

如果 $\mathbf A$ 列满秩，解唯一且为 $(\mathbf A^T\mathbf A)^{-1}\mathbf A^T\mathbf b$；**但数值实现一般不要显式计算这个逆或直接形成正规方程**：可优先使用 QR 或 SVD，以避免平方条件数引起的精度恶化

## 核心 4. 特征值、正定性与动力学

对实对称矩阵 $\mathbf H=\mathbf H^T$，存在正交特征分解

$$
\mathbf H=\mathbf Q\boldsymbol\Lambda\mathbf Q^T
$$

$\mathbf H\succ0$ 等价于其所有特征值大于零，也等价于任何非零 $\mathbf x$ 都满足 $\mathbf x^T\mathbf H\mathbf x>0$；若无额外约束和退化建模，独立广义坐标下刚体机器人惯性矩阵 $\mathbf M(\mathbf q)$ 应对称正定

二次型 $E=\tfrac12\dot{\mathbf q}^T\mathbf M\dot{\mathbf q}$ 是机器人动能的典型表示，这说明正定性不仅是数学技巧，还与实际正动能有关

## 进阶 5. SVD：把输入与输出方向对齐

任意实矩阵均可奇异值分解

$$
\mathbf A=\mathbf U\boldsymbol\Sigma\mathbf V^T
$$

其中 $\mathbf U,\mathbf V$ 为正交矩阵，$\boldsymbol\Sigma$ 对角部分为非负奇异值 $\sigma_i$；左奇异向量是输出空间方向，右奇异向量是输入空间方向；对雅可比 $\mathbf J$，关系 $\mathbf J\mathbf v_i=\sigma_i\mathbf u_i$ 给出某个关节速度方向产生的末端速度放大倍数

- $\sigma_i\approx0$：对应输出方向难以激发，控制求逆会放大噪声
- $\sigma_i=0$：映射降秩，出现瞬时运动约束
- $\sigma_{\max}/\sigma_{\min}$：在满秩情况下给出 $2$-范数条件数；零奇异值时条件数无穷

伪逆利用非零奇异值的倒数

$$
\mathbf A^\dagger=\mathbf V\boldsymbol\Sigma^\dagger\mathbf U^T
$$

它给出最小二乘问题中的**最小范数解**，但靠近奇异位置时依然可能使求得的输入非常大，因此工程上常引入阻尼

## 进阶 6. 坐标尺度与条件数陷阱

同时包含线速度（m/s）和角速度（rad/s）的六维速度向量，若未经特征长度或任务权重归一化，直接对其雅可比计算一个“通用灵巧度/条件数”，结果会依赖单位尺度；更换米与毫米会改变数值，却不会改变真实机械臂

因此，比较不同机器人时必须说明任务维度、归一化和度量选择；仅报告一个最小奇异值是不够的

## 实践 7. 两个计算实验

1. 分析 $\mathbf A=\begin{bmatrix}1&1\\0&\epsilon\end{bmatrix}$ 随 $\epsilon\to0$ 时的奇异值与条件数，观察最小二乘解对噪声的敏感程度
2. 对平面二连杆的雅可比计算 SVD，比对手臂伸直时的奇异值与 M03 的交互实验结果

**自检：** 为什么 $\mathbf A^T\mathbf A$ 的逆可能不存在？为什么最小二乘解不一定使残差为零？为什么冗余机械臂可以做零空间运动

**参考：** Strang, *Introduction to Linear Algebra*, matrix factorizations；Lynch & Park, *Modern Robotics*, Section 5.4；Golub & Van Loan, *Matrix Computations*
