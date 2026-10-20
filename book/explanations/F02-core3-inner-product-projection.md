# F02-核心3.内积、范数与正交投影-详细讲解

> **先记住一句话：** 内积看一个向量在另一个方向上有多少贡献，范数衡量长度，正交投影把目标放到“能够实现的输出”里最近的位置

上一节讲了“哪些输入能推动输出，哪些输入会互相抵消”；这一节接着问：**如果目标根本不在能实现的方向里，怎样做到尽可能接近**

先用拉箱子理解内积，再用一条斜轨道理解投影，最后回到机械臂的雅可比与最小二乘；下面默认实向量、欧氏内积，比较的坐标分量已经采用一致单位或经过明确的归一化

## 1. 拉箱子：真正做功的是哪个方向的力

箱子沿水平地面向右移动 $2\,\mathrm m$，你用大小为 $10\,\mathrm N$ 的恒力拉它，力与位移的夹角为 $\theta$

$$
W=\mathbf F^T\Delta\mathbf p
=\|\mathbf F\|_2\|\Delta\mathbf p\|_2\cos\theta
=20\cos\theta\ \mathrm J
$$

- 水平向右拉，$\theta=0^\circ$：这股力做功 $20\,\mathrm J$
- 斜向上拉，$\theta=60^\circ$：沿位移方向的力只有 $5\,\mathrm N$，做功 $10\,\mathrm J$
- 竖直向上拉，$\theta=90^\circ$：与水平位移垂直，这股力做功为零
- 向左拉，$\theta=180^\circ$：力与位移相反，做功 $-20\,\mathrm J$，相当于阻碍运动

这里固定了位移，只计算**这一股力的功**，不求箱子的实际加速度；竖直力也可能改变支持力或摩擦力，“功为零”不等于“对系统完全没有影响”

<!-- INTERACTIVE:PROJECTION_WORK -->

对任意两个非零实向量，同一条关系写成

$$
\mathbf a^T\mathbf b=\sum_i a_i b_i
=\|\mathbf a\|_2\|\mathbf b\|_2\cos\theta
$$

**坐标算法**是“对应分量相乘再相加”，**几何含义**是“长度乘长度，再乘方向对齐的程度”；两个非零向量内积为零，就表示它们正交

零向量与任何向量的内积都为零，但它没有确定方向，不能给它指定一个夹角；若力随位置变化，做功应使用沿实际路径的积分 $W=\int\mathbf F^T\,d\mathbf p$

## 2. 范数：一根箭头有多长，两根箭头差多远

向量 $\mathbf x=[3,4]^T$ 就像从原点向右走 3、向上走 4，其欧氏长度由勾股定理得到

$$
\|\mathbf x\|_2=\sqrt{3^2+4^2}=5,
\qquad \|\mathbf x\|_2^2=\mathbf x^T\mathbf x=25
$$

注意 $\|\mathbf x\|_2$ 是**长度**，$\|\mathbf x\|_2^2$ 是**长度的平方**；两者不能在数值或单位上互换

如果机器人想去 $\mathbf b$，实际只能到 $\mathbf p$，则

$$
\mathbf r=\mathbf b-\mathbf p,\qquad
\|\mathbf r\|_2=\text{两点间的距离}
$$

用平方距离做目标不会改变最小值的位置，因为平方在非负数上单调递增；写成 $\tfrac12\|\mathbf r\|_2^2$，只是为了求导时消去因子 2，不是在额外引入一个物理能量模型

**单位提醒：** 若一个分量是米、另一个是弧度，直接平方相加不能当成一个天然的物理距离；需要先约定特征长度、归一化或任务权重

## 3. 斜轨道：投影不是随便找个落点

想象一个滑块只能沿穿过原点、方向为 $\mathbf a=[2,1]^T$ 的直线运动；输入一个标量 $t$，它的位置就是

$$
\mathbf p(t)=t\mathbf a=\begin{bmatrix}2t\\t\end{bmatrix}
$$

目标点在 $\mathbf b=[1,2]^T$；它不在轨道上，所以不论怎样调整 $t$，都到不了目标

**最合理的落点是垂足**：从目标向轨道作垂线，交点记作 $\mathbf p_*$；若沿轨道继续挪动，只会额外增加一段距离

令残差 $\mathbf r_*=\mathbf b-t_*\mathbf a$ 与轨道方向正交

$$
\mathbf a^T(\mathbf b-t_*\mathbf a)=0
\quad\Longrightarrow\quad
\boxed{t_*=\frac{\mathbf a^T\mathbf b}{\mathbf a^T\mathbf a}}
$$

于是投影向量为

$$
\boxed{\operatorname{proj}_{\mathbf a}\mathbf b
=\mathbf a\frac{\mathbf a^T\mathbf b}{\mathbf a^T\mathbf a}}
\qquad(\mathbf a\ne\mathbf0)
$$

不要漏掉分母：$t_*$ 是乘在 $\mathbf a$ 前面的**系数**，不是投影的长度；沿单位方向 $\widehat{\mathbf a}=\mathbf a/\|\mathbf a\|_2$ 的带符号投影长度才是 $\widehat{\mathbf a}^T\mathbf b$，它可以为负

把本例数字代进去

$$
t_*=\frac{2\times1+1\times2}{2^2+1^2}=\frac45=0.8
$$

$$
\mathbf p_*=\begin{bmatrix}1.6\\0.8\end{bmatrix},\qquad
\mathbf r_*=\begin{bmatrix}-0.6\\1.2\end{bmatrix},\qquad
\mathbf a^T\mathbf r_*=2(-0.6)+1.2=0
$$

**输入是 $0.8$，最接近的输出是 $[1.6,0.8]^T$，剩余误差是 $[-0.6,1.2]^T$**；这三个量生活在不同角色里，不要混在一起

## 4. 动手找垂足：为什么最优误差还不等于零

先保持目标 $\mathbf b=(1,2)$，拖动输入 $t$，看橙色残差线段怎样变短、又怎样变长；然后点“移到最优投影”，观察残差与轨道垂直

<!-- INTERACTIVE:PROJECTION_FIT -->

可以依次尝试：把目标改为 $(2,1)$，投影处残差变成零；再改为 $(-2,-1)$，最优输入变成负数；最后改为 $(0,0)$，零输入就能完全实现目标

本例最小二乘目标是

$$
f(t)=\frac12\|t\mathbf a-\mathbf b\|_2^2
=\frac12\big[(2t-1)^2+(t-2)^2\big]
=\frac52(t-0.8)^2+0.9
$$

因此最优 $t=0.8$，最小目标值是 $0.9$，最小距离是 $\sqrt{1.8}\approx1.342$，**目标值不是距离本身**

几何上，任意轨道位置到目标的误差可拆成“沿轨道多走的部分”和“无法消掉的垂直部分”；因为两者正交，勾股定理给出

$$
\|\mathbf b-t\mathbf a\|_2^2
=\|\mathbf r_*\|_2^2+(t-t_*)^2\|\mathbf a\|_2^2
$$

离开垂足时，第二项只能增加，第一项则不会因为沿轨道移动而消失

## 5. 从一条线到一个平面：读懂截图中的列空间

当矩阵有多列时

$$
\mathbf A\mathbf x=x_1\mathbf a_1+\cdots+x_n\mathbf a_n
$$

可以实现的输出是这些列向量的所有线性组合，称为**列空间** $\operatorname{col}(\mathbf A)$；一列非零向量张成一条线，两列独立向量可以张成一个平面

<figure class="concept-diagram web-source-figure" data-asset="projection">
  <img src="../../assets/figures/web-png/projection.png" alt="目标向量投影到两列向量张成的平面，剩余误差垂直于平面" loading="lazy"/>
  <figcaption>先找红色目标向量 b，再找两根绿色列向量张成的淡紫色平面；平面内的投影是能够实现的最近输出，从投影指向目标的线段是剩余残差；图中空间坐标轴 x 与公式中输入向量 x 的角色不同 · 来源：<a href="https://commons.wikimedia.org/wiki/File:Least_squares_projection_geometry.png" target="_blank" rel="noopener noreferrer">Eteri byazrova (GraphMath)</a> · <a href="https://creativecommons.org/licenses/by-sa/4.0" target="_blank" rel="noopener noreferrer">CC BY-SA 4.0</a></figcaption>
</figure>

看图时按这个顺序读：**列向量决定能去哪，目标向量说明想去哪，投影给出最近的能去位置，残差指出还差多少**

对于无约束欧氏最小二乘

$$
\min_{\mathbf x}\frac12\|\mathbf A\mathbf x-\mathbf b\|_2^2
$$

最优残差 $\mathbf r_*=\mathbf b-\mathbf A\mathbf x_*$ 必须与每一列都垂直；否则，就能沿某列方向再挪一点，让距离变小

$$
\mathbf a_i^T\mathbf r_*=0\quad\text{对每一列都成立}
\quad\Longleftrightarrow\quad
\mathbf A^T\mathbf r_*=\mathbf0
$$

这就是正文中 $\mathbf A^T(\mathbf A\mathbf x_*-\mathbf b)=\mathbf0$ 的几何含义；括号里用的是 $-\mathbf r_*$，换一个符号不会改变“等于零”

## 6. 正规方程：把“再挪一点也不能改善”写成求导

设 $f(\mathbf x)=\tfrac12\|\mathbf A\mathbf x-\mathbf b\|_2^2$；给输入一个小改变量 $\delta\mathbf x$，展开得到

$$
f(\mathbf x+\delta\mathbf x)-f(\mathbf x)
=(\mathbf A\mathbf x-\mathbf b)^T\mathbf A\delta\mathbf x
+\frac12\|\mathbf A\delta\mathbf x\|_2^2
$$

第一项是一阶变化，它的系数就是梯度

$$
\nabla f(\mathbf x)=\mathbf A^T(\mathbf A\mathbf x-\mathbf b)
$$

最优时一阶项必须对任意方向都为零，于是得到**正规方程**

$$
\boxed{\mathbf A^T\mathbf A\mathbf x_*=\mathbf A^T\mathbf b}
$$

对于这个无约束线性最小二乘问题，驻点不仅是候选解：一阶项消失后，剩余项 $\tfrac12\|\mathbf A\delta\mathbf x\|_2^2\ge0$，所以它确实是全局最小点；这个结论不能直接套到一般非线性优化上

**一个测量例子：** 三次测同一个长度，读数为 $0.9,1.0,1.1\,\mathrm m$，用同一个数 $x$ 解释三次读数，取

$$
\mathbf A=\begin{bmatrix}1\\1\\1\end{bmatrix},\quad
\mathbf b=\begin{bmatrix}0.9\\1.0\\1.1\end{bmatrix},\quad
3x_*=3\quad\Longrightarrow\quad x_*=1.0\,\mathrm m
$$

最优残差是 $[-0.1,0,0.1]^T\,\mathrm m$，三个分量相加为零；在这里，“与列向量 $[1,1,1]^T$ 正交”就是“剩余误差不再整体偏向正或负”

普通最小二乘对三个读数同等对待；测量精度不同或相关时，应进一步考虑加权模型，参见 F04

## 7. 输出唯一，不代表输入唯一

如果 $\mathbf A$ 列满秩，$\mathbf A^T\mathbf A$ 正定，最优输入唯一；数学表达式为

$$
\mathbf x_*=(\mathbf A^T\mathbf A)^{-1}\mathbf A^T\mathbf b
$$

如果列不独立，最近的**投影输出仍然唯一**，但可以有许多输入实现同一个输出

回到斜轨道，让两个输入都沿 $\mathbf a=[2,1]^T$ 产生贡献

$$
\mathbf A=\begin{bmatrix}2&2\\1&1\end{bmatrix},\qquad
\mathbf A\mathbf x=(x_1+x_2)\mathbf a
$$

对于目标 $(1,2)$，只要 $x_1+x_2=0.8$，就得到同一个最优输出：$(0.8,0)$、$(0.4,0.4)$、$(1.8,-1)$ 都可以；输入之间的差位于零空间

伪逆会从所有最小二乘解中选择欧氏范数最小的一个，本例是 $(0.4,0.4)$；它最小化的是输入平方和，**不自动等于最省电、最安全或最符合关节限位**

这正好与上一节衔接：[回看秩与零空间](#/explain/F02-core2-rank-nullspace)

## 8. 回到机器人：想往右动，却只能瞬间向上动

取长度均为 $1\,\mathrm m$ 的平面二连杆机械臂，两段都沿水平方向向右伸直；此时位置雅可比为

$$
\mathbf J=\begin{bmatrix}0&0\\2&1\end{bmatrix},\qquad
\mathbf v=\mathbf J\dot{\mathbf q}
$$

它的列空间只有竖直速度方向；若期望速度 $\mathbf v_d=[0.1,0.2]^T\,\mathrm{m/s}$，在无约束欧氏最小二乘中，最近的可实现速度为

$$
\mathbf v_*=\begin{bmatrix}0\\0.2\end{bmatrix}\,\mathrm{m/s},\qquad
\mathbf v_d-\mathbf v_*=\begin{bmatrix}0.1\\0\end{bmatrix}\,\mathrm{m/s}
$$

水平方向的剩余误差与竖直列空间正交；一组最小范数关节速度是 $\dot{\mathbf q}_*=[0.08,0.04]^T\,\mathrm{rad/s}$，因为 $2\times0.08+0.04=0.2$

这里解释的是**当前构型下的瞬时速度**；机械臂弯曲后雅可比会变化，不能把这次投影当成整个运动过程的固定轨迹；实际控制还需处理速度限幅、碰撞及任务权重

若加入输入不等式约束，最优点可能停在可行集边界，残差不再必须与整个列空间正交；加权最小二乘的条件也变为 $\mathbf A^T\mathbf W(\mathbf A\mathbf x-\mathbf b)=\mathbf0$，不能沿用原坐标中的欧氏垂直关系

## 9. 为什么公式写着逆矩阵，代码却推荐 QR 或 SVD

正规方程告诉我们最优解满足什么关系，不等于要求计算机先算一个逆矩阵

对列满秩矩阵，$2$-范数条件数满足

$$
\kappa_2(\mathbf A^T\mathbf A)=\kappa_2(\mathbf A)^2
$$

如果两列几乎平行，原问题已经对误差敏感，再形成 $\mathbf A^T\mathbf A$ 会平方条件数；显式求逆通常也没有必要

在 Python 中可以直接求最小二乘解，再自己检查输出和残差

```python
import numpy as np

A = np.array([[2.0], [1.0]])
b = np.array([1.0, 2.0])
x, _, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)
p = A @ x
r = b - p
print(x)            # [0.8]
print(p)            # [1.6 0.8]
print(r)            # [-0.6  1.2]
print(A.T @ r)      # 接近 0，允许浮点舍入误差
print(0.5 * (r @ r))  # 约 0.9
```

QR 可以避免显式构造正规方程；SVD 还能帮助观察秩亏和小奇异值，实际数值秩的判断取决于尺度与阈值；这些方法改善数值求解，不能消除原问题本身的病态性

### 自检：用三句话解释给别人听

1. 为什么竖直拉力对水平位移做功为零，却仍可能影响运动？
2. 为什么 $\mathbf A^T(\mathbf A\mathbf x-\mathbf b)=0$ 不等于 $\mathbf A\mathbf x=\mathbf b$？
3. 为什么最近输出唯一时，输入仍然可能不唯一？

**核对思路：** 内积只保留沿位移的力分量，其他分量仍可能改变约束反力；最小二乘只消掉列空间内可修正的残差，垂直分量可以保留；若存在零空间，就能改变输入而不改变输出

**参考：** Strang, *Introduction to Linear Algebra*, orthogonality and least squares；Golub & Van Loan, *Matrix Computations*, least squares；Lynch & Park, *Modern Robotics*, Chapters 5–6
