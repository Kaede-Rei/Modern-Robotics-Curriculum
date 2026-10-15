# 刚体空间表示：坐标变换不再靠猜

> **本章问题：** 相机测得的目标点如何换算到机械臂基座？为什么旋转矩阵不能随意相加？欧拉角和四元数各有什么陷阱

**先修：** F02；本文严格使用 `book/notation.md` 的坐标约定：${}^A\mathbf R_B$ 将 $B$ 系向量坐标转换为 $A$ 系坐标

## 核心 1. 先分清点、位移向量和坐标系

设 $B$ 原点相对于 $A$ 系的位置为 ${}^A\mathbf p_B$；同一个空间点 $P$ 在两坐标系中的坐标满足

$$
{}^A\mathbf p_P={}^A\mathbf R_B{}^B\mathbf p_P+{}^A\mathbf p_B
$$

当 $\mathbf p_P$ 表示点坐标时需要平移项；当变换纯自由向量（如同一个方向向量）时没有平移项；若要将视觉的相机系目标点转换到机械臂基座系，必须知道 ${}^{base}\mathbf T_{camera}$，不能只使用旋转矩阵

## 核心 2. 为什么旋转矩阵属于 SO(3)

一个合法的三维刚体旋转 $\mathbf R$ 满足

$$
\mathbf R^T\mathbf R=\mathbf I,\quad\det\mathbf R=+1,
\quad SO(3)=\{\mathbf R\in\mathbb R^{3\times3}\mid\mathbf R^T\mathbf R=\mathbf I,\det\mathbf R=1\}
$$

正交性保证长度、夹角不变，行列式为 $+1$ 排除镜像翻转；旋转矩阵的逆是转置：$\mathbf R^{-1}=\mathbf R^T$；旋转可相乘，但有限旋转通常**不可交换**

例如绕固定 $z$ 轴转 $90^\circ$ 后绕固定 $x$ 轴转 $90^\circ$，与反过来旋转的最终姿态一般不同；写出矩阵乘积时，必须说明是绕固定轴还是随体轴旋转

## 核心 3. 齐次变换一次处理旋转与平移

定义

$$
{}^A\mathbf T_B=\begin{bmatrix}{}^A\mathbf R_B&{}^A\mathbf p_B\\\mathbf0^T&1\end{bmatrix}\in SE(3)
$$

点的齐次坐标映射为 $[{}^A\mathbf p_P;1]={}^A\mathbf T_B[{}^B\mathbf p_P;1]$

变换链必须匹配相邻坐标系

$$
{}^A\mathbf T_C={}^A\mathbf T_B{}^B\mathbf T_C
$$

逆变换由

$$
\mathbf T^{-1}=\begin{bmatrix}\mathbf R^T&-\mathbf R^T\mathbf p\\\mathbf0^T&1\end{bmatrix}
$$

得到；不要通过单纯将平移向量取负来计算逆坐标变换——必须同时把它转回新坐标系

### 3.1 数值算例：相机到基座

相机坐标轴与基座平行，相机原点在基座系的位置是 $(0.20,0.05,0.30)\ \mathrm m$；相机测得目标点 ${}^c\mathbf p_t=(0.10,-0.02,0.50)^T\ \mathrm m$；此时旋转是单位阵，故基座系目标点为

$$
{}^{base}\mathbf p_t=(0.30,0.03,0.80)^T\ \mathrm m
$$

如果相机相对基座还有旋转，必须先旋转再平移；即使只有几度的外参误差，在远距离也能产生明显定位偏差

## 核心 4. 欧拉角不是三维欧氏坐标

欧拉角以三个连续轴旋转参数化姿态；必须写明序列，例如外禀 XYZ 与内禀 ZYX 对应关系；只说 roll–pitch–yaw 容易产生不同约定的歧义

欧拉角可读性好，但存在奇异参数化点（俗称万向节死锁）以及角度跳变；**欧拉角出现奇异，并不等于刚体真实姿态发生物理奇异**

## 进阶 5. 单位四元数与姿态插值

采用标量优先四元数 $\mathbf q_R=(w,x,y,z)$，要求

$$
w^2+x^2+y^2+z^2=1
$$

用 $\mathbf u\in\mathbb R^3$ 表示单位旋转轴、$\theta$ 表示右手旋转角时

$$
\mathbf q_R=\left(\cos\frac\theta2,\ \mathbf u\sin\frac\theta2\right)
$$

$\mathbf q_R$ 和 $-\mathbf q_R$ 代表同一个物理姿态，称作双覆盖；如果做姿态插值，常先通过四元数内积选择短路径一侧，再做球面线性插值（SLERP）；四元数数值积分应注意归一化漂移

**注意：** 单位四元数不同于关节坐标 $\mathbf q$；本教程用下标 $R$ 区分两者

## 进阶 6. 姿态误差与矩阵对数

对期望旋转 $\mathbf R_d$、当前旋转 $\mathbf R$，相对旋转可写为 $\mathbf R_d^T\mathbf R$ 或 $\mathbf R^T\mathbf R_d$，它们具有不同方向；若采用李群对数定义误差，必须同时说明左/右误差、表达坐标系及接近 $\pi$ 旋转时的分支不连续性

微小角位移可以局部写为 $\mathbf R(\delta\boldsymbol\theta)\approx\mathbf I+[\delta\boldsymbol\theta]_\times$，但有限旋转不能通过直接相加角度向量替代复合旋转

## 实践 7. 变换链检查

1. 在纸上画出 base、camera、tool 三个坐标系，并写 ${}^{base}\mathbf T_{camera}$、${}^{base}\mathbf T_{tool}$ 的方向；求 ${}^{tool}\mathbf T_{camera}$
2. 在 Python 中生成一个正交矩阵，验证 $\mathbf R^T\mathbf R\approx\mathbf I$ 与 $\det\mathbf R\approx1$；计算 $\mathbf T\mathbf T^{-1}$ 与单位矩阵的误差
3. 将目标点变换到基座，再经逆变换还原到相机，检查误差来源


## 交互观察

拖动下方参数，比较模型输出与本章公式的联系，交互演示采用简化假设，不替代 Python 数值检查或真实机器人验证

<!-- INTERACTIVE:FRAMES -->

**参考：** Lynch & Park, *Modern Robotics*, Sections 3.2–3.3 and Appendix B；Murray, Li & Sastry, *A Mathematical Introduction to Robotic Manipulation*, Chapter 2
