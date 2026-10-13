# 全书统一数学符号约定

> 本章说明全书的主要数学符号和坐标系约定，阅读公式时应结合各章给出的具体条件

## 1. 基本数学约定

- 标量：普通斜体 $t,m,k$；列向量：加粗小写 $\mathbf q,\mathbf p$；矩阵：加粗大写 $\mathbf M,\mathbf J$
- $\mathbf q\in\mathbb R^n$ 为关节**广义坐标**；$\dot{\mathbf q}$、$\ddot{\mathbf q}$ 为其时间导数；$n$ 为广义坐标维数
- $\mathbf 0$、$\mathbf I$ 表示形状适当的零量和单位阵；转置、逆、伪逆分别为 $^T$、$^{-1}$、$^\dagger$
- 连续时间 $t\in\mathbb R$；离散采样时刻 $k\in\mathbb Z$，采样周期 $T_s>0$
- 向量和矩阵的维度、物理单位首次出现时必须说明；公式成立条件不得省略

## 2. 坐标系、旋转与刚体位姿

- 坐标系为右手正交系，约定 $\{A\}$、$\{B\}$，基座系可记作 $\{s\}$，末端工具系 $\{b\}$
- ${}^{A}\mathbf R_B\in SO(3)$ 将**在 $B$ 中表达**的向量坐标变为**在 $A$ 中表达**：${}^{A}\mathbf v={}^{A}\mathbf R_B{}^{B}\mathbf v$
- ${}^{A}\mathbf p_B$ 表示 $B$ 原点相对 $A$ 原点、**用 $A$ 坐标表示**的平移向量
- ${}^{A}\mathbf T_B = \begin{bmatrix}{}^A\mathbf R_B & {}^A\mathbf p_B\\\mathbf 0^T & 1\end{bmatrix}\in SE(3)$，映射 $B$ 中点的齐次坐标到 $A$
- 坐标连乘：${}^{A}\mathbf T_C={}^A\mathbf T_B{}^B\mathbf T_C$；反向变换：${}^{B}\mathbf T_A=({}^A\mathbf T_B)^{-1}$
- 齐次变换按**被动坐标变换**定义；如讨论主动刚体动作，必须显式说明并区分
- 旋转矩阵、欧拉角和单位四元数均用于姿态表示；四元数采用标量优先 $(w,x,y,z)$ 并明确 $q$ 与 $-q$ 表示同一姿态

## 3. 旋量、雅可比与力旋量

- 三维向量叉乘的反对称矩阵：$[\boldsymbol\omega]_{\times}\mathbf v=\boldsymbol\omega\times\mathbf v$
- 6 维运动旋量（twist）固定采用**角分量优先**：$\mathbf V=[\boldsymbol\omega^T,\mathbf v^T]^T$；这里 $\mathbf v$ 为 twist 的线分量，不要未经说明直接等同于某任意点的线速度
- 6 维力旋量（wrench）与上述约定配对：$\mathbf F=[\mathbf m^T,\mathbf f^T]^T$；瞬时功率 $P=\mathbf F^T\mathbf V$
- 空间旋量 $\mathbf V_s=(\dot{\mathbf T}\mathbf T^{-1})^\vee$；本体旋量 $\mathbf V_b=(\mathbf T^{-1}\dot{\mathbf T})^\vee$，其中 $\mathbf T={}^s\mathbf T_b$
- $\mathbf V_s=\operatorname{Ad}_{\mathbf T}\mathbf V_b$；${}^{s}\mathbf J_s(\mathbf q)$、${}^{b}\mathbf J_b(\mathbf q)$ 满足相应的 $\mathbf V_s=\mathbf J_s\dot{\mathbf q}$ 与 $\mathbf V_b=\mathbf J_b\dot{\mathbf q}$（简写 $\mathbf J_s$、$\mathbf J_b$ 时默认表达系已明确）
- 同一物理功率在两个 twist 表达中不变：若 $\mathbf V_s=\operatorname{Ad}_{\mathbf T}\mathbf V_b$，则 $\mathbf F_b=\operatorname{Ad}_{\mathbf T}^T\mathbf F_s$
- 对作用于机器人末端的 wrench，统一写 $\boldsymbol\tau_{\mathrm{ext}}=\mathbf J(\mathbf q)^T\mathbf F_{\mathrm{ext}}$，二者**必须使用匹配的表达系、参考点及排列**
- 平面二连杆末端的 $\dot{\mathbf p}=\mathbf J_p(\mathbf q)\dot{\mathbf q}$ 属于**位置雅可比**关系，不与 6D 刚体空间雅可比混称

## 4. 动力学、摩擦与外力正方向

- 广义关节力矩/力为 $\boldsymbol\tau\in\mathbb R^n$，单位按关节类型为 N·m 或 N
- 基本机器人动力学：$\mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\mathbf g(\mathbf q)+\boldsymbol\tau_f = \boldsymbol\tau_{\mathrm{cmd}}+\mathbf J^T\mathbf F_{\mathrm{ext}}$
- $\mathbf F_{\mathrm{ext}}$ 定义为**环境施加给机器人**的外力旋量；正号在等式右侧；若表达“机器人施加给环境的力”，其方向在互为作用反作用的同一接触点处相反
- $\mathbf M$ 是惯性矩阵、$\mathbf C\dot{\mathbf q}$ 是速度相关广义力项、$\mathbf g$ 是所选重力势能约定下的重力广义力、$\boldsymbol\tau_f$ 是摩擦广义力；$\mathbf C$ 矩阵本身**不唯一**，不要把它的具体元素当成独立物理不变量
- 任意使用质量、质心、惯性张量时说明是连杆坐标系还是质心系、以及惯性张量关于哪个点定义

## 5. 控制误差、稳定性与估计

- 默认关节位置跟踪误差：$\mathbf e=\mathbf q_d-\mathbf q$；由此 $\dot{\mathbf e}=\dot{\mathbf q}_d-\dot{\mathbf q}$
- 若引用采用 $\mathbf q-\mathbf q_d$ 的教材或论文，必须同时对应修改控制律与 Lyapunov 推导，不得单独替换误差符号
- 姿态误差必须附完整矩阵定义，并注明是左不变/右不变、在哪个坐标系表达；禁止只写 $\mathbf e_R$ 而不定义
- $\mathbf x$ 为状态、$\hat{\mathbf x}$ 为估计状态、$\mathbf y$ 为观测、$\mathbf u$ 为控制输入，$\mathbf P$ 为状态估计误差协方差（首次使用应明确条件）
- 稳定性须区分 Lyapunov 稳定、渐近稳定、指数稳定、局部/全局、名义/鲁棒；$\dot V\le0$ 一般不自动推出渐近稳定
- 控制律给出时须标明连续/离散、理想/实际模型、执行器饱和、测量噪声和延迟适用条件

## 6. 符号使用说明

- 本教材中的符号采用全书统一约定，具体推导条件以所在章节为准
- 有不同合理表示方式时，优先说明换算关系（如 twist $[\boldsymbol\omega;\mathbf v]$ 与 $[\mathbf v;\boldsymbol\omega]$）
- 章节正文在首次出现新符号时仍需就地定义，不能要求读者只靠本文件理解全部公式
- 不同参考资料可能采用不同记号，应先核对定义再进行换算
