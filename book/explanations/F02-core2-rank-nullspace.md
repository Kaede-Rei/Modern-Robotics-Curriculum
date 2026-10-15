# F02-核心2.秩、零空间与几何约束-详细讲解

> **先记住一句话：** 秩说明输入能独立推动多少个输出方向，零空间说明输入还能怎样变化却不改变输出

为什么六轴机械臂有时会突然进入奇异位形，而七轴机械臂通常可以在末端保持不动时调整手肘？这两个现象背后，其实是同一个线性代数定理

## 1. 从两个电机开始：动了却等于没动

想象一个只有前后运动能力的小车，左右两个电机都能向同一方向提供速度贡献，先把它简化成

$$
v=v_1+v_2
$$

电机 1 输出 $+2$、电机 2 输出 $-2$，那么小车速度就是 $0$，不是电机都停了，而是**两个输入的作用刚好抵消**

这里的加法只是一种教学用线性模型，不是在模拟真实差速驱动小车的完整运动学

<!-- INTERACTIVE:RANK_MOTORS -->

用矩阵表示刚才的例子

$$
\underbrace{\begin{bmatrix}1&1\end{bmatrix}}_{\mathbf A}
\underbrace{\begin{bmatrix}1\\-1\end{bmatrix}}_{\mathbf x}
=\underbrace{0}_{\mathbf A\mathbf x}
$$

这里 $\mathbf x\ne\mathbf0$，但 $\mathbf A\mathbf x=\mathbf0$，因此 $\mathbf x$ 落在矩阵 $\mathbf A$ 的**零空间**里

严格来说，零空间是所有满足 $\mathbf A\mathbf x=\mathbf0$ 的输入组成的集合，包括零向量本身；我们特别关注非零向量，是为了找到可以相互抵消的输入方向

## 2. 秩到底在数什么

上面的矩阵 $\mathbf A=[1\quad1]$ 有两列，对应两个输入；但是两列都指向同一个一维输出方向，因此只有 **1 个独立输出方向**

$$
\operatorname{rank}(\mathbf A)=1
$$

输入有两维 $v_1,v_2$，但能够独立影响输出的只有一维，其余一维可以用于抵消

$$
\underbrace{\operatorname{rank}(\mathbf A)}_{\text{独立输出方向数}}
+\underbrace{\dim\ker(\mathbf A)}_{\text{可抵消输入方向数}}
=\underbrace{n}_{\text{输入维数}}
$$

对于这个例子，就是 $1+1=2$，这正是**秩–零度定理**

可以把它理解为对输入自由度的一次分账：一部分用来真正改变输出，剩余部分形成零空间，但这不是在说所有输入方向只能做一种用途，任意输入都可以分解为这两类方向的分量

## 3. 机械臂雅可比：把输入换成关节速度

机械臂控制中的局部线性映射是

$$
\underbrace{\mathbf V}_{\text{末端瞬时速度}}
=\underbrace{\mathbf J(\mathbf q)}_{\text{雅可比}}
\underbrace{\dot{\mathbf q}}_{\text{关节速度}}
$$

若任务是完整的空间末端位姿，$\mathbf V$ 有六个分量，即三个线速度和三个角速度，六轴机械臂的 $\mathbf J$ 为 $6\times6$，七轴机械臂的 $\mathbf J$ 为 $6\times7$

<div class="rank-dimension" role="img" aria-label="秩与零空间维数对照，六轴正常为六个有效方向，六轴奇异为五个有效方向一个零空间方向，七轴满行秩为六个有效方向一个零空间方向">
  <div class="rank-dimension-key"><span><i class="rank-key-task"></i>影响末端输出的独立方向</span><span><i class="rank-key-null"></i>零空间方向</span></div>
  <div class="rank-dimension-row"><strong>六轴 · 正常</strong><div class="rank-dimension-bar"><span class="rank-dimension-task" style="flex:6">6</span></div><small>6 = 6 + 0</small></div>
  <div class="rank-dimension-row"><strong>六轴 · 奇异</strong><div class="rank-dimension-bar"><span class="rank-dimension-task" style="flex:5">5</span><span class="rank-dimension-null" style="flex:1">1</span></div><small>6 = 5 + 1</small></div>
  <div class="rank-dimension-row"><strong>七轴 · 满行秩</strong><div class="rank-dimension-bar"><span class="rank-dimension-task" style="flex:6">6</span><span class="rank-dimension-null" style="flex:1">1</span></div><small>7 = 6 + 1</small></div>
</div>

| 情况 | 雅可比尺寸 | 秩 | 零空间维数 | 直观理解 |
|:--|:--:|:--:|:--:|:--|
| 六轴正常 | $6\times6$ | $6$ | $0$ | 六维任务刚好够用 |
| 六轴奇异，例如秩降至 5 | $6\times6$ | $5$ | $1$ | 末端丢失一个独立的瞬时速度方向 |
| 七轴满行秩 | $6\times7$ | $6$ | $1$ | 末端六维速度可独立控制，仍有内部调整方向 |
| 七轴奇异，例如秩降至 5 | $6\times7$ | $5$ | $2$ | 零空间变大，却丢失一个末端速度方向 |

**尤其容易误会的一点：** 六轴奇异时的零空间维度虽然从 0 变成 1，但这不等于它更灵活，而是两个关节速度方向的效果发生重叠，以至于某个末端速度方向无法独立产生

## 4. 动手看一次奇异：二连杆伸直

想象一只只能在桌面上运动的二关节机械臂，末端任务只有平面 $x,y$ 两维，它的雅可比是 $2\times2$

$$
\mathbf J=\begin{bmatrix}
-l_1\sin q_1-l_2\sin(q_1+q_2)&-l_2\sin(q_1+q_2)\\
l_1\cos q_1+l_2\cos(q_1+q_2)&l_2\cos(q_1+q_2)
\end{bmatrix}
$$

行列式恰好是 $\det\mathbf J=l_1l_2\sin q_2$，当 $q_2=0$、两段手臂完全伸直时，行列式为零，秩从 2 降为 1

<!-- INTERACTIVE:RANK_SINGULARITY -->

观察两个关节分别能给末端贡献的瞬时速度箭头，在伸直时它们指向同一条直线，所以无法独立合成两个平面速度方向

这只是六轴腕部轴线重合等奇异现象的**二维类比**，并非所有六轴奇异都表现为整条手臂伸直

## 5. 七轴为什么能调整手肘：先看三关节类比

考虑一个平面三关节机械臂，只让它完成**末端 $x,y$ 位置任务**，它有三个关节输入、两个任务输出

一般的非奇异姿态下 $\operatorname{rank}(\mathbf J)=2$，所以 $\dim\ker(\mathbf J)=3-2=1$

这个多出的一维意味着我们可以沿着某个关节运动方向调整手臂形状，同时让末端位置的瞬时变化为零

<!-- INTERACTIVE:RANK_REDUNDANCY -->

这里为了便于观察，演示使用逆运动学直接求出多种**末端位置精确相同**的平面三连杆构型，注意这些构型的**末端朝向可以不同**，因为本例只约束二维位置

七轴机械臂的对应情况更严格：它的任务通常是六维末端速度；当 $6\times7$ 雅可比满行秩时，存在一维非零的关节速度零空间，**瞬时末端六维任务速度不变**

要让真实机械臂在有限时间内始终保持完整末端位姿，还需要连续求解运动轨迹，并遵守关节、碰撞与动力学约束，不能简单地把一次求出的零空间速度一直执行下去

## 6. 最后要分清的三个层次

- **秩：** 映射能够独立生成多少个输出方向
- **零空间：** 哪些非零输入组合可以在映射中相互抵消
- **冗余：** 满足整个任务时，还能利用零空间调整内部构型的能力

**不要把数学可行误认为工程安全：** 雅可比满行秩，不代表关节不触及限位、运动不会碰撞，也不保证给定速度在电机约束内可以执行；雅可比描述的是局部瞬时映射，不直接保证有限距离的连续运动可行

### 试着回答

1. 两个电机模型中，$v_1=3,v_2=-3$ 时，小车为什么不动
2. 二连杆伸直时，零空间变大，为什么反而称为奇异
3. 三连杆实验里末端 $x,y$ 不动，是否意味着末端朝向也保持不变
4. 七轴机械臂的 $6\times7$ 雅可比从秩 6 降到秩 5 时，末端能力和零空间维数各发生什么变化

<details class="rank-answer"><summary>展开查看答案提示</summary>

1. 两个非零输入刚好相互抵消，合成速度为 0
2. 奇异时雅可比只剩一个独立输出方向，虽然出现非零零空间向量，但末端不能再独立产生整个平面的速度
3. 不一定，本例只保持末端二维位置，末端朝向可以改变
4. 秩由 6 降到 5，丢失一个独立末端速度方向；零空间维数从 1 变成 2

</details>

读完这里，再回到 [F02 原章节的核心 3：正交投影](#/F02/核心-3.-内积范数与正交投影)，后面就能进一步理解雅可比伪逆与零空间投影
