# 现代机器人学 · Modern Robotics Curriculum

基于公开机器人学教材、课程和论文整理的个人学习项目，包含六篇 49 章中文讲义、离线 HTML 阅读器与配套 Python 教学实验

本项目采用 **Vibe Coding** 方式编写和组织内容，相关讲义、代码和可视化由 AI 辅助生成，参考资料不代表原作者参与或认可本项目

**项目仅供个人学习与技术实验使用，不属于官方教材，也不保证所有数学推导及工程实现已通过独立学术审查**

## 项目特点

- 六篇 49 章，覆盖数学与机构、建模、规划与控制、感知与估计、机器人学习和系统工程
- 按 **核心 → 进阶 → 实践** 组织章节，结合物理直觉、公式推导、几何图示、例题和练习
- `dist/index.html` 包含正文、全部配图、数学公式、样式和交互，复制单个文件即可断网阅读
- 提供离线章节导航、全文搜索、数学公式、响应式阅读、12 项参数交互实验与二维机械臂运动示意
- 提供独立的「部分教学详细讲解」入口，支持从正文知识点进入补充讲解及返回原文
- `labs/python/` 提供可复现实验，包括运动学、动力学、控制、规划、感知、学习和系统工程示例

## 交互学习

教材在坐标变换、正运动学、重力力矩、PD 控制、柔顺控制、轨迹生成、卡尔曼测量更新、相机投影、占据概率、模仿学习、强化学习和实时调度等章节设有可调参数的浏览器实验

所有交互均在 `dist/index.html` 内离线运行，不需要外部库或服务器，所得结果只用于解释理想模型，与配套 Python 数值检查和真机试验应明确区分

## 快速开始

**阅读教材**

打开 [`dist/index.html`](dist/index.html)，无需安装 Python 或启动服务，文字、全部图片、公式和交互实验均离线可用

仓库已经包含全部图片，无需首次联网下载；图注的外部来源链接只用于查阅原出处，阅读教材不会请求这些网站

**重新构建教材**

需要 Python、Pandoc 和 `requirements-build.txt` 中的依赖

```bash
python -m pip install -r requirements-build.txt
# 检查随仓库附带的 PNG，不访问网络
python scripts/fetch_web_png.py --check
python scripts/build_book.py
python scripts/check_book.py
```

章节正文维护在 `book/`，请勿直接修改构建产物 `dist/index.html`

**配图与来源**

- 49 章配图已放在对应知识点旁，图注说明图中对象与公式的关系及适用边界
- 41 项现成网络 PNG 随仓库保存，包含真实机器人照片、官方展示图、机构和算法示意图；新增最小二乘投影、二维高斯分布与 SVD 数据图
- F02 详细页保留 UR16e 实物照片、OpenArm 官方展示图，以及原有秩、奇异性与冗余交互
- [图片来源清单](assets/figures/SOURCES.md) 记录作者、许可和教学位置；[JSON 清单](assets/figures/web-png-sources.json) 记录原始 URL、实际下载 URL、尺寸与 SHA-256
- 构建器禁止在线图片地址；缺图或 PNG 损坏时停止，不会退回网络加载；检查脚本检查所有章节、补充讲解及图片完整性
- 如需恢复缺失文件，可联网运行 `python scripts/fetch_web_png.py`；已有文件先校验，下载通过解码和哈希验证后才原子写入，遇到源站限流即停止，稍后可续跑
- 源站重编码可能使下载内容与归档哈希不符，此时需核对版本与来源再更新清单，不应忽略校验错误

**验证离线阅读**

```bash
python -m unittest discover -s tests -v
python scripts/check_book.py
# 可选：安装浏览器验收工具后，在断网模式访问全部页面并操作交互控件
python -m pip install playwright
python -m playwright install chromium
python scripts/check_reader_browser.py
```

浏览器验收将 HTML 单独复制到临时目录，禁止网络访问，检查全部配图解码、12 项实验滑块、二维机械臂和 F02 三项演示，以及窄屏布局

**运行教学实验**

```bash
python -m pip install -r requirements-lab.txt
python labs/python/arm2r/main.py --check
python labs/python/kinematics/main.py --check
python labs/python/dynamics/main.py --check
python labs/python/control/main.py --check
python labs/python/planning/main.py --check
python labs/python/perception/main.py --check
python labs/python/robot_learning/main.py --check
python labs/python/systems/main.py --check
```

实验使用简化模型或合成数据，数值检查不等于真实机械臂性能验证

## 项目结构

```text
Modern-Robotics-Curriculum/
├── README.md
├── book/                  # 正式教材、课程目录与符号约定
├── reader/                # HTML 模板、样式与阅读交互
├── dist/index.html        # 可独立阅读的 HTML 教材
├── assets/figures/        # 章节插图
├── labs/python/           # Python 教学实验
├── references/            # 参考文献数据
├── scripts/               # 教材构建与检查
├── requirements-build.txt
└── requirements-lab.txt
```

## 参考资料

本项目学习和整理过程中参考了以下公开课程、教材及论文，具体章节还包含对应的原始文献链接

- [Modern Robotics — Northwestern University](https://modernrobotics.northwestern.edu/)
- [Underactuated Robotics — MIT](https://underactuated.mit.edu/)
- [Robotic Manipulation — MIT](https://manipulation.mit.edu/)
- [Stanford CS223A — Introduction to Robotics](https://cs.stanford.edu/groups/manips/teaching/cs223a/)
- [Robot Dynamics — ETH Zurich](https://rsl.ethz.ch/education-students/lectures/robotdynamics.html)
- [ROS 2 Documentation](https://docs.ros.org/)

完整的参考资料条目见 `references/references.bib`

## 声明

- 项目为个人自用的学习笔记与实验集合，以公开资料为参考并通过 Vibe Coding 方式构建
- 讲义和演示不代表所参考高校、作者、开源软件社区的官方观点或背书
- AI 辅助生成的推导与代码可能存在错误，涉及真实机器人时必须独立检查模型、限位、实时性与安全措施
- 第三方教材、论文、软件和品牌的版权及相应许可仍归原权利人所有
- 本仓库暂未指定开源许可证，公开代码不等于自动授予修改、分发或商业使用许可
