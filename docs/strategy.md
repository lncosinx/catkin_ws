# 策略子系统技术文档（strategy_node）

`strategy_node` 负责**装箱决策**：把「在 $14\times10$ 网格里尽可能多得分」建模为一个
**带计分的精确覆盖问题**，用手写 Dancing Links（DLX）搜索求解，输出逐块抓放方案。本文
从数据处理角度说明它经过哪些步骤、每步的方法与**数学公式**。

参考代码：
- 节点逻辑：[strategy_node.cpp](../src/lucky/src/strategy_node.cpp)
- 求解引擎（ROS-free）：[include/lucky/tetris_solver.hpp](../src/lucky/include/lucky/tetris_solver.hpp)
- 仅策略测试：[test_strategy.launch](../src/lucky/launch/test_strategy.launch)

---

## 0. 数据流总览

```
/vision/board_state (库存 + 散料像素)
   │
   ▼ [1] 稳定门控        数量门槛 + 库存连续 K 帧不变
   ▼ [2] 方案生成        PlanConfig：目标块数/起始行/禁用格/理论满分
   ▼ [3] 子库存枚举      Σn_i=target 且 n_T 偶 的所有组合
   ▼ [4] DLX 精确覆盖    Algorithm X + 计分，收集同分最优多候选
   ▼ [5] 补放 + 拓扑排序 贪心塞空位 + Kahn 保证先底后顶
   ▼ 发布 /tetris_plan(首选) + /tetris_plan_candidates(同分多套)
```

输出**不直接给控制器**，而是给 `path_planner_node`：同分候选由它按机械臂运动代价二次
择优（见 [path_plan.md](path_plan.md)）。

---

## 1. 步骤 1 — 稳定门控

避免视觉只识别到一部分就提前出方案。设当前总块数 $N=\sum_{i=0}^{6}\text{inv}[i]$：

1. **数量门槛**（普通模式）：$N<\texttt{min\_usable\_total\_blocks}$（默认 34）不规划。
2. **库存稳定**：库存向量 $\mathbf{inv}$ 需连续 $K$ 帧完全不变；所需帧数
   $$K=\begin{cases}\texttt{inventory\_stable\_required\_frames}=5 & N=\texttt{expected\_total\_blocks}=35\\ \texttt{min\_usable\_stable\_required\_frames}=3 & N=34\ (\text{差 1 兜底})\end{cases}$$
3. 一次任务只规划一次（`task_completed` 锁死），`is_robot_busy` 期间不规划。

> 进阶模式的门控是**峰值保持**（等识别数不再上升）+ 最优稳定帧超时兜底，见节点日志。

---

## 2. 步骤 2 — 方案生成（PlanConfig）

装箱不是一次搜整盘，而是构造一组「用满底部若干行」的**子目标方案**，按理论满分从高到
低逐一尝试，一旦某方案求出解就锁定。每个方案 `PlanConfig` 含：目标块数 `target`、起始行
`start_r`（只在 $[\texttt{start\_r},13]$ 行放置）、禁用格集合、理论满分、搜索节点上限、重启次数。

- **34 块特化方案**（$N\ge34$ 时）：$\text{target}=34,\ \text{start\_r}=0$，禁用第 0 行第
  6\~9 列 4 格，理论满分 $260$。
- **通用方案**：设可用满行数上界
  $$r_\max=\min\!\Big(12,\ \Big\lfloor\tfrac{4N}{10}\Big\rfloor\Big)\ \text{（向下取偶）}$$
  从 $r=r_\max$ 每次减 2 直到 2，构造方案：目标块数 $\text{target}=\frac{10r}{4}$（填满 $r$
  行需 $10r$ 格、每块 4 格），起始行 $\text{start\_r}=14-r$（占底部 $r$ 行），理论满分
  $\text{max\_score}=20r$（每满行最高 20 分，见 §4.3）。

---

## 3. 步骤 3 — 子库存枚举

对方案目标块数 $\text{target}$，枚举所有「从 7 种库存各取 $n_i$ 块」的组合，满足

$$\sum_{i=0}^{6} n_i=\text{target},\qquad 0\le n_i\le\text{inv}[i],\qquad n_2\equiv0\ (\mathrm{mod}\ 2)$$

T 形（id=2）取偶数是放置对称性约束。递归深搜生成；每方案最多评估前 20 个子库存。

---

## 4. 步骤 4 — DLX 精确覆盖求解

### 4.1 精确覆盖建模
**精确覆盖问题**：给定全集 $U$ 与子集族 $\mathcal S=\{S_1,\dots\}$，求子族
$\mathcal S^*\subseteq\mathcal S$ 使得每个元素被**恰好覆盖一次**：

$$\bigcup_{S\in\mathcal S^*}S=U,\qquad S_i\cap S_j=\varnothing\ (\forall S_i,S_j\in\mathcal S^*,\ i\ne j)$$

本问题的映射：
- **列（$U$ 的元素）**：$\text{target}$ 个「棋子列」（每个棋子必须用一次）+ 每个可用棋盘
  格一列（$\text{start\_r}\sim13$ 行、未禁用、未占用）。
- **行（子集 $S$）**：每个棋子实例的每个**唯一旋转** × 每个合法落位 = 一行，覆盖「自身
  棋子列 + 所占格子列」。另给每棋子一行「只占自身列」的空行 → 表示**可以不放**。

**唯一旋转**：base 形状按 90° 旋转 4 次去重。90° 逆时针旋转把 $(x,y)$ 映为

$$r{=}1:(x,y)\!\to\!(-y,x),\quad r{=}2:(-x,-y),\quad r{=}3:(y,-x)$$

再平移归一化（$\min$ 移到 0）后去重。故田(O)只 1 种、一字(I)/Z/S 各 2 种、T/L 各 4 种。

**同形多实例去对称**：`piece_prev` 链强制同种形状的多实例按顺序使用（第 $j$ 个未用前不
能用第 $j{+}1$ 个），消掉等价排列，防止搜索爆炸。

### 4.2 Algorithm X + Dancing Links
DLX 是 Knuth 的 Algorithm X 在**双向十字循环链表**上的高效实现。核心：

- **列选择启发**（最小剩余值 MRV）：选当前剩余列里 size 最小者，$c^*=\arg\min_c |c|$，
  最大化剪枝、减少分支因子。
- **覆盖/取消覆盖**（cover/uncover）：选中一行后，$O(1)$ 指针拆除/恢复相关行列——这正是
  "Dancing Links" 名字来源，回溯无需重建。
- **带重启随机化**：每方案做 `max_restarts` 次，每次 `shuffle` 行顺序后重建 DLX 搜索
  （`search` 受 `max_search_nodes` 与最大解数限制），对所有解逐一计分。

### 4.3 计分规则
把解落盘得占用矩阵后，逐行计分：

$$\text{score}=\sum_{\text{row }y}\Big(10\cdot\mathbf 1[\text{row }y\text{ 满}]+10\cdot\mathbf 1[\text{row }y\text{ 满}\ \wedge\ \text{该行颜色种类}\ge4]\Big)$$

外加**规则④**：整盘出现的形状种类 $<3$ → 该解 $\text{score}=0$。达方案理论满分即提前返回。

> 「满行且 ≥4 色 +10」在普通模式恒定生效；进阶模式由 `seq_reward_four_colors` 控制（默认关）。

### 4.4 收集同分多候选
`solveForMaskMulti` 不止取一个最优解：把达到**最高分**的解**去重**（按排序后行索引集合）后
收集最多 `max_candidates`（默认 8）套布局。它们得分等价但物理抓取顺序/落位不同，交给
path_planner 用运动代价二次择优。`solveForMask` 是 `max_candidates=1` 的薄封装。

---

## 5. 步骤 5 — 补放、拓扑排序与输出

对**每一套**同分候选布局都做（`buildPlanMsgFromSolution`）：

### 5.1 补放剩余块
把没用上的库存块，从底行往上贪心塞进空位，条件是**不破坏已放置**且**下方有支撑**：
一个候选落位 $\{(r_k,c_k)\}$ 合法当且仅当每格空、且

$$\text{fully\_supported}=\bigwedge_k\Big(r_k+1\ge14\ \vee\ \text{board}[r_k{+}1][c_k]\ne0\Big)$$

即每格要么触底、要么下面压着别的块。

### 5.2 放置顺序拓扑排序（Kahn 算法）
真机必须**先放底层再放上层**，否则上层块悬空。构造依赖 DAG：若格 $(r,c)$ 与其下方
$(r+1,c)$ 属**不同棋子**，加有向边「下 → 上」（下方块须先放）。用 **Kahn 算法**做拓扑排序：

1. 计算各节点入度 $\deg^-$，入度 0 的入优先队列。
2. 反复取队首 $u$ 加入序列，对每条 $u\to v$ 做 $\deg^-(v){-}{-}$，减到 0 则入队。

同层（无依赖）用优先级决定先后：**底行更靠下优先，其次更靠左优先**。设棋子的最大行
$p_\text{bottom}$、最小列 $p_\text{left}$，优先队列序为

$$u\prec v\iff p_\text{bottom}(u)>p_\text{bottom}(v)\ \text{或}\ \big(p_\text{bottom}(u)=p_\text{bottom}(v)\ \wedge\ p_\text{left}(u)<p_\text{left}(v)\big)$$

### 5.3 组装动作
按拓扑序为每棋子从对应形状的散料池弹出一个像素抓取点，组装 17 整数动作：

```
[shape, way, sum_r, sum_c, pick_u, pick_v, pick_angle, geom_u, geom_v,
 cell0_r, cell0_c, ... cell3_r, cell3_c]
```

- `way`∈{0,1,2,3} → 旋转 $0/90/180/270°$。
- `sum_r`/`sum_c` 是 4 格行/列坐标**绝对总和**，下游 $\div4$ 得中心。
- 各候选**各自独立回填抓取像素**（拷贝散料池后 `pop_back`），故具体用哪个物理块由
  path_planner 连同腕部翻转一并优化。

发布：`/tetris_plan` 发**首选**（第一套），`/tetris_plan_candidates` 发**全部** $K$ 套
（帧布局 `[K, len0, plan0, len1, plan1, ...]`），随后 `task_completed=true`。

---

## 6. 进阶模式（形状序列硬约束）

竞赛进阶：裁判给定**形状抽签序列**，棋盘从空开始，只放其中一部分、按规则边放边搭。开关
`advanced_mode:=true`。求解器 `solveSequence` 用**束搜索（beam search）**：

### 6.1 有效序列
`seqBuildEffective` 三种模式：
- **分组（`seq_group_by_shape=true`，新默认）**：先放完第一个形状（其全部库存），再到下一个。
  形状完成顺序 = `shape_sequence` **去重**（按首次出现），每种形状展开成其库存/`pick_limit` 份，
  总数上限 = 库存之和。此模式**忽略 `cyclic`**。
- 交错有限（`group_by_shape=false, cyclic=false`）：直接用原序列。
- 交错循环（`group_by_shape=false, cyclic=true`）：循环重复并按库存跳过已用尽的形状。

### 6.2 束搜索
逐步放置：对当前束内每个状态，枚举该步形状的**全部合法放置**扩展成子状态，按启发值排序
只保留前 `beam_width`（默认 120）个：

$$\text{beam}_{t+1}=\operatorname{top-}W\big\{\text{expand}(s)\mid s\in\text{beam}_t\big\}\ \text{按}\ h(\cdot)\ \text{降序}$$

**启发值**（主项得分、次项偏好填满行、低行优先）：

$$h(\text{board})=10^6\cdot\text{score}+\sum_{\text{row }r}\text{filled}(r)^2\,(1+r)\ \big[+\ \text{颜色多样次项}\big]-\lambda\cdot U(\text{board})$$

$\text{filled}(r)^2$ 使接近填满的行权重超线性上升，$(1+r)$ 让低行（$r$ 大）权重略高。
末项仅**终态支撑模式**生效：$U$ = 悬空格数、$\lambda=\texttt{float\_penalty}$（默认 1000），
仅作排序倾向（不改计分），引导束优先选可被支撑的布局。

### 6.3 支撑约束（规则③）
两种口径，由 `seq_final_support` 选择：

- **逐步支撑（`seq_final_support=false`，新默认）**：每块放置**当场**下方即需有支撑，落最底行
  靠盘面支撑属例外（故第一块被迫落最底行）：

$$\text{supported}=\bigvee_{k}\big(r_k+1\ge R\ \vee\ \text{board}[r_k{+}1][c_k]\ne0\big)$$

  （`seq_require_support=false` 为宽松实验：下方支撑 **或** 竖直相邻。）此时**分组求解顺序本身
  就是先底后顶合法的执行序**——每块放置时其支撑已在盘上——故策略**直接按分组顺序发布**（不做
  拓扑重排），机器人真正「先放完一种形状再到下一种」，真机不会悬空。

- **终态支撑（`seq_final_support=true`）**：支撑按**终态整盘**判——允许某块放置时暂悬空，其正
  下方留待**后续形状/组**填上。搜索时放置只需**锚定**（触底，或任一四邻已被现有结构占据）以剪枝，
  纯悬空块仅在其后还有方块可填其底时允许；合法解由 `seqFinalValid` 统一校验：**终态每个已填格
  都受支撑**（触底或正下方有块）**且支撑依赖 DAG 无环**。装填自由度更大（可先摆上层、后由别的
  形状填底），但求解序不是先底后顶——策略须先按支撑 DAG 做 **Kahn 拓扑排序**（先底后顶，同
  §5.2）再打包，**执行会跨形状交错**。

计分同普通模式（§4.3）。配色加分（规则②）由 `seq_reward_four_colors` 控制，但**分组模式恒关闭
四色约束**。输出沿用 17-int 格式，以单候选集（$K{=}1$）发到候选话题，供 `path_planner` 在 DAG 内
二次择优（`lucky.launch` 默认 `allow_reorder=false`，即保留策略给的顺序）。

---

## 7. 接口与参数

### 输入 `/vision/board_state`
`[7 库存][140 占用栅格][num_blocks][每块 stride 个整数]`；`stride` 运行时推断：剩余整数
$\ge 6\,\text{num\_blocks}$ 按 6 解析（带 `geom_u/v`），否则按 4。`data.size()>=147` 才处理。

### 关键参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `expected_total_blocks` / `min_usable_total_blocks` | 35 / 34 | 满量 / 差 1 兜底门槛 |
| `inventory_stable_required_frames` / `min_usable_stable_required_frames` | 5 / 3 | 稳定帧数 $K$ |
| `num_strategy_candidates` | 8 | 同分最优候选布局数（$>1$ 交 path_planner 择优）|
| `advanced_mode` | false | 进阶开关 |
| `seq_group_by_shape` | true | 分组求解：先放完一种形状再到下一种（新默认策略）|
| `seq_final_support` | false | false=逐步支撑（按分组顺序发布、逐形状完成、真机不悬空）；true=终态支撑（装填更自由，但执行先底后顶、跨形状交错）|
| `seq_beam_width` | 1200 | 分组束搜索束宽。**默认求解器内 120 太小会明显欠搜索**（满行数腰斩）；方案只算一次再执行，可用大束宽换分（实测 600≈0.8s→8 满行，1200≈1.9s 到顶）|
| `seq_cyclic` / `seq_require_support` | false / true | 循环（仅旧交错模式）/ 逐步支撑是否用竞赛规则③ |
| `seq_reward_four_colors` | false | 进阶配色加分（规则②）；**分组模式恒关闭** |
| `float_penalty` | 1000 | 终态支撑模式悬空格启发惩罚（仅 `tetris_solver.hpp`，非 rosparam）|
| `shape_sequence` | — | 进阶形状 id 序列（`rosparam`）；分组模式作形状完成顺序（去重）|

DLX 内部：34 块方案 `max_search_nodes=5×10⁶, max_restarts=10, max_sols=400`；通用方案
`10⁶ / 5 / 100`。

---

## 8. 单独测试

```bash
roslaunch lucky test_strategy.launch                                # 普通
roslaunch lucky test_strategy.launch advanced_mode:=true seq_cyclic:=true
```

手动喂 `/vision/board_state`（需**连续**发够 $K$ 帧库存不变才触发，故用 `-r` 持续发布，
而非发一帧），观察 `/tetris_plan` 与 `/tetris_plan_candidates`。节点日志会打印每步评估方案、
锁定分数、逐条 `action [i/N]`。

布局 = `7 库存 + 140 占用栅格(14×10 行优先) [+ num_blocks + 每块 (shape,u,v,angle[,geom_u,geom_v])]`。
`strategy_node` 核心求解只需前 147 个（散料明细仅 `size>147` 时解析，供 path_planner 用）。最小
可触发帧：库存共 35 块 + 空棋盘（140 个 0）：

```bash
# 空棋盘、库存 [5×7]=35，10Hz 连发；稳定 K 帧后触发一次规划
rostopic pub -r 10 /vision/board_state std_msgs/Int32MultiArray \
  "{data: [5,5,5,5,5,5,5, $(python3 -c 'print(",".join(["0"]*140))')]}"
# 另开终端看结果
rostopic echo /tetris_plan
```

改库存分布即改 `[5,5,5,5,5,5,5]`（7 形状 id 计数，和须达 `expected_total_blocks`／兜底
`min_usable_total_blocks`）；把某些栅格 0 改 1 即模拟已占格。进阶模式（`advanced_mode:=true`）
不要求库存达 34/35，稳定即解。
