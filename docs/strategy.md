# 策略节点（strategy_node）

`strategy_node` 是流水线中负责"装箱决策"的核心。它接收视觉节点输出的库存与散料像素坐标，把"在 14×10 网格里尽可能多得分"当作一个**带计分的精确覆盖问题**求解，最终发布逐块抓放方案。

输出**不直接给控制器**，而是给 `path_planner_node`：普通模式会保留**若干同分最优布局**作为候选集发到 `/tetris_plan_candidates`，由 `path_planner_node` 按机械臂运动代价择优、解算坐标与腕部翻转，再喂给纯执行器 `xarm_controller_node`（见 [docs/path_plan.md](path_plan.md)）。同时仍向 `/tetris_plan`（latched）发一份「首选」计划，向后兼容 `test_path` / `test_controller` 的单解流程。

代码：
- 节点逻辑：[src/tly/src/strategy_node.cpp](../src/tly/src/strategy_node.cpp)
- 求解引擎（ROS-free，可单独单测）：[src/tly/include/tly/tetris_solver.hpp](../src/tly/include/tly/tetris_solver.hpp)
- 仅策略测试 launch：[src/tly/launch/test_strategy.launch](../src/tly/launch/test_strategy.launch)

---

## 1. 接口契约

### 输入：`/vision/board_state`（`std_msgs/Int32MultiArray`）
扁平整型数组，布局为：

| 段 | 长度 | 含义 |
|---|---|---|
| 库存 | 7 | 7 种形状各自的散料数量（`inventory[0..6]`） |
| 棋盘占用栅格 | 140 | 14×10 行优先；`>0` 表示该格已被占用 |
| `num_blocks` | 1 | 散料块数量（数组长度 > 147 时才有） |
| 散料块 | `num_blocks × stride` | 每块 `stride=4`：`[shape, pick_u, pick_v, angle]`；或 `stride=6`：再带 `geom_u, geom_v` |

- `data.size() >= 147`（7+140）才会被处理；不足直接丢弃。
- 散料块的 `stride` 由运行时推断：若 `num_blocks > 0 且 剩余整数 >= num_blocks*6`，则按 6 解析（带几何中心），否则按 4。`geom_u/geom_v`（方块几何中心）供控制节点补偿 hybrid 吸点偏离几何中心造成的放置平移误差。

### 输出 1：`/tetris_plan`（`std_msgs/Int32MultiArray`，latched）
「首选」单份计划（同分候选里的第一份），向后兼容单解流程。
- 首元素：动作数 `N`。
- 之后每个动作 **17 个整数**：
  ```
  [shape, way, sum_r, sum_c, pick_u, pick_v, pick_angle, geom_u, geom_v,
   cell0_r, cell0_c, cell1_r, cell1_c, cell2_r, cell2_c, cell3_r, cell3_c]
  ```
  - `shape`：形状 id（见下）；`way`：放置旋转档位（0/1/2/3 → 0/90/180/270°）。
  - `sum_r`/`sum_c`：该块 4 个格子行/列坐标的**绝对总和**（下游除以 4 得中心）。
  - `pick_u/v/angle`：抓取像素点与视觉角度；`geom_u/v`：几何中心像素（补偿用）。
  - `cell0..3`：4 个目标格子的 `(row, col)`。
- 进阶模式输出格式相同。

### 输出 2：`/tetris_plan_candidates`（`std_msgs/Int32MultiArray`，latched）
同分最优的**多套布局**打包成一帧，供 `path_planner_node` 按运动代价择优（`publishCandidateSet`）。帧布局：

```
[K, len0, plan0(len0 个 int), len1, plan1(len1 个 int), ...]
```

- `K`：候选数；每个 `plani` 就是一份完整的上面那种 17-int 计划（含首元素动作数）。
- 各候选**各自独立回填抓取像素**（`buildPlanMsgFromSolution` 内部拷贝散料池后 `pop_back`），故不同布局用哪个物理块由 path_planner 连同腕部翻转一起优化。
- 话题名可用 `plan_candidates_topic` 改；候选数上限 `num_strategy_candidates`（默认 8）。
- 进阶模式只有一个确定解，也以**单候选集**（`K=1`）形式发到此话题，供 path_planner 统一消费。

### 状态：`/robot_status`（`std_msgs/Bool`，订阅）
机器人忙/闲标志。`is_robot_busy` 期间不规划，避免与执行交叠。

### 形状 id 契约（与 vision / path_planner / controller 共享，改动需同步全部）
`0` 一字（红）、`1` 田字（橙）、`2` T（棕）、`3` L 左（紫）、`4` L 右（黄）、`5` Z 左（蓝）、`6` Z 右（绿）。定义见 `BASE_SHAPES`。

---

## 2. 触发与稳定门控

`visionCallback` 在真正规划前要过几道门，避免视觉只识别到一部分（20~30 块）就提前出方案：

1. **正在规划 / 机器人忙 / 已完成**：直接返回（一次任务只规划一次，`task_completed` 锁死）。
2. **数量门槛**（普通模式）：`total_blocks < min_usable_total_blocks`（默认 34）时不规划，重置稳定计数。
3. **库存稳定**：库存数组需连续若干帧不变。所需帧数：
   - 满 `expected_total_blocks`（默认 35）→ `inventory_stable_required_frames`（默认 5）；
   - 仅达 34（差 1 的 fallback）→ `min_usable_stable_required_frames`（默认 3）。
4. 稳定后即开始规划；若只有 34 块，会打 fallback 警告但仍继续。

> 这套"差 1 也能跑"的兜底是为了避免视觉永远差一个方块时策略永远空等——求解器内部本就支持 34 块特判方案。

---

## 3. 普通模式求解（最大化得分）

核心思想：把"哪些块放到哪些格"建模为**精确覆盖**，用手写 Dancing Links（DLX）搜索，并在多套"目标方案"间挑分最高的。

### 3.1 候选方案（`PlanConfig`）
按当前总块数构造一组待评估方案 `plans`：
- 若 `total_blocks >= 34`：加入一个 34 块、禁用第 0 行 6~9 列 4 格的特化方案（`max_possible_score=260`）。
- 通用方案：从 `max_even_rows`（`total*4/10`，封顶 12，取偶）开始，每次减 2 行，构造若干"用满底部 r 行"的方案，理论满分 `r*20`。

逐方案评估，一旦某方案找到解就锁定并停止（方案按分数潜力从高到低排列）。

### 3.2 子库存枚举（`generateSubInventories`）
对每个方案的目标块数 `target_blocks`，枚举所有"从 7 种库存中取出恰好 target 块"的组合，且约束 **T 形（id=2）取偶数个**（放置/对称性需要）。每方案最多评估前 20 个子库存。

### 3.3 DLX 精确覆盖（`solveForMaskMulti`）
- **列**：`target_blocks` 个"每个棋子必须用一次"的棋子列 + 起始行 `start_r` 到 13、未禁用、未被占用的每个棋盘格列。
- **行（矩阵行）**：每个棋子实例的每个唯一旋转、每个合法落位生成一行（覆盖"自身棋子列 + 占据的格子列"）。每个棋子还有一行"只占自身列"的空行，表示**可以不放**（覆盖棋子列但不占格）。
- **同形多实例去对称**：`piece_prev` 链让同种形状的多个实例必须按顺序使用（`is_piece_used` 约束），避免等价排列爆搜。
- **带重启的随机化**：每个方案做 `max_restarts` 次，每次 `shuffle` 行顺序后重新建 DLX 搜索（`search` 受 `max_search_nodes` 与最大解数限制），对所有找到的解逐一计分。
- **收集同分多候选**：`solveForMaskMulti` 不只取一个最优解，而是把达到**最高分**的解**去重**后收集最多 `max_candidates`（= `num_strategy_candidates`，默认 8）套布局，一并返回（`solveForMask` 是它 `max_candidates=1` 的薄封装）。这些同分布局在得分上等价，但物理抓取顺序/落位不同，交给 path_planner 用运动代价二次择优。
- **计分规则**：
  - 满行 +10；满行且该行 **≥4 种颜色** 再 +10（普通模式此加分恒定生效）；
  - 规则④：整盘出现的形状种类 < 3 → 该解 0 分。
  - 达到方案理论满分即提前返回。

### 3.4 补放、拓扑排序与输出
对**每一套**同分候选布局（`buildPlanMsgFromSolution`）都做：
1. **补放剩余块**：把没用上的库存块，在不破坏已放置、且"下方有支撑"（`fully_supported`）的前提下，从底行往上贪心塞进空位，尽量多放。
2. **放置顺序拓扑排序**：构造"上方块依赖下方块"的依赖图（同列上下相邻且属不同棋子 → 下→上 有向边），用 Kahn 算法做拓扑排序，保证**先放底层再放上层**。同层用优先级 `(底行更靠下优先, 更靠左优先)` 决定先后。
3. 按拓扑序为每个棋子从对应形状的散料里弹出一个像素抓取点（`available_blocks[shape].back()`），组装 17-int 动作。无对应像素时告警并用 `(0,0)`。仅首选候选打印逐块日志（`verbose`），避免刷屏。

打包完成后：向 `/tetris_plan` 发**首选**（第一套）计划，向 `/tetris_plan_candidates` 发**全部** K 套（见输出 2），随后 `task_completed = true`，本次任务结束（一次任务只规划一次）。

---

## 4. 进阶模式（按形状序列硬约束）

竞赛进阶任务：裁判给定一个**形状抽签序列**，只放其中一部分，按规则边放边搭。开关 `advanced_mode:=true`。

- 不要求库存达 34/35，只要稳定 + 非空即求解。
- 棋盘视为**从空开始**（裁判已清盘）。
- 求解器 `solveSequence`（`tetris_solver.hpp`）：
  - `seqBuildEffective` 构造有效放置序列：`cyclic=false` 用原序列；`cyclic=true` 循环重复并按库存跳过已用尽的形状，长度上限=库存之和。
  - **束搜索（beam search）**：每步对当前束内每个状态枚举该形状全部合法放置，扩展后按启发值 `seqHeuristic` 保留前 `beam_width`（默认 120）个。
  - **支撑约束（规则③）**：`seq_require_support=true`（竞赛规则）要求每块下方有方块支撑，落在最底行靠盘面支撑属例外（故第一块被迫落最底行）；`false` 为宽松实验（下方支撑 *或* 竖直相邻）。
  - 计分：满行 +10；整盘形状种类 <3 则 0 分。**「满行且 ≥4 色再 +10」由 `seq_reward_four_colors` 控制（默认 `false`）**——普通模式此加分恒定生效，但进阶模式实际比赛未必有配色加分，故默认关闭、只追满行（同时影响 `seqHeuristic` 的配色次项，与计分口径一致）。
- 输出沿用同样的 17-int 格式；按放置顺序直接打包（跳过拓扑排序），从散料弹出抓取像素。发布到 `/tetris_plan` 的同时，也以**单候选集**（`K=1`）发到 `/tetris_plan_candidates`，供 path_planner 统一消费。

---

## 5. 参数一览（`~private`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `expected_total_blocks` | 35 | 期望识别到的总块数（满量） |
| `min_usable_total_blocks` | 34 | 普通模式允许规划的最少块数（差 1 兜底） |
| `inventory_stable_required_frames` | 5 | 满量时所需连续稳定帧 |
| `min_usable_stable_required_frames` | 3 | 仅 34 块兜底时所需稳定帧 |
| `num_strategy_candidates` | 8 | 普通模式保留的同分最优候选布局数（>1 交 path_planner 按运动代价择优；<1 会夹到 1） |
| `plan_candidates_topic` | `/tetris_plan_candidates` | 候选集发布话题 |
| `advanced_mode` | false | 开启进阶（形状序列）模式 |
| `seq_cyclic` | false | 进阶序列是否循环放置 |
| `seq_require_support` | true | 进阶支撑约束：竞赛规则③ / 宽松实验 |
| `seq_reward_four_colors` | false | 进阶模式「满行且 ≥4 色 +10」加分（竞赛规则②）；默认关闭 |
| `shape_sequence` | — | 进阶模式形状 id 列表（`rosparam`，如 `[0,1,2,3,4,5,6]`） |

---

## 6. 单独测试

`test_strategy.launch` 只起 `strategy_node`，**无任何硬件依赖**——纯算法验证。

```bash
roslaunch tly test_strategy.launch
# 进阶模式：
roslaunch tly test_strategy.launch advanced_mode:=true seq_cyclic:=true
```

手动喂入 `/vision/board_state` 触发规划，观察输出（`/tetris_plan` 首选计划、`/tetris_plan_candidates` 同分候选集）：

```bash
# 终端 A：观察输出（首选计划 + 候选集）
rostopic echo /tetris_plan
rostopic echo /tetris_plan_candidates

# 终端 B：构造一帧 board_state（7 库存 + 140 全 0 栅格 + num_blocks + 每块 6 个像素整数）
# 注意：需连续发够 inventory_stable_required_frames 帧（库存数组保持不变）才会触发
rostopic pub -r 10 /vision/board_state std_msgs/Int32MultiArray "data: [ ... ]"
```

节点会在日志里打印每步评估的方案、锁定分数、以及逐条 `action [i/N]` 抓放序列，便于核对 `way` / 中心 / 抓取像素。
