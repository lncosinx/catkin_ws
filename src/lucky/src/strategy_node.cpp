#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <random>
#include <string>
#include <cstdio>
#include <lucky/tetris_solver.hpp>

using namespace std;

// ==============================================================================
// ROS 节点逻辑
// ==============================================================================
ros::Publisher plan_pub;
ros::Publisher candidates_pub; // 同分多候选集 (/tetris_plan_candidates)
std::string plan_candidates_topic = "/tetris_plan_candidates";
// 非进阶任务：策略节点保留的同分最优候选布局数（>1 时交规划节点按运动代价择优）。
int num_strategy_candidates = 8;
bool is_planning = false;
bool is_robot_busy = false;
bool task_completed = false;

// 等待视觉结果完整且连续稳定，避免只识别到 20~30 个方块时策略节点提前规划。
int expected_total_blocks = 35;
// 如果视觉长期只差 1 个，可以允许 34 个稳定后先规划，避免永远等待。
// strategy 内部本来就支持 34 块特判方案。
int min_usable_total_blocks = 34;
int inventory_stable_required_frames = 5;
int min_usable_stable_required_frames = 3;
int inventory_stable_count = 0;
vector<int> last_inventory(7, -1);

// 进阶任务模式：按外部形状序列(硬约束)求解。进阶任务只放 35 块中的一部分，
// 故不要求库存达到 34/35，只要稳定即可。
bool advanced_mode = false;
vector<int> shape_sequence;          // 形状 id 顺序，例如 [0,1,2,3,...]
bool seq_cyclic = false;             // 是否按序列循环放置
bool seq_require_support = true;     // true=竞赛规则③(下方支撑)；false=宽松实验
bool seq_reward_four_colors = false; // 满行且≥4色额外 +10(竞赛规则②)。默认 false：
                                     // 实际比赛可能没有此配色加分，关掉只追求满行。
// 进阶模式：每种形状“可吸取上限”。即使场上识别到更多，也只允许吸取指定数量的该形状；
// 例如场上有 4 个 id=0，设 shape_pick_limits[0]=2 则最多只放/吸 2 个。<0=不限制(默认，
// 用视觉全部库存)。长度 7，一一对应 7 种形状 id。仅进阶模式生效。
vector<int> shape_pick_limits(7, -1);

// 进阶模式「峰值保持」：仅「稳定」不够——视觉可能停在偏低的误识别值并稳定住。故等识别总数
// 「不再上升」(current==peak) 才规划；若某帧曾见更高峰值，就再等它稳定恢复，直到 wait 预算耗尽，
// 用见过的「最优稳定帧」整帧兜底（整帧一致，避免库存计数与像素表错位）。
int adv_peak_total = -1;                                 // 本轮见过的最大识别总数（含瞬时帧）
int adv_best_stable_total = -1;                          // 已达稳定的帧中总数最大者
std_msgs::Int32MultiArray::ConstPtr adv_best_stable_msg; // 上者对应的整帧（兜底规划用）
ros::Time adv_wait_start;                                // 本轮等待起点（首帧）
bool adv_wait_started = false;
double advanced_peak_max_wait_sec = 10.0; // 峰值保持超时预算（秒）

void statusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    is_robot_busy = msg->data;
    if (!is_robot_busy && !task_completed)
    {
        ROS_INFO("[STATUS] Robot IDLE. Strategy Engine ready for the first vision frame.");
    }
}

struct BlockInfo
{
    int u = 0, v = 0, ang = 0;  // pick point + 当前视觉角度
    int geom_u = 0, geom_v = 0; // 几何中心，用于控制节点补偿 hybrid 吸点偏移
    bool has_geom = false;
}; // 统一存放每个积木的物理属性

// 把若干份 17-int 计划打包成候选集消息并发布: [K, len0, plan0..., len1, plan1...]。
static void publishCandidateSet(ros::Publisher &pub,
                                const vector<std_msgs::Int32MultiArray> &plans)
{
    std_msgs::Int32MultiArray msg;
    msg.data.push_back((int)plans.size());
    for (const auto &p : plans)
    {
        msg.data.push_back((int)p.data.size());
        msg.data.insert(msg.data.end(), p.data.begin(), p.data.end());
    }
    pub.publish(msg);
}

// 由一个候选解(行索引序列) + 其 action 表 + 棋盘 + 库存，构建一份 17-int 抓放计划：
// 复刻原单解流程(提取放置 + 补余块填充 + 放置依赖拓扑排序 + 扁平打包)。
// available_blocks 为按形状分组的视觉散块，函数内部拷贝后 pop_back，不影响调用方；
// 多个候选因此可各自独立回填抓取像素。verbose 仅对首选候选打印逐块日志，避免刷屏。
static bool buildPlanMsgFromSolution(const vector<int> &solution,
                                     const vector<Action> &actions,
                                     const int board[14][10],
                                     const vector<int> &inventory,
                                     const vector<BlockInfo> available_blocks[7],
                                     std_msgs::Int32MultiArray &plan_msg,
                                     bool verbose)
{
    vector<BlockInfo> avail[7];
    for (int s = 0; s < 7; ++s)
        avail[s] = available_blocks[s];

    vector<Action> final_actions;
    int used_count[7] = {0};
    for (int r : solution)
    {
        if (actions[r].rot_idx == -1)
            continue;
        final_actions.push_back(actions[r]);
        used_count[actions[r].shape_type]++;
    }

    int board_piece[14][10];
    for (int i = 0; i < 14; ++i)
        for (int j = 0; j < 10; ++j)
            board_piece[i][j] = board[i][j] > 0 ? -2 : -1;
    for (const auto &act : final_actions)
        for (auto pt : act.absolute_coords)
            board_piece[pt.x][pt.y] = act.piece_id;

    int next_piece_id = 100;
    for (int type = 0; type < 7; ++type)
    {
        int leftover = inventory[type] - used_count[type];
        for (int k = 0; k < leftover; ++k)
        {
            auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
            bool placed = false;
            for (int r = 13; r >= 0 && !placed; --r)
                for (int c = 0; c < 10 && !placed; ++c)
                    for (int rot_idx = 0; rot_idx < unique_rots.size() && !placed; ++rot_idx)
                    {
                        auto &variant = unique_rots[rot_idx];
                        auto &shape = variant.coords;
                        bool valid = true, fully_supported = true;
                        vector<Point> abs_coords;
                        for (auto &p : shape)
                        {
                            int nr = r + p.y, nc = c + p.x;
                            if (nr < 0 || nr >= 14 || nc < 0 || nc >= 10 || board_piece[nr][nc] != -1)
                            {
                                valid = false;
                                break;
                            }
                            if (nr < 13 && board_piece[nr + 1][nc] == -1)
                                fully_supported = false;
                            abs_coords.push_back({nr, nc});
                        }
                        if (valid && fully_supported)
                        {
                            Action act = {next_piece_id++, type, rot_idx, variant.real_way, r, c, abs_coords};
                            final_actions.push_back(act);
                            for (auto &pt : abs_coords)
                                board_piece[pt.x][pt.y] = act.piece_id;
                            placed = true;
                        }
                    }
        }
    }

    vector<int> adj[200];
    int in_degree[200] = {0};
    for (int r = 0; r < 13; ++r)
        for (int c = 0; c < 10; ++c)
        {
            int curr = board_piece[r][c];
            int below = board_piece[r + 1][c];
            if (curr >= 0 && below >= 0 && curr != below)
            {
                bool exist = false;
                for (int t : adj[below])
                    if (t == curr)
                        exist = true;
                if (!exist)
                {
                    adj[below].push_back(curr);
                    in_degree[curr]++;
                }
            }
        }

    int p_bottom[200] = {0};
    int p_left[200] = {0};
    for (const auto &act : final_actions)
    {
        int pid = act.piece_id;
        int max_r = -1;
        int min_c = 999;
        for (auto &pt : act.absolute_coords)
        {
            if (pt.x > max_r)
                max_r = pt.x;
            if (pt.y < min_c)
                min_c = pt.y;
        }
        p_bottom[pid] = max_r;
        p_left[pid] = min_c;
    }

    auto cmp = [&](int a, int b)
    {
        if (p_bottom[a] != p_bottom[b])
            return p_bottom[a] < p_bottom[b];
        return p_left[a] > p_left[b];
    };

    priority_queue<int, vector<int>, decltype(cmp)> pq(cmp);
    for (const auto &act : final_actions)
        if (in_degree[act.piece_id] == 0)
            pq.push(act.piece_id);

    vector<int> seq;
    while (!pq.empty())
    {
        int u = pq.top();
        pq.pop();
        seq.push_back(u);
        for (int v : adj[u])
            if (--in_degree[v] == 0)
                pq.push(v);
    }

    if (seq.empty())
        return false;

    static const char *SHAPE_NAMES[] = {
        "linear_red", "grid_orange", "T_shape_brown",
        "L_left_purple", "L_right_yellow", "Z_left_blue", "Z_right_green"};

    plan_msg.data.clear();
    plan_msg.data.push_back(seq.size()); // 每个动作使用扩展 17-int 格式

    for (size_t i = 0; i < seq.size(); ++i)
    {
        for (const auto &act : final_actions)
        {
            if (act.piece_id != seq[i])
                continue;
            int pu = 0, pv = 0, p_ang = 0, geom_u = 0, geom_v = 0;
            bool has_geom = false;
            if (!avail[act.shape_type].empty())
            {
                BlockInfo b = avail[act.shape_type].back();
                pu = b.u;
                pv = b.v;
                p_ang = b.ang;
                geom_u = b.geom_u;
                geom_v = b.geom_v;
                has_geom = b.has_geom;
                avail[act.shape_type].pop_back();
            }

            int sum_r = 0, sum_c = 0;
            for (auto &pt : act.absolute_coords)
            {
                sum_r += pt.x;
                sum_c += pt.y;
            }

            plan_msg.data.push_back(act.shape_type);
            plan_msg.data.push_back(act.real_way);
            plan_msg.data.push_back(sum_r);
            plan_msg.data.push_back(sum_c);
            plan_msg.data.push_back(pu);
            plan_msg.data.push_back(pv);
            plan_msg.data.push_back(p_ang);
            plan_msg.data.push_back(geom_u);
            plan_msg.data.push_back(geom_v);
            for (auto &pt : act.absolute_coords)
            {
                plan_msg.data.push_back(pt.x); // row
                plan_msg.data.push_back(pt.y); // col
            }

            if (verbose)
                ROS_INFO("action [%2zu/%lu]: select block => %-18s | center=(%.2f, %.2f) | rotation => %3d degree | pick=(%d,%d,%d) geom=(%d,%d) has_geom=%s",
                         i + 1, seq.size(), SHAPE_NAMES[act.shape_type],
                         sum_r / 4.0, sum_c / 4.0, act.real_way * 90, pu, pv, p_ang,
                         geom_u, geom_v, has_geom ? "true" : "false");
            break;
        }
    }
    return true;
}

void visionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (task_completed)
        return;
    if (is_planning || is_robot_busy)
        return;
    if (msg->data.size() < 147)
        return;

    vector<int> current_inventory(7, 0);
    int total_blocks = 0;
    for (int i = 0; i < 7; ++i)
    {
        current_inventory[i] = msg->data[i];
        total_blocks += current_inventory[i];
    }

    // 进阶峰值保持：每帧（含未稳定帧）更新峰值与等待起点，供后面的「不再上升」判定。
    if (advanced_mode)
    {
        if (!adv_wait_started)
        {
            adv_wait_start = ros::Time::now();
            adv_wait_started = true;
        }
        if (total_blocks > adv_peak_total)
            adv_peak_total = total_blocks;
    }

    // 1) 数量太少时不规划；但允许“差 1 个”的 34 块稳定库存作为可用 fallback。
    //    进阶模式只放一部分方块，跳过此数量门槛（仅要求稳定 + 非空）。
    if (!advanced_mode && total_blocks < min_usable_total_blocks)
    {
        inventory_stable_count = 0;
        last_inventory = current_inventory;
        ROS_WARN_THROTTLE(1.0,
                          "[WAIT_VISION] recognized %d/%d blocks, below min usable %d. inv=[%d,%d,%d,%d,%d,%d,%d]",
                          total_blocks, expected_total_blocks, min_usable_total_blocks,
                          current_inventory[0], current_inventory[1], current_inventory[2],
                          current_inventory[3], current_inventory[4], current_inventory[5], current_inventory[6]);
        return;
    }

    // 2) 库存数组连续 N 帧不变，避免刚识别满时仍在抖动。
    if (current_inventory == last_inventory)
        inventory_stable_count++;
    else
    {
        inventory_stable_count = 1;
        last_inventory = current_inventory;
    }

    int required_stable = advanced_mode
                              ? inventory_stable_required_frames
                              : ((total_blocks >= expected_total_blocks)
                                     ? inventory_stable_required_frames
                                     : min_usable_stable_required_frames);

    if (inventory_stable_count < required_stable)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[WAIT_VISION] recognized %d/%d blocks, stable frame %d/%d. inv=[%d,%d,%d,%d,%d,%d,%d]",
                          total_blocks, expected_total_blocks,
                          inventory_stable_count, required_stable,
                          current_inventory[0], current_inventory[1], current_inventory[2],
                          current_inventory[3], current_inventory[4], current_inventory[5], current_inventory[6]);
        return;
    }

    if (!advanced_mode && total_blocks < expected_total_blocks)
    {
        ROS_WARN("[WAIT_VISION] fallback planning with %d/%d blocks after stable inventory. inv=[%d,%d,%d,%d,%d,%d,%d]",
                 total_blocks, expected_total_blocks,
                 current_inventory[0], current_inventory[1], current_inventory[2],
                 current_inventory[3], current_inventory[4], current_inventory[5], current_inventory[6]);
    }

    // 规划所用的帧：默认当前帧；进阶超时兜底时切到「最优稳定帧」。
    std_msgs::Int32MultiArray::ConstPtr frame = msg;

    // 进阶峰值保持决策：已稳定，但若识别数还没回到峰值，就再等（直到超时兜底）。
    if (advanced_mode)
    {
        // 记录已达稳定的最优（总数最大）帧——整帧存下，兜底时其库存/棋盘/像素表内部一致。
        if (total_blocks > adv_best_stable_total)
        {
            adv_best_stable_total = total_blocks;
            adv_best_stable_msg = msg;
        }
        const double waited = adv_wait_started ? (ros::Time::now() - adv_wait_start).toSec() : 0.0;
        const bool at_peak = (total_blocks >= adv_peak_total);
        const bool timed_out = (waited >= advanced_peak_max_wait_sec);

        if (!at_peak && !timed_out)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[WAIT_VISION][PEAK-HOLD] stable at %d but peak %d seen; waiting %.1f/%.0fs for count to recover. inv=[%d,%d,%d,%d,%d,%d,%d]",
                              total_blocks, adv_peak_total, waited, advanced_peak_max_wait_sec,
                              current_inventory[0], current_inventory[1], current_inventory[2],
                              current_inventory[3], current_inventory[4], current_inventory[5], current_inventory[6]);
            return;
        }
        if (!at_peak && timed_out && adv_best_stable_msg && adv_best_stable_total > total_blocks)
        {
            // 超时兜底：改用见过的「最优稳定帧」整帧规划。
            frame = adv_best_stable_msg;
            for (int i = 0; i < 7; ++i)
                current_inventory[i] = frame->data[i];
            total_blocks = adv_best_stable_total;
            ROS_WARN("[WAIT_VISION][PEAK-HOLD] timeout %.1fs: peak %d never held stable; falling back to best stable frame total=%d.",
                     waited, adv_peak_total, total_blocks);
        }
        else if (!at_peak && timed_out)
        {
            ROS_WARN("[WAIT_VISION][PEAK-HOLD] timeout %.1fs: proceeding with current stable total=%d (peak %d).",
                     waited, total_blocks, adv_peak_total);
        }
    }

    is_planning = true;
    ROS_INFO("===============================================================");
    ROS_INFO("=      Vision inventory stable. Launching Dynamic Engine V3    =");
    ROS_INFO("=      total=%d inv=[%d,%d,%d,%d,%d,%d,%d]", total_blocks,
             current_inventory[0], current_inventory[1], current_inventory[2],
             current_inventory[3], current_inventory[4], current_inventory[5], current_inventory[6]);
    ROS_INFO("===============================================================");

    int current_board[14][10];
    int idx = 7;
    for (int r = 0; r < 14; ++r)
    {
        for (int c = 0; c < 10; ++c)
        {
            current_board[r][c] = frame->data[idx++];
        }
    }

    // 解析视觉散落块。兼容两种格式：
    // 旧格式每块 4 个整数：[shape, pick_u, pick_v, angle]
    // 新格式每块 6 个整数：[shape, pick_u, pick_v, angle, geom_u, geom_v]
    // hybrid 吸取时需要 geom_u/geom_v 给控制节点补偿“吸点不在几何中心”的放置偏差。
    vector<BlockInfo> available_blocks[7];
    if (frame->data.size() > 147)
    {
        int num_blocks = frame->data[idx++];
        int remaining = (int)frame->data.size() - idx;
        bool vision_has_geom = (num_blocks > 0 && remaining >= num_blocks * 6);
        int stride = vision_has_geom ? 6 : 4;

        for (int i = 0; i < num_blocks && idx + stride - 1 < (int)frame->data.size(); ++i)
        {
            BlockInfo b;
            int shape = frame->data[idx++];
            b.u = frame->data[idx++];
            b.v = frame->data[idx++];
            b.ang = frame->data[idx++];
            if (vision_has_geom)
            {
                b.geom_u = frame->data[idx++];
                b.geom_v = frame->data[idx++];
                b.has_geom = true;
            }
            else
            {
                b.geom_u = b.u;
                b.geom_v = b.v;
                b.has_geom = false;
            }

            if (shape >= 0 && shape < 7)
                available_blocks[shape].push_back(b);
        }
        ROS_INFO("Parsed %d scattered blocks. vision_payload_stride=%d (%s geom center).",
                 num_blocks, stride, vision_has_geom ? "with" : "without");
    }

    if (total_blocks == 0)
    {
        ROS_WARN("No blocks recognized. Idle.");
        is_planning = false;
        return;
    }

    // ============================================================
    // 进阶任务：按外部形状序列(硬约束)求解，按放置顺序直接打包(跳过拓扑排序)。
    // 棋盘从空开始(进阶任务裁判已清盘)；输出沿用 /tetris_plan 17-int 格式。
    // ============================================================
    if (advanced_mode)
    {
        if (shape_sequence.empty())
        {
            ROS_WARN("[ADVANCED] shape_sequence param is empty; nothing to solve.");
            is_planning = false;
            return;
        }

        // 可选“可吸取上限”：某形状即使场上识别到更多，也只允许吸取指定数量。
        // shape_pick_limits[s] < 0 表示不限制(用视觉全部库存)；否则对该形状取 min。
        vector<int> effective_inventory = current_inventory;
        bool limits_applied = false;
        for (int s = 0; s < 7; ++s)
        {
            int lim = (s < (int)shape_pick_limits.size()) ? shape_pick_limits[s] : -1;
            if (lim >= 0 && lim < effective_inventory[s])
            {
                effective_inventory[s] = lim;
                limits_applied = true;
            }
        }
        if (limits_applied)
            ROS_INFO("[ADVANCED] pick limits applied: vision inv=[%d,%d,%d,%d,%d,%d,%d] -> capped=[%d,%d,%d,%d,%d,%d,%d]",
                     current_inventory[0], current_inventory[1], current_inventory[2], current_inventory[3],
                     current_inventory[4], current_inventory[5], current_inventory[6],
                     effective_inventory[0], effective_inventory[1], effective_inventory[2], effective_inventory[3],
                     effective_inventory[4], effective_inventory[5], effective_inventory[6]);

        SeqConfig cfg;
        cfg.sequence = shape_sequence;
        cfg.cyclic = seq_cyclic;
        cfg.inventory = effective_inventory;
        cfg.require_support = seq_require_support;
        cfg.reward_four_colors = seq_reward_four_colors;
        cfg.board_rows = 14;
        cfg.board_cols = 10;
        SeqResult res = solveSequence(cfg);
        ROS_INFO("[ADVANCED] solveSequence: placed=%d score=%d (seq_len=%lu cyclic=%d support=%d four_colors=%d)",
                 res.placed, res.score, shape_sequence.size(), (int)seq_cyclic, (int)seq_require_support,
                 (int)seq_reward_four_colors);

        if (res.placements.empty())
        {
            ROS_WARN("[ADVANCED] no valid placement found.");
            is_planning = false;
            return;
        }

        const char *SHAPE_NAMES[] = {
            "linear_red", "grid_orange", "T_shape_brown",
            "L_left_purple", "L_right_yellow", "Z_left_blue", "Z_right_green"};

        std_msgs::Int32MultiArray plan_msg;
        plan_msg.data.push_back((int)res.placements.size());

        ROS_INFO("====== ADVANCED pick-and-place sequence ======");
        for (size_t i = 0; i < res.placements.size(); ++i)
        {
            const auto &pl = res.placements[i];
            int pu = 0, pv = 0, p_ang = 0, geom_u = 0, geom_v = 0;
            bool has_geom = false;
            if (pl.shape_type >= 0 && pl.shape_type < 7 && !available_blocks[pl.shape_type].empty())
            {
                BlockInfo b = available_blocks[pl.shape_type].back();
                pu = b.u;
                pv = b.v;
                p_ang = b.ang;
                geom_u = b.geom_u;
                geom_v = b.geom_v;
                has_geom = b.has_geom;
                available_blocks[pl.shape_type].pop_back();
            }
            else
            {
                ROS_WARN("[ADVANCED] no pixel coordinate for shape %d! Using 0,0.", pl.shape_type);
            }

            int sum_r = 0, sum_c = 0;
            for (auto &pt : pl.cells)
            {
                sum_r += pt.x;
                sum_c += pt.y;
            }

            // 17-int 格式：与普通模式一致
            plan_msg.data.push_back(pl.shape_type);
            plan_msg.data.push_back(pl.real_way);
            plan_msg.data.push_back(sum_r);
            plan_msg.data.push_back(sum_c);
            plan_msg.data.push_back(pu);
            plan_msg.data.push_back(pv);
            plan_msg.data.push_back(p_ang);
            plan_msg.data.push_back(geom_u);
            plan_msg.data.push_back(geom_v);
            for (auto &pt : pl.cells)
            {
                plan_msg.data.push_back(pt.x); // row
                plan_msg.data.push_back(pt.y); // col
            }

            ROS_INFO("adv action [%2lu/%lu]: %-18s | center=(%.2f, %.2f) | rotation => %3d degree | pick=(%d,%d,%d) has_geom=%s",
                     i + 1, res.placements.size(), SHAPE_NAMES[pl.shape_type],
                     sum_r / 4.0, sum_c / 4.0, pl.real_way * 90, pu, pv, p_ang,
                     has_geom ? "true" : "false");
        }
        ROS_INFO("===============================================================");
        plan_pub.publish(plan_msg);
        // 进阶模式只有一个确定解：以单候选集形式同时发往候选话题，供规划节点统一消费。
        publishCandidateSet(candidates_pub, {plan_msg});
        ROS_INFO("[ADVANCED][SUCCESS] score=%d! Sent %lu blocks to execution.", res.score, res.placements.size());
        task_completed = true;
        is_planning = false;
        return;
    }

    vector<PlanConfig> plans;
    if (total_blocks >= 34)
    {
        plans.push_back({34, 0, {{0, 6}, {0, 7}, {0, 8}, {0, 9}}, 260, 5000000, 10});
    }

    int max_even_rows = (total_blocks * 4) / 10;
    if (max_even_rows > 12)
        max_even_rows = 12;
    if (max_even_rows % 2 != 0)
        max_even_rows--;

    for (int r = max_even_rows; r >= 2; r -= 2)
    {
        plans.push_back({(r * 10) / 4, 14 - r, {}, r * 20, 1000000, 5});
    }

    int global_best_score = -1;
    vector<Action> global_best_actions;
    vector<vector<int>> global_best_candidates; // 同分最优的多个布局(去重)，交规划节点按运动代价择优
    PlanConfig optimal_plan;

    for (const auto &plan : plans)
    {
        ROS_INFO(">>> Evaluation Strategy: Target usage of %d blocks, Starting row: %d, Theoretical maximum score: %d points.", plan.target_blocks, plan.start_r, plan.max_possible_score);

        vector<vector<int>> valid_subs;
        vector<int> current_sub;
        generateSubInventories(0, 0, plan.target_blocks, current_sub, current_inventory, valid_subs);

        int eval_limit = min((int)valid_subs.size(), 20);
        for (int i = 0; i < eval_limit; ++i)
        {
            vector<Action> temp_actions;
            vector<vector<int>> temp_cands;
            int current_score = 0;

            if (solveForMaskMulti(plan, valid_subs[i], current_board, temp_actions, temp_cands,
                                  current_score, num_strategy_candidates))
            {
                if (current_score > global_best_score)
                {
                    global_best_score = current_score;
                    global_best_actions = temp_actions;
                    global_best_candidates = temp_cands;
                    optimal_plan = plan;
                    if (global_best_score >= plan.max_possible_score)
                        break;
                }
            }
        }
        if (global_best_score != -1)
        {
            ROS_INFO("Strategic Lock-on! score=%d with %lu candidate layout(s).",
                     global_best_score, global_best_candidates.size());
            break;
        }
    }

    if (global_best_score != -1)
    {
        // 为每个同分候选布局各打一份 17-int 计划(独立回填抓取像素)。
        ROS_INFO("====== Packing %lu candidate plan(s) for path planner ======",
                 global_best_candidates.size());
        vector<std_msgs::Int32MultiArray> plan_msgs;
        plan_msgs.reserve(global_best_candidates.size());
        for (size_t ci = 0; ci < global_best_candidates.size(); ++ci)
        {
            std_msgs::Int32MultiArray pm;
            if (buildPlanMsgFromSolution(global_best_candidates[ci], global_best_actions, current_board,
                                         current_inventory, available_blocks, pm, /*verbose=*/(ci == 0)))
                plan_msgs.push_back(std::move(pm));
        }

        if (plan_msgs.empty())
        {
            ROS_WARN("Failed to pack any candidate plan from %lu solution(s).",
                     global_best_candidates.size());
            is_planning = false;
            return;
        }

        ROS_INFO("===============================================================");
        // 1) 向后兼容：/tetris_plan 仍发最优(首个)候选，供 test_path/test_controller 单解流程使用。
        plan_pub.publish(plan_msgs.front());
        // 2) 候选集：/tetris_plan_candidates 帧格式 [K, len0, plan0..., len1, plan1...]，
        //    规划节点对每个候选求关节代价后取最小者执行。
        publishCandidateSet(candidates_pub, plan_msgs);
        ROS_INFO("[SUCCESS] Max Score=%d. Published %lu candidate plan(s); first has %d task(s).",
                 global_best_score, plan_msgs.size(),
                 plan_msgs.front().data.empty() ? 0 : plan_msgs.front().data[0]);
        task_completed = true;
        is_planning = false;
    }
    else
    {
        ROS_WARN("Failed to find any valid placement plan with current blocks.");
    }

    is_planning = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tetris_strategy_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("expected_total_blocks", expected_total_blocks, expected_total_blocks);
    pnh.param("min_usable_total_blocks", min_usable_total_blocks, min_usable_total_blocks);
    pnh.param("inventory_stable_required_frames", inventory_stable_required_frames, inventory_stable_required_frames);
    pnh.param("min_usable_stable_required_frames", min_usable_stable_required_frames, min_usable_stable_required_frames);

    // 非进阶任务：同分最优候选数（>1 时由规划节点按运动代价择优）。
    pnh.param("num_strategy_candidates", num_strategy_candidates, num_strategy_candidates);
    if (num_strategy_candidates < 1)
        num_strategy_candidates = 1;
    pnh.param("plan_candidates_topic", plan_candidates_topic, plan_candidates_topic);

    pnh.param("advanced_mode", advanced_mode, advanced_mode);
    pnh.param("advanced_peak_max_wait_sec", advanced_peak_max_wait_sec, advanced_peak_max_wait_sec);
    pnh.param("seq_cyclic", seq_cyclic, seq_cyclic);
    pnh.param("seq_require_support", seq_require_support, seq_require_support);
    pnh.param("seq_reward_four_colors", seq_reward_four_colors, seq_reward_four_colors);
    pnh.getParam("shape_sequence", shape_sequence); // 形状 id 列表，例如 [0,1,2,3]

    // 可选“可吸取上限”：list<int>，长度≤7，一一对应形状 id；<0=该形状不限制。
    // 缺省(不传)则保持默认全 -1(不限制)。传入不足 7 项时其余保持 -1。
    {
        vector<int> user_limits;
        if (pnh.getParam("shape_pick_limits", user_limits))
        {
            for (size_t i = 0; i < user_limits.size() && i < 7; ++i)
                shape_pick_limits[i] = user_limits[i];
            if (user_limits.size() > 7)
                ROS_WARN("[ADVANCED] shape_pick_limits has %lu entries; only first 7 used.", user_limits.size());
        }
    }

    if (advanced_mode)
        ROS_INFO("[ADVANCED] mode ON: seq_len=%lu cyclic=%d require_support=%d reward_four_colors=%d pick_limits=[%d,%d,%d,%d,%d,%d,%d]",
                 shape_sequence.size(), (int)seq_cyclic, (int)seq_require_support,
                 (int)seq_reward_four_colors,
                 shape_pick_limits[0], shape_pick_limits[1], shape_pick_limits[2], shape_pick_limits[3],
                 shape_pick_limits[4], shape_pick_limits[5], shape_pick_limits[6]);

    plan_pub = nh.advertise<std_msgs::Int32MultiArray>("/tetris_plan", 10, true);
    candidates_pub = nh.advertise<std_msgs::Int32MultiArray>(plan_candidates_topic, 10, true);
    ROS_INFO("[STRATEGY] num_strategy_candidates=%d, candidates topic='%s'",
             num_strategy_candidates, plan_candidates_topic.c_str());

    // 【核心修复3】：把订阅频道改回正确的 /vision/board_state ！！！
    ros::Subscriber vision_sub = nh.subscribe("/vision/board_state", 1, visionCallback);

    ROS_INFO("Strategy Node Started. Awaiting Int32MultiArray on '/vision/board_state'...");
    ROS_INFO("[WAIT_VISION] expected_total_blocks=%d min_usable=%d stable_required_frames=%d min_usable_stable_frames=%d",
             expected_total_blocks, min_usable_total_blocks,
             inventory_stable_required_frames, min_usable_stable_required_frames);
    if (advanced_mode)
        ROS_INFO("[WAIT_VISION] advanced peak-hold: wait for count to stop climbing, timeout=%.0fs -> best stable frame.",
                 advanced_peak_max_wait_sec);

    ros::spin();
    return 0;
}