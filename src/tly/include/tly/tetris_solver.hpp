#pragma once

// 俄罗斯方块放置求解核心（ROS-free，可单独测试）。
// 从 strategy_node.cpp 抽出：形状定义 + DLX 精确覆盖引擎 + 子库存枚举 + 单掩码求解。
// 设计为按可执行文件各自包含（strategy_node 与未来的单测程序各自一个 TU）。
// 进阶任务（带形状序列硬约束的求解）将在此模块扩展。

#include <vector>
#include <algorithm>
#include <random>
#include <set>

using namespace std;

// ==============================================================================
// 核心数据结构与形状定义
// ==============================================================================
struct Point
{
    int x, y;
    bool operator<(const Point &o) const { return y != o.y ? y < o.y : x < o.x; }
};

struct ShapeVariant
{
    vector<Point> coords;
    int real_way; // 0=0°, 1=90°, 2=180°, 3=270°
};

struct Action
{
    int piece_id;
    int shape_type;
    int rot_idx;
    int real_way;
    int start_r, start_c;
    vector<Point> absolute_coords;
};

struct PlanConfig
{
    int target_blocks;
    int start_r;
    vector<Point> disabled_cells;
    int max_possible_score;
    int max_search_nodes;
    int max_restarts;
};

// 7 种基础形状（每个可执行文件各自一份，static 避免多 TU 链接冲突）
static vector<vector<Point>> BASE_SHAPES = {
    {{0, 0}, {1, 0}, {2, 0}, {3, 0}}, // 0: 红色一字形
    {{0, 0}, {1, 0}, {0, 1}, {1, 1}}, // 1: 橙色田字形
    {{0, 0}, {1, 0}, {2, 0}, {1, 1}}, // 2: 棕色T

    {{1, 0}, {1, 1}, {1, 2}, {0, 2}}, // 3: 紫色L左
    {{0, 0}, {0, 1}, {0, 2}, {1, 2}}, // 4: 黄色L右

    {{0, 0}, {1, 0}, {1, 1}, {2, 1}}, // 5: 蓝色Z左
    {{1, 0}, {2, 0}, {0, 1}, {1, 1}}  // 6: 绿色Z右
};

// ==============================================================================
// DLX 极速精确覆盖引擎 (省略未改动的方法...)
// ==============================================================================
const int MAX_NODES = 500000;
struct DLXNode
{
    int r, c, up, down, left, right;
};

class DLX
{
public:
    DLXNode nodes[MAX_NODES];
    int col_size[200], head[200], node_count, col_count, max_solutions;
    vector<int> current_ans;
    vector<vector<int>> all_solutions;
    vector<int> row_to_piece, piece_prev;
    bool is_piece_used[40];
    int search_nodes_count, max_search_nodes;

    void init(int c_count, int max_sol, vector<int> p_prev, int limit_nodes = 5000000)
    {
        col_count = c_count;
        max_solutions = max_sol;
        piece_prev = p_prev;
        search_nodes_count = 0;
        max_search_nodes = limit_nodes;
        for (int i = 0; i < 40; ++i)
            is_piece_used[i] = false;
        for (int i = 0; i <= col_count; ++i)
        {
            nodes[i].left = i - 1;
            nodes[i].right = i + 1;
            nodes[i].up = i;
            nodes[i].down = i;
            col_size[i] = 0;
            head[i] = i;
        }
        nodes[0].left = col_count;
        nodes[col_count].right = 0;
        node_count = col_count + 1;
        all_solutions.clear();
        current_ans.clear();
        row_to_piece.clear();
    }

    void addRow(int row_idx, const vector<int> &columns, int piece_id)
    {
        if (row_idx >= row_to_piece.size())
            row_to_piece.resize(row_idx + 1, -1);
        row_to_piece[row_idx] = piece_id;
        int first_node = node_count;
        for (int i = 0; i < columns.size(); ++i)
        {
            int c = columns[i] + 1;
            int id = node_count++;
            nodes[id].r = row_idx;
            nodes[id].c = c;
            nodes[id].down = head[c];
            nodes[id].up = nodes[head[c]].up;
            nodes[nodes[head[c]].up].down = id;
            nodes[head[c]].up = id;
            col_size[c]++;
            if (i == 0)
            {
                nodes[id].left = id;
                nodes[id].right = id;
            }
            else
            {
                nodes[id].left = first_node;
                nodes[id].right = nodes[first_node].right;
                nodes[nodes[first_node].right].left = id;
                nodes[first_node].right = id;
            }
        }
    }

    void remove(int c)
    {
        nodes[nodes[c].right].left = nodes[c].left;
        nodes[nodes[c].left].right = nodes[c].right;
        for (int i = nodes[c].down; i != c; i = nodes[i].down)
            for (int j = nodes[i].right; j != i; j = nodes[j].right)
            {
                nodes[nodes[j].down].up = nodes[j].up;
                nodes[nodes[j].up].down = nodes[j].down;
                col_size[nodes[j].c]--;
            }
    }

    void resume(int c)
    {
        for (int i = nodes[c].up; i != c; i = nodes[i].up)
            for (int j = nodes[i].left; j != i; j = nodes[j].left)
            {
                nodes[nodes[j].down].up = j;
                nodes[nodes[j].up].down = j;
                col_size[nodes[j].c]++;
            }
        nodes[nodes[c].right].left = c;
        nodes[nodes[c].left].right = c;
    }

    void search()
    {
        if (all_solutions.size() >= max_solutions)
            return;
        if (search_nodes_count++ > max_search_nodes)
            return;
        if (nodes[0].right == 0)
        {
            all_solutions.push_back(current_ans);
            return;
        }

        int c = nodes[0].right;
        for (int i = nodes[0].right; i != 0; i = nodes[i].right)
            if (col_size[i] < col_size[c])
                c = i;

        remove(c);
        for (int i = nodes[c].down; i != c; i = nodes[i].down)
        {
            int r = nodes[i].r;
            int p_id = row_to_piece[r];
            int prev_p = piece_prev[p_id];
            if (prev_p != -1 && !is_piece_used[prev_p])
                continue;

            is_piece_used[p_id] = true;
            current_ans.push_back(r);
            for (int j = nodes[i].right; j != i; j = nodes[j].right)
                remove(nodes[j].c);
            search();
            for (int j = nodes[i].left; j != i; j = nodes[j].left)
                resume(nodes[j].c);
            current_ans.pop_back();
            is_piece_used[p_id] = false;
        }
        resume(c);
    }
};

// ==============================================================================
// 图形处理与规划引擎
// ==============================================================================
vector<Point> normalize(vector<Point> shape)
{
    int min_x = 999, min_y = 999;
    for (auto &p : shape)
    {
        min_x = min(min_x, p.x);
        min_y = min(min_y, p.y);
    }
    vector<Point> norm;
    for (auto &p : shape)
        norm.push_back({p.x - min_x, p.y - min_y});
    sort(norm.begin(), norm.end(), [](const Point &a, const Point &b)
         {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x; });
    return norm;
}

vector<ShapeVariant> getUniqueRotations(vector<Point> base)
{
    vector<ShapeVariant> unique_rots;
    for (int r = 0; r < 4; ++r)
    {
        vector<Point> rot;
        if (r == 0)
            rot = base;
        else if (r == 1)
        {
            for (auto &p : base)
                rot.push_back({-p.y, p.x});
        }
        else if (r == 2)
        {
            for (auto &p : base)
                rot.push_back({-p.x, -p.y});
        }
        else if (r == 3)
        {
            for (auto &p : base)
                rot.push_back({p.y, -p.x});
        }

        vector<Point> norm = normalize(rot);

        bool is_dup = false;
        for (auto &existing : unique_rots)
        {
            if (existing.coords.size() == norm.size())
            {
                bool match = true;
                for (size_t i = 0; i < norm.size(); ++i)
                {
                    if (existing.coords[i].x != norm[i].x || existing.coords[i].y != norm[i].y)
                    {
                        match = false;
                        break;
                    }
                }
                if (match)
                {
                    is_dup = true;
                    break;
                }
            }
        }
        if (!is_dup)
            unique_rots.push_back({norm, r});
    }
    return unique_rots;
}

bool solveForMask(const PlanConfig &plan, const vector<int> &sub_inv, const int current_board[14][10],
                  vector<Action> &out_actions, vector<int> &out_best_sol, int &out_best_score)
{
    int total_pieces = plan.target_blocks;

    int cell_col_map[14][10];
    for (int r = 0; r < 14; ++r)
        for (int c = 0; c < 10; ++c)
            cell_col_map[r][c] = -1;

    int col_idx = total_pieces;
    for (int r = plan.start_r; r <= 13; ++r)
    {
        for (int c = 0; c < 10; ++c)
        {
            bool disabled = false;
            for (auto &p : plan.disabled_cells)
            {
                if (p.x == r && p.y == c)
                {
                    disabled = true;
                    break;
                }
            }
            if (!disabled)
                cell_col_map[r][c] = col_idx++;
        }
    }

    int col_count = col_idx;
    vector<vector<int>> matrix;
    vector<Action> all_actions;
    vector<int> piece_prev(total_pieces, -1);

    int p_id = 0;
    for (int type = 0; type < 7; ++type)
    {
        auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
        for (int inst = 0; inst < sub_inv[type]; ++inst)
        {
            if (inst > 0)
                piece_prev[p_id] = p_id - 1;

            matrix.push_back({p_id});
            all_actions.push_back({p_id, type, -1, -1, -1, -1, {}});

            for (int rot_idx = 0; rot_idx < unique_rots.size(); ++rot_idx)
            {
                auto &variant = unique_rots[rot_idx];
                auto &shape = variant.coords;
                int max_x = 0, max_y = 0;
                for (auto &p : shape)
                {
                    max_x = max(max_x, p.x);
                    max_y = max(max_y, p.y);
                }

                int valid_height = 14 - plan.start_r;
                for (int local_r = 0; local_r <= valid_height - max_y - 1; ++local_r)
                {
                    for (int c = 0; c <= 10 - max_x - 1; ++c)
                    {
                        bool valid = true;
                        vector<Point> abs_coords;
                        vector<int> row_cols = {p_id};

                        int global_r = plan.start_r + local_r;
                        for (auto &p : shape)
                        {
                            int nr = global_r + p.y;
                            int nc = c + p.x;
                            if (nr < plan.start_r || nr >= 14 || nc < 0 || nc >= 10 || current_board[nr][nc] != 0 || cell_col_map[nr][nc] == -1)
                            {
                                valid = false;
                                break;
                            }
                            abs_coords.push_back({nr, nc}); // x为行，y为列
                            row_cols.push_back(cell_col_map[nr][nc]);
                        }
                        if (valid)
                        {
                            all_actions.push_back({p_id, type, rot_idx, variant.real_way, global_r, c, abs_coords});
                            matrix.push_back(row_cols);
                        }
                    }
                }
            }
            p_id++;
        }
    }

    mt19937 rng(1337 + plan.target_blocks);
    int max_sols = (plan.target_blocks >= 34) ? 400 : 100;
    int global_best_score_internal = -1;

    for (int attempt = 0; attempt < plan.max_restarts; ++attempt)
    {
        vector<int> row_indices(matrix.size());
        for (int i = 0; i < matrix.size(); ++i)
            row_indices[i] = i;
        shuffle(row_indices.begin(), row_indices.end(), rng);

        DLX *dlx = new DLX();
        dlx->init(col_count, max_sols, piece_prev, plan.max_search_nodes);
        for (int i = 0; i < matrix.size(); ++i)
        {
            int idx = row_indices[i];
            dlx->addRow(idx, matrix[idx], all_actions[idx].piece_id);
        }
        dlx->search();

        if (!dlx->all_solutions.empty())
        {
            for (const auto &sol : dlx->all_solutions)
            {
                int temp_board[14][10];
                for (int i = 0; i < 14; ++i)
                    for (int j = 0; j < 10; ++j)
                        temp_board[i][j] = current_board[i][j];

                for (int row_idx : sol)
                {
                    if (all_actions[row_idx].rot_idx == -1)
                        continue;
                    for (const auto &pt : all_actions[row_idx].absolute_coords)
                    {
                        temp_board[pt.x][pt.y] = all_actions[row_idx].shape_type + 1;
                    }
                }

                int score = 0;
                for (int y = plan.start_r; y < 14; ++y)
                {
                    std::set<int> unique_colors;
                    bool is_full = true;
                    for (int x = 0; x < 10; ++x)
                    {
                        if (temp_board[y][x] == 0)
                            is_full = false;
                        else
                            unique_colors.insert(temp_board[y][x]);
                    }
                    if (is_full)
                    {
                        score += 10;
                        if (unique_colors.size() >= 4)
                            score += 10;
                    }
                }
                // 规则④：整盘形状种类 <3 则不计分
                {
                    std::set<int> board_shapes;
                    for (int y = 0; y < 14; ++y)
                        for (int x = 0; x < 10; ++x)
                            if (temp_board[y][x] != 0)
                                board_shapes.insert(temp_board[y][x]);
                    if ((int)board_shapes.size() < 3)
                        score = 0;
                }
                if (score > global_best_score_internal)
                {
                    global_best_score_internal = score;
                    out_best_sol = sol;
                }
            }

            if (global_best_score_internal >= plan.max_possible_score)
            {
                out_actions = all_actions;
                out_best_score = global_best_score_internal;
                delete dlx;
                return true;
            }
        }
        delete dlx;
    }

    if (global_best_score_internal != -1)
    {
        out_actions = all_actions;
        out_best_score = global_best_score_internal;
        return true;
    }
    return false;
}

void generateSubInventories(int type, int current_sum, int target_sum, vector<int> &current_sub,
                            const vector<int> &max_inv, vector<vector<int>> &valid_subs)
{
    if (type == 7)
    {
        if (current_sum == target_sum && current_sub[2] % 2 == 0)
            valid_subs.push_back(current_sub);
        return;
    }
    for (int take = 0; take <= max_inv[type]; ++take)
    {
        if (current_sum + take <= target_sum)
        {
            current_sub.push_back(take);
            generateSubInventories(type + 1, current_sum + take, target_sum, current_sub, max_inv, valid_subs);
            current_sub.pop_back();
        }
    }
}

// ==============================================================================
// 进阶任务求解器（带形状序列硬约束）
//   - 按给定形状序列(可循环)逐个放置；放不下即停，目标最大化得分。
//   - 硬约束(竞赛规则③)：每块下方必须有方块支撑(重叠相连)，落在最底行(第一层)
//     则靠盘面支撑、属例外。第一个方块因此被迫落在最底行。require_support=true 即
//     此规则；false 为宽松实验(支撑或上方相连)。
//   - 计分与普通模式一致：满行 +10，满行且 ≥4 色再 +10；规则④：整盘形状种类 <3 则 0 分。
//   - 采用 beam search（束搜索），在束宽内寻找高分放置序列；纯算法、可离线单测。
// 坐标约定：BASE_SHAPES 内 Point{x=列偏移, y=行偏移}；输出 cells 用 Point{x=行, y=列}。
// ==============================================================================
struct SeqPlacement
{
    int shape_type;
    int real_way;        // 0/1/2/3 -> 0/90/180/270 度
    vector<Point> cells; // 绝对坐标 Point{x=row, y=col}
};

struct SeqConfig
{
    vector<int> sequence;         // 形状 id (0..6) 放置顺序
    bool cyclic = false;          // 是否循环重复该序列
    vector<int> inventory;        // size 7，各形状可用数量
    bool require_support = true; // true=竞赛规则③(下方有方块支撑/第一层除外)；false=宽松实验
    int board_rows = 14;
    int board_cols = 10;
    int beam_width = 120;         // 束宽
    long max_total_placements = 0; // 0 = 取 inventory 之和
};

struct SeqResult
{
    vector<SeqPlacement> placements; // 按放置顺序
    int score = 0;
    int placed = 0;
};

// 计分：board 为 rows*cols 扁平数组，0=空，否则 shape_type+1（颜色）。
inline int seqScore(const vector<int> &board, int rows, int cols)
{
    int s = 0;
    for (int r = 0; r < rows; ++r)
    {
        bool full = true;
        set<int> colors;
        for (int c = 0; c < cols; ++c)
        {
            int v = board[r * cols + c];
            if (v == 0)
                full = false;
            else
                colors.insert(v);
        }
        if (full)
        {
            s += 10;
            if ((int)colors.size() >= 4)
                s += 10;
        }
    }
    // 规则④：整盘形状种类 <3 则不计分
    set<int> board_shapes;
    for (int v : board)
        if (v)
            board_shapes.insert(v);
    if ((int)board_shapes.size() < 3)
        return 0;
    return s;
}

// 启发值：主项 score；次项偏好填满行(filled^2)、低行优先、颜色多样，引导束搜索。
inline long seqHeuristic(const vector<int> &board, int rows, int cols, int score)
{
    long h = (long)score * 1000000L;
    for (int r = 0; r < rows; ++r)
    {
        int filled = 0;
        set<int> colors;
        for (int c = 0; c < cols; ++c)
        {
            int v = board[r * cols + c];
            if (v)
            {
                filled++;
                colors.insert(v);
            }
        }
        h += (long)filled * filled * (1 + r); // 低行(r 大)权重略高
        if (filled > 0)
            h += (long)colors.size();
    }
    return h;
}

struct SeqState
{
    vector<int> board; // rows*cols
    vector<SeqPlacement> placements;
    int score = 0;
    int placed = 0;
    long heuristic = 0;
};

// 枚举形状 shape 在 st 上的全部合法放置。
inline vector<SeqPlacement> seqEnumPlacements(const SeqState &st, int shape, const SeqConfig &cfg)
{
    vector<SeqPlacement> out;
    const int rows = cfg.board_rows, cols = cfg.board_cols;
    auto rots = getUniqueRotations(BASE_SHAPES[shape]);
    for (auto &var : rots)
    {
        int max_x = 0, max_y = 0;
        for (auto &p : var.coords)
        {
            max_x = max(max_x, p.x);
            max_y = max(max_y, p.y);
        }
        for (int r0 = 0; r0 + max_y <= rows - 1; ++r0)
        {
            for (int c0 = 0; c0 + max_x <= cols - 1; ++c0)
            {
                vector<Point> cells;
                bool ok = true, connected = false, supported = false;
                for (auto &p : var.coords)
                {
                    int nr = r0 + p.y; // 行
                    int nc = c0 + p.x; // 列
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols || st.board[nr * cols + nc] != 0)
                    {
                        ok = false;
                        break;
                    }
                    cells.push_back({nr, nc});
                    // 规则③：下方有方块支撑，或落在最底行(第一层，靠盘面支撑)
                    if ((nr + 1 >= rows) || st.board[(nr + 1) * cols + nc])
                        supported = true;
                    // 宽松实验项：竖直方向(上或下)与已有方块相邻
                    if ((nr - 1 >= 0 && st.board[(nr - 1) * cols + nc]) ||
                        (nr + 1 < rows && st.board[(nr + 1) * cols + nc]))
                        connected = true;
                }
                if (!ok)
                    continue;
                // 规则③ = 下方支撑(第一层/触底除外)。require_support=true 即竞赛规则；
                // false 为宽松实验(支撑或上方相连)。第一个方块在两种模式下都因"需支撑"
                // 而被迫落在最底行(此时仅盘面可支撑)。
                bool valid = cfg.require_support ? supported : (supported || connected);
                if (!valid)
                    continue;
                out.push_back({shape, var.real_way, cells});
            }
        }
    }
    return out;
}

// 构造有效放置序列：finite=原序列；cyclic=循环并按库存跳过已用尽的形状，长度上限=库存之和。
inline vector<int> seqBuildEffective(const SeqConfig &cfg)
{
    vector<int> inv = cfg.inventory;
    inv.resize(7, 0);
    int total = 0;
    for (int v : inv)
        total += max(0, v);
    if (cfg.max_total_placements > 0)
        total = min(total, (int)cfg.max_total_placements);

    vector<int> eff;
    if (cfg.sequence.empty() || total <= 0)
        return eff;

    if (!cfg.cyclic)
    {
        eff = cfg.sequence; // finite：超出库存的项会在搜索中无处可放而自然终止
        return eff;
    }
    vector<int> used(7, 0);
    int idx = 0;
    long guard = 0, guard_max = (long)total * (long)cfg.sequence.size() * 2 + 16;
    while ((int)eff.size() < total && guard++ < guard_max)
    {
        int s = cfg.sequence[idx % cfg.sequence.size()];
        idx++;
        if (s >= 0 && s < 7 && used[s] < inv[s])
        {
            eff.push_back(s);
            used[s]++;
        }
    }
    return eff;
}

inline SeqResult solveSequence(const SeqConfig &cfg)
{
    SeqResult best;
    const int rows = cfg.board_rows, cols = cfg.board_cols;
    vector<int> eff = seqBuildEffective(cfg);
    if (eff.empty())
        return best;

    SeqState init;
    init.board.assign(rows * cols, 0);
    vector<SeqState> beam = {init};

    auto consider = [&](const SeqState &st)
    {
        if (st.score > best.score || (st.score == best.score && st.placed > best.placed))
        {
            best.score = st.score;
            best.placed = st.placed;
            best.placements = st.placements;
        }
    };

    for (size_t step = 0; step < eff.size() && !beam.empty(); ++step)
    {
        int shape = eff[step];
        vector<SeqState> next;
        next.reserve(beam.size() * 8);
        for (auto &st : beam)
        {
            vector<SeqPlacement> places = seqEnumPlacements(st, shape, cfg);
            if (places.empty())
            {
                consider(st); // 该状态无法继续，记为候选解
                continue;
            }
            for (auto &p : places)
            {
                SeqState ns = st;
                for (auto &cell : p.cells)
                    ns.board[cell.x * cols + cell.y] = shape + 1;
                ns.placements.push_back(p);
                ns.placed = st.placed + 1;
                ns.score = seqScore(ns.board, rows, cols);
                ns.heuristic = seqHeuristic(ns.board, rows, cols, ns.score);
                consider(ns);
                next.push_back(std::move(ns));
            }
        }
        // 束剪枝：按启发值降序保留前 beam_width 个
        if ((int)next.size() > cfg.beam_width)
        {
            std::nth_element(next.begin(), next.begin() + cfg.beam_width, next.end(),
                             [](const SeqState &a, const SeqState &b)
                             { return a.heuristic > b.heuristic; });
            next.resize(cfg.beam_width);
        }
        beam = std::move(next);
    }
    for (auto &st : beam)
        consider(st);
    return best;
}
