// 进阶求解器离线测试（不依赖 ROS）。
// 编译： g++ -std=c++14 -O2 -I <pkg>/include test_tetris_solver.cpp -o /tmp/tst
// 运行： /tmp/tst
//
// 验证：第一个方块触底、其后每块满足连接/支撑、计分自洽、能放下相当数量方块。

#include <lucky/tetris_solver.hpp>
#include <cstdio>
#include <cassert>

static void render(const SeqResult &res, int rows, int cols)
{
    vector<int> board(rows * cols, 0);
    for (auto &pl : res.placements)
        for (auto &c : pl.cells)
            board[c.x * cols + c.y] = pl.shape_type + 1;
    for (int r = 0; r < rows; ++r)
    {
        printf("  ");
        for (int c = 0; c < cols; ++c)
        {
            int v = board[r * cols + c];
            putchar(v == 0 ? '.' : char('0' + (v - 1)));
        }
        putchar('\n');
    }
}

// 重新独立校验每个放置的硬约束是否满足。
static bool verify(const SeqResult &res, const SeqConfig &cfg)
{
    int rows = cfg.board_rows, cols = cfg.board_cols;
    vector<int> board(rows * cols, 0);
    for (size_t i = 0; i < res.placements.size(); ++i)
    {
        const auto &pl = res.placements[i];
        bool connected = false, supported = false;
        for (auto &c : pl.cells)
        {
            if (c.x < 0 || c.x >= rows || c.y < 0 || c.y >= cols)
            {
                printf("FAIL: cell out of bounds\n");
                return false;
            }
            if (board[c.x * cols + c.y] != 0)
            {
                printf("FAIL: overlap at (%d,%d)\n", c.x, c.y);
                return false;
            }
            if ((c.x + 1 >= rows) || board[(c.x + 1) * cols + c.y])
                supported = true; // 规则③：下方有方块或触底(第一层)
            if ((c.x - 1 >= 0 && board[(c.x - 1) * cols + c.y]) ||
                (c.x + 1 < rows && board[(c.x + 1) * cols + c.y]))
                connected = true;
        }
        bool valid = cfg.require_support ? supported : (supported || connected);
        if (!valid)
        {
            printf("FAIL: block %zu floating (violates rule3 support)\n", i);
            return false;
        }
        for (auto &c : pl.cells)
            board[c.x * cols + c.y] = pl.shape_type + 1;
    }
    int sc = seqScore(board, rows, cols, cfg.reward_four_colors);
    if (sc != res.score)
    {
        printf("FAIL: score mismatch reported=%d recomputed=%d\n", res.score, sc);
        return false;
    }
    return true;
}

static void run_case(const char *name, SeqConfig cfg)
{
    SeqResult res = solveSequence(cfg);
    printf("=== %s ===\n", name);
    printf("placed=%d  score=%d\n", res.placed, res.score);
    render(res, cfg.board_rows, cfg.board_cols);
    bool ok = verify(res, cfg);
    printf("verify: %s\n\n", ok ? "OK" : "FAILED");
    assert(ok);
    assert(res.placed > 0);
}

int main()
{
    // 用例1：循环序列 0..6，库存各 5（共 35），竞赛规则③(下方支撑)。
    {
        SeqConfig cfg;
        cfg.sequence = {0, 1, 2, 3, 4, 5, 6};
        cfg.cyclic = true;
        cfg.inventory = {5, 5, 5, 5, 5, 5, 5};
        cfg.require_support = true;
        run_case("cyclic 0..6, inv=5 each, rule3 support", cfg);
    }
    // 用例2：同上，但用宽松实验模式(支撑或上方相连)。
    {
        SeqConfig cfg;
        cfg.sequence = {0, 1, 2, 3, 4, 5, 6};
        cfg.cyclic = true;
        cfg.inventory = {5, 5, 5, 5, 5, 5, 5};
        cfg.require_support = false;
        run_case("cyclic 0..6, inv=5 each, loose", cfg);
    }
    // 用例3：5 个田字(单一形状) + 规则③。可并排铺满最底两行，但规则④(整盘 <3 种形状)
    // 使其不计分 → score 必须为 0。
    {
        SeqConfig cfg;
        cfg.sequence = {1, 1, 1, 1, 1};
        cfg.cyclic = false;
        cfg.inventory = {0, 5, 0, 0, 0, 0, 0};
        cfg.require_support = true;
        SeqResult res = solveSequence(cfg);
        printf("=== finite squares x5, rule4 gate (single shape) ===\n");
        printf("placed=%d  score=%d\n", res.placed, res.score);
        render(res, cfg.board_rows, cfg.board_cols);
        bool ok = verify(res, cfg);
        printf("verify: %s\n\n", ok ? "OK" : "FAILED");
        assert(ok);
        assert(res.placed > 0);
        assert(res.score == 0); // 规则④：单一形状不计分
    }
    // 用例4：分组 + 终态支撑(新进阶模式)。先放完形状 0 的全部库存，再 1，再 2；无四色加分。
    // 校验：(a) 放置顺序按形状分组(0..0,1..1,2..2)；(b) 终态整盘每个已填格受支撑(触底或正下
    // 方有块)；(c) 支撑依赖 DAG 无环(可先底后顶执行)；(d) 计分自洽。
    {
        SeqConfig cfg;
        cfg.sequence = {0, 1, 2};
        cfg.cyclic = false;
        cfg.inventory = {4, 4, 4, 0, 0, 0, 0};
        cfg.group_by_shape = true;
        cfg.final_support = true;
        cfg.reward_four_colors = false;
        SeqResult res = solveSequence(cfg);
        printf("=== grouped + final-support (new advanced) ===\n");
        printf("placed=%d  score=%d\n", res.placed, res.score);
        render(res, cfg.board_rows, cfg.board_cols);
        assert(res.placed > 0);

        // (a) 分组：形状按 sequence 首次出现顺序非递减出现。
        int last_group = -1;
        int group_of[7];
        for (int i = 0; i < 7; ++i) group_of[i] = -1;
        for (size_t i = 0; i < cfg.sequence.size(); ++i)
            if (group_of[cfg.sequence[i]] < 0) group_of[cfg.sequence[i]] = (int)i;
        for (auto &pl : res.placements)
        {
            int g = group_of[pl.shape_type];
            assert(g >= 0 && g >= last_group && "placements must be grouped by shape");
            last_group = g;
        }

        // (b) 终态每格受支撑 + (d) 计分自洽。
        int rows = cfg.board_rows, cols = cfg.board_cols;
        vector<int> board(rows * cols, 0);
        for (auto &pl : res.placements)
            for (auto &c : pl.cells)
                board[c.x * cols + c.y] = pl.shape_type + 1;
        for (int r = 0; r < rows - 1; ++r)
            for (int c = 0; c < cols; ++c)
                assert(!(board[r * cols + c] && board[(r + 1) * cols + c] == 0) &&
                       "final board must be fully supported");
        assert(seqScore(board, rows, cols, cfg.reward_four_colors) == res.score);

        // (c) 支撑 DAG 无环：借 seqFinalValid(整盘支撑 + Kahn 可排完)复核。
        SeqState st;
        st.board = board;
        st.placements = res.placements;
        assert(seqFinalValid(st, rows, cols) && "support DAG must be acyclic & fully supported");
        printf("verify: OK (grouped, final-support, acyclic)\n\n");
    }

    // 用例5：分组 + 逐步支撑(新默认进阶模式)。先放完形状 0 全部库存再 1 再 2；每块放置当场即需
    // 支撑，故分组求解序本身就是先底后顶合法执行序(策略直接按此发布，不拓扑重排)。
    // 校验：(a) 分组顺序；(b) 沿放置顺序每块满足规则③支撑(verify)；(c) 计分自洽。
    {
        SeqConfig cfg;
        cfg.sequence = {0, 1, 2};
        cfg.cyclic = false;
        cfg.inventory = {4, 4, 4, 0, 0, 0, 0};
        cfg.group_by_shape = true;
        cfg.final_support = false;   // 逐步支撑
        cfg.require_support = true;  // 竞赛规则③
        cfg.reward_four_colors = false;
        SeqResult res = solveSequence(cfg);
        printf("=== grouped + step-support (new advanced default) ===\n");
        printf("placed=%d  score=%d\n", res.placed, res.score);
        render(res, cfg.board_rows, cfg.board_cols);
        assert(res.placed > 0);

        // (a) 分组：形状按 sequence 首次出现顺序非递减。
        int group_of[7];
        for (int i = 0; i < 7; ++i) group_of[i] = -1;
        for (size_t i = 0; i < cfg.sequence.size(); ++i)
            if (group_of[cfg.sequence[i]] < 0) group_of[cfg.sequence[i]] = (int)i;
        int last_group = -1;
        for (auto &pl : res.placements)
        {
            int g = group_of[pl.shape_type];
            assert(g >= 0 && g >= last_group && "placements must be grouped by shape");
            last_group = g;
        }
        // (b)+(c) 沿放置顺序逐块支撑合法 + 计分自洽。
        assert(verify(res, cfg) && "each block supported when placed (rule3) & score consistent");
        printf("verify: OK (grouped, step-support, order is executable as-is)\n\n");
    }

    printf("ALL TESTS PASSED\n");
    return 0;
}
