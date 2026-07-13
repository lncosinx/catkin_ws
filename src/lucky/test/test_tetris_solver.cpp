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
    printf("ALL TESTS PASSED\n");
    return 0;
}
