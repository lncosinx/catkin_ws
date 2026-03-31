#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <set>
#include <queue>
#include <algorithm>
#include <random>

using namespace std;

// ==============================================================================
// 核心数据结构与形状定义 (完全内嵌，告别外部 txt 文件)
// ==============================================================================
struct Point { int x, y; bool operator<(const Point& o) const { return y != o.y ? y < o.y : x < o.x; } };
struct Action { int piece_id; int shape_type; int rot_idx; int start_r, start_c; vector<Point> absolute_coords; };

// 7种基础形状(左上角对齐的原点相对坐标)
vector<vector<Point>> BASE_SHAPES = {
    {{0,0}, {1,0}, {2,0}, {3,0}}, // 0: 红色一字形
    {{0,0}, {1,0}, {0,1}, {1,1}}, // 1: 橙色田字形
    {{0,0}, {1,0}, {2,0}, {1,1}}, // 2: 棕色山字形 (T型)
    {{0,0}, {0,1}, {0,2}, {1,2}}, // 3: 紫色L字形右
    {{1,0}, {1,1}, {1,2}, {0,2}}, // 4: 黄色L字形左
    {{0,0}, {1,0}, {1,1}, {2,1}}, // 5: 蓝色Z字形左
    {{1,0}, {2,0}, {0,1}, {1,1}}  // 6: 绿色Z字形右
};

// ==============================================================================
// DLX 精确覆盖算法引擎
// ==============================================================================
const int MAX_NODES = 500000;
struct DLXNode { int r, c, up, down, left, right; };

class DLX {
public:
    DLXNode nodes[MAX_NODES];
    int col_size[200], head[200], node_count, col_count, max_solutions;
    vector<int> current_ans;
    vector<vector<int>> all_solutions;
    vector<int> row_to_piece, piece_prev; 
    bool is_piece_used[40];
    int search_nodes_count, max_search_nodes;

    void init(int c_count, int max_sol, vector<int> p_prev, int limit_nodes = 50000) {
        col_count = c_count; max_solutions = max_sol; piece_prev = p_prev;
        search_nodes_count = 0; max_search_nodes = limit_nodes; 
        for(int i=0; i<40; ++i) is_piece_used[i] = false;
        for (int i = 0; i <= col_count; ++i) {
            nodes[i].left = i - 1; nodes[i].right = i + 1;
            nodes[i].up = i; nodes[i].down = i; col_size[i] = 0; head[i] = i;
        }
        nodes[0].left = col_count; nodes[col_count].right = 0; node_count = col_count + 1;
        all_solutions.clear(); current_ans.clear(); row_to_piece.clear();
    }

    void addRow(int row_idx, const vector<int>& columns, int piece_id) {
        if (row_idx >= row_to_piece.size()) row_to_piece.resize(row_idx + 1, -1);
        row_to_piece[row_idx] = piece_id;
        int first_node = node_count;
        for (int i = 0; i < columns.size(); ++i) {
            int c = columns[i] + 1; int id = node_count++;
            nodes[id].r = row_idx; nodes[id].c = c;
            nodes[id].down = head[c]; nodes[id].up = nodes[head[c]].up;
            nodes[nodes[head[c]].up].down = id; nodes[head[c]].up = id; col_size[c]++;
            if (i == 0) { nodes[id].left = id; nodes[id].right = id; } 
            else {
                nodes[id].left = first_node; nodes[id].right = nodes[first_node].right;
                nodes[nodes[first_node].right].left = id; nodes[first_node].right = id;
            }
        }
    }

    void remove(int c) {
        nodes[nodes[c].right].left = nodes[c].left; nodes[nodes[c].left].right = nodes[c].right;
        for (int i = nodes[c].down; i != c; i = nodes[i].down)
            for (int j = nodes[i].right; j != i; j = nodes[j].right) {
                nodes[nodes[j].down].up = nodes[j].up; nodes[nodes[j].up].down = nodes[j].down; col_size[nodes[j].c]--;
            }
    }

    void resume(int c) {
        for (int i = nodes[c].up; i != c; i = nodes[i].up)
            for (int j = nodes[i].left; j != i; j = nodes[j].left) {
                nodes[nodes[j].down].up = j; nodes[nodes[j].up].down = j; col_size[nodes[j].c]++;
            }
        nodes[nodes[c].right].left = c; nodes[nodes[c].left].right = c;
    }

    void search() {
        if (all_solutions.size() >= max_solutions) return;
        if (search_nodes_count++ > max_search_nodes) return; // 防卡死熔断
        if (nodes[0].right == 0) { all_solutions.push_back(current_ans); return; }
        
        int c = nodes[0].right;
        for (int i = nodes[0].right; i != 0; i = nodes[i].right)
            if (col_size[i] < col_size[c]) c = i;
            
        remove(c);
        for (int i = nodes[c].down; i != c; i = nodes[i].down) {
            int r = nodes[i].r; int p_id = row_to_piece[r];
            int prev_p = piece_prev[p_id];
            if (prev_p != -1 && !is_piece_used[prev_p]) continue; // 强力对称性剪枝
            
            is_piece_used[p_id] = true;
            current_ans.push_back(r);
            for (int j = nodes[i].right; j != i; j = nodes[j].right) remove(nodes[j].c);
            search();
            for (int j = nodes[i].left; j != i; j = nodes[j].left) resume(nodes[j].c);
            current_ans.pop_back();
            is_piece_used[p_id] = false;
        }
        resume(c);
    }
};

// ==============================================================================
// 图形学处理与方案求解器
// ==============================================================================
vector<Point> normalize(vector<Point> shape) {
    int min_x = 999, min_y = 999;
    for (auto& p : shape) { min_x = min(min_x, p.x); min_y = min(min_y, p.y); }
    vector<Point> norm; for (auto& p : shape) norm.push_back({p.x - min_x, p.y - min_y});
    sort(norm.begin(), norm.end()); return norm;
}

vector<vector<Point>> getUniqueRotations(vector<Point> base) {
    set<vector<Point>> unique_rots; vector<Point> curr = base;
    for (int r = 0; r < 4; ++r) {
        vector<Point> rot; for (auto& p : curr) rot.push_back({-p.y, p.x});
        curr = normalize(rot); unique_rots.insert(curr);
    }
    return vector<vector<Point>>(unique_rots.begin(), unique_rots.end());
}

bool solveForSubInventory(int target_rows, const vector<int>& sub_inv, const int current_board[14][10], 
                          vector<Action>& out_actions, vector<int>& out_best_sol, int& out_best_score) {
    int total_pieces = 0;
    for (int count : sub_inv) total_pieces += count;

    int col_count = total_pieces + (target_rows * 10);
    vector<vector<int>> matrix;
    vector<Action> all_actions;
    vector<int> piece_prev(total_pieces, -1);
    
    int p_id = 0;
    for (int type = 0; type < 7; ++type) {
        auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
        for (int inst = 0; inst < sub_inv[type]; ++inst) {
            if (inst > 0) piece_prev[p_id] = p_id - 1; 
            
            // 虚拟行：丢弃动作
            matrix.push_back({p_id});
            all_actions.push_back({p_id, type, -1, -1, -1, {}});

            for (int rot_idx = 0; rot_idx < unique_rots.size(); ++rot_idx) {
                auto& shape = unique_rots[rot_idx];
                int max_x = 0, max_y = 0;
                for (auto& p : shape) { max_x = max(max_x, p.x); max_y = max(max_y, p.y); }
                
                for (int r = 0; r <= target_rows - max_y - 1; ++r) {
                    for (int c = 0; c <= 10 - max_x - 1; ++c) {
                        bool valid = true;
                        vector<Point> abs_coords;
                        vector<int> row_cols = {p_id};
                        
                        int real_r_start = 14 - target_rows + r; // 映射到底部坐标
                        for (auto& p : shape) {
                            int nr = real_r_start + p.y;
                            int nc = c + p.x;
                            // 检测是否越界或碰到视觉传递过来的已存在方块障碍物
                            if (nr < 0 || nr >= 14 || nc < 0 || nc >= 10 || current_board[nr][nc] != 0) {
                                valid = false; break;
                            }
                            abs_coords.push_back({nr, nc});
                            row_cols.push_back(total_pieces + ((nr - (14 - target_rows)) * 10 + nc));
                        }
                        if (valid) {
                            all_actions.push_back({p_id, type, rot_idx, real_r_start, c, abs_coords});
                            matrix.push_back(row_cols);
                        }
                    }
                }
            }
            p_id++;
        }
    }

    // 行洗牌打乱，极速跳跃出高分色块组合
    vector<int> row_indices(matrix.size());
    for(int i=0; i<matrix.size(); ++i) row_indices[i] = i;
    mt19937 rng(1337); 
    shuffle(row_indices.begin(), row_indices.end(), rng);

    DLX* dlx = new DLX();
    dlx->init(col_count, 100, piece_prev, 50000); 
    for (int i = 0; i < matrix.size(); ++i) {
        int idx = row_indices[i];
        dlx->addRow(idx, matrix[idx], all_actions[idx].piece_id);
    }
    dlx->search();

    if (dlx->all_solutions.empty()) {
        delete dlx; return false;
    }

    int best_score = -1;
    for (const auto& sol : dlx->all_solutions) {
        int temp_board[14][10];
        for(int i=0; i<14; ++i) for(int j=0; j<10; ++j) temp_board[i][j] = current_board[i][j];
        
        for (int row_idx : sol) {
            if (all_actions[row_idx].rot_idx == -1) continue;
            for (const auto& pt : all_actions[row_idx].absolute_coords) {
                // 已彻底修复此处：使用 pt.x 和 pt.y
                temp_board[pt.x][pt.y] = all_actions[row_idx].shape_type + 1; // 1-7代表颜色
            }
        }
        
        int score = 0;
        for (int y = 14 - target_rows; y < 14; ++y) {
            set<int> unique_colors;
            bool is_full = true;
            for (int x = 0; x < 10; ++x) {
                if (temp_board[y][x] == 0) is_full = false;
                else unique_colors.insert(temp_board[y][x]);
            }
            if (is_full) {
                score += 10; 
                if (unique_colors.size() >= 4) score += 10; 
            }
        }
        if (score > best_score) { best_score = score; out_best_sol = sol; }
    }

    out_actions = all_actions; out_best_score = best_score;
    delete dlx; return true;
}

void generateSubInventories(int type, int current_sum, int target_sum, vector<int>& current_sub, 
                            const vector<int>& max_inv, vector<vector<int>>& valid_subs) {
    if (type == 7) {
        if (current_sum == target_sum && current_sub[2] % 2 == 0) valid_subs.push_back(current_sub);
        return;
    }
    for (int take = 0; take <= max_inv[type]; ++take) {
        if (current_sum + take <= target_sum) {
            current_sub.push_back(take);
            generateSubInventories(type + 1, current_sum + take, target_sum, current_sub, max_inv, valid_subs);
            current_sub.pop_back();
        }
    }
}

// ==============================================================================
// ROS 节点逻辑
// ==============================================================================
ros::Publisher plan_pub;
bool is_planning = false;

void visionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (is_planning) return; // 防止连续回调阻塞
    if (msg->data.size() != 147) {
        ROS_ERROR("Vision data format error! Expected length 147, got %lu.", msg->data.size());
        return;
    }

    is_planning = true;
    ROS_INFO("Received vision data. Starting Dynamic Strategy V2...");

    vector<int> current_inventory(7, 0);
    int total_blocks = 0;
    for (int i = 0; i < 7; ++i) {
        current_inventory[i] = msg->data[i];
        total_blocks += current_inventory[i];
    }

    int current_board[14][10];
    int idx = 7;
    for (int r = 0; r < 14; ++r) {
        for (int c = 0; c < 10; ++c) {
            current_board[r][c] = msg->data[idx++];
        }
    }

    if (total_blocks == 0) {
        ROS_WARN("No blocks recognized. Idle.");
        is_planning = false;
        return;
    }

    int max_possible_rows = (total_blocks * 4) / 10;
    if (max_possible_rows % 2 != 0) max_possible_rows--;

    int global_best_score = -1;
    vector<Action> global_best_actions;
    vector<int> global_best_solution;
    int optimal_target_rows = 0;

    for (int target_rows = max_possible_rows; target_rows >= 2; target_rows -= 2) {
        int target_blocks = (target_rows * 10) / 4;
        vector<vector<int>> valid_subs;
        vector<int> current_sub;
        generateSubInventories(0, 0, target_blocks, current_sub, current_inventory, valid_subs);
        
        int eval_limit = min((int)valid_subs.size(), 20);
        for (int i = 0; i < eval_limit; ++i) {
            vector<Action> temp_actions;
            vector<int> temp_sol;
            int current_score = 0;
            
            if (solveForSubInventory(target_rows, valid_subs[i], current_board, temp_actions, temp_sol, current_score)) {
                if (current_score > global_best_score) {
                    global_best_score = current_score;
                    global_best_actions = temp_actions;
                    global_best_solution = temp_sol;
                    optimal_target_rows = target_rows;
                    if (global_best_score >= (target_rows * 20) - 10) break; // 提前熔断
                }
            }
        }
        if (global_best_score != -1) break; 
    }

    if (global_best_score != -1) {
        // 1. 提取完美矩形用到的核心方块
        vector<Action> final_actions;
        int used_count[7] = {0};
        for(int r : global_best_solution) {
            if (global_best_actions[r].rot_idx == -1) continue;
            final_actions.push_back(global_best_actions[r]);
            used_count[global_best_actions[r].shape_type]++;
        }

        // 2. 映射当前棋盘 (用于物理支撑和防碰撞检测)
        int board_piece[14][10];
        for(int i=0; i<14; ++i) for(int j=0; j<10; ++j) board_piece[i][j] = current_board[i][j] > 0 ? -2 : -1; 
        
        for(const auto& act : final_actions) {
            for(auto pt : act.absolute_coords) 
                board_piece[pt.x][pt.y] = act.piece_id;
        }

        // 3. 【核心提分补丁】：贪心放置剩余方块 (凑齐34块以触发比赛时间奖励)
        int next_piece_id = 100; // 给剩余方块分配独立的新ID，避免和前35个冲突
        for (int type = 0; type < 7; ++type) {
            int leftover = current_inventory[type] - used_count[type];
            for (int k = 0; k < leftover; ++k) {
                auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
                bool placed = false;
                
                // 从棋盘底部往上扫描，只要找到能放的地方就塞进去
                for (int r = 13; r >= 0 && !placed; --r) {
                    for (int c = 0; c < 10 && !placed; ++c) {
                        for (int rot_idx = 0; rot_idx < unique_rots.size() && !placed; ++rot_idx) {
                            auto& shape = unique_rots[rot_idx];
                            bool valid = true;
                            bool fully_supported = true;
                            vector<Point> abs_coords;
                            
                            for (auto& p : shape) {
                                int nr = r + p.y;
                                int nc = c + p.x;
                                // 检查碰撞与越界
                                if (nr < 0 || nr >= 14 || nc < 0 || nc >= 10 || board_piece[nr][nc] != -1) {
                                    valid = false; break;
                                }
                                // 检查绝对支撑：为了机械臂物理安全，要求每个悬空网格正下方都必须有方块 (!= -1 代表下方有自己或已有障碍物)
                                if (nr < 13 && board_piece[nr+1][nc] == -1) {
                                    fully_supported = false; 
                                }
                                abs_coords.push_back({nr, nc});
                            }
                            
                            // 找到合法且物理绝对安全的支撑点
                            if (valid && fully_supported) {
                                Action act = {next_piece_id++, type, rot_idx, r, c, abs_coords};
                                final_actions.push_back(act);
                                for (auto& pt : abs_coords) board_piece[pt.x][pt.y] = act.piece_id;
                                placed = true;
                            }
                        }
                    }
                }
            }
        }
                
        // 4. 生成符合重力规则的自底向上拓扑排序序列
        vector<int> adj[200]; int in_degree[200] = {0}; // 容量扩大以适应附加的方块ID (100+)
        for(int r=0; r<13; ++r) {
            for(int c=0; c<10; ++c) {
                int curr = board_piece[r][c];
                int below = board_piece[r+1][c];
                // 如果当前悬空块需要下方支撑，记录拓扑依赖关系
                if(curr >= 0 && below >= 0 && curr != below) {
                    bool exist = false;
                    for(int t : adj[below]) if(t == curr) exist = true;
                    if(!exist) { adj[below].push_back(curr); in_degree[curr]++; }
                }
            }
        }
        
        queue<int> q; vector<int> seq;
        for(const auto& act : final_actions) {
            int pid = act.piece_id;
            if(in_degree[pid] == 0) q.push(pid);
        }
        while(!q.empty()) {
            int u = q.front(); q.pop(); seq.push_back(u);
            for(int v : adj[u]) if(--in_degree[v] == 0) q.push(v);
        }

        // 5. 打包发送给控制节点
        std_msgs::Int32MultiArray plan_msg;
        plan_msg.data.push_back(seq.size());
        
        const char* SHAPE_NAMES[] = {
            "linear_red", "grid_orange", "T_shape_brown", 
            "L_right_purple", "L_left_yellow", "Z_left_blue", "Z_right_green"
        };

        ROS_INFO("====== Starting output of detailed pick-and-place sequence ======");
        for(int i=0; i<seq.size(); ++i) {
            for(const auto& act : final_actions) {
                if(act.piece_id == seq[i]) {
                    int shape = act.shape_type;
                    int rot = act.rot_idx;
                    int start_r = act.start_r;
                    int start_c = act.start_c;

                    plan_msg.data.push_back(shape); 
                    plan_msg.data.push_back(rot);   
                    plan_msg.data.push_back(start_r); 
                    plan_msg.data.push_back(start_c); 
                    
                    ROS_INFO("action [%2d/%lu]: select block => %-18s | place grid => row:%2d column:%2d | rotation => %3d 度", 
                             i + 1, seq.size(), SHAPE_NAMES[shape], start_r, start_c, rot * 90);
                    break;
                }
            }
        }
        ROS_INFO("===============================================================");
        
        plan_pub.publish(plan_msg);
        ROS_INFO("Plan SUCCESS! Perfect Rows: %d, Max Score: %d. Sent %lu steps (including leftovers) to Controller.", optimal_target_rows, global_best_score, seq.size());
    } else {
        ROS_WARN("Failed to find a valid placement plan with current blocks.");
    }
    
    is_planning = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tetris_strategy_node");
    ros::NodeHandle nh;

    plan_pub = nh.advertise<std_msgs::Int32MultiArray>("/tetris_plan", 10, true);
    ros::Subscriber vision_sub = nh.subscribe("/vision_data", 1, visionCallback);

    ROS_INFO("Strategy Node Started. Awaiting Int32MultiArray(size 147) on '/vision_data'...");

    ros::spin();
    return 0;
}