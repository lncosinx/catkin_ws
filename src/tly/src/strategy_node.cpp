#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <random>

using namespace std;

// ==============================================================================
// 核心数据结构与形状定义
// ==============================================================================
struct Point { int x, y; bool operator<(const Point& o) const { return y != o.y ? y < o.y : x < o.x; } };

struct ShapeVariant {
    vector<Point> coords;
    int real_way; // 0=0°, 1=90°, 2=180°, 3=270°
};

struct Action { int piece_id; int shape_type; int rot_idx; int real_way; int start_r, start_c; vector<Point> absolute_coords; };

struct PlanConfig {
    int target_blocks;
    int start_r;
    vector<Point> disabled_cells;
    int max_possible_score;
    int max_search_nodes;
    int max_restarts;
};

// 【积木形状定义省略内部...】
vector<vector<Point>> BASE_SHAPES = {
    {{0,0}, {1,0}, {2,0}, {3,0}}, // 0: 红色一字形
    {{0,0}, {1,0}, {0,1}, {1,1}}, // 1: 橙色田字形
    {{0,0}, {1,0}, {2,0}, {1,1}}, // 2: 棕色山字(T)
    {{0,0}, {0,1}, {0,2}, {1,2}}, // 3: 紫色L右
    {{1,0}, {1,1}, {1,2}, {0,2}}, // 4: 黄色L左
    {{0,0}, {1,0}, {1,1}, {2,1}}, // 5: 蓝色Z左
    {{1,0}, {2,0}, {0,1}, {1,1}}  // 6: 绿色Z右
};

// ==============================================================================
// DLX 极速精确覆盖引擎 (省略未改动的方法...)
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

    void init(int c_count, int max_sol, vector<int> p_prev, int limit_nodes = 5000000) {
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
        if (search_nodes_count++ > max_search_nodes) return; 
        if (nodes[0].right == 0) { all_solutions.push_back(current_ans); return; }
        
        int c = nodes[0].right;
        for (int i = nodes[0].right; i != 0; i = nodes[i].right)
            if (col_size[i] < col_size[c]) c = i;
            
        remove(c);
        for (int i = nodes[c].down; i != c; i = nodes[i].down) {
            int r = nodes[i].r; int p_id = row_to_piece[r];
            int prev_p = piece_prev[p_id];
            if (prev_p != -1 && !is_piece_used[prev_p]) continue; 
            
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
// 图形处理与规划引擎
// ==============================================================================
vector<Point> normalize(vector<Point> shape) {
    int min_x = 999, min_y = 999;
    for (auto& p : shape) { min_x = min(min_x, p.x); min_y = min(min_y, p.y); }
    vector<Point> norm; for (auto& p : shape) norm.push_back({p.x - min_x, p.y - min_y});
    sort(norm.begin(), norm.end(), [](const Point& a, const Point& b) {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });
    return norm;
}

vector<ShapeVariant> getUniqueRotations(vector<Point> base) {
    vector<ShapeVariant> unique_rots;
    for (int r = 0; r < 4; ++r) {
        vector<Point> rot;
        if (r == 0) rot = base;
        else if (r == 1) { for(auto& p : base) rot.push_back({-p.y, p.x}); }
        else if (r == 2) { for(auto& p : base) rot.push_back({-p.x, -p.y}); }
        else if (r == 3) { for(auto& p : base) rot.push_back({p.y, -p.x}); }
        
        vector<Point> norm = normalize(rot);
        
        bool is_dup = false;
        for (auto& existing : unique_rots) {
            if (existing.coords.size() == norm.size()) {
                bool match = true;
                for(size_t i=0; i<norm.size(); ++i) {
                    if(existing.coords[i].x != norm[i].x || existing.coords[i].y != norm[i].y) {
                        match = false; break;
                    }
                }
                if(match) { is_dup = true; break; }
            }
        }
        if (!is_dup) unique_rots.push_back({norm, r});
    }
    return unique_rots;
}

bool solveForMask(const PlanConfig& plan, const vector<int>& sub_inv, const int current_board[14][10], 
                  vector<Action>& out_actions, vector<int>& out_best_sol, int& out_best_score) {
    int total_pieces = plan.target_blocks;
    
    int cell_col_map[14][10];
    for(int r=0; r<14; ++r) for(int c=0; c<10; ++c) cell_col_map[r][c] = -1;

    int col_idx = total_pieces;
    for (int r = plan.start_r; r <= 13; ++r) {
        for (int c = 0; c < 10; ++c) {
            bool disabled = false;
            for (auto& p : plan.disabled_cells) {
                if (p.x == r && p.y == c) { disabled = true; break; }
            }
            if (!disabled) cell_col_map[r][c] = col_idx++;
        }
    }
    
    int col_count = col_idx;
    vector<vector<int>> matrix;
    vector<Action> all_actions;
    vector<int> piece_prev(total_pieces, -1);
    
    int p_id = 0;
    for (int type = 0; type < 7; ++type) {
        auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
        for (int inst = 0; inst < sub_inv[type]; ++inst) {
            if (inst > 0) piece_prev[p_id] = p_id - 1; 
            
            matrix.push_back({p_id});
            all_actions.push_back({p_id, type, -1, -1, -1, -1, {}});

            for (int rot_idx = 0; rot_idx < unique_rots.size(); ++rot_idx) {
                auto& variant = unique_rots[rot_idx];
                auto& shape = variant.coords;
                int max_x = 0, max_y = 0;
                for (auto& p : shape) { max_x = max(max_x, p.x); max_y = max(max_y, p.y); }
                
                int valid_height = 14 - plan.start_r;
                for (int local_r = 0; local_r <= valid_height - max_y - 1; ++local_r) {
                    for (int c = 0; c <= 10 - max_x - 1; ++c) {
                        bool valid = true;
                        vector<Point> abs_coords;
                        vector<int> row_cols = {p_id};
                        
                        int global_r = plan.start_r + local_r;
                        for (auto& p : shape) {
                            int nr = global_r + p.y;
                            int nc = c + p.x;
                            if (nr < plan.start_r || nr >= 14 || nc < 0 || nc >= 10 || current_board[nr][nc] != 0 || cell_col_map[nr][nc] == -1) {
                                valid = false; break;
                            }
                            abs_coords.push_back({nr, nc}); // x为行，y为列
                            row_cols.push_back(cell_col_map[nr][nc]);
                        }
                        if (valid) {
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

    for (int attempt = 0; attempt < plan.max_restarts; ++attempt) {
        vector<int> row_indices(matrix.size());
        for(int i=0; i<matrix.size(); ++i) row_indices[i] = i;
        shuffle(row_indices.begin(), row_indices.end(), rng);

        DLX* dlx = new DLX();
        dlx->init(col_count, max_sols, piece_prev, plan.max_search_nodes); 
        for (int i = 0; i < matrix.size(); ++i) {
            int idx = row_indices[i];
            dlx->addRow(idx, matrix[idx], all_actions[idx].piece_id);
        }
        dlx->search();

        if (!dlx->all_solutions.empty()) {
            for (const auto& sol : dlx->all_solutions) {
                int temp_board[14][10];
                for(int i=0; i<14; ++i) for(int j=0; j<10; ++j) temp_board[i][j] = current_board[i][j];
                
                for (int row_idx : sol) {
                    if (all_actions[row_idx].rot_idx == -1) continue;
                    for (const auto& pt : all_actions[row_idx].absolute_coords) {
                        temp_board[pt.x][pt.y] = all_actions[row_idx].shape_type + 1; 
                    }
                }
                
                int score = 0;
                for (int y = plan.start_r; y < 14; ++y) {
                    std::set<int> unique_colors;
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
                if (score > global_best_score_internal) { 
                    global_best_score_internal = score; 
                    out_best_sol = sol; 
                }
            }
            
            if (global_best_score_internal >= plan.max_possible_score) {
                out_actions = all_actions;
                out_best_score = global_best_score_internal;
                delete dlx;
                return true;
            }
        }
        delete dlx; 
    }

    if (global_best_score_internal != -1) {
        out_actions = all_actions;
        out_best_score = global_best_score_internal;
        return true;
    }
    return false;
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

struct BlockInfo { int u, v, ang; }; // 统一存放每个积木的物理属性

void visionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (is_planning) return; 
    if (msg->data.size() < 147) return;

    is_planning = true;
    ROS_INFO("===============================================================");
    ROS_INFO("=      Received Vision Data. Launching Dynamic Engine V3      =");
    ROS_INFO("===============================================================");

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
    
    // 【核心修复1】：按照 4 个元素 (shape, u, v, angle) 去解析视觉数据，防止数组错位！
    vector<BlockInfo> available_blocks[7];
    if (msg->data.size() > 147) {
        int num_blocks = msg->data[idx++];
        for (int i = 0; i < num_blocks && idx + 3 < msg->data.size(); ++i) {
            int shape = msg->data[idx++];
            int u = msg->data[idx++];
            int v = msg->data[idx++];
            int ang = msg->data[idx++]; // 读出绝对角度
            available_blocks[shape].push_back({u, v, ang});
        }
        ROS_INFO("Parsed true pixel coordinates AND angle for %d scattered blocks.", num_blocks);
    }

    if (total_blocks == 0) {
        ROS_WARN("No blocks recognized. Idle.");
        is_planning = false; return;
    }

    vector<PlanConfig> plans;
    if (total_blocks >= 34) {
        plans.push_back({34, 0, {{0,6},{0,7},{0,8},{0,9}}, 260, 5000000, 10}); 
    }
    
    int max_even_rows = (total_blocks * 4) / 10;
    if (max_even_rows > 12) max_even_rows = 12; 
    if (max_even_rows % 2 != 0) max_even_rows--;

    for (int r = max_even_rows; r >= 2; r -= 2) {
        plans.push_back({(r * 10) / 4, 14 - r, {}, r * 20, 1000000, 5});
    }

    int global_best_score = -1;
    vector<Action> global_best_actions;
    vector<int> global_best_solution;
    PlanConfig optimal_plan;

    for (const auto& plan : plans) {
        ROS_INFO(">>> 评估战略: 目标使用 %d 块, 起始行: %d, 理论极限得分: %d 分", plan.target_blocks, plan.start_r, plan.max_possible_score);
        
        vector<vector<int>> valid_subs;
        vector<int> current_sub;
        generateSubInventories(0, 0, plan.target_blocks, current_sub, current_inventory, valid_subs);
        
        int eval_limit = min((int)valid_subs.size(), 20);
        for (int i = 0; i < eval_limit; ++i) {
            vector<Action> temp_actions;
            vector<int> temp_sol;
            int current_score = 0;
            
            if (solveForMask(plan, valid_subs[i], current_board, temp_actions, temp_sol, current_score)) {
                if (current_score > global_best_score) {
                    global_best_score = current_score;
                    global_best_actions = temp_actions;
                    global_best_solution = temp_sol;
                    optimal_plan = plan;
                    if (global_best_score >= plan.max_possible_score) break; 
                }
            }
        }
        if (global_best_score != -1) {
            ROS_INFO("★ 战略锁定！成功找到得分高达 %d 分的完美解。", global_best_score);
            break; 
        }
    }

    if (global_best_score != -1) {
        vector<Action> final_actions;
        int used_count[7] = {0};
        for(int r : global_best_solution) {
            if (global_best_actions[r].rot_idx == -1) continue;
            final_actions.push_back(global_best_actions[r]);
            used_count[global_best_actions[r].shape_type]++;
        }

        int board_piece[14][10];
        for(int i=0; i<14; ++i) for(int j=0; j<10; ++j) board_piece[i][j] = current_board[i][j] > 0 ? -2 : -1; 
        for(const auto& act : final_actions) {
            for(auto pt : act.absolute_coords) board_piece[pt.x][pt.y] = act.piece_id;
        }

        int next_piece_id = 100; 
        for (int type = 0; type < 7; ++type) {
            int leftover = current_inventory[type] - used_count[type];
            for (int k = 0; k < leftover; ++k) {
                auto unique_rots = getUniqueRotations(BASE_SHAPES[type]);
                bool placed = false;
                for (int r = 13; r >= 0 && !placed; --r) {
                    for (int c = 0; c < 10 && !placed; ++c) {
                        for (int rot_idx = 0; rot_idx < unique_rots.size() && !placed; ++rot_idx) {
                            auto& variant = unique_rots[rot_idx];
                            auto& shape = variant.coords;
                            bool valid = true, fully_supported = true;
                            vector<Point> abs_coords;
                            for (auto& p : shape) {
                                int nr = r + p.y, nc = c + p.x;
                                if (nr < 0 || nr >= 14 || nc < 0 || nc >= 10 || board_piece[nr][nc] != -1) {
                                    valid = false; break;
                                }
                                if (nr < 13 && board_piece[nr+1][nc] == -1) fully_supported = false; 
                                abs_coords.push_back({nr, nc});
                            }
                            if (valid && fully_supported) {
                                Action act = {next_piece_id++, type, rot_idx, variant.real_way, r, c, abs_coords};
                                final_actions.push_back(act);
                                for (auto& pt : abs_coords) board_piece[pt.x][pt.y] = act.piece_id;
                                placed = true;
                            }
                        }
                    }
                }
            }
        }
                
        vector<int> adj[200]; int in_degree[200] = {0}; 
        for(int r=0; r<13; ++r) {
            for(int c=0; c<10; ++c) {
                int curr = board_piece[r][c];
                int below = board_piece[r+1][c];
                if(curr >= 0 && below >= 0 && curr != below) {
                    bool exist = false;
                    for(int t : adj[below]) if(t == curr) exist = true;
                    if(!exist) { adj[below].push_back(curr); in_degree[curr]++; }
                }
            }
        }
        
        int p_bottom[200] = {0};
        int p_left[200] = {0};
        for(const auto& act : final_actions) {
            int pid = act.piece_id;
            int max_r = -1;
            int min_c = 999;
            for(auto& pt : act.absolute_coords) {
                if(pt.x > max_r) max_r = pt.x; 
                if(pt.y < min_c) min_c = pt.y; 
            }
            p_bottom[pid] = max_r;
            p_left[pid] = min_c;
        }

        auto cmp = [&](int a, int b) {
            if (p_bottom[a] != p_bottom[b]) return p_bottom[a] < p_bottom[b]; 
            return p_left[a] > p_left[b]; 
        };
        
        priority_queue<int, vector<int>, decltype(cmp)> pq(cmp);
        for(const auto& act : final_actions) {
            int pid = act.piece_id;
            if(in_degree[pid] == 0) pq.push(pid);
        }
        
        vector<int> seq;
        while(!pq.empty()) {
            int u = pq.top(); pq.pop(); seq.push_back(u);
            for(int v : adj[u]) if(--in_degree[v] == 0) pq.push(v);
        }

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
                    int pu = 0, pv = 0, p_ang = 0;
                    if (!available_blocks[act.shape_type].empty()) {
                        pu = available_blocks[act.shape_type].back().u;
                        pv = available_blocks[act.shape_type].back().v;
                        p_ang = available_blocks[act.shape_type].back().ang;
                        available_blocks[act.shape_type].pop_back();
                    } else {
                        ROS_WARN("No pixel coordinate found for shape %d! Using 0,0.", act.shape_type);
                    }

                    // 【核心修复2】：重新引入 Center-to-Center！求放置网格的真实中心
                    int min_r = 999, max_r = -1, min_c = 999, max_c = -1;
                    for(auto& pt : act.absolute_coords) {
                        if(pt.x < min_r) min_r = pt.x;
                        if(pt.x > max_r) max_r = pt.x;
                        if(pt.y < min_c) min_c = pt.y;
                        if(pt.y > max_c) max_c = pt.y;
                    }
                    int center_r_x2 = min_r + max_r; 
                    int center_c_x2 = min_c + max_c;

                    plan_msg.data.push_back(act.shape_type); 
                    plan_msg.data.push_back(act.real_way);   
                    plan_msg.data.push_back(center_r_x2); // 不发左上角，改发包围盒中心行 * 2
                    plan_msg.data.push_back(center_c_x2); // 不发左上角，改发包围盒中心列 * 2
                    plan_msg.data.push_back(pu); 
                    plan_msg.data.push_back(pv); 
                    plan_msg.data.push_back(p_ang);       // 【补上丢失的角度】
                    
                    ROS_INFO("action [%2d/%lu]: select block => %-18s | place grid center(x2) => row:%2d col:%2d | pick_yaw:%3d", 
                             i + 1, seq.size(), SHAPE_NAMES[act.shape_type], center_r_x2, center_c_x2, p_ang);
                    break;
                }
            }
        }
        ROS_INFO("===============================================================");
        plan_pub.publish(plan_msg);
        ROS_INFO("[SUCCESS] Max Score Achieved: %d 分! Sent %lu blocks to execution.", global_best_score, seq.size());
    } else {
        ROS_WARN("Failed to find any valid placement plan with current blocks.");
    }
    
    is_planning = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tetris_strategy_node");
    ros::NodeHandle nh;

    plan_pub = nh.advertise<std_msgs::Int32MultiArray>("/tetris_plan", 10, true);
    
    // 【核心修复3】：把订阅频道改回正确的 /vision/board_state ！！！
    ros::Subscriber vision_sub = nh.subscribe("/vision/board_state", 1, visionCallback);

    ROS_INFO("Strategy Node Started. Awaiting Int32MultiArray on '/vision/board_state'...");

    ros::spin();
    return 0;
}