# 一个抓放任务。所有位姿均为 base 系、已可直接 move_line 执行。
# 由“路径”节点（path_planner_node）完成像素->base 抓取(含深度Z)、格子->base 放置、
# 腕部 yaw 翻转决策与沿 board 法向的悬停解算的全部坐标计算；控制节点（纯执行器）
# 只消费这些位姿，不再做任何坐标换算或腕部决策。
int32 shape
int32 way
geometry_msgs/Pose pick_pose          # 抓取下压目标 (base)
geometry_msgs/Pose pick_hover_pose    # 抓取上方悬停 (base, 沿 board 法向抬升)
geometry_msgs/Pose place_pose         # 放置下压目标 (base)
geometry_msgs/Pose place_hover_pose   # 放置上方悬停 (base, 沿 board 法向抬升)
