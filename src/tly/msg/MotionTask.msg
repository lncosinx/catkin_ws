# 一个抓放任务。pick_pose / place_pose 均为 base 系、已可直接 move_line 执行的位姿。
# 由“路径”节点完成像素->base 抓取(含深度Z)与格子->base 放置的全部坐标解算，
# 控制节点只需消费这些位姿，不再做任何坐标换算。
int32 shape
int32 way
geometry_msgs/Pose pick_pose      # 抓取下压目标 (base)
geometry_msgs/Pose place_pose     # 放置下压目标 (base)
