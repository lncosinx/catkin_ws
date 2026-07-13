// xArm6 正/逆运动学（仅依赖 tf2，无 MoveIt）。
//
// 用途：path_planner_node 把每个 base 系 TCP 位姿转成 6 个关节角 q，用于沿 move_line 直线
// 路径积分的"运动时间"代价（见 path_planner 的 evalTransit），并据此联合优化抓放顺序、同形状
// 抓取分配与腕部 180° 翻转。
//
// 连杆几何取自 xarm_description/xarm6_full.urdf 的关节原点/轴（已核对）：
//   j1 z=0.267 | j2 rpy=(-π/2) | j3 xyz=(0.0535,-0.2845) | j4 xyz=(0.0775,0.3425) rpy=(-π/2)
//   | j5 rpy=(π/2) | j6 xyz=(0.076,0.097) rpy=(-π/2) | link_tcp z=0.061
// 每个关节： T_i(q) = preTf(i) · Rz(q_i)，FK = T0·…·T5·toolTf，得 base ← link_tcp。
//
// FK 是精确链乘。IK 用阻尼最小二乘（DLS / Levenberg–Marquardt），从 seed（上一路点或测量
// 关节）出发迭代——seed 既保证收敛，又把解锁在 seed 所在分支上，复刻 xArm 固件笛卡尔指令的
// "就近解" 行为（尤其 J4/J6 圈数连续）。这样 6 个关节全精确、分支稳定，且无需手推偏置腕闭式解。
//
// 注：FK 目标系是 URDF 的 link_tcp，与 robot_state_publisher 发布的 base→link_tcp TF 完全一致，
// 故 path_planner 可用 fk(measured_q) 对照 TF 做启动自检。命令 TCP 若另带工具偏置，与 link_tcp
// 仅差沿工具轴的常量，对 J1/J6 主导的关节代价影响可忽略。
#pragma once

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <array>
#include <cmath>

namespace lucky
{

    class XArm6Kinematics
    {
    public:
        using Joints = std::array<double, 6>;

        // 正运动学：给定关节角 q，返回 base ← link_tcp 位姿。
        tf2::Transform fk(const Joints &q) const
        {
            tf2::Transform frames[6];
            return fkFrames(q, frames);
        }

        // 逆运动学：求达到 target(base ← link_tcp) 的关节角，从 seed 出发（就近解分支）。
        // 收敛返回 true 并写 q_out；不收敛/越硬限返回 false。
        bool ik(const tf2::Transform &target, const Joints &seed, Joints &q_out) const
        {
            Joints q = seed;
            const tf2::Vector3 p_t = target.getOrigin();
            const tf2::Matrix3x3 R_t(target.getRotation());

            for (int iter = 0; iter < max_iters_; ++iter)
            {
                tf2::Transform frames[6];
                tf2::Transform cur = fkFrames(q, frames);

                // 6 维位姿误差：平移 + 旋转向量（base 系）。
                tf2::Vector3 ep = p_t - cur.getOrigin();
                tf2::Vector3 er = rotError(R_t, tf2::Matrix3x3(cur.getRotation()));
                double e[6] = {ep.x(), ep.y(), ep.z(), er.x(), er.y(), er.z()};

                if (ep.length() < tol_pos_ && er.length() < tol_rot_)
                {
                    q_out = q;
                    return withinHardLimits(q);
                }

                // 几何雅可比 J(6x6)：列 i = [z_i × (p_tcp − p_i); z_i]。
                double J[6][6];
                const tf2::Vector3 p_tcp = cur.getOrigin();
                for (int i = 0; i < 6; ++i)
                {
                    tf2::Vector3 z_i = frames[i].getBasis().getColumn(2);
                    tf2::Vector3 p_i = frames[i].getOrigin();
                    tf2::Vector3 lin = z_i.cross(p_tcp - p_i);
                    J[0][i] = lin.x();
                    J[1][i] = lin.y();
                    J[2][i] = lin.z();
                    J[3][i] = z_i.x();
                    J[4][i] = z_i.y();
                    J[5][i] = z_i.z();
                }

                // A = JᵀJ + λ²I ; b = Jᵀe ; 解 A·dq = b。
                double A[6][6], b[6];
                for (int a = 0; a < 6; ++a)
                {
                    double bs = 0.0;
                    for (int k = 0; k < 6; ++k)
                        bs += J[k][a] * e[k];
                    b[a] = bs;
                    for (int c = 0; c < 6; ++c)
                    {
                        double s = 0.0;
                        for (int k = 0; k < 6; ++k)
                            s += J[k][a] * J[k][c];
                        A[a][c] = s + (a == c ? lambda_ * lambda_ : 0.0);
                    }
                }

                double dq[6];
                if (!solve6(A, b, dq))
                    return false;

                // 限步，避免大跨步越过解。
                double m = 0.0;
                for (int i = 0; i < 6; ++i)
                    m = std::max(m, std::fabs(dq[i]));
                double scale = (m > max_step_) ? max_step_ / m : 1.0;
                for (int i = 0; i < 6; ++i)
                    q[i] += dq[i] * scale;
            }
            return false;
        }

        // URDF 硬限位（rad）。J1/J4/J6 ±2π；J2/J3/J5 见 urdf。
        static const Joints &lowerLimits()
        {
            static const Joints lo = {-6.283185307179586, -2.059, -3.927,
                                      -6.283185307179586, -1.69297, -6.283185307179586};
            return lo;
        }
        static const Joints &upperLimits()
        {
            static const Joints hi = {6.283185307179586, 2.0944, 0.19198,
                                      6.283185307179586, 3.141592653589793, 6.283185307179586};
            return hi;
        }

        bool withinHardLimits(const Joints &q, double margin = 0.0) const
        {
            const Joints &lo = lowerLimits();
            const Joints &hi = upperLimits();
            for (int i = 0; i < 6; ++i)
                if (q[i] < lo[i] - margin || q[i] > hi[i] + margin)
                    return false;
            return true;
        }

    private:
        int max_iters_ = 80;
        double tol_pos_ = 1e-5;  // m
        double tol_rot_ = 1e-5;  // rad
        double lambda_ = 0.05;   // DLS 阻尼
        double max_step_ = 0.30; // 每迭代最大关节步长 (rad)

        static tf2::Transform makeRot(double r, double p, double y)
        {
            tf2::Quaternion q;
            q.setRPY(r, p, y);
            return tf2::Transform(q, tf2::Vector3(0, 0, 0));
        }
        static tf2::Transform makeTrans(double x, double y, double z)
        {
            return tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(x, y, z));
        }

        // 关节 i 的固定（旋转前）变换：T_i(q) = preTf(i) · Rz(q_i)。
        static const tf2::Transform &preTf(int i)
        {
            static const double PI2 = M_PI / 2.0;
            static const tf2::Transform pre[6] = {
                makeTrans(0.0, 0.0, 0.267),
                makeRot(-PI2, 0.0, 0.0),
                makeTrans(0.0535, -0.2845, 0.0),
                makeTrans(0.0775, 0.3425, 0.0) * makeRot(-PI2, 0.0, 0.0),
                makeRot(PI2, 0.0, 0.0),
                makeTrans(0.076, 0.097, 0.0) * makeRot(-PI2, 0.0, 0.0)};
            return pre[i];
        }
        static const tf2::Transform &toolTf()
        {
            static const tf2::Transform t = makeTrans(0.0, 0.0, 0.061); // link6 → link_tcp
            return t;
        }

        // FK 同时输出每个关节旋转后的 base← 帧（供雅可比用），返回 base← link_tcp。
        tf2::Transform fkFrames(const Joints &q, tf2::Transform frames[6]) const
        {
            tf2::Transform T;
            T.setIdentity();
            for (int i = 0; i < 6; ++i)
            {
                tf2::Quaternion rz;
                rz.setRPY(0, 0, q[i]);
                T = T * preTf(i) * tf2::Transform(rz, tf2::Vector3(0, 0, 0));
                frames[i] = T;
            }
            return T * toolTf();
        }

        // R_err = R_target · R_curᵀ 的旋转向量（轴×角，base 系）。seeded 迭代下角度小，公式稳。
        static tf2::Vector3 rotError(const tf2::Matrix3x3 &R_t, const tf2::Matrix3x3 &R_c)
        {
            tf2::Matrix3x3 Re = R_t * R_c.transpose();
            double tr = Re[0][0] + Re[1][1] + Re[2][2];
            double cos_a = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
            double a = std::acos(cos_a);
            if (a < 1e-9)
                return tf2::Vector3(0, 0, 0);
            double s = 2.0 * std::sin(a);
            tf2::Vector3 axis((Re[2][1] - Re[1][2]) / s,
                              (Re[0][2] - Re[2][0]) / s,
                              (Re[1][0] - Re[0][1]) / s);
            return axis * a;
        }

        // 6x6 线性方程组高斯消元（部分主元）。奇异返回 false。
        static bool solve6(double A[6][6], const double b_in[6], double x[6])
        {
            double M[6][7];
            for (int i = 0; i < 6; ++i)
            {
                for (int j = 0; j < 6; ++j)
                    M[i][j] = A[i][j];
                M[i][6] = b_in[i];
            }
            for (int col = 0; col < 6; ++col)
            {
                int piv = col;
                double best = std::fabs(M[col][col]);
                for (int r = col + 1; r < 6; ++r)
                    if (std::fabs(M[r][col]) > best)
                    {
                        best = std::fabs(M[r][col]);
                        piv = r;
                    }
                if (best < 1e-12)
                    return false;
                if (piv != col)
                    for (int j = 0; j < 7; ++j)
                        std::swap(M[col][j], M[piv][j]);
                double d = M[col][col];
                for (int j = col; j < 7; ++j)
                    M[col][j] /= d;
                for (int r = 0; r < 6; ++r)
                {
                    if (r == col)
                        continue;
                    double f = M[r][col];
                    if (f == 0.0)
                        continue;
                    for (int j = col; j < 7; ++j)
                        M[r][j] -= f * M[col][j];
                }
            }
            for (int i = 0; i < 6; ++i)
                x[i] = M[i][6];
            return true;
        }
    };

} // namespace lucky
