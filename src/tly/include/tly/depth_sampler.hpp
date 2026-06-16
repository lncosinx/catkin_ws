#pragma once

// 共享深度采样器：订阅 RealSense 对齐深度 (aligned_depth_to_color, 16UC1, 毫米)，
// 在给定像素邻域内取有效深度中值，返回相机系深度 Z（米）。
//
// 经典版与 DexiNed 版视觉节点共用此组件，避免重复实现。
//
// 重要（坐标系一致性）：realsense2_camera 的 aligned_depth_to_color 是在“彩色
// 原图(/color/image_raw)的坐标系与分辨率”下生成的，使用彩色内参（D4xx 彩色为
// (Modified) Brown-Conrady，可能带非零畸变）。而本节点检测用的是 image_rect_color
// (image_proc 去畸变后)。仅当彩色畸变系数 D≈0 时二者像素才一致。
// 因此采样深度前，节点需用 CameraModel::rectified_to_raw() 把检测得到的 rect 像素
// 映射回 raw 像素，再传入本类的 sampleZ()。本类只负责在给定像素处取深度。

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace tly
{

class DepthSampler
{
public:
    DepthSampler() = default;

    // 订阅深度话题；需在节点构造时调用一次。
    void init(ros::NodeHandle &nh, const std::string &topic)
    {
        sub_ = nh.subscribe(topic, 1, &DepthSampler::cb, this);
    }

    // 是否已收到至少一帧深度。
    bool ready() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return !depth_.empty();
    }

    // 在 (u,v) 周围 radius 像素的方形邻域内取非零深度中值。
    // 成功返回 true 并写 z_m（米）；邻域内无有效深度返回 false。
    bool sampleZ(int u, int v, int radius, double &z_m) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (depth_.empty())
            return false;

        std::vector<uint16_t> vals;
        const int r = std::max(0, radius);
        for (int dy = -r; dy <= r; ++dy)
        {
            const int y = v + dy;
            if (y < 0 || y >= depth_.rows)
                continue;
            const uint16_t *row = depth_.ptr<uint16_t>(y);
            for (int dx = -r; dx <= r; ++dx)
            {
                const int x = u + dx;
                if (x < 0 || x >= depth_.cols)
                    continue;
                const uint16_t d = row[x];
                if (d > 0) // 0 表示无效/空洞
                    vals.push_back(d);
            }
        }
        if (vals.empty())
            return false;

        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        z_m = static_cast<double>(vals[vals.size() / 2]) / 1000.0; // mm -> m
        return true;
    }

private:
    void cb(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv::Mat d = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            std::lock_guard<std::mutex> lk(mtx_);
            d.copyTo(depth_); // 拷贝，避免底层缓冲被下一帧覆盖
        }
        catch (const cv_bridge::Exception &)
        {
            // 编码不符等异常静默忽略，保持节点存活
        }
    }

    ros::Subscriber sub_;
    mutable std::mutex mtx_;
    cv::Mat depth_; // 16UC1，毫米
};

} // namespace tly
