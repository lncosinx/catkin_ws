#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>

// 替换为你的服务头文件
#include <tly/GetPrecisePose.h> 

#include <vector>
#include <deque>
#include <map>
#include <mutex>
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace std;
using namespace cv;

// --- 全局常量定义 ---
const map<int, string> SHAPE_NAMES = {
    {0, "line"}, {1, "square"}, {2, "T"}, {3, "L_left"}, 
    {4, "L_right"}, {5, "Z_left"}, {6, "Z_right"}
};

const map<int, vector<Point>> BASE_SHAPES = {
    {0, {{0,0}, {1,0}, {2,0}, {3,0}}},
    {1, {{0,0}, {1,0}, {0,1}, {1,1}}},
    {2, {{0,0}, {1,0}, {2,0}, {1,1}}},
    {3, {{1,0}, {1,1}, {1,2}, {0,2}}},
    {4, {{0,0}, {0,1}, {0,2}, {1,2}}},
    {5, {{0,0}, {1,0}, {1,1}, {2,1}}},
    {6, {{1,0}, {2,0}, {0,1}, {1,1}}}
};

const map<int, Scalar> DRAW_COLORS = {
    {0, Scalar(0, 0, 255)},   {1, Scalar(0, 165, 255)}, {2, Scalar(42, 42, 165)},
    {3, Scalar(255, 0, 255)}, {4, Scalar(0, 255, 255)}, {5, Scalar(255, 0, 0)},
    {6, Scalar(0, 255, 0)}
};

// --- 基础数据结构 ---
struct Detection {
    int shape_id;
    string name;
    vector<Point> contour;
    double area;
    Point2f geom_px;
    Point2f pick_px;
    double angle_deg;
    double score;
    double iou;
    double stamp;
    Point3f geom_table;
    Point3f pick_table;
    bool has_table_points = false;
};

// --- 数学与辅助工具 ---
namespace utils {
    double meanPeriodicAngle180(const vector<double>& angles) {
        if (angles.empty()) return 0.0;
        double s = 0, c = 0;
        for (double a : angles) {
            double a2 = a * 2.0 * M_PI / 180.0;
            s += sin(a2);
            c += cos(a2);
        }
        double res = atan2(s, c) * 180.0 / M_PI / 2.0;
        return fmod(res + 360.0, 180.0);
    }

    double angleStdDevDeg(const vector<double>& angles) {
        if (angles.size() < 2) return numeric_limits<double>::infinity();
        double mean = meanPeriodicAngle180(angles) * 2.0 * M_PI / 180.0;
        vector<double> diffs;
        double sum_sq = 0;
        for (double a : angles) {
            double a2 = a * 2.0 * M_PI / 180.0;
            double d = atan2(sin(a2 - mean), cos(a2 - mean));
            sum_sq += d * d;
        }
        return sqrt(sum_sq / angles.size()) * 180.0 / M_PI / 2.0;
    }

    Point2f medianPoint2f(vector<Point2f> pts) {
        if (pts.empty()) return Point2f(0, 0);
        vector<float> xs, ys;
        for (auto& p : pts) { xs.push_back(p.x); ys.push_back(p.y); }
        nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
        nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
        return Point2f(xs[xs.size() / 2], ys[ys.size() / 2]);
    }

    Point3f medianPoint3f(vector<Point3f> pts) {
        if (pts.empty()) return Point3f(0, 0, 0);
        vector<float> xs, ys, zs;
        for (auto& p : pts) { xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z); }
        nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
        nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
        nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
        return Point3f(xs[xs.size() / 2], ys[ys.size() / 2], zs[zs.size() / 2]);
    }
}

// --- 追踪器 ---
class BlockTrack {
public:
    int track_id;
    int shape_id;
    string name;
    deque<Detection> history;
    int history_len;
    int missed;

    BlockTrack(int id, const Detection& det, int h_len) 
        : track_id(id), history_len(h_len), missed(0) {
        update(det);
    }

    void update(const Detection& det) {
        shape_id = det.shape_id;
        name = det.name;
        if (history.size() >= history_len) history.pop_front();
        history.push_back(det);
        missed = 0;
    }

    void mark_missed() { missed++; }
    int count() const { return history.size(); }

    Point2f stable_px_geom() const {
        vector<Point2f> pts; for (auto& d : history) pts.push_back(d.geom_px);
        return utils::medianPoint2f(pts);
    }

    Point2f stable_px_pick() const {
        vector<Point2f> pts; for (auto& d : history) pts.push_back(d.pick_px);
        return utils::medianPoint2f(pts);
    }

    Point3f stable_table_pick() const {
        vector<Point3f> pts; 
        for (auto& d : history) if(d.has_table_points) pts.push_back(d.pick_table);
        return utils::medianPoint3f(pts);
    }

    bool has_stable_table_pick() const {
        for (auto& d : history) if(d.has_table_points) return true;
        return false;
    }

    double stable_angle() const {
        vector<double> angs; for (auto& d : history) angs.push_back(d.angle_deg);
        return utils::meanPeriodicAngle180(angs);
    }

    double stable_score() const {
        vector<double> scores; for (auto& d : history) scores.push_back(d.score);
        nth_element(scores.begin(), scores.begin() + scores.size() / 2, scores.end());
        return scores[scores.size() / 2];
    }

    double position_std_px() const {
        if (history.size() < 2) return numeric_limits<double>::infinity();
        Point2f mean(0,0);
        for(auto& d : history) mean += d.pick_px;
        mean.x /= history.size(); mean.y /= history.size();
        double var_x = 0, var_y = 0;
        for(auto& d : history) {
            var_x += pow(d.pick_px.x - mean.x, 2);
            var_y += pow(d.pick_px.y - mean.y, 2);
        }
        return sqrt((var_x + var_y) / history.size());
    }

    double angle_std_deg() const {
        vector<double> angs; for (auto& d : history) angs.push_back(d.angle_deg);
        return utils::angleStdDevDeg(angs);
    }

    bool is_stable(int min_frames, double max_px_std, double max_angle_std) const {
        return count() >= min_frames && missed == 0 && 
               position_std_px() <= max_px_std && angle_std_deg() <= max_angle_std;
    }
};

// --- 相机模型 ---
class CameraModel {
public:
    bool ready = false;
    string frame_id;
    Mat K, D, P;
    bool use_rectified = true;
    mutex mtx;

    void update(const sensor_msgs::CameraInfoConstPtr& msg, bool prefer_rect) {
        lock_guard<mutex> lock(mtx);
        frame_id = msg->header.frame_id;
        K = Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
        D = msg->D.empty() ? Mat::zeros(5, 1, CV_64F) : Mat(msg->D).clone();
        P = Mat(3, 4, CV_64F, (void*)msg->P.data()).clone();
        use_rectified = prefer_rect && abs(P.at<double>(0,0)) > 1e-9;
        ready = true;
    }

    bool get_intrinsics(double& fx, double& fy, double& cx, double& cy) {
        lock_guard<mutex> lock(mtx);
        if (!ready) return false;
        if (use_rectified) {
            fx = P.at<double>(0,0); fy = P.at<double>(1,1);
            cx = P.at<double>(0,2); cy = P.at<double>(1,2);
        } else {
            fx = K.at<double>(0,0); fy = K.at<double>(1,1);
            cx = K.at<double>(0,2); cy = K.at<double>(1,2);
        }
        return true;
    }

    Point3f pixel_to_ray(double u, double v) {
        lock_guard<mutex> lock(mtx);
        if (use_rectified) {
            double fx = P.at<double>(0,0), fy = P.at<double>(1,1);
            double cx = P.at<double>(0,2), cy = P.at<double>(1,2);
            return Point3f((u - cx)/fx, (v - cy)/fy, 1.0);
        } else {
            Mat pt(1, 1, CV_64FC2, Scalar(u, v));
            Mat undistorted;
            undistortPoints(pt, undistorted, K, D);
            return Point3f(undistorted.at<Vec2d>(0,0)[0], undistorted.at<Vec2d>(0,0)[1], 1.0);
        }
    }
};

// --- 模板匹配库 ---
class TemplateBank {
public:
    int canvas_size, angle_step, refine_step, cell_size, margin;
    struct Tmpl { int sid; double angle; Mat mask; };
    vector<Tmpl> coarse_templates;

    TemplateBank(int c_size=96, int a_step=5, int r_step=1) 
        : canvas_size(c_size), angle_step(a_step), refine_step(r_step), cell_size(20), margin(14) {
        for (auto const& [sid, pts] : BASE_SHAPES) {
            for (int angle = 0; angle < 180; angle += angle_step) {
                coarse_templates.push_back({sid, (double)angle, render_shape(sid, angle)});
            }
        }
    }

    Mat render_shape(int sid, double angle_deg) {
        auto raw = BASE_SHAPES.at(sid);
        int min_x = 999, min_y = 999, max_x = -999, max_y = -999;
        for (auto& p : raw) {
            min_x = min(min_x, p.x); min_y = min(min_y, p.y);
            max_x = max(max_x, p.x); max_y = max(max_y, p.y);
        }
        int w = (max_x - min_x + 1) * cell_size + 2 * margin;
        int h = (max_y - min_y + 1) * cell_size + 2 * margin;
        Mat base = Mat::zeros(h, w, CV_8UC1);
        for (auto& p : raw) {
            int x0 = margin + (p.x - min_x) * cell_size;
            int y0 = margin + (p.y - min_y) * cell_size;
            rectangle(base, Rect(x0, y0, cell_size, cell_size), Scalar(255), FILLED);
        }
        morphologyEx(base, base, MORPH_CLOSE, Mat::ones(3, 3, CV_8UC1));

        int diag = ceil(sqrt(h*h + w*w)) + 8;
        Mat canvas = Mat::zeros(diag, diag, CV_8UC1);
        base.copyTo(canvas(Rect((diag-w)/2, (diag-h)/2, w, h)));
        
        Mat rot_mat = getRotationMatrix2D(Point2f(diag/2.0, diag/2.0), angle_deg, 1.0);
        Mat rot;
        warpAffine(canvas, rot, rot_mat, Size(diag, diag), INTER_NEAREST);
        return normalize_binary_mask(rot, canvas_size);
    }

    static Mat normalize_binary_mask(const Mat& mask, int size) {
        vector<Point> pts;
        findNonZero(mask, pts);
        if (pts.empty()) return Mat::zeros(size, size, CV_8UC1);
        Rect bb = boundingRect(pts);
        Mat crop = mask(bb);
        double scale = min((size - 10) / (double)max(bb.width, 1), (size - 10) / (double)max(bb.height, 1));
        int nw = max(1, (int)round(bb.width * scale));
        int nh = max(1, (int)round(bb.height * scale));
        Mat resized;
        resize(crop, resized, Size(nw, nh), 0, 0, INTER_NEAREST);
        Mat out = Mat::zeros(size, size, CV_8UC1);
        resized.copyTo(out(Rect((size - nw) / 2, (size - nh) / 2, nw, nh)));
        threshold(out, out, 0, 255, THRESH_BINARY); // 确保严格二值化
        return out;
    }

    tuple<int, double, double> match(const Mat& mask_norm) {
        int best_sid = -1;
        double best_angle = 0;
        double best_iou = -1.0;

        // CPU 上计算 96x96 的 IoU 速度极快，无需上 GPU
        for (auto& tmpl : coarse_templates) {
            double iou = binary_iou(mask_norm, tmpl.mask);
            if (iou > best_iou) { best_iou = iou; best_sid = tmpl.sid; best_angle = tmpl.angle; }
        }

        // 精细调整
        double ref_angle = best_angle;
        for (int angle = best_angle - angle_step; angle <= best_angle + angle_step; angle += refine_step) {
            int a = (angle + 180) % 180;
            Mat tmpl = render_shape(best_sid, a);
            double iou = binary_iou(mask_norm, tmpl);
            if (iou > best_iou) { best_iou = iou; ref_angle = a; }
        }
        return {best_sid, fmod(ref_angle, 180.0), best_iou};
    }

    static double binary_iou(const Mat& a, const Mat& b) {
        Mat inter, uni;
        bitwise_and(a, b, inter);
        bitwise_or(a, b, uni);
        int u_px = countNonZero(uni);
        if (u_px == 0) return 0.0;
        return (double)countNonZero(inter) / u_px;
    }
};

// --- 主 ROS 节点类 ---
class VisionProcessorNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    
    // 参数
    string image_topic, camera_info_topic, table_frame, camera_frame_override;
    bool assume_rectified;
    double target_plane_z, hover_z;
    
    int scale_percent;
    int roi_x_min, roi_y_min, roi_x_max, roi_y_max;
    string threshold_mode;
    int manual_dark_threshold, blur_kernel, close_kernel, open_kernel;
    double min_area, max_area, min_template_iou, min_fill, max_fill;
    
    string pick_point_mode;
    vector<int> distance_pick_shapes;
    int template_size, template_angle_step, template_refine_step;
    
    int track_history_len, stable_min_frames, max_missed_frames;
    double track_match_gate_px, track_match_gate_m, stable_max_px_std, stable_max_angle_std_deg;
    bool publish_only_stable;
    
    double precise_timeout, precise_center_gate_px;
    bool precise_require_stable;

    // 发布/订阅/服务
    ros::Subscriber info_sub;
    image_transport::Subscriber img_sub;
    ros::Publisher state_pub;
    image_transport::Publisher debug_pub, fg_pub, edges_pub;
    ros::Publisher pose_array_pub;
    ros::ServiceServer precise_srv;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    CameraModel cam_model;
    TemplateBank* templates;

    mutex state_mtx;
    Mat latest_frame;
    std_msgs::Header latest_header;
    vector<BlockTrack> tracks;
    int next_track_id = 1;

    // CUDA 滤镜指针 (重用以避免重复分配)
    Ptr<cuda::Filter> cuda_blur_filter;
    Ptr<cuda::Filter> cuda_close_filter;
    Ptr<cuda::Filter> cuda_open_filter;

public:
    VisionProcessorNode() : pnh_("~"), it_(nh_), tf_listener(tf_buffer) {
        // 加载参数
        pnh_.param("image_topic", image_topic, string("/camera/color/image_rect_color"));
        pnh_.param("camera_info_topic", camera_info_topic, string("/camera/color/camera_info"));
        pnh_.param("assume_image_rectified", assume_rectified, true);
        pnh_.param("table_frame", table_frame, string("table_frame"));
        pnh_.param("camera_frame", camera_frame_override, string(""));
        
        nh_.param("/tetris/PICK_Z", target_plane_z, 0.0);
        nh_.param("/tetris/HOVER_Z", hover_z, 0.10);
        pnh_.param("target_plane_z", target_plane_z, target_plane_z);
        pnh_.param("hover_z", hover_z, hover_z);

        pnh_.param("scale_percent", scale_percent, 100);
        pnh_.param("roi_x_min", roi_x_min, 0); pnh_.param("roi_y_min", roi_y_min, 0);
        pnh_.param("roi_x_max", roi_x_max, 0); pnh_.param("roi_y_max", roi_y_max, 0);
        
        pnh_.param("threshold_mode", threshold_mode, string("otsu"));
        pnh_.param("manual_dark_threshold", manual_dark_threshold, 105);
        pnh_.param("blur_kernel", blur_kernel, 5);
        pnh_.param("close_kernel", close_kernel, 3);
        pnh_.param("open_kernel", open_kernel, 3);
        
        pnh_.param("min_area", min_area, 900.0);
        pnh_.param("max_area", max_area, 50000.0);
        pnh_.param("min_template_iou", min_template_iou, 0.42);
        pnh_.param("max_contour_fill_ratio", max_fill, 0.98);
        pnh_.param("min_contour_fill_ratio", min_fill, 0.18);
        
        pnh_.param("pick_point_mode", pick_point_mode, string("centroid"));
        pnh_.param("template_size", template_size, 96);
        pnh_.param("template_angle_step", template_angle_step, 5);
        pnh_.param("template_refine_step", template_refine_step, 1);

        pnh_.param("track_history_len", track_history_len, 9);
        pnh_.param("stable_min_frames", stable_min_frames, 5);
        pnh_.param("max_missed_frames", max_missed_frames, 8);
        pnh_.param("track_match_gate_px", track_match_gate_px, 55.0);
        pnh_.param("track_match_gate_m", track_match_gate_m, 0.035);
        pnh_.param("stable_max_px_std", stable_max_px_std, 3.0);
        pnh_.param("stable_max_angle_std_deg", stable_max_angle_std_deg, 5.0);
        pnh_.param("publish_only_stable", publish_only_stable, true);

        pnh_.param("precise_timeout", precise_timeout, 0.8);
        pnh_.param("precise_require_stable", precise_require_stable, true);
        pnh_.param("precise_center_gate_px", precise_center_gate_px, 280.0);

        vector<int> def_dist = {3, 4};
        pnh_.param("distance_pick_shapes", distance_pick_shapes, def_dist);

        templates = new TemplateBank(template_size, template_angle_step, template_refine_step);

        // 初始化 CUDA 滤镜
        int b_k = blur_kernel % 2 == 1 ? blur_kernel : blur_kernel + 1;
        cuda_blur_filter = cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(b_k, b_k), 0);
        if (close_kernel > 1) cuda_close_filter = cuda::createMorphologyFilter(MORPH_CLOSE, CV_8UC1, Mat::ones(close_kernel, close_kernel, CV_8UC1));
        if (open_kernel > 1) cuda_open_filter = cuda::createMorphologyFilter(MORPH_OPEN, CV_8UC1, Mat::ones(open_kernel, open_kernel, CV_8UC1));

        info_sub = nh_.subscribe(camera_info_topic, 1, &VisionProcessorNode::info_cb, this);
        img_sub = it_.subscribe(image_topic, 1, &VisionProcessorNode::image_cb, this);
        state_pub = nh_.advertise<std_msgs::Int32MultiArray>("/vision/board_state", 10);
        debug_pub = it_.advertise("/vision/debug_image", 1);
        fg_pub = it_.advertise("/vision/debug_foreground", 1);
        edges_pub = it_.advertise("/vision/debug_edges", 1);
        pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("/vision/tracked_blocks_table", 1);
        precise_srv = nh_.advertiseService("/vision/get_precise_pose", &VisionProcessorNode::handle_precise, this);

        ROS_INFO("CUDA Vision node ready. GPU acceleration enabled.");
    }

    ~VisionProcessorNode() { delete templates; }

    void info_cb(const sensor_msgs::CameraInfoConstPtr& msg) {
        cam_model.update(msg, assume_rectified);
    }

    Point2f contour_centroid(const vector<Point>& cnt) {
        Moments m = moments(cnt);
        if (abs(m.m00) > 1e-6) return Point2f(m.m10 / m.m00, m.m01 / m.m00);
        RotatedRect rect = minAreaRect(cnt);
        return rect.center;
    }

    Point2f distance_transform_center(Size sz, const vector<Point>& cnt) {
        Mat single = Mat::zeros(sz, CV_8UC1);
        vector<vector<Point>> cnts = {cnt};
        drawContours(single, cnts, 0, Scalar(255), FILLED);
        Mat dist;
        distanceTransform(single, dist, DIST_L2, 5);
        Point max_loc;
        minMaxLoc(dist, nullptr, nullptr, nullptr, &max_loc);
        return Point2f(max_loc.x, max_loc.y);
    }

    void process_foreground_cuda(const Mat& frame, Mat& small_out, Mat& mask_out, double& scale) {
        Mat small_frame;
        scale = scale_percent / 100.0;
        if (scale_percent == 100) small_frame = frame;
        else resize(frame, small_frame, Size(), scale, scale, INTER_AREA);
        
        int x1 = max(0, roi_x_min), y1 = max(0, roi_y_min);
        int x2 = (roi_x_max <= 0) ? small_frame.cols : min(small_frame.cols, roi_x_max);
        int y2 = (roi_y_max <= 0) ? small_frame.rows : min(small_frame.rows, roi_y_max);
        Mat roi = small_frame(Rect(x1, y1, x2 - x1, y2 - y1));

        // --- GPU 加速管线开始 ---
        cuda::GpuMat d_roi, d_gray, d_mask;
        d_roi.upload(roi);
        cuda::cvtColor(d_roi, d_gray, COLOR_BGR2GRAY);
        
        if (blur_kernel > 1) cuda_blur_filter->apply(d_gray, d_gray);

        if (threshold_mode == "otsu") {
            // 混合策略：CPU快速计算Otsu阈值，GPU执行Threshold以获得最高吞吐量
            Mat tiny_gray;
            Mat cpu_gray; d_gray.download(cpu_gray);
            resize(cpu_gray, tiny_gray, Size(), 0.25, 0.25);
            double otsu_thresh = threshold(tiny_gray, tiny_gray, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
            cuda::threshold(d_gray, d_mask, otsu_thresh, 255, THRESH_BINARY_INV);
        } else {
            cuda::threshold(d_gray, d_mask, manual_dark_threshold, 255, THRESH_BINARY_INV);
        }

        if (close_kernel > 1) cuda_close_filter->apply(d_mask, d_mask);
        if (open_kernel > 1) cuda_open_filter->apply(d_mask, d_mask);

        Mat roi_mask_cpu;
        d_mask.download(roi_mask_cpu); // 回收至 CPU 用于 findContours
        // --- GPU 加速管线结束 ---

        mask_out = Mat::zeros(small_frame.size(), CV_8UC1);
        roi_mask_cpu.copyTo(mask_out(Rect(x1, y1, x2 - x1, y2 - y1)));
        small_out = small_frame;
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg) {
        if (!cam_model.ready) return;
        Mat frame;
        try { frame = cv_bridge::toCvShare(msg, "bgr8")->image; } 
        catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); return; }

        double stamp = msg->header.stamp.toSec();
        Mat small_frame, fg_mask; double scale;
        
        // 1. GPU 预处理
        process_foreground_cuda(frame, small_frame, fg_mask, scale);
        
        // 2. 轮廓提取与分类 (CPU)
        double inv_scale = 1.0 / scale;
        vector<vector<Point>> contours;
        findContours(fg_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<Detection> detections;
        for (auto& cnt : contours) {
            double area = contourArea(cnt);
            if (area < min_area || area > max_area) continue;
            Rect bb = boundingRect(cnt);
            double fill = area / max(1.0, (double)bb.width * bb.height);
            if (fill < min_fill || fill > max_fill || bb.width < 10 || bb.height < 10) continue;

            Mat c_mask = Mat::zeros(fg_mask.size(), CV_8UC1);
            vector<vector<Point>> tmp_cnt = {cnt};
            drawContours(c_mask, tmp_cnt, 0, Scalar(255), FILLED);
            Mat norm = TemplateBank::normalize_binary_mask(c_mask(bb), template_size);
            
            auto [sid, angle, iou] = templates->match(norm);
            if (iou < min_template_iou) continue;

            Point2f centroid = contourCentroid(cnt);
            Point2f pick_pt = centroid;
            if (pick_point_mode == "distance" || 
               (pick_point_mode == "hybrid" && find(distance_pick_shapes.begin(), distance_pick_shapes.end(), sid) != distance_pick_shapes.end())) {
                pick_pt = distance_transform_center(fg_mask.size(), cnt);
            }

            vector<Point> cnt_full;
            for(auto& p : cnt) cnt_full.push_back(Point(p.x * inv_scale, p.y * inv_scale));
            
            Detection d;
            d.shape_id = sid; d.name = SHAPE_NAMES.at(sid);
            d.contour = cnt_full; d.area = area * inv_scale * inv_scale;
            d.geom_px = Point2f(centroid.x * inv_scale, centroid.y * inv_scale);
            d.pick_px = Point2f(pick_pt.x * inv_scale, pick_pt.y * inv_scale);
            d.angle_deg = angle; d.score = iou; d.stamp = stamp;
            detections.push_back(d);
        }

        // 3. 坐标转换与追踪更新
        geometry_msgs::TransformStamped tx;
        bool has_tx = false;
        string c_frame = camera_frame_override.empty() ? msg->header.frame_id : camera_frame_override;
        try {
            tx = tf_buffer.lookupTransform(table_frame, c_frame, ros::Time(0), ros::Duration(0.05));
            has_tx = true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "TF error: %s", ex.what());
        }

        if (has_tx) attach_table_points(detections, tx);

        {
            lock_guard<mutex> lock(state_mtx);
            latest_frame = frame.clone();
            latest_header = msg->header;
            update_tracks(detections);
        }

        // 4. 发布结果
        vector<BlockTrack> pub_tracks;
        for (auto& t : tracks) {
            if (publish_only_stable) {
                if (t.is_stable(stable_min_frames, stable_max_px_std, stable_max_angle_std_deg)) pub_tracks.push_back(t);
            } else if (t.missed == 0) pub_tracks.push_back(t);
        }

        publish_board_state(pub_tracks);
        if (pose_array_pub.getNumSubscribers() > 0) publish_pose_array(pub_tracks, msg->header);
        
        if (debug_pub.getNumSubscribers() > 0) {
            Mat debug = draw_debug(frame, detections, tracks);
            debug_pub.publish(cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
        }
        if (fg_pub.getNumSubscribers() > 0) fg_pub.publish(cv_bridge::CvImage(msg->header, "mono8", fg_mask).toImageMsg());
    }

    void attach_table_points(vector<Detection>& detections, const geometry_msgs::TransformStamped& tx) {
        tf2::Quaternion q(tx.transform.rotation.x, tx.transform.rotation.y, tx.transform.rotation.z, tx.transform.rotation.w);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 T(tx.transform.translation.x, tx.transform.translation.y, tx.transform.translation.z);

        for (auto& d : detections) {
            auto try_proj = [&](Point2f px, Point3f& out) {
                Point3f ray = cam_model.pixel_to_ray(px.x, px.y);
                tf2::Vector3 v_ray(ray.x, ray.y, ray.z);
                tf2::Vector3 table_ray = R * v_ray;
                if (abs(table_ray.z()) < 1e-9) return false;
                double t = (target_plane_z - T.z()) / table_ray.z();
                if (t < 0) return false;
                tf2::Vector3 pt = T + table_ray * t;
                out = Point3f(pt.x(), pt.y(), pt.z());
                return true;
            };
            bool g_ok = try_proj(d.geom_px, d.geom_table);
            bool p_ok = try_proj(d.pick_px, d.pick_table);
            d.has_table_points = g_ok && p_ok;
        }
    }

    double match_score(const BlockTrack& tr, const Detection& det) {
        double penalty = (tr.shape_id == det.shape_id) ? 0.0 : 1.5;
        if (tr.has_stable_table_pick() && det.has_table_points) {
            Point3f p1 = tr.stable_table_pick(), p2 = det.pick_table;
            double d = norm(p1 - p2);
            if (d <= track_match_gate_m) return d / max(track_match_gate_m, 1e-6) + penalty;
            return numeric_limits<double>::infinity();
        }
        Point2f p1 = tr.stable_px_pick(), p2 = det.pick_px;
        double dpx = norm(p1 - p2);
        if (dpx <= track_match_gate_px) return dpx / max(track_match_gate_px, 1e-6) + penalty;
        return numeric_limits<double>::infinity();
    }

    void update_tracks(const vector<Detection>& detections) {
        vector<int> unmatched_t(tracks.size()), unmatched_d(detections.size());
        iota(unmatched_t.begin(), unmatched_t.end(), 0);
        iota(unmatched_d.begin(), unmatched_d.end(), 0);
        
        struct Pair { double score; int ti, di; };
        vector<Pair> pairs;
        for (size_t ti = 0; ti < tracks.size(); ++ti) {
            for (size_t di = 0; di < detections.size(); ++di) {
                double s = match_score(tracks[ti], detections[di]);
                if (!isinf(s)) pairs.push_back({s, (int)ti, (int)di});
            }
        }
        sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b){ return a.score < b.score; });

        for (auto& p : pairs) {
            auto it_t = find(unmatched_t.begin(), unmatched_t.end(), p.ti);
            auto it_d = find(unmatched_d.begin(), unmatched_d.end(), p.di);
            if (it_t != unmatched_t.end() && it_d != unmatched_d.end()) {
                tracks[p.ti].update(detections[p.di]);
                unmatched_t.erase(it_t); unmatched_d.erase(it_d);
            }
        }
        for (int ti : unmatched_t) tracks[ti].mark_missed();
        for (int di : unmatched_d) tracks.push_back(BlockTrack(next_track_id++, detections[di], track_history_len));
        
        tracks.erase(remove_if(tracks.begin(), tracks.end(), 
            [this](const BlockTrack& t){ return t.missed > max_missed_frames; }), tracks.end());
    }

    void publish_board_state(const vector<BlockTrack>& pub_tracks) {
        std_msgs::Int32MultiArray msg;
        vector<int> inventory(7, 0);
        vector<int> board_state(140, 0);
        vector<int> payload;
        for (auto& tr : pub_tracks) {
            if (tr.shape_id < 0 || tr.shape_id >= 7) continue;
            inventory[tr.shape_id]++;
            Point2f p = tr.stable_px_geom();
            payload.push_back(tr.shape_id);
            payload.push_back(round(p.x)); payload.push_back(round(p.y));
            payload.push_back(round(tr.stable_angle()));
        }
        msg.data.insert(msg.data.end(), inventory.begin(), inventory.end());
        msg.data.insert(msg.data.end(), board_state.begin(), board_state.end());
        msg.data.push_back(payload.size() / 4);
        msg.data.insert(msg.data.end(), payload.begin(), payload.end());
        state_pub.publish(msg);
    }

    void publish_pose_array(const vector<BlockTrack>& pub_tracks, const std_msgs::Header& header) {
        geometry_msgs::PoseArray arr;
        arr.header.stamp = header.stamp.isZero() ? ros::Time::now() : header.stamp;
        arr.header.frame_id = table_frame;
        for (auto& tr : pub_tracks) {
            if (!tr.has_stable_table_pick()) continue;
            Point3f p = tr.stable_table_pick();
            geometry_msgs::Pose pose;
            pose.position.x = p.x; pose.position.y = p.y; pose.position.z = p.z;
            tf2::Quaternion q; q.setRPY(M_PI, 0.0, tr.stable_angle() * M_PI / 180.0);
            pose.orientation = tf2::toMsg(q);
            arr.poses.push_back(pose);
        }
        pose_array_pub.publish(arr);
    }

    Mat draw_debug(const Mat& frame, const vector<Detection>& det, const vector<BlockTrack>& trk) {
        Mat out = frame.clone();
        for (auto& d : det) {
            vector<vector<Point>> cnts = {d.contour};
            drawContours(out, cnts, 0, DRAW_COLORS.at(d.shape_id), 1);
            circle(out, d.geom_px, 3, Scalar(180,180,180), -1);
        }
        for (auto& tr : trk) {
            Point2f geom = tr.stable_px_geom(), pick = tr.stable_px_pick();
            bool stable = tr.is_stable(stable_min_frames, stable_max_px_std, stable_max_angle_std_deg);
            Scalar color = stable ? Scalar(0, 255, 0) : Scalar(0, 200, 255);
            circle(out, geom, 5, color, -1);
            drawMarker(out, pick, Scalar(0,255,255), MARKER_CROSS, 14, 2);
            char label[64];
            sprintf(label, "#%d s%d %s a%d iou%.2f", tr.track_id, tr.shape_id, 
                    SHAPE_NAMES.at(tr.shape_id).c_str(), (int)round(tr.stable_angle()), tr.stable_score());
            putText(out, label, Point(geom.x + 8, geom.y - 8), FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        }
        return out;
    }

    bool handle_precise(tly::GetPrecisePose::Request &req, tly::GetPrecisePose::Response &res) {
        double deadline = ros::Time::now().toSec() + precise_timeout;
        int target_sid = req.target_shape_type;
        
        while (ros::Time::now().toSec() <= deadline) {
            lock_guard<mutex> lock(state_mtx);
            if (latest_frame.empty()) continue;
            
            double fx, fy, cx, cy;
            if (!cam_model.get_intrinsics(fx, fy, cx, cy)) continue;
            Point2f center_px(cx, cy);

            BlockTrack* chosen = nullptr;
            double min_dist = numeric_limits<double>::infinity();

            for (auto& tr : tracks) {
                if (tr.shape_id != target_sid || tr.missed != 0) continue;
                if (precise_require_stable && !tr.is_stable(stable_min_frames, stable_max_px_std, stable_max_angle_std_deg)) continue;
                Point2f px = tr.stable_px_pick();
                double dpx = norm(px - center_px);
                if (dpx > precise_center_gate_px) continue;
                if (dpx < min_dist) { min_dist = dpx; chosen = &tr; }
            }

            if (chosen) {
                res.success = true;
                res.angle = round(chosen->stable_angle());
                
                string c_frame = camera_frame_override.empty() ? latest_header.frame_id : camera_frame_override;
                geometry_msgs::TransformStamped tx;
                try {
                    tx = tf_buffer.lookupTransform(table_frame, c_frame, ros::Time(0), ros::Duration(0.01));
                    
                    // 获取中心点的 Table 坐标
                    tf2::Quaternion q(tx.transform.rotation.x, tx.transform.rotation.y, tx.transform.rotation.z, tx.transform.rotation.w);
                    tf2::Matrix3x3 R(q); tf2::Vector3 T(tx.transform.translation.x, tx.transform.translation.y, tx.transform.translation.z);
                    Point3f ray_c = cam_model.pixel_to_ray(cx, cy);
                    tf2::Vector3 table_ray = R * tf2::Vector3(ray_c.x, ray_c.y, ray_c.z);
                    double t = (target_plane_z - T.z()) / table_ray.z();
                    tf2::Vector3 center_table = T + table_ray * t;

                    if (chosen->has_stable_table_pick()) {
                        Point3f target_table = chosen->stable_table_pick();
                        double dx_t = target_table.x - center_table.x();
                        double dy_t = target_table.y - center_table.y();
                        double z_dist = max(abs(hover_z - target_plane_z), 1e-4);
                        res.dx = round(dy_t * fx / z_dist);
                        res.dy = round(dx_t * fy / z_dist);
                        return true;
                    }
                } catch (...) { /* fallback to px */ }
                
                res.dx = round(chosen->stable_px_pick().x - cx);
                res.dy = round(chosen->stable_px_pick().y - cy);
                return true;
            }
            ros::Duration(0.03).sleep();
        }
        res.success = false; res.dx = 0; res.dy = 0; res.angle = 0;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_processor_node_cuda");
    VisionProcessorNode node;
    ros::AsyncSpinner spinner(4); // 开启多线程以同时处理图像和 TF
    spinner.start();
    ros::waitForShutdown();
    return 0;
}