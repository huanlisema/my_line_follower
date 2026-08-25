#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "my_line_follower/pid.hpp"
#include <algorithm>
#include <cmath>
using namespace std::placeholders;
const double MAX_SCAN_ANGLE = 240.0; // 激光的扫描角度,去掉总是被遮挡的部分
const double PI = 3.14159265358979323846;
struct Roi
{
    double y1;
    double y2;
    double x1;
    double x2;
    double weight;
};
// 算法处理节点类
class MyLineFollower
{
public:
    MyLineFollower() : weight_sum(1.0)
    {
        // 裁剪图片的范围和权重设置
        rois = {
            {0.9, 0.95, 0.0, 1.0, 0.7},
            {0.8, 0.85, 0.0, 1.0, 0.2},
            {0.7, 0.75, 0.0, 1.0, 0.1}};
    }

    double get_deflection_angle(const cv::Mat &cv_image, cv::Mat &result_image, double threshold)
    {
        (void)threshold;
        // 设置黑线的颜色阈值
        cv::Scalar min_black(0, 0, 0);
        cv::Scalar max_black(180, 255, 60);
        int h = cv_image.rows;
        int w = cv_image.cols;
        double centroid_sum = 0;

        for (auto &roi : rois)
        {

            // 1. 裁剪ROI
            int y_start = roi.y1 * h;
            int y_end = roi.y2 * h;
            int x_start = roi.x1 * w;
            int x_end = roi.x2 * w;
            cv::Mat roi_image = cv_image(cv::Rect(x_start, y_start, x_end - x_start, y_end - y_start));
            // 2. BGR->HSV
            cv::Mat hsv_image;
            cv::cvtColor(roi_image, hsv_image, cv::COLOR_BGR2HSV);
            // 3. 高斯滤波
            cv::Mat blur;
            cv::GaussianBlur(hsv_image, blur, cv::Size(3, 3), 3);
            // 4. 二值化
            cv::Mat mask;
            cv::inRange(blur, min_black, max_black, mask);
            // 5. 形态学处理(腐蚀膨胀)
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 卷积核
            cv::Mat eroded;
            cv::erode(mask, eroded, kernel);
            cv::Mat dilated;
            cv::dilate(eroded, dilated, kernel);
            // 7. 找轮廓
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);
            // 8. 找最大轮廓
            int max_index = get_max_area_contour(contours, 30);
            if (max_index == -1)
            {
                continue;
            }
            // 9. 算中心
            cv::RotatedRect rect = cv::minAreaRect(contours[max_index]); // 取最大轮廓
            cv::Point2f box[4];                                          // 提取轮廓4个角
            rect.points(box);
            for (int i = 0; i < 4; i++) // 加上起始点还原在整图上的位置
            {
                box[i].x += x_start;
                box[i].y += y_start;
            }
            cv::Point2f center = (box[0] + box[2]) / 2.0; // 对角中点坐标
                                                          // 画中心点
            cv::circle(result_image, center, 5, cv::Scalar(0, 0, 255), -1);
            // 累加不同roi块中心点x坐标的权重
            centroid_sum += center.x * roi.weight;
        }
        if (centroid_sum == 0)
        {
            return 999; // 没找到线
        }
        double center_pos = centroid_sum / weight_sum; // 当权重和不等于1会有偏差,除以权重之和
        double deflection_angle = -std::atan((center_pos - w / 2.0) / (h / 2.0));
        return deflection_angle;
    }
    int get_max_area_contour(const std::vector<std::vector<cv::Point>> &contours, double min_area)
    {
        double max_area = 0;
        int max_index = -1;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                max_index = i;
            }
        }
        // 没有满足面积要求的轮廓
        if (max_area < min_area)
        {
            return -1;
        }
        return max_index;
    }

private:
    std::vector<Roi> rois;
    double weight_sum;
};
// ros2通信节点类
class MyLineFollowingNode : public rclcpp::Node
{
public:
    MyLineFollowingNode() : Node("my_line_following_node_cpp"),
                            threshold(0.5),
                            scan_angle(45.0),
                            stop_threshold(0.4),
                            stop(false),
                            yolo_stop(false),
                            count(0),
                            yolo_false_count(0)
    {
        mylinefollower = std::make_shared<MyLineFollower>();
        pid = std::make_shared<PID>(1.1, 0.0, 0.0);
        image_sub = create_subscription<sensor_msgs::msg::Image>("/ascamera/camera_publisher/rgb0/image", 10,
                                                                 std::bind(&MyLineFollowingNode::image_cb, this, _1));
        result_image_pub = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);
        mecanum_pub = create_publisher<geometry_msgs::msg::Twist>("/controller/cmd_vel", 1);
        lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>("/scan_raw", 10,
                                                                     std::bind(&MyLineFollowingNode::lidar_cb, this, _1));
        stop_sub = create_subscription<std_msgs::msg::Bool>("/stop_detected", 10, std::bind(&MyLineFollowingNode::stop_callback, this, _1));
    }

private:
    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            yolo_false_count = 0;
            if (!yolo_stop)
            {
                yolo_stop = true;
                RCLCPP_WARN(this->get_logger(), "YOLO检测到STOP标志，小车停车！");
            }
        }
        else
        { 
            yolo_false_count++;
            // 连续5次false,才解除STOP 
            if (yolo_false_count >= 5)
            {
                if (yolo_stop)
                {
                    yolo_stop = false;
                }
                yolo_false_count = 0;
            }
        }
    }
    void image_cb(const sensor_msgs::msg::Image &ros_image)
    {
        // 转换成opencv图像
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(ros_image, "bgr8");
        cv::Mat cv_image = cv_ptr->image;
        image_height = cv_image.rows;
        image_width = cv_image.cols;
        cv::Mat result_image = cv_image.clone(); // 复制图像，用于修改
        // 组织速度消息
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.1;
        deflection_angle = mylinefollower->get_deflection_angle(cv_image, result_image, threshold);
        // 显示带中心点图像
        publish_result(result_image);
        // 没找到线
        if (deflection_angle == 999)
        {
            // 停止，或者保持上一帧速度
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            mecanum_pub->publish(twist);
            return;
        }
        if (stop)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            mecanum_pub->publish(twist);
            RCLCPP_INFO(this->get_logger(), "检测到障碍物，停车！");
            return;
        }
        if (yolo_stop)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            mecanum_pub->publish(twist);
            RCLCPP_INFO(this->get_logger(), "识别到stop标志，停车！");
            return;
        }
        // 偏差角 -> PID
        pid->update(deflection_angle);
        // 获取 PID 输出
        double pid_output = pid->getOutput();
        // 输出限幅,防止猛转
        twist.angular.z = set_range(-pid_output, -1.0, 1.0);
        // 调试信息
        RCLCPP_INFO(this->get_logger(), "angle = %.3f, pid = %.3f, vx = %.3f, wz = %.3f",
                    deflection_angle,
                    pid_output,
                    twist.linear.x,
                    twist.angular.z);
        // 发布速度
        mecanum_pub->publish(twist);
    }
    void lidar_cb(const sensor_msgs::msg::LaserScan &lidar_data)
    {
        // 处理雷达范围数据
        // 数据大小 = 扫描角度/每扫描一次增加的角度
        // 一半范围的弧度
        double half_scan_angle = MAX_SCAN_ANGLE / 2.0 * PI / 180.0;
        // 计算半边需要取多少个激光点
        int max_index = static_cast<int>(half_scan_angle / lidar_data.angle_increment);
        max_index = std::min(max_index, static_cast<int>(lidar_data.ranges.size())); // 防越界
        // 左半边数据
        std::vector<float> left_ranges(lidar_data.ranges.begin(), lidar_data.ranges.begin() + max_index);
        // 右半边数据：反向取
        std::vector<float> right_ranges(lidar_data.ranges.rbegin(), lidar_data.ranges.rbegin() + max_index);
        // 处理雷达避障范围数据
        double angle = scan_angle / 2.0 * PI / 180.0;
        int angle_index = static_cast<int>(angle / lidar_data.angle_increment + 0.5);
        angle_index = std::min(angle_index, static_cast<int>(left_ranges.size()));
        std::vector<float> left_range(left_ranges.begin(), left_ranges.begin() + angle_index);
        std::vector<float> right_range(right_ranges.begin(), right_ranges.begin() + angle_index);
        // 过滤无效数据
        std::vector<float> left_valid;
        std::vector<float> right_valid;
        filter_ranges(left_range, left_valid);
        filter_ranges(right_range, right_valid);
        bool obstacle = false;
        // 判断左边数据集
        if (!left_valid.empty())
        {
            // 取最近距离
            float min_dist_left = *std::min_element(left_valid.begin(), left_valid.end());
            if (min_dist_left < stop_threshold)
                obstacle = true;
        }
        // 判断右边数据集
        if (!right_valid.empty())
        {
            // 取最近距离
            float min_dist_right = *std::min_element(right_valid.begin(), right_valid.end());
            if (min_dist_right < stop_threshold)
                obstacle = true;
        }
        if (obstacle)
        {
            // 判断是否需要停车
            stop = true;
            count = 0;
        }
        else
        {
            count++;
            if (count > 5)
            {
                count = 0;
                stop = false;
            }
        }
    }
    void filter_ranges(const std::vector<float> &ranges, std::vector<float> &valid_ranges)
    {
        valid_ranges.clear();
        for (float distance : ranges)
        {
            if (distance != 0.0f && std::isfinite(distance)) // 去除零和非数字的数据
            {
                valid_ranges.push_back(distance);
            }
        }
    }
    void publish_result(cv::Mat &result_image)
    {
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_image).toImageMsg();
        result_image_pub->publish(*image_msg);
    }
    double set_range(double value, double min_value, double max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;     // 图片订阅
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub; // 发布图片
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mecanum_pub;    // 发布速度
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub; // 雷达订阅
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
    int image_height; // 图片高
    int image_width;  // 图片宽
    double threshold; // 颜色偏差系数
    std::shared_ptr<MyLineFollower> mylinefollower;
    double deflection_angle; // 偏差角
    std::shared_ptr<PID> pid;
    double scan_angle;     // 雷达避障角度范围(角度制)
    double stop_threshold; // 雷达避障距离
    bool stop;
    bool yolo_stop;
    int count;
    int yolo_false_count;
    
};

int main(int argc, char const *argv[])
{
    // 初始化 ROS2 客户端
    rclcpp::init(argc, argv);
    // 调用 spin 函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<MyLineFollowingNode>());
    // 资源释放
    rclcpp::shutdown();
    return 0;
}