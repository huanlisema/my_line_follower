#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "my_line_follower/pid.hpp"
#include <algorithm>
using namespace std::placeholders;
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
                            threshold(0.5)
    {
        mylinefollower = std::make_shared<MyLineFollower>();
        pid = std::make_shared<PID>(1.1, 0.0, 0.0);
        image_sub = create_subscription<sensor_msgs::msg::Image>("/ascamera/camera_publisher/rgb0/image", 10,
                                                                 std::bind(&MyLineFollowingNode::image_cb, this, _1));
        result_image_pub = create_publisher<sensor_msgs::msg::Image>("~/image_result", 1);
        mecanum_pub = create_publisher<geometry_msgs::msg::Twist>("/controller/cmd_vel", 1);
    }

private:
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
        // 偏差角 -> PID
        pid->update(deflection_angle);
        //获取 PID 输出
        double pid_output = pid->getOutput();
        //输出限幅,防止猛转
        twist.angular.z = set_range(-pid_output,-1.0,1.0);
        //调试信息
        RCLCPP_INFO(this->get_logger(),"angle = %.3f, pid = %.3f, vx = %.3f, wz = %.3f",
            deflection_angle,
            pid_output,
            twist.linear.x,
            twist.angular.z);
        //发布速度
        mecanum_pub->publish(twist);
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
    int image_height;                                                       // 图片高
    int image_width;                                                        // 图片宽
    double threshold;                                                       // 颜色偏差系数
    std::shared_ptr<MyLineFollower> mylinefollower;
    double deflection_angle; // 偏差角
    std::shared_ptr<PID> pid;
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