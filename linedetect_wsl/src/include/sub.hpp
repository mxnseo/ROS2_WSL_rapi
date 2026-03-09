#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp" // ROS 2 기본 헤더
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp" // OpenCV 헤더
#include <memory> 
#include <chrono> 

using namespace std;

class LineDetector : public rclcpp::Node { // 클래스 상속
public:
    LineDetector(); // 생성자
private:
    void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg); // 메인 콜백 함수 선언

    cv::Mat preprocess_image(const cv::Mat& frame_color); // 전처리 함수 선언
    int find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids); // 라인 탐색 함수 선언
    void draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int target_idx); // 시각화 함수 선언

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_; // 서브스크라이버 변수

    cv::Point tmp_pt_; // 라인 위치 기억할 변수
    bool first_run_; // 첫 실행인지 확인할 플래그
};

#endif // SUB_HPP_