/*
 *    SLLIDAR ROS2 CLIENT
 *
 *    Copyright (c) 2009 - 2014 RoboPeak Team
 *    http://www.robopeak.com
 *    Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *    http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;

    // 윈도우 창 생성 및 이름 지정, 흰색
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

	// 십자가 표시
	cv::line(scan_video, cv::Point(250, 240), cv::Point(250, 260), cv::Scalar(255, 0, 0), 1);
    cv::line(scan_video, cv::Point(240, 250), cv::Point(260, 250), cv::Scalar(255, 0, 0), 1);

    printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
                 RAD2DEG(scan->angle_max));

	float scale = 8.0; // 보이는 범위 조정 scale
    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // 각도 환산
        printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);

		// 극좌표 -> 직교좌표로 변환하기, x = r * cos(theta), y = r * sin(theta)
		float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
		float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
		
        // 스캔 영상 그리기 코드
		cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
    }
	cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
	cv::Mat result;
	cv::warpAffine(scan_video, result, rotate, scan_video.size());
	cv::flip(result, result, 1);
	cv::namedWindow("Lidar 2D map");

    // 스캔 영상 화면 출력 및 동영상 저장 코드
    static cv::VideoWriter video_writer; // static으로 선언하여 함수가 종료되어도 객체 유지
    static bool is_writer_initialized = false; // 초기화 여부 확인 플래그

    // 아직 초기화가 안 됐다면 (첫 번째 프레임이 들어왔을 때)
    if (!is_writer_initialized) {
        // 저장할 파일 이름, 코덱, FPS, 프레임 크기(가로, 세로), 컬러 여부
        video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(result.cols, result.rows), true);
        
        if (video_writer.isOpened()) {
            is_writer_initialized = true; // [핵심] 성공했으면 true로 바꿔야 함!! (이게 없어서 안됐던 것)
            printf("Video Writer Initialized Successfully.\n");
        } else {
            printf("Failed to open Video Writer.\n");
        }
    }

    // 비디오 파일에 현재 프레임 쓰기
    if (is_writer_initialized) {
        video_writer.write(result);
    }

    // 창 띄우기
    cv::imshow("Lidar 2D map", result);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), scanCb);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
