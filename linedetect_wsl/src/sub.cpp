#include "include/sub.hpp"

LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) { // 괄호에 들어가기 전에 변수들의 초기값 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
	// 토픽, 콜백 넣음
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
}

// 이미지 전처리 함수
cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    // 흑백으로 변환
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    // 밝기 보정
    cv::Scalar bright_avg = cv::mean(frame_gray); // 평균 밝기를 계산해서
    frame_gray = frame_gray + (100 - bright_avg[0]); // 차이만큼 값을 더하거나 뺌

    // 이진화
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);

    // ROI 잘라서 리턴
    return frame_binary(cv::Rect(0, 240, 640, 120));
}

// 라인 탐색
int LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;
    
    // 첫 실행이면 중앙이랑 제일 가까운 라인 잡음
    if (first_run_) {
        int best_idx = -1;
        int min_center_dist = 10000;

        for (int i = 1; i < cnt; i++) { // 0번의 배경 제외하고 1번부터 탐색
            int area = stats.at<int>(i, 4);
            if (area > 100) { // 면적 100 이상만 봄 (노이즈 제거)
                int cx = cvRound(centroids.at<double>(i, 0)); // 객체의 중심좌표 x
                // 중앙(320)과의 거리 계산
                int dist_from_center = abs(320 - cx);

                // 중앙이랑 제일 가까운 라인으로 갱신
                if (dist_from_center < min_center_dist) {
                    min_center_dist = dist_from_center;
                    best_idx = i;
                }
            }
        }
        
        // 찾았으면 초기화하고 플래그 끔
        if (best_idx != -1) {
            tmp_pt_ = cv::Point(cvRound(centroids.at<double>(best_idx, 0)), cvRound(centroids.at<double>(best_idx, 1)));
            first_run_ = false;
        }
    }

    // 화면에서 라인이 사라지면 기존 위치랑 가까운 라인 찾음
    int min_idx = -1;
    int min_dist = roi.cols;
    int search_radius = 60; // 탐색 범위 제한, 이전 위치에서 60픽셀 이상 멀리 떨어진 객체는 무시

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int dist = cv::norm(cv::Point(x, y) - tmp_pt_); // 현재 객체 위치랑 이전 프레임 위치 사이의 거리 계산

            // 거리 가깝고 범위 안이면 선택
            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    // 찾았으면 멤버변수 위치 갱신
    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
    }

    return min_idx; // 찾은 인덱스 반환
}

// 시각화
// 시각화 함수 (수정됨)
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int target_idx)
{
    // 컬러로 변환
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
    
    int cnt = stats.rows;

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            
            int left = stats.at<int>(i, 0);
            int top = stats.at<int>(i, 1);
            int width = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);

            cv::rectangle(result, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 1);
            cv::circle(result, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
        }
    }

    // 타겟을 찾았든(idx != -1), 못 찾았든(idx == -1) 무조건 기억된 위치(tmp_pt_)에 빨간 점을 찍음
    if (target_idx != -1) {
        int left = stats.at<int>(target_idx, 0);
        int top = stats.at<int>(target_idx, 1);
        int width = stats.at<int>(target_idx, 2);
        int height = stats.at<int>(target_idx, 3);
        cv::rectangle(result, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
    }

    cv::circle(result, tmp_pt_, 8, cv::Scalar(0, 0, 255), -1); 
}

// 메인 콜백
void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;

	// 전처리 함수
    cv::Mat roi = preprocess_image(frame_color);

    // 레이블링 수행
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 알고리즘 돌려서 타겟 인덱스 찾아옴
    int target_idx = find_target_line(roi, stats, centroids);

    // 결과 영상 그림
    cv::Mat result_view = roi.clone();
    draw_result(result_view, stats, centroids, target_idx);

    // 에러 계산
    int error = 320 - tmp_pt_.x;

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, time:%.2f ms", error, totalTime);

    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}