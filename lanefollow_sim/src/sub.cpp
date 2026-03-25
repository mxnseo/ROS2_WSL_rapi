#include "lanefollow_sim/sub.hpp"

LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_l(160, 60), tmp_pt_r(480, 60), first_run_(true), mode(false) {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
        
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "topic_dxlpub",
        qos_profile);
}

// 관심영역 설정 + 전처리
cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    // ROI: 하단 1/4 영역
    cv::Mat roi = frame_color(cv::Rect(
        cv::Point(0, frame_color.rows * 3 / 4),
        cv::Point(frame_color.cols, frame_color.rows)
    )).clone();

    // 그레이스케일
    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);

    // 밝기 보정
    cv::Scalar bright_avg = cv::mean(roi);
    roi = roi + (100 - bright_avg[0]);

    // 이진화
    cv::threshold(roi, roi, 120, 255, cv::THRESH_BINARY);

    // first_run_ 시 초기 탐색 기준점: x축 1/4, 3/4 지점
    if (first_run_) {
        tmp_pt_l = cv::Point(roi.cols / 4, roi.rows / 2);
        tmp_pt_r = cv::Point(roi.cols * 3 / 4, roi.rows / 2);
    }

    return roi;
}

// 라인 추적 함수
std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;

    int left_idx  = -1;
    int right_idx = -1;
    double left_best  = roi.cols; // 가장 가까운 거리 초기값
    double right_best = roi.cols;

    // 0번은 배경이니까 제외
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));

            double d_left  = cv::norm(cv::Point(x, y) - tmp_pt_l);
            double d_right = cv::norm(cv::Point(x, y) - tmp_pt_r);

            // 150px 이내에서 가장 가까운 것을 좌/우 후보로 저장
            if (d_left < left_best && d_left <= 150) {
                left_best = d_left;
                left_idx  = i;
            }
            if (d_right < right_best && d_right <= 150) {
                right_best = d_right;
                right_idx  = i;
            }
        }
    }

    // 좌측 차선 갱신
    bool left_ok  = (left_idx  != -1 && left_best  <= 150);
    bool right_ok = (right_idx != -1 && right_best <= 150);

    if (left_ok) {
        tmp_pt_l = cv::Point(
            cvRound(centroids.at<double>(left_idx,  0)),
            cvRound(centroids.at<double>(left_idx,  1))
        );
    } else {
        left_idx = -1;
    }

    // 우측 차선 갱신
    if (right_ok) {
        tmp_pt_r = cv::Point(
            cvRound(centroids.at<double>(right_idx, 0)),
            cvRound(centroids.at<double>(right_idx, 1))
        );
    } else {
        right_idx = -1;
    }

    if (left_ok || right_ok) first_run_ = false;

    return std::make_pair(left_idx, right_idx);
}

// 라인 시각화 함수
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int labels = stats.rows;
    bool l_valid = (left_idx  >= 1 && left_idx  < labels);
    bool r_valid = (right_idx >= 1 && right_idx < labels);

    for (int i = 1; i < labels; i++) {
        int area = stats.at<int>(i, 4);
        if (area < 100) continue;

        int left   = stats.at<int>(i, 0);
        int top    = stats.at<int>(i, 1);
        int width  = stats.at<int>(i, 2);
        int height = stats.at<int>(i, 3);
        int cx     = cvRound(centroids.at<double>(i, 0));
        int cy     = cvRound(centroids.at<double>(i, 1));

        cv::Scalar color(255, 0, 0); // 파란색: 추적 안됨
        if (l_valid && i == left_idx) color = cv::Scalar(0, 0, 255); // 빨간색
        else if (r_valid && i == right_idx) color = cv::Scalar(0, 0, 255); // 빨간색

        cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
        cv::circle(result, cv::Point(cx, cy), 3, color, -1);
    }

    // 라인 못 찾았을 때: 마지막 위치에 작은 사각형 표시
    if (!l_valid) {
        cv::rectangle(result,
            cv::Rect(tmp_pt_l.x - 2, tmp_pt_l.y - 2, 4, 4),
            cv::Scalar(0, 0, 255), 2);
    }
    if (!r_valid) {
        cv::rectangle(result,
            cv::Rect(tmp_pt_r.x - 2, tmp_pt_r.y - 2, 4, 4),
            cv::Scalar(0, 0, 255), 2);
    }
}

int LineDetector::getch(void) 
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool LineDetector::kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;

    cv::Mat roi = preprocess_image(frame_color);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    std::pair<int, int> targets = find_target_line(roi, stats, centroids);
    int left_idx  = targets.first;
    int right_idx = targets.second;

    cv::Mat result_view = roi.clone();
    draw_result(result_view, stats, centroids, left_idx, right_idx);

    // error = 영상 중심 x - 라인의 무게중심 중심 x / 2
    int center_x = roi.cols / 2;
    int error = center_x - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    if(kbhit()) {
        int ch = getch();
        if(ch == 'q')      mode = false;
        else if(ch == 's') mode = true;
    }

    if (mode) {
        vel.x =  100 - k * error;
        vel.y = -(100 + k * error);
    } else {
        vel.x = 0;
        vel.y = 0;
    }

    pub_->publish(vel);

    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);
}