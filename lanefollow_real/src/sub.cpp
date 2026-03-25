#include "lanefollow_real/sub.hpp"

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

cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Scalar bright_avg = cv::mean(frame_gray); 
    frame_gray = frame_gray + (90 - bright_avg[0]); // 의도된 밝기 보정값

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 120, 255, cv::THRESH_BINARY);

    // [수정 1] ROI 동적 계산 (해상도 독립적)
    int h = frame_binary.rows;
    int w = frame_binary.cols;
    int roi_y = h * 3 / 4;
    int roi_h = h / 4;
    if (roi_y + roi_h > h) roi_h = h - roi_y;

    return frame_binary(cv::Rect(0, roi_y, w, roi_h)).clone();
}

std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt    = stats.rows;
    int half_w = roi.cols / 2;

    int l_idx = -1;
    int r_idx = -1;

    // first_run_: 좌/우 영역 분리해서 초기 탐색
    if (first_run_) {
        int min_left_dist  = 10000;
        int min_right_dist = 10000;

        for (int i = 1; i < cnt; i++) { 
            int area = stats.at<int>(i, 4);
            if (area > 100) { 
                int cx = cvRound(centroids.at<double>(i, 0));

                // [수정 2] 초기 기준점 동적 계산 (roi.cols 기반)
                if (cx < half_w) {
                    int dist = abs(roi.cols / 4 - cx);
                    if (dist < min_left_dist) {
                        min_left_dist = dist;
                        l_idx = i;
                    }
                } else {
                    int dist = abs(roi.cols * 3 / 4 - cx);
                    if (dist < min_right_dist) {
                        min_right_dist = dist;
                        r_idx = i;
                    }
                }
            }
        }
        
        if (l_idx != -1) {
            tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
        }
        if (r_idx != -1) {
            tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
        }
        // [수정 3] 둘 다 찾았을 때만 first_run_ 해제
        if (l_idx != -1 && r_idx != -1) first_run_ = false;

        return std::make_pair(l_idx, r_idx);
    }

    // [수정 4] search_radius 150으로 복원 (50은 너무 작음)
    int search_radius = 50;
    int min_area = 100;

    int min_dist_l = 10000;
    int min_dist_r = 10000;
    int idx_l = -1;
    int idx_r = -1;

    // 좌측 탐색 (x < half_w 영역만)
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area < min_area) continue;

        int x = cvRound(centroids.at<double>(i, 0));
        int y = cvRound(centroids.at<double>(i, 1));
        if (x >= half_w) continue;

        int dist_l = cv::norm(cv::Point(x, y) - tmp_pt_l);
        if (dist_l < min_dist_l && dist_l <= search_radius) {
            min_dist_l = dist_l;
            idx_l = i;
        }
    }

    // 우측 탐색 (x >= half_w 영역만, idx_l 제외)
    for (int i = 1; i < cnt; i++) {
        if (i == idx_l) continue;
        int area = stats.at<int>(i, 4);
        if (area < min_area) continue;

        int x = cvRound(centroids.at<double>(i, 0));
        int y = cvRound(centroids.at<double>(i, 1));
        if (x < half_w) continue;

        int dist_r = cv::norm(cv::Point(x, y) - tmp_pt_r);
        if (dist_r < min_dist_r && dist_r <= search_radius) {
            min_dist_r = dist_r;
            idx_r = i;
        }
    }

    if (idx_l != -1) {
        tmp_pt_l = cv::Point(cvRound(centroids.at<double>(idx_l, 0)), cvRound(centroids.at<double>(idx_l, 1)));
    }
    if (idx_r != -1) {
        tmp_pt_r = cv::Point(cvRound(centroids.at<double>(idx_r, 0)), cvRound(centroids.at<double>(idx_r, 1)));
    }

    return std::make_pair(idx_l, idx_r);
}

void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
    
    int cnt = stats.rows;

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            
            int left   = stats.at<int>(i, 0);
            int top    = stats.at<int>(i, 1);
            int width  = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);

            cv::Scalar color(255, 0, 0); 
            if (i == left_idx || i == right_idx) {
                color = cv::Scalar(0, 0, 255);
            }

            cv::rectangle(result, cv::Rect(left, top, width, height), color, 1);
            cv::circle(result, cv::Point(x, y), 5, color, -1);
        }
    }

    cv::circle(result, tmp_pt_l, 5, cv::Scalar(0, 0, 255), -1); 
    cv::circle(result, tmp_pt_r, 5, cv::Scalar(0, 0, 255), -1); 
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

    // [수정 5] error 기준점 동적 계산
    int center_x = roi.cols / 2;
    int error = center_x - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false;
        else if(ch == 's') mode = true;
    }

    if (mode) {
        vel.x = 100 - k * error;
        vel.y = -(100 + k * error);
    } 
    else {
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