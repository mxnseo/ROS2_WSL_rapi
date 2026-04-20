#include "lidardrive/sub.hpp" // 헤더 파일 포함함

// 생성자: 노드 이름 설정, 초기 좌표 세팅, 플래그 및 주행 모드(mode) 초기화함
LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
    vel.x = 0; 
    vel.y = 0; 
    vel.z = 0; 

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "topic_dxlpub",
        qos_profile);
}

// 2D 맵 이미지를 흑백/이진화 처리해서 전처리하는 함수
cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat frame_gray; 
    cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);

    return frame_binary(cv::Rect(0, 0, 500, 250)); 
}

// 화면에서 양쪽 가장 가까운 장애물(타겟) 인덱스 찾는 함수
std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows; // 검출된 객체 총 개수 가져옴
    int l_idx = -1; // 왼쪽 타겟 인덱스 (-1은 아직 못 찾았단 뜻)
    int r_idx = -1;
    
    double min_dist_l = 100000.0; // 최소 거리 (비교를 위해 엄청 큰 값 넣어둠)
    double min_dist_r = 100000.0;

    cv::Point robot_pos(250, 250); 

    for (int i = 1; i < cnt; i++) {

        int cx = cvRound(centroids.at<double>(i, 0)); // 객체의 중심점 좌표
        int cy = cvRound(centroids.at<double>(i, 1));
        cv::Point obj_pos(cx, cy); // 객체 위치 좌표로 묶음

        double dist = cv::norm(robot_pos - obj_pos); // 로봇과 객체 사이의 직선 거리 계산

        if (cx < 250) { // 중심이 화면 왼쪽(x<250)에 있음 & 기존 최소 거리보다 가까우면 갱신
            if (dist < min_dist_l) { 
                min_dist_l = dist; 
                l_idx = i;
            }
        }
        else { 
            if (dist < min_dist_r) {
                min_dist_r = dist;
                r_idx = i; 
            }
        }
    }
    
    // [좌표 업데이트] 왼쪽 객체 찾았으면 임시 좌표에 저장
    if (l_idx != -1) {
        tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
    } else { 
        tmp_pt_l = cv::Point(0, 0); // 못 찾았으면 왼쪽 맨 위 구석으로 좌표 이동
    }

    if (r_idx != -1) {
        tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
    } else { 
        tmp_pt_r = cv::Point(500, 0);
    }

    return std::make_pair(l_idx, r_idx);
}

// 화면에 박스, 화살표 등 시각화해서 그리는 함수
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    // 입력 이미지가 흑백(1채널)이면 색깔 입히기 위해 컬러(3채널)로 변환함
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int cnt = stats.rows;
    cv::Point robot_pos(250, 250);

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4); // 객체의 픽셀 면적 가져옴
        if (area > 0) { // 면적이 있는 객체만 처리함 (노이즈 필터링 느낌)
            if (i == left_idx || i == right_idx) { // 양쪽 타겟으로 선정된 애들만
                int x = cvRound(centroids.at<double>(i, 0)); // 중심 x
                int y = cvRound(centroids.at<double>(i, 1)); // 중심 y
                
                int left = stats.at<int>(i, 0); // 바운딩 박스 왼쪽 위 x
                int top = stats.at<int>(i, 1); // 바운딩 박스 왼쪽 위 y
                int width = stats.at<int>(i, 2); // 바운딩 박스 너비
                int height = stats.at<int>(i, 3); // 바운딩 박스 높이

                cv::Scalar color; // 색깔 변수
                if (i == left_idx) color = cv::Scalar(0, 255, 0); // 왼쪽은 초록색
                else color = cv::Scalar(0, 0, 255); // 오른쪽은 빨간색

                // 객체 테두리에 박스 그림 (두께 2)
                cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
                // 객체 중심에 점
                cv::circle(result, cv::Point(x, y), 3, color, -1);
            }
        }
    }

    // 왼쪽 타겟 찾았으면
    if (left_idx != -1) {
        int left = stats.at<int>(left_idx, 0);
        int top = stats.at<int>(left_idx, 1);
        int width = stats.at<int>(left_idx, 2);
        int height = stats.at<int>(left_idx, 3);
        
        cv::Point l_box_bottom_right(left + width, top + height); // 박스의 우측 하단 좌표 계산함
        
        // 로봇 위치에서 타겟 박스 우측 하단으로 초록색 화살표 그음
        cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);
    }

    // 오른쪽 타겟 찾았으면
    if (right_idx != -1) {
        int left = stats.at<int>(right_idx, 0);
        int top = stats.at<int>(right_idx, 1);
        int height = stats.at<int>(right_idx, 3);
        
        cv::Point r_box_bottom_left(left, top + height); // 박스의 좌측 하단 좌표 계산함
        
        // 로봇 위치에서 타겟 박스 좌측 하단으로 빨간색 화살표 그음
        cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0, 0, 255), 1);
    }

    // 주행 목표 지점 (왼쪽 타겟과 오른쪽 타겟의 딱 중간지점 x좌표)
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::Point target_pos(target_x, 100); // 목표 높이(y)는 화면 앞쪽인 100으로 고정함
    
    // 로봇 위치에서 목표 지점으로 파란색 화살표 그음 (가야 할 방향)
    cv::arrowedLine(result, robot_pos, target_pos, cv::Scalar(255, 0, 0), 1);

    // 타겟 중심 좌표에 각각 초록, 빨간색 작은 점 하나씩 찍어줌
    cv::circle(result, tmp_pt_l, 3, cv::Scalar(0, 255, 0), -1); 
    cv::circle(result, tmp_pt_r, 3, cv::Scalar(0, 0, 255), -1); 
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

// 메인 루프: 라이다 데이터 들어올 때마다 실행되는 콜백 함수
void LineDetector::mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto startTime = chrono::steady_clock::now(); // FPS 계산하려고 시작 시간 기록
    int count = scan->scan_time / scan->time_increment; // 한 바퀴 스캔 데이터 포인트 개수 계산
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    float scale = 8.0; // 라이다 거리를 이미지 픽셀 크기에 맞게 키울 배율
    for (int i = 0; i < count; i++) {
        // 극좌표계(거리, 각도)를 직교좌표계(x, y)로 변환함 (화면 중앙 250,250 기준)
        float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
        float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
        cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
    }
    
    // 로봇 진행 방향에 맞추려고 이미지를 180도 돌리고 좌우 반전하는 작업
    cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
    cv::Mat result;
    cv::warpAffine(scan_video, result, rotate, scan_video.size()); // 180도 회전 적용
    cv::flip(result, result, 1); // 좌우 반전
    cv::namedWindow("Lidar 2D map");

    // 위에서 만든 2D 맵 이미지를 흑백/이진화 전처리
    cv::Mat roi = preprocess_image(result);

    cv::Mat labels, stats, centroids;
    // 이진화된 이미지에서 객체들(라벨, 면적 크기, 중심점 등) 추출
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 양쪽 장애물 인덱스
    std::pair<int, int> targets = find_target_line(roi, stats, centroids);

    int left_idx = targets.first;
    int right_idx = targets.second;

    draw_result(result, stats, centroids, left_idx, right_idx);
    // 중앙(250)과 양쪽 타겟 중앙값의 차이를 계산해서 조향 '오차(Error)' 값 만듦
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    // 키보드 눌렸는지 확인
    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false; // 'q' 누르면 정지 모드
        else if(ch == 's') mode = true; // 's' 누르면 주행(출발) 모드
    }

    if (mode) { // true일 때
        // 오차가 0이거나 너무 심하면(-60, 60 벗어남) 회전 안 하고 일정하게 직진
        if (error == 0 || error < -60 || error > 60){
            vel.x = 50;
            vel.y = -50;
        }
        else { // 오차가 적당하면 P(비례) 제어 넣어서 바퀴 속도 조절함 (조향)
            vel.x = 50 - k * error; 
            vel.y = -(50 + k * error);
        }
    } 
    else { // false
        vel.x = 0; 
        vel.y = 0; 
    }

    pub_->publish(vel);

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();
    static cv::VideoWriter video_writer; 
    static bool is_writer_initialized = false;

    if (!is_writer_initialized) {
        video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(result.cols, result.rows), true);
        if (video_writer.isOpened()) { 
            is_writer_initialized = true; 
            printf("Video Writer Initialized Successfully.\n");
        } else {
            printf("Failed to open Video Writer.\n");
        }
    }

    if (is_writer_initialized) {
        video_writer.write(result);
    }

    cv::imshow("Lidar 2D map", result);
    cv::waitKey(1);

    // 터미널 창에 현재 오차, 양쪽 바퀴 속도, 연산 걸린 시간 출력함
    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);
}
