#include "lidardrive/sub.hpp" // 헤더 파일 포함함

// 생성자: 노드 이름 설정, 초기 좌표 세팅, 플래그 및 주행 모드(mode) 초기화함
LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
    vel.x = 0; // x축 속도 0으로 초기화함
    vel.y = 0; // y축 속도 0으로 초기화함
    vel.z = 0; // z축 속도 0으로 초기화함

    // 통신 상태 안 좋으면 최근 10개까지만 보관하라는 설정임
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 라이다 스캔 데이터(LaserScan) 구독하는 Subscriber 만듦
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
        
    // 바퀴 굴릴 속도(Vector3) 발행하는 Publisher 만듦
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "topic_dxlpub",
        qos_profile);
}

// 2D 맵 이미지를 흑백/이진화 처리해서 전처리하는 함수임
cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat frame_gray; // 흑백 이미지 담을 변수
    cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY); // 컬러 이미지를 흑백으로 바꿈

    cv::Mat frame_binary; // 이진화(흑백 두 색깔뿐인) 이미지 담을 변수
    // 밝기 100을 기준으로 이진화하고, 색깔을 반전(INV)시킴 (배경 까맣게, 장애물 하얗게)
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);

    // 전체 이미지 말고 위쪽 절반(0~250 높이)만 잘라서 관심 영역(ROI)으로 반환함
    return frame_binary(cv::Rect(0, 0, 500, 250)); 
}

// 화면에서 양쪽 가장 가까운 장애물(타겟) 인덱스 찾는 함수임
std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows; // 검출된 객체 총 개수 가져옴
    int l_idx = -1; // 왼쪽 타겟 인덱스 (-1은 아직 못 찾았단 뜻)
    int r_idx = -1; // 오른쪽 타겟 인덱스
    
    double min_dist_l = 100000.0; // 왼쪽 최소 거리 (비교를 위해 엄청 큰 값 넣어둠)
    double min_dist_r = 100000.0; // 오른쪽 최소 거리

    cv::Point robot_pos(250, 250); // 로봇의 현재 위치 (화면 중앙 하단) 설정함

    // 0번은 배경이니까 빼고 1번 객체부터 끝까지 반복함
    for (int i = 1; i < cnt; i++) {

        int cx = cvRound(centroids.at<double>(i, 0)); // 객체의 중심점 x좌표
        int cy = cvRound(centroids.at<double>(i, 1)); // 객체의 중심점 y좌표
        cv::Point obj_pos(cx, cy); // 객체 위치 좌표로 묶음

        double dist = cv::norm(robot_pos - obj_pos); // 로봇과 객체 사이의 직선 거리 계산함

        if (cx < 250) { // 중심이 화면 왼쪽(x<250)에 있으면
            if (dist < min_dist_l) { // 기존 최소 거리보다 더 가까우면
                min_dist_l = dist; // 최소 거리 갱신함
                l_idx = i; // 제일 가까운 왼쪽 객체 인덱스 갱신함
            }
        }
        else { // 중심이 화면 오른쪽(x>=250)에 있으면
            if (dist < min_dist_r) { // 기존 최소 거리보다 더 가까우면
                min_dist_r = dist; // 최소 거리 갱신함
                r_idx = i; // 제일 가까운 오른쪽 객체 인덱스 갱신함
            }
        }
    }
    
    // [좌표 업데이트] 왼쪽 객체 찾았으면 임시 좌표에 저장
    if (l_idx != -1) {
        tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
    } else { // 못 찾았으면
        tmp_pt_l = cv::Point(0, 0); // 왼쪽 맨 위 구석으로 좌표 던져버림
    }

    // [좌표 업데이트] 오른쪽 객체 찾았으면 임시 좌표에 저장
    if (r_idx != -1) {
        tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
    } else { // 못 찾았으면
        tmp_pt_r = cv::Point(500, 0); // 오른쪽 맨 위 구석으로 좌표 던져버림
    }

    // 왼쪽, 오른쪽 타겟 인덱스 쌍으로 묶어서 반환함
    return std::make_pair(l_idx, r_idx);
}

// 화면에 박스, 화살표 등 시각화해서 그리는 함수임
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    // 입력 이미지가 흑백(1채널)이면 색깔 입히기 위해 컬러(3채널)로 변환함
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int cnt = stats.rows; // 검출된 객체 수

    cv::Point robot_pos(250, 250); // 로봇 위치 기준점

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

                // 객체 테두리에 네모 박스 그림 (두께 2)
                cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
                // 객체 중심에 점 찍음
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

// 리눅스 터미널에서 엔터 안 치고 키보드 한 알만 눌러도 바로 인식하게 해주는 함수임
int LineDetector::getch(void) 
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt); // 현재 터미널 설정 백업
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 터미널 모드 변경 (엔터 대기 없앰, 입력한 키 화면에 안 보이게 함)
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 새 설정 적용
    ch = getchar(); // 키보드 입력 하나 받음
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 터미널 설정 원래대로 복구
    return ch; // 누른 키 반환
}

// 프로그램 안 멈추고(Non-block) 키보드가 눌렸는지 확인만 해주는 함수임
bool LineDetector::kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt); // 터미널 백업
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 입력 대기, 에코 끔
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 설정 적용
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0); // 파일 디스크립터 상태 가져옴
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 논블로킹(기다리지 않음) 모드로 전환
    ch = getchar(); // 문자 읽기 시도
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 터미널 복구
    fcntl(STDIN_FILENO, F_SETFL, oldf); // 디스크립터 상태 복구
    if (ch != EOF) // 뭔가 눌려서 문자가 들어왔다면
    {
        ungetc(ch, stdin); // 읽은 거 다시 버퍼에 돌려놓음 (getch로 다시 읽어야 하니까)
        return true; // 키 눌렸음! (true 반환)
    }
    return false; // 안 눌렸음! (false 반환)
}

// 메인 루프: 라이다 데이터 들어올 때마다 실행되는 콜백 함수임
void LineDetector::mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto startTime = chrono::steady_clock::now(); // FPS 계산하려고 시작 시간 기록함
    int count = scan->scan_time / scan->time_increment; // 한 바퀴 스캔 데이터 포인트 개수 계산함

    // 500x500 사이즈의 하얀색 도화지(이미지) 만듦
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    float scale = 8.0; // 라이다 거리를 이미지 픽셀 크기에 맞게 키울 배율임
    for (int i = 0; i < count; i++) {
        // 극좌표계(거리, 각도)를 직교좌표계(x, y)로 변환함 (화면 중앙 250,250 기준)
        float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
        float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
        
        // 변환된 좌표 위치에 빨간색 작은 점(장애물) 찍음
        cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
    }
    
    // 로봇 진행 방향에 맞추려고 이미지를 180도 돌리고 좌우 반전하는 작업임
    cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
    cv::Mat result;
    cv::warpAffine(scan_video, result, rotate, scan_video.size()); // 180도 회전 적용
    cv::flip(result, result, 1); // 좌우 반전
    cv::namedWindow("Lidar 2D map"); // 화면 띄울 창 이름 지정

    // 위에서 만든 2D 맵 이미지를 흑백/이진화 전처리함
    cv::Mat roi = preprocess_image(result);

    cv::Mat labels, stats, centroids;
    // 이진화된 이미지에서 객체들(라벨, 면적 크기, 중심점 등) 추출함
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 타겟 찾는 함수 돌려서 양쪽 목표물 인덱스 가져옴
    std::pair<int, int> targets = find_target_line(roi, stats, centroids);

    int left_idx = targets.first; // 왼쪽 타겟 인덱스
    int right_idx = targets.second; // 오른쪽 타겟 인덱스

    // 타겟 정보 바탕으로 화면에 네모, 점, 화살표 다 그려줌
    draw_result(result, stats, centroids, left_idx, right_idx);

    // 중앙(250)과 양쪽 타겟 중앙값의 차이를 계산해서 조향 '오차(Error)' 값 만듦
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    // 키보드 눌렸는지 확인
    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false; // 'q' 누르면 정지 모드
        else if(ch == 's') mode = true; // 's' 누르면 주행(출발) 모드
    }

    if (mode) { // 출발 모드일 때
        // 오차가 0이거나 너무 심하면(-60, 60 벗어남) 회전 안 하고 일정하게 직진함
        if (error == 0 || error < -60 || error > 60){
            vel.x = 50;
            vel.y = -50;
        }
        else { // 오차가 적당하면 P(비례) 제어 넣어서 바퀴 속도 조절함 (조향)
            vel.x = 50 - k * error; // 오차만큼 왼쪽 바퀴 속도 뺌
            vel.y = -(50 + k * error); // 오차만큼 오른쪽 바퀴 속도 더함 (역방향)
        }
    } 
    else { // 정지 모드일 때
        vel.x = 0; // 모터 스톱
        vel.y = 0; // 모터 스톱
    }

    // 계산된 속도값 모터쪽으로 날림(publish)
    pub_->publish(vel);

    // 코드 실행 다 끝난 시간 기록하고, 시작 시간 빼서 총 걸린 시간(ms) 계산함
    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    // 동영상 저장용 변수를 static으로 선언함 (콜백 끝난다고 초기화 안 되게 유지)
    static cv::VideoWriter video_writer; 
    static bool is_writer_initialized = false; // 비디오 파일 열렸는지 체크하는 플래그

    // 프로그램 켜고 처음 한 번만 실행되는 비디오 저장 초기화 블록임
    if (!is_writer_initialized) {
        // mp4 코덱으로 10FPS, 화면 사이즈 맞춰서 컬러 영상으로 파일 엶
        video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(result.cols, result.rows), true);
        
        if (video_writer.isOpened()) { // 잘 열렸으면
            is_writer_initialized = true; // 열렸다고 플래그 true로 바꿈 (이거 중요함!)
            printf("Video Writer Initialized Successfully.\n"); // 성공 로그 찍음
        } else {
            printf("Failed to open Video Writer.\n"); // 실패 로그 찍음
        }
    }

    // 비디오 파일 정상적으로 열려있으면 현재 프레임 1장 녹화함
    if (is_writer_initialized) {
        video_writer.write(result);
    }

    // 터미널 창에 현재 오차, 양쪽 바퀴 속도, 연산 걸린 시간 출력함
    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);

    // 라이다 2D 맵 창 화면에 띄워줌
    cv::imshow("Lidar 2D map", result);
    cv::waitKey(1); // 1ms 대기 (이거 안 쓰면 OpenCV 창 렉걸리고 안 뜸)
}
