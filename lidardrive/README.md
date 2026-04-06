# ROS2 로보틱스 수업 - 라이다 기반 주행 (실제 로봇)

## 과제 개요

ROS2 환경에서 LiDAR 스캔 데이터를 구독하여 2D 맵으로 변환하고,  
Connected Components 알고리즘으로 장애물을 검출하여 실제 로봇의 좌/우 모터 속도를 제어합니다.

---

## 소스 코드 설명

### `main.cpp`

```cpp
rclcpp::init(argc, argv);
```
- ROS2 통신 시스템 초기화

```cpp
auto node = make_shared<LineDetector>();
rclcpp::spin(node);
rclcpp::shutdown();
```
- `LineDetector` 노드 생성 후 실행, 종료 시 정리

---

### `sub.cpp`

#### 1. 생성자 - `LineDetector::LineDetector()`

```cpp
LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
```
- `Node("sllidar_client")` : 노드 이름 설정
- `tmp_pt_l(125, 125)` : 왼쪽 장애물 초기 위치 (화면 좌측 1/4 지점)
- `tmp_pt_r(375, 125)` : 오른쪽 장애물 초기 위치 (화면 우측 3/4 지점)

```cpp
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile, bind(&LineDetector::mysub_callback, this, placeholders::_1));
```
- 카메라 대신 LiDAR의 `scan` 토픽을 구독

```cpp
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
```
- `topic_dxlpub` 토픽으로 좌/우 모터 속도 퍼블리시

---

#### 2. 이미지 전처리 - `preprocess_image()`

```cpp
    cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);
```
- 라이다 시각화 이미지를 흑백으로 변환 후 반전 이진화
- `THRESH_BINARY_INV`: 픽셀값 100 이하(어두운 점 = 장애물)를 흰색으로 변환하여 객체로 인식

```cpp
    return frame_binary(cv::Rect(0, 0, 500, 250));
```
- 전체 500x500 이미지에서 상단 절반(로봇 전방 영역)만 ROI로 사용

---

#### 3. 장애물 탐색 - `find_target_line()`

```cpp
    cv::Point robot_pos(250, 250);
```
- 라이다 맵에서 로봇 위치는 항상 화면 중앙(250, 250)

```cpp
    double dist = cv::norm(robot_pos - obj_pos);

    if (cx < 250) {
        if (dist < min_dist_l) { l_idx = i; }
    } else {
        if (dist < min_dist_r) { r_idx = i; }
    }
```
- x좌표가 250 미만이면 왼쪽, 이상이면 오른쪽 장애물로 분류
- 각 영역에서 로봇과 가장 가까운 객체를 좌/우 타겟으로 선택

```cpp
    } else { tmp_pt_l = cv::Point(0, 0); }     // 못 찾으면 왼쪽 구석으로
    } else { tmp_pt_r = cv::Point(500, 0); }   // 못 찾으면 오른쪽 구석으로
```
- 해당 방향에 장애물이 없으면 기준점을 화면 가장자리로 설정하여 오차 계산에 반영

---

#### 4. 시각화 - `draw_result()`

```cpp
    if (i == left_idx) color = cv::Scalar(0, 255, 0);  // 왼쪽: 초록
    else               color = cv::Scalar(0, 0, 255);  // 오른쪽: 빨강
```
- 왼쪽 장애물은 초록색, 오른쪽 장애물은 빨간색으로 구분 표시

```cpp
    cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);
    cv::arrowedLine(result, robot_pos, r_box_bottom_left,  cv::Scalar(0, 0, 255), 1);
```
- 로봇 위치(중앙)에서 각 장애물을 향한 화살표를 그려 방향 표시

```cpp
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::arrowedLine(result, robot_pos, cv::Point(target_x, 100), cv::Scalar(255, 0, 0), 1);
```
- 좌/우 장애물의 중간점을 향한 파란 화살표로 목표 주행 방향 표시

---

#### 5. 키보드 입력 - `getch()` / `kbhit()`

```cpp
    newt.c_lflag &= ~(ICANON | ECHO);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
```
- 터미널 raw 모드 + non-blocking 설정으로 엔터 없이 즉시 키 입력 감지

---

#### 6. 메인 콜백 - `mysub_callback()`

```cpp
    float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
    float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
```
- 극좌표 `(r, θ)`를 직교좌표 `(x, y)`로 변환하여 2D 맵에 빨간 점으로 표시

```cpp
    cv::warpAffine(scan_video, result, rotate, scan_video.size());
    cv::flip(result, result, 1);
```
- 180도 회전 후 좌우 반전으로 라이다 좌표계와 화면 좌표계의 방향 보정

```cpp
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
```
- 화면 중앙(250)과 좌/우 장애물 중간점의 x좌표 차이로 오차 계산

```cpp
    if (error == 0 || error < -60 || error > 60) {
        vel.x = 50;
        vel.y = -50;
    } else {
        vel.x = 50 - k * error;
        vel.y = -(50 + k * error);
    }
```
- 오차가 60픽셀 이상으로 크거나 0이면 직진 (이상 상황 처리)
- 그 외에는 오차에 비례해 좌/우 모터 속도를 차등 조정하여 조향
- `k = 0.45` : 오차에 대한 속도 보정 비율

```cpp
    video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, ...);
    if (video_writer.isOpened()) { is_writer_initialized = true; }
```
- 첫 프레임에서 한 번만 VideoWriter 초기화, 이후 매 프레임을 mp4로 저장

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 구독 토픽 | `scan` |
| 퍼블리시 토픽 | `topic_dxlpub` |
| 노드 이름 | `sllidar_client` |
| ROI 영역 | 500x250 (라이다 맵 상단 절반) |
| 조향 게인 (k) | 0.45 |
| 저장 파일 | `lidar_video.mp4` |

```
[INFO] [sllidar_client]: Received Image : err:-8, leftvel: 53.60, rightvel: -46.40, time:5.21 ms
[INFO] [sllidar_client]: Received Image : err:0,  leftvel: 50.00, rightvel: -50.00, time:4.98 ms
Video Writer Initialized Successfully.
```

- `err` : 양수면 주행 경로가 왼쪽, 음수면 오른쪽으로 치우침
- `leftvel` / `rightvel` : 각 모터에 전달되는 속도값
- `time` : 한 프레임 처리 시간 (밀리초)
