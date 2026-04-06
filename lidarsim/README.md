# ROS2 로보틱스 수업 - 라이다 기반 주행 (시뮬레이터)

## 과제 개요

ROS2 환경에서 카메라 이미지를 구독하여 라이다 스타일의 2D 맵으로 변환하고,  
Connected Components 알고리즘으로 장애물을 검출하여 시뮬레이터 로봇의 좌/우 모터 속도를 제어합니다.

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
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
```
- `Node("camsub_wsl_12")` : 노드 이름 설정
- `tmp_pt_l(125, 125)` : 왼쪽 장애물 초기 위치 (화면 좌측 1/4 지점)
- `tmp_pt_r(375, 125)` : 오른쪽 장애물 초기 위치 (화면 우측 3/4 지점)

```cpp
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", qos_profile, bind(&LineDetector::mysub_callback, this, placeholders::_1));
```
- 실제 로봇(lidardrive)과 달리 카메라 압축 이미지 토픽(`image/compressed_12`)을 구독

```cpp
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
```
- `topic_dxlpub` 토픽으로 좌/우 모터 속도 퍼블리시

---

#### 2. 이미지 전처리 - `preprocess_image()`

```cpp
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);
```
- 카메라 이미지를 흑백으로 변환 후 반전 이진화
- `THRESH_BINARY_INV`: 어두운 영역(장애물)을 흰색으로 반전하여 객체로 인식

```cpp
    return frame_binary(cv::Rect(0, 0, 500, 250));
```
- 500x250 영역을 ROI로 사용 (전방 장애물 탐색 영역)

---

#### 3. 장애물 탐색 - `find_target_line()`

```cpp
    if (area <= 30) continue;
```
- 면적 30 이하인 객체는 노이즈로 간주하고 무시
- 실제 로봇(lidardrive)은 면적 필터 없이 모든 객체를 탐색

```cpp
    cv::Point robot_pos(250, 250);
    double dist = cv::norm(robot_pos - obj_pos);

    if (cx < 250) {
        if (dist < min_dist_l) { l_idx = i; }
    } else {
        if (dist < min_dist_r) { r_idx = i; }
    }
```
- 화면 중앙(250, 250)을 로봇 위치로 설정
- x좌표 기준으로 좌/우 영역을 나누고 각 영역에서 가장 가까운 객체 선택

```cpp
    } else { tmp_pt_l = cv::Point(0, 0); }
    } else { tmp_pt_r = cv::Point(500, 0); }
```
- 해당 방향에 장애물이 없으면 기준점을 화면 가장자리로 설정

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

```cpp
    draw_result(frame_color, stats, centroids, left_idx, right_idx);
```
- 원본 컬러 이미지 위에 직접 시각화 결과를 덧그림 (lidardrive와 다른 점)

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
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;
```
- 압축 이미지를 OpenCV Mat으로 디코딩, 실패 시 함수 종료

```cpp
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
```
- 화면 중앙(250)과 좌/우 장애물 중간점의 x좌표 차이로 오차 계산

```cpp
    if (mode) {
        vel.x = 100 - k * error;
        vel.y = -(100 + k * error);
    }
```
- 기본 속도 100에서 오차에 비례해 좌/우 모터 속도 차등 조정
- `k = 0.3` : 오차에 대한 속도 보정 비율 (실제 로봇의 0.45보다 낮게 설정)

```cpp
    cv::imshow("frame_color", frame_color);
```
- 원본 카메라 이미지(시각화 결과 포함)를 화면에 출력 (별도 라이다 맵 창 없음)

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 구독 토픽 | `image/compressed_12` |
| 퍼블리시 토픽 | `topic_dxlpub` |
| 노드 이름 | `camsub_wsl_12` |
| ROI 영역 | 500x250 |
| 조향 게인 (k) | 0.3 |
| 노이즈 필터 면적 | 30 픽셀 이하 제거 |

```
[INFO] [camsub_wsl_12]: Received Image : err:-5, leftvel: 101.50, rightvel: -98.50, time:3.84 ms
[INFO] [camsub_wsl_12]: Received Image : err:12, leftvel: 96.40,  rightvel: -103.60, time:3.71 ms
```

- `err` : 양수면 주행 경로가 왼쪽, 음수면 오른쪽으로 치우침
- `leftvel` / `rightvel` : 각 모터에 전달되는 속도값
- `time` : 한 프레임 처리 시간 (밀리초)
