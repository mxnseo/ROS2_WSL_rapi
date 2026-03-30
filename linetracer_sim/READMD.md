# ROS2 로보틱스 수업 - 라인 트레이서 (시뮬레이터 / 실제 로봇 공용)

## 과제 개요

ROS2 환경에서 카메라 이미지를 구독하여 단일 라인을 검출하고,  
화면 중심과의 오차를 기반으로 로봇의 좌/우 모터 속도를 제어합니다.  
시뮬레이터와 실제 로봇 환경 모두에서 동작합니다.

---

## 소스 코드 설명

### `main.cpp`

```cpp
rclcpp::init(argc, argv);
```
- ROS2 통신 시스템 초기화

```cpp
auto node = make_shared<LineDetector>();
```
- `LineDetector` 노드 객체를 스마트 포인터로 생성

```cpp
rclcpp::spin(node);
```
- 노드를 실행 상태로 유지하며 콜백 대기

```cpp
rclcpp::shutdown();
```
- 노드 종료 시 ROS2 시스템 정리

---

### `sub.cpp`

#### 1. 생성자 - `LineDetector::LineDetector()`

```cpp
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) {
```
- `Node("camsub_wsl_12")` : 노드 이름 설정
- `tmp_pt_(320, 60)` : 이전 프레임의 라인 위치를 기억하는 변수를 화면 중앙(320, 60)으로 초기화
- `first_run_(true)` : 첫 실행 플래그 초기화

```cpp
    vel.x = 0; vel.y = 0; vel.z = 0;
```
- 모터 속도 벡터 초기화

```cpp
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", qos_profile, bind(&LineDetector::mysub_callback, this, placeholders::_1));
```
- `image/compressed_12` 토픽을 구독하고 콜백 함수 바인딩

```cpp
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
```
- `topic_dxlpub` 토픽으로 좌/우 모터 속도 퍼블리시

---

#### 2. 이미지 전처리 - `preprocess_image()`

```cpp
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
```
- BGR 컬러 이미지를 흑백(Grayscale)으로 변환

```cpp
    cv::Scalar bright_avg = cv::mean(frame_gray);
    frame_gray = frame_gray + (120 - bright_avg[0]);
```
- 목표 밝기 120을 기준으로 평균 밝기와의 차이만큼 보정

```cpp
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);
```
- 픽셀값 130을 기준으로 이진화

```cpp
    return frame_binary(cv::Rect(0, 360, 640, 120));
```
- y=360부터 높이 120픽셀 영역을 ROI로 잘라서 반환 (640x480 해상도 기준 하단 영역)

---

#### 3. 라인 탐색 - `find_target_line()`

##### 첫 실행 처리

```cpp
    if (first_run_) {
        for (int i = 1; i < cnt; i++) {
            int cx = cvRound(centroids.at<double>(i, 0));
            int dist_from_center = abs(320 - cx);
```
- 첫 실행 시 화면 중앙(320)과 가장 가까운 라인을 초기 타겟으로 설정
- `i = 1`부터 시작하여 0번 배경 인덱스 제외

```cpp
            if (area > 100) { ... }
```
- 면적 100 이하인 객체는 노이즈로 간주하고 무시

```cpp
        if (best_idx != -1) {
            tmp_pt_ = cv::Point(...);
            first_run_ = false;
        }
```
- 타겟을 찾았으면 `tmp_pt_`를 해당 위치로 초기화하고 플래그 해제

##### 연속 추적

```cpp
    int search_radius = 60;
```
- 이전 위치에서 60픽셀 이상 떨어진 객체는 다른 라인으로 판단하여 무시

```cpp
    int dist = cv::norm(cv::Point(x, y) - tmp_pt_);
```
- 현재 객체와 이전 프레임의 타겟 위치 사이의 유클리드 거리 계산

```cpp
    if (dist < min_dist && dist <= search_radius) {
        min_dist = dist;
        min_idx = i;
    }
```
- 탐색 범위 안에서 이전 위치와 가장 가까운 객체를 타겟으로 선택

```cpp
    if (min_idx != -1) {
        tmp_pt_ = cv::Point(...);
    }
    return min_idx;
```
- 타겟을 찾았으면 `tmp_pt_`를 갱신하고 인덱스 반환, 못 찾았으면 `-1` 반환

---

#### 4. 시각화 - `draw_result()`

```cpp
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
```
- 이진화 이미지를 다시 컬러로 변환 (색깔 표시를 위해)

```cpp
    cv::rectangle(result, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 1);
    cv::circle(result, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
```
- 모든 객체(라인 후보)를 파란색 사각형과 점으로 표시

```cpp
    if (target_idx != -1) {
        cv::rectangle(result, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
    }
    cv::circle(result, tmp_pt_, 5, cv::Scalar(0, 0, 255), -1);
```
- 타겟 라인은 빨간색 사각형으로 강조
- 라인을 놓쳤을 때도 `tmp_pt_`(마지막 기억 위치)에 빨간 점을 항상 표시

---

#### 5. 키보드 입력 - `getch()` / `kbhit()`

```cpp
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
```
- 터미널을 raw 모드로 변경하여 엔터 없이 즉시 키 입력 감지

```cpp
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
```
- `kbhit()`에서 non-blocking 모드로 설정하여 키 입력이 없어도 즉시 리턴

---

#### 6. 메인 콜백 - `mysub_callback()`

```cpp
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;
```
- 압축된 이미지 메시지를 OpenCV Mat으로 디코딩, 실패 시 함수 종료

```cpp
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
```
- 이진화 이미지에서 연결된 영역을 레이블링
- `stats`: 각 객체의 위치, 크기, 면적 / `centroids`: 각 객체의 중심 좌표

```cpp
    int error = (320 - tmp_pt_.x);
```
- 화면 중앙(320)과 현재 라인 위치의 x좌표 차이로 오차 계산
- 양수: 라인이 왼쪽으로 치우침, 음수: 오른쪽으로 치우침

```cpp
    if(ch == 'q') mode = false;
    else if(ch == 's') mode = true;
```
- `s` 키: 로봇 구동 시작, `q` 키: 정지

```cpp
    vel.x = 110 - k * error;   // 왼쪽 모터
    vel.y = -(110 + k * error); // 오른쪽 모터
```
- 기본 속도 110에서 오차에 비례하여 좌/우 모터 속도를 차등 조정
- `k = 0.07` : 오차에 대한 속도 보정 비율

```cpp
    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);
```
- 오차값, 좌/우 모터 속도, 처리 시간을 로그로 출력

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 구독 토픽 | `image/compressed_12` |
| 퍼블리시 토픽 | `topic_dxlpub` |
| 노드 이름 | `camsub_wsl_12` |
| ROI 영역 | y=360 ~ 480 (640x480 기준) |
| 조향 게인 (k) | 0.07 |
| 탐색 반경 | 60 픽셀 |

```
[INFO] [camsub_wsl_12]: Received Image : err:-12, leftvel: 110.84, rightvel: -109.16, time:4.03 ms
[INFO] [camsub_wsl_12]: Received Image : err:5,   leftvel: 109.65, rightvel: -110.35, time:3.78 ms
```

- `err` : 양수면 라인이 왼쪽, 음수면 오른쪽으로 치우침
- `leftvel` / `rightvel` : 각 모터에 전달되는 속도값
- `time` : 한 프레임 처리 시간 (밀리초)
