# ROS2 로보틱스 수업 - 차선 추종 (시뮬레이터)

## 과제 개요

ROS2 환경에서 카메라 이미지를 구독하여 좌/우 두 차선을 동시에 검출하고,  
차선 중심과 화면 중심의 오차를 기반으로 시뮬레이터 로봇의 좌/우 모터 속도를 제어합니다.

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
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_l(160, 60), tmp_pt_r(480, 60), first_run_(true), mode(false) {
```
- `Node("camsub_wsl_12")` : 노드 이름 설정
- `tmp_pt_l(160, 60)` : 왼쪽 차선의 초기 추적 위치 (화면 좌측 1/4 지점)
- `tmp_pt_r(480, 60)` : 오른쪽 차선의 초기 추적 위치 (화면 우측 3/4 지점)
- `first_run_(true)` : 첫 실행 플래그 초기화
- `mode(false)` : 로봇 구동 모드 초기값 (정지 상태)

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
    cv::Mat roi = frame_color(cv::Rect(
        cv::Point(0, frame_color.rows * 3 / 4),
        cv::Point(frame_color.cols, frame_color.rows)
    )).clone();
```
- 입력 이미지의 하단 1/4 영역을 ROI로 잘라냄
- 시작점: `(0, 높이×3/4)`, 끝점: `(너비, 높이)` — 비율 기반으로 해상도에 무관하게 동작

```cpp
    cv::Scalar bright_avg = cv::mean(roi);
    roi = roi + (100 - bright_avg[0]);
```
- 목표 밝기 100을 기준으로 평균 밝기와의 차이만큼 보정

```cpp
    cv::threshold(roi, roi, 120, 255, cv::THRESH_BINARY);
```
- 픽셀값 120을 기준으로 이진화

```cpp
    if (first_run_) {
        tmp_pt_l = cv::Point(roi.cols / 4, roi.rows / 2);
        tmp_pt_r = cv::Point(roi.cols * 3 / 4, roi.rows / 2);
    }
```
- 첫 실행 시 초기 탐색 기준점을 ROI 크기에 맞게 동적으로 설정 (좌: 1/4, 우: 3/4 지점)

---

#### 3. 라인 탐색 - `find_target_line()`

```cpp
    double left_best  = roi.cols;
    double right_best = roi.cols;
```
- 가장 가까운 거리의 초기값을 화면 너비로 설정 (모든 거리보다 크게 초기화)

```cpp
    double d_left  = cv::norm(cv::Point(x, y) - tmp_pt_l);
    double d_right = cv::norm(cv::Point(x, y) - tmp_pt_r);
```
- 각 객체와 이전 프레임의 좌/우 차선 위치 사이의 유클리드 거리 계산

```cpp
    if (d_left < left_best && d_left <= 150) {
        left_best = d_left; left_idx = i;
    }
    if (d_right < right_best && d_right <= 150) {
        right_best = d_right; right_idx = i;
    }
```
- 150픽셀 이내에서 좌/우 기준점과 가장 가까운 객체를 각각 후보로 선택
- 실제 패키지(탐색 반경 50px)보다 넓게 설정하여 시뮬레이터 환경에 유연하게 대응

```cpp
    bool left_ok  = (left_idx  != -1 && left_best  <= 150);
    bool right_ok = (right_idx != -1 && right_best <= 150);
    if (left_ok || right_ok) first_run_ = false;
```
- 좌/우 중 **하나라도 찾으면** 첫 실행 플래그 해제 (실제 패키지와 다른 점)

```cpp
    } else { left_idx = -1; }
    } else { right_idx = -1; }
```
- 탐색 범위 밖이면 인덱스를 -1로 설정하고 `tmp_pt_`는 이전 값 유지

---

#### 4. 시각화 - `draw_result()`

```cpp
    cv::Scalar color(255, 0, 0);
    if (l_valid && i == left_idx) color = cv::Scalar(0, 0, 255);
    else if (r_valid && i == right_idx) color = cv::Scalar(0, 0, 255);
```
- 타겟 차선은 빨간색, 나머지는 파란색으로 구분

```cpp
    if (!l_valid) {
        cv::rectangle(result, cv::Rect(tmp_pt_l.x - 2, tmp_pt_l.y - 2, 4, 4), cv::Scalar(0, 0, 255), 2);
    }
    if (!r_valid) {
        cv::rectangle(result, cv::Rect(tmp_pt_r.x - 2, tmp_pt_r.y - 2, 4, 4), cv::Scalar(0, 0, 255), 2);
    }
```
- 차선을 놓쳤을 때 마지막 위치에 작은 빨간 사각형(4x4px)을 표시하여 위치 유지 확인

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
    int error = center_x - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
```
- 좌/우 차선의 중간점을 기준으로 화면 중심과의 오차 계산
- 양수: 차선 중심이 왼쪽으로 치우침, 음수: 오른쪽으로 치우침

```cpp
    if(ch == 'q') mode = false;
    else if(ch == 's') mode = true;
```
- `s` 키: 로봇 구동 시작, `q` 키: 정지

```cpp
    vel.x =  100 - k * error;  // 왼쪽 모터
    vel.y = -(100 + k * error); // 오른쪽 모터
```
- `k = 0.18` : 시뮬레이터 환경에 맞게 조정된 오차 보정 비율 (실제 로봇의 0.2보다 작음)
- 오차가 클수록 한쪽 모터를 빠르게, 반대쪽을 느리게 하여 조향

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
| ROI 영역 | 하단 1/4 (해상도 비례) |
| 조향 게인 (k) | 0.18 |
| 탐색 반경 | 150 픽셀 |

```
[INFO] [camsub_wsl_12]: Received Image : err:-10, leftvel: 98.20, rightvel: -101.80, time:3.95 ms
[INFO] [camsub_wsl_12]: Received Image : err:4,  leftvel: 100.72, rightvel: -99.28, time:3.61 ms
```

- `err` : 양수면 차선 중심이 왼쪽, 음수면 오른쪽으로 치우침
- `leftvel` / `rightvel` : 각 모터에 전달되는 속도값
- `time` : 한 프레임 처리 시간 (밀리초)
