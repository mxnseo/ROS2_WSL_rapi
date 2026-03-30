# ROS2 로보틱스 수업 - 차선 추종 (실제 로봇)

## 과제 개요

ROS2 환경에서 카메라 이미지를 구독하여 좌/우 두 차선을 동시에 검출하고,  
차선 중심과 화면 중심의 오차를 기반으로 실제 로봇의 좌/우 모터 속도를 제어합니다.

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
    int h = frame_binary.rows;
    int w = frame_binary.cols;
    int roi_y = h * 3 / 4;
    int roi_h = h / 4;
```
- ROI 영역을 해상도에 맞게 동적으로 계산 (하단 1/4 영역 사용)
- 고정값 대신 비율로 계산하여 해상도 변화에 유연하게 대응

```cpp
    cv::Scalar bright_avg = cv::mean(frame_gray);
    frame_gray = frame_gray + (90 - bright_avg[0]);
```
- 목표 밝기 90을 기준으로 평균 밝기와의 차이만큼 보정 (실제 환경 조명 조건 반영)

```cpp
    cv::threshold(frame_gray, frame_binary, 120, 255, cv::THRESH_BINARY);
```
- 픽셀값 120을 기준으로 이진화

---

#### 3. 라인 탐색 - `find_target_line()`

```cpp
    int half_w = roi.cols / 2;
```
- 화면을 좌/우로 나누는 기준선 (동적 계산)

##### 첫 실행 처리

```cpp
    if (first_run_) {
        if (cx < half_w) {
            int dist = abs(roi.cols / 4 - cx);   // 왼쪽: x=1/4 기준
        } else {
            int dist = abs(roi.cols * 3 / 4 - cx); // 오른쪽: x=3/4 기준
        }
    }
```
- 화면 왼쪽 절반에서는 1/4 지점과 가장 가까운 객체를 왼쪽 차선으로 초기화
- 화면 오른쪽 절반에서는 3/4 지점과 가장 가까운 객체를 오른쪽 차선으로 초기화

```cpp
    if (l_idx != -1 && r_idx != -1) first_run_ = false;
```
- 좌/우 차선을 **모두 찾았을 때만** 첫 실행 플래그 해제 (한쪽만 찾은 경우 재시도)

##### 연속 추적

```cpp
    int search_radius = 50;
```
- 이전 위치에서 50픽셀 이내의 객체만 같은 차선으로 인식

```cpp
    // 좌측 탐색 (x < half_w 영역만)
    if (x >= half_w) continue;

    // 우측 탐색 (x >= half_w 영역만, idx_l 제외)
    if (i == idx_l) continue;
    if (x < half_w) continue;
```
- 좌측 탐색은 왼쪽 영역만, 우측 탐색은 오른쪽 영역만 보도록 분리
- 우측 탐색 시 이미 왼쪽 차선으로 선택된 객체는 제외

```cpp
    int dist_l = cv::norm(cv::Point(x, y) - tmp_pt_l);
    int dist_r = cv::norm(cv::Point(x, y) - tmp_pt_r);
```
- 현재 객체와 이전 프레임의 좌/우 차선 위치 사이의 유클리드 거리 계산

---

#### 4. 시각화 - `draw_result()`

```cpp
    cv::Scalar color(255, 0, 0); // 파란색: 추적 안됨
    if (i == left_idx || i == right_idx) color = cv::Scalar(0, 0, 255); // 빨간색
```
- 타겟 차선은 빨간색, 나머지 객체는 파란색으로 구분 표시

```cpp
    cv::circle(result, tmp_pt_l, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(result, tmp_pt_r, 5, cv::Scalar(0, 0, 255), -1);
```
- 차선을 놓쳤을 때도 기억된 마지막 위치를 빨간 점으로 항상 표시

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
- `k = 0.2` : 오차에 대한 속도 보정 비율
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
| 조향 게인 (k) | 0.2 |
| 탐색 반경 | 50 픽셀 |

```
[INFO] [camsub_wsl_12]: Received Image : err:-10, leftvel: 98.00, rightvel: -102.00, time:4.12 ms
[INFO] [camsub_wsl_12]: Received Image : err:3,  leftvel: 100.60, rightvel: -99.40, time:3.87 ms
```

- `err` : 양수면 차선 중심이 왼쪽, 음수면 오른쪽으로 치우침
- `leftvel` / `rightvel` : 각 모터에 전달되는 속도값
- `time` : 한 프레임 처리 시간 (밀리초)
