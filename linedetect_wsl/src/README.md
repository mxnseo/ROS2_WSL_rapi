# ROS2 로보틱스 수업 - 라인 검출 과제

## 과제 개요

ROS2 환경에서 카메라 이미지를 구독하여 라인을 검출하는 노드를 구현합니다.  
OpenCV의 Connected Components 알고리즘을 사용하여 이진화된 이미지에서 라인을 추적합니다.

-----

## 폴더 구조

```
ROS2_WSL_rapi/
└── linedetect_wsl/
    └── src/
        └── sub.cpp   # 라인 검출 메인 소스파일
```

-----

## 소스 코드 설명 (`sub.cpp`)

### 1. 생성자 - `LineDetector::LineDetector()`

```cpp
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) {
```

- `Node("camsub_wsl_12")` : ROS2 노드 이름을 `camsub_wsl_12`로 설정
- `tmp_pt_(320, 60)` : 이전 프레임의 라인 위치를 기억하는 변수를 화면 중앙(320, 60)으로 초기화
- `first_run_(true)` : 첫 실행 여부를 나타내는 플래그를 `true`로 초기화

```cpp
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
```

- QoS 프로파일을 설정. 최근 10개의 메시지만 유지 (`KeepLast(10)`)

```cpp
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
```

- `image/compressed_12` 토픽을 구독
- 메시지가 수신될 때마다 `mysub_callback` 함수가 호출되도록 바인딩

-----

### 2. 이미지 전처리 - `preprocess_image()`

```cpp
cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
```

- 컬러 이미지를 입력받아 전처리된 이진화 이미지를 반환하는 함수

```cpp
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
```

- BGR 컬러 이미지를 흑백(Grayscale) 이미지로 변환

```cpp
    cv::Scalar bright_avg = cv::mean(frame_gray);
    frame_gray = frame_gray + (100 - bright_avg[0]);
```

- `cv::mean()`으로 이미지 전체의 평균 밝기를 계산
- 목표 밝기(100)와의 차이만큼 픽셀값을 더하거나 빼서 밝기를 보정

```cpp
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);
```

- 픽셀값이 130보다 크면 255(흰색), 작으면 0(검정)으로 이진화

```cpp
    return frame_binary(cv::Rect(0, 240, 640, 120));
```

- 전체 이미지(640x360)에서 아래쪽 ROI 영역만 잘라서 반환
- `Rect(x, y, width, height)` : y=240부터 높이 120픽셀 영역 (240~360 행)

-----

### 3. 라인 탐색 - `find_target_line()`

```cpp
int LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;
```

- `stats.rows` : Connected Components로 찾은 객체(라벨)의 수

#### 첫 실행 처리

```cpp
    if (first_run_) {
        int best_idx = -1;
        int min_center_dist = 10000;
```

- 첫 실행일 때는 화면 중앙과 가장 가까운 라인을 초기 타겟으로 설정

```cpp
        for (int i = 1; i < cnt; i++) {
            int area = stats.at<int>(i, 4);
            if (area > 100) {
```

- `i = 1`부터 시작 : 0번 인덱스는 배경이므로 제외
- `stats.at<int>(i, 4)` : 해당 객체의 면적(픽셀 수). 100 이하는 노이즈로 무시

```cpp
                int cx = cvRound(centroids.at<double>(i, 0));
                int dist_from_center = abs(320 - cx);
```

- `centroids.at<double>(i, 0)` : 객체의 중심 x좌표
- 화면 중앙(320)과의 거리를 계산

```cpp
                if (dist_from_center < min_center_dist) {
                    min_center_dist = dist_from_center;
                    best_idx = i;
                }
```

- 중앙과 가장 가까운 객체를 갱신하며 탐색

```cpp
        if (best_idx != -1) {
            tmp_pt_ = cv::Point(cvRound(centroids.at<double>(best_idx, 0)), cvRound(centroids.at<double>(best_idx, 1)));
            first_run_ = false;
        }
```

- 타겟을 찾았으면 `tmp_pt_`를 해당 위치로 초기화하고 플래그를 `false`로 변경

#### 이후 프레임 처리 (연속 추적)

```cpp
    int min_idx = -1;
    int min_dist = roi.cols;
    int search_radius = 60;
```

- `search_radius = 60` : 이전 위치에서 60픽셀 이상 떨어진 객체는 다른 라인으로 판단하여 무시

```cpp
            int dist = cv::norm(cv::Point(x, y) - tmp_pt_);
```

- 현재 객체의 중심좌표와 이전 프레임의 타겟 위치 사이의 유클리드 거리 계산

```cpp
            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
```

- 탐색 범위 안에서 이전 위치와 가장 가까운 객체를 타겟으로 선택

```cpp
    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
    }
    return min_idx;
```

- 타겟을 찾았으면 `tmp_pt_`를 갱신하고 인덱스를 반환
- 못 찾았으면 `tmp_pt_`는 이전 값을 유지하고 `-1` 반환

-----

### 4. 시각화 - `draw_result()`

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
        ...
        cv::rectangle(result, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
    }
    cv::circle(result, tmp_pt_, 8, cv::Scalar(0, 0, 255), -1);
```

- 타겟으로 선택된 라인을 빨간색 사각형으로 강조
- `tmp_pt_` 위치(기억된 위치)에 빨간 점을 항상 표시 (타겟을 잃었을 때도 마지막 위치 표시)

-----

### 5. 메인 콜백 - `mysub_callback()`

```cpp
    auto startTime = chrono::steady_clock::now();
```

- 처리 시작 시각 기록 (처리 시간 측정용)

```cpp
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;
```

- 압축된 이미지 메시지를 OpenCV Mat으로 디코딩
- 디코딩 실패 시 함수 종료

```cpp
    cv::Mat roi = preprocess_image(frame_color);
```

- 전처리 함수를 호출하여 ROI 이진화 이미지를 획득

```cpp
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
```

- Connected Components 알고리즘으로 이진화 이미지에서 연결된 영역(객체)을 레이블링
- `stats` : 각 객체의 위치, 크기, 면적 정보
- `centroids` : 각 객체의 중심 좌표

```cpp
    int target_idx = find_target_line(roi, stats, centroids);
```

- 탐색 알고리즘으로 추적할 라인의 인덱스를 반환받음

```cpp
    int error = 320 - tmp_pt_.x;
```

- 화면 중앙(320)과 현재 라인 위치의 x좌표 차이로 오차(error) 계산
- 이 값은 로봇의 조향 제어에 사용됨

```cpp
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();
    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, time:%.2f ms", error, totalTime);
```

- 처리 시간을 밀리초 단위로 계산하여 로그로 출력

```cpp
    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
```

- 원본 컬러 이미지와 처리된 ROI 이미지를 창으로 출력

-----

## 실행 결과

|항목       |내용                    |
|---------|----------------------|
|구독 토픽    |`image/compressed_12` |
|노드 이름    |`camsub_wsl_12`       |
|ROI 영역   |640 x 120 (y: 240~360)|
|오차(error)|화면 중앙(320) - 라인 x좌표   |
|탐색 반경    |60 픽셀                 |

```
[INFO] [camsub_wsl_12]: Received Image : err:-15, time:3.24 ms
[INFO] [camsub_wsl_12]: Received Image : err:2, time:2.97 ms
```

- `err` : 양수면 라인이 왼쪽, 음수면 오른쪽으로 치우침
- `time` : 한 프레임 처리에 걸린 시간 (밀리초)

-----

## 소스 파일 링크

[sub.cpp 바로가기](https://github.com/mxnseo/ROS2_WSL_rapi/tree/main/linedetect_wsl/src)