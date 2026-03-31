# ROS2 로보틱스 수업 - 라이다 2D 맵 시각화

## 과제 개요

ROS2 환경에서 LiDAR 스캔 데이터를 구독하여 2D 맵으로 시각화하고,  
결과 영상을 mp4 파일로 저장합니다.

---

## 소스 코드 설명 (`lidarplot.cpp`)

### 1. 매크로 정의

```cpp
#define RAD2DEG(x) ((x)*180./M_PI)
```
- 라디안 단위의 각도를 도(degree) 단위로 변환하는 매크로

---

### 2. 스캔 콜백 - `scanCb()`

```cpp
int count = scan->scan_time / scan->time_increment;
```
- 한 번의 스캔에서 측정된 총 포인트 수 계산
- `scan_time`: 한 회전에 걸린 시간 / `time_increment`: 포인트 간 시간 간격

```cpp
cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
```
- 500x500 픽셀의 흰색 배경 이미지 생성 (3채널 컬러)

```cpp
cv::line(scan_video, cv::Point(250, 240), cv::Point(250, 260), cv::Scalar(255, 0, 0), 1);
cv::line(scan_video, cv::Point(240, 250), cv::Point(260, 250), cv::Scalar(255, 0, 0), 1);
```
- 화면 중앙(250, 250)에 파란색 십자선을 그려 로봇 위치 표시

#### 극좌표 → 직교좌표 변환

```cpp
float scale = 8.0;
```
- 라이다 거리 데이터를 화면에 맞게 축소하는 배율값. 값이 클수록 더 넓은 범위를 표시

```cpp
float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
```
- i번째 포인트의 각도를 라디안에서 도(degree)로 변환

```cpp
float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
```
- 극좌표 `(r, θ)`를 직교좌표 `(x, y)`로 변환
- `x = r × sin(θ)`, `y = r × cos(θ)` (이미지 좌표계 기준)
- 화면 중앙(250, 250)을 원점으로 설정

```cpp
cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
```
- 변환된 좌표에 반지름 1픽셀의 빨간 점을 찍어 스캔 포인트 표시

#### 영상 회전 및 반전

```cpp
cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
cv::warpAffine(scan_video, result, rotate, scan_video.size());
cv::flip(result, result, 1);
```
- 중앙을 기준으로 180도 회전 후 좌우 반전
- 라이다 좌표계와 화면 좌표계의 방향 차이를 보정

---

### 3. 동영상 저장

```cpp
static cv::VideoWriter video_writer;
static bool is_writer_initialized = false;
```
- `static`으로 선언하여 콜백 함수가 반복 호출되어도 객체가 유지됨
- `is_writer_initialized`: 중복 초기화 방지 플래그

```cpp
if (!is_writer_initialized) {
    video_writer.open("lidar_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(result.cols, result.rows), true);
```
- 첫 번째 프레임이 들어왔을 때 한 번만 VideoWriter 초기화
- 저장 파일명: `lidar_video.mp4`, 코덱: mp4v, FPS: 10

```cpp
    if (video_writer.isOpened()) {
        is_writer_initialized = true;
    }
```
- 초기화 성공 시 플래그를 `true`로 설정 (없으면 매 프레임마다 재초기화 시도)

```cpp
if (is_writer_initialized) {
    video_writer.write(result);
}
```
- 초기화가 완료된 경우에만 현재 프레임을 파일에 기록

---

### 4. `main()`

```cpp
auto node = rclcpp::Node::make_shared("sllidar_client");
```
- `sllidar_client` 이름의 노드 생성

```cpp
auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), scanCb);
```
- `scan` 토픽을 구독하고 `scanCb` 콜백 함수 등록
- `SensorDataQoS()`: 센서 데이터에 적합한 QoS 설정 (Best Effort, 빠른 전송 우선)

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 구독 토픽 | `scan` |
| 노드 이름 | `sllidar_client` |
| 출력 창 | `Lidar 2D map` |
| 저장 파일 | `lidar_video.mp4` |
| 화면 배율 (scale) | 8.0 |

```
[SLLIDAR INFO]: I heard a laser scan laser[360]:
[SLLIDAR INFO]: angle_range : [-180.000000, 180.000000]
[SLLIDAR INFO]: angle-distance : [-180.000000, 0.823000]
[SLLIDAR INFO]: angle-distance : [-179.000000, 0.821000]
...
Video Writer Initialized Successfully.
```

- 흰색 배경에 빨간 점으로 스캔 포인트를 표시
- 화면 중앙의 파란 십자선이 로봇 위치
